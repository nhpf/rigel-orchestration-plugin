from __future__ import annotations

import subprocess
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any

# Kubernetes client library
from kubernetes import config
from kubernetes.client import ApiException, AppsV1Api, CoreV1Api

# Rigel imports
from rigel.loggers import get_logger
from rigel.models.builder import ModelBuilder

if TYPE_CHECKING:
    from rigel.models.application import Application
    from rigel.models.plugin import PluginRawData
    from rigel.models.rigelfile import RigelfileGlobalData

# Import our Pydantic models
from rigel.plugins import Plugin as PluginBase

from .models import KubernetesOrchestrationModel, OrchestrationPluginModel
from .utils.dict_operations import deep_merge

DEPLOYMENT_NAME = "rigel-k8s-application"
UNAVAILABLE_STATUS = 404
LOGGER = get_logger()


class OrchestrationPlugin(PluginBase):
    """A Rigel plugin that orchestrates a ROS application in Kubernetes."""

    def __init__(
        self,
        raw_data: PluginRawData,
        global_data: RigelfileGlobalData,
        application: Application,
        providers_data: dict[str, Any],
        shared_data: dict[str, Any] | None = None,
    ) -> None:
        """Create a new plugin instance."""
        super().__init__(raw_data, global_data, application, providers_data, shared_data or {})

        # Build and validate user data against our Pydantic model.
        self.model: OrchestrationPluginModel = ModelBuilder(OrchestrationPluginModel).build([], self.raw_data)

        # For convenience, store a reference to the orchestration data
        self.orch: KubernetesOrchestrationModel = self.model.orchestration

        # Set up a K8s client (in-cluster or local). For local dev, ensure your kubeconfig is present.
        # In many real cases, you'd do this in 'setup()', but we can do it here too.
        try:
            config.load_kube_config()  # If running locally
        except Exception as e:  # noqa: BLE001
            LOGGER.warning(f"Error loading kubeconfig: {e}")
            # If in-cluster, you could do config.load_incluster_config()
            # or handle exceptions gracefully.
            config.load_incluster_config()

        self._apps_api = AppsV1Api()
        self._core_api = CoreV1Api()

        LOGGER.info("Initialized OrchestrationPlugin with validated schema.")

    # ----------------------------------------------------------------
    # Jobs
    # ----------------------------------------------------------------

    def job_deploy_ros_master(self) -> None:
        """Deploy or update a ROS master node as a K8s Deployment."""
        deployment_name = "ros-master"
        namespace = "default"
        try:
            self._apps_api.read_namespaced_deployment(deployment_name, namespace)
            LOGGER.info("ros-master is already deployed; skipping creation.")
            # LOGGER.info("ros-master is already deployed; patching it.")  # noqa: ERA001
            # self._apps_api.patch_namespaced_deployment(deployment_name, namespace, final_depl)  # noqa: ERA001
        except ApiException as e:
            if e.status == UNAVAILABLE_STATUS:
                # Create a new deployment
                LOGGER.info("Deploying ros-master in K8s.")
                ros_master_depl = {
                    "apiVersion": "apps/v1",
                    "kind": "Deployment",
                    "metadata": {"name": deployment_name},
                    "spec": {
                        "replicas": 1,
                        "selector": {"matchLabels": {"app": "ros-master"}},
                        "template": {
                            "metadata": {"labels": {"app": "ros-master"}},
                            "spec": {
                                "containers": [
                                    {
                                        "name": "ros-master-container",
                                        "image": f"ros:{self.application.distro}",
                                        "ports": [{"containerPort": 11311}],
                                        "env": [
                                            {"name": "ROS_HOSTNAME", "value": "ros-master"},
                                            {"name": "ROS_MASTER_URI", "value": "http://ros-master:11311"},
                                        ],
                                        # Example readiness probe if desired:
                                        # "readinessProbe": {  # noqa: ERA001
                                        #     "exec": {  # noqa: ERA001
                                        #     "command": ["/bin/sh", "-c", "/opt/ros/noetic/scripts/readiness_probe.sh"]
                                        #     },
                                        #     "initialDelaySeconds": 5,  # noqa: ERA001
                                        #     "periodSeconds": 5
                                        # },
                                    },
                                ],
                            },
                        },
                    },
                }
                # Merge any additional user overrides for ROS master if needed
                final_depl = deep_merge(ros_master_depl, self.orch.additional_k8s_params.get("ros_master", {}))
                self._apps_api.create_namespaced_deployment(namespace=namespace, body=final_depl)
                LOGGER.info("ros-master deployed successfully.")
            else:
                LOGGER.warning(f"Error reading ros-master deployment: {e}")

    def job_create_persistent_storage(self) -> None:
        """Create or update the required PVs and PVCs for the user-specified volumes."""
        if not self.orch.persistent_storage:
            return

        for vol in self.orch.persistent_storage.volumes:
            # 1. Create PV
            pv_body = {
                "apiVersion": "v1",
                "kind": "PersistentVolume",
                "metadata": {"name": f"{vol.name}-pv"},
                "spec": {
                    "capacity": {"storage": vol.size},
                    "accessModes": ["ReadWriteOnce"],
                    "storageClassName": vol.storage_class,
                    "hostPath": {
                        "path": f"/mnt/data/{vol.name}",  # Example host path
                    },
                },
            }
            # Attempt creation or skip if it exists
            try:
                self._core_api.read_persistent_volume(name=f"{vol.name}-pv")
                LOGGER.info(f"PersistentVolume {vol.name}-pv already exists; skipping creation.")
            except ApiException as e:
                if e.status == UNAVAILABLE_STATUS:
                    self._core_api.create_persistent_volume(body=pv_body)
                    LOGGER.info(f"Created PersistentVolume '{vol.name}-pv'.")
                else:
                    LOGGER.warning(f"Error reading PV for {vol.name}: {e}")

            # 2. Create PVC in default namespace
            pvc_body = {
                "apiVersion": "v1",
                "kind": "PersistentVolumeClaim",
                "metadata": {"name": f"{vol.name}-pvc"},
                "spec": {
                    "accessModes": ["ReadWriteOnce"],
                    "resources": {"requests": {"storage": vol.size}},
                    "storageClassName": vol.storage_class,
                },
            }
            try:
                self._core_api.read_namespaced_persistent_volume_claim(f"{vol.name}-pvc", "default")
                LOGGER.info(f"PVC {vol.name}-pvc already exists; skipping creation.")
            except ApiException as e:
                if e.status == UNAVAILABLE_STATUS:
                    self._core_api.create_namespaced_persistent_volume_claim(namespace="default", body=pvc_body)
                    LOGGER.info(f"Created PVC '{vol.name}-pvc'.")
                else:
                    LOGGER.warning(f"Error reading PVC for {vol.name}: {e}")

    def job_deploy_application(self) -> None:
        """Deploy or patch the main ROS application.

        TODO: In practice, get more details (images, command, etc.)
        from self.application or additional plugin data.
        """
        namespace = "default"

        # Minimal base deployment spec
        base_deployment = {
            "apiVersion": "apps/v1",
            "kind": "Deployment",
            "metadata": {"name": DEPLOYMENT_NAME},
            "spec": {
                "replicas": 1,
                "selector": {"matchLabels": {"app": DEPLOYMENT_NAME}},
                "template": {
                    "metadata": {"labels": {"app": DEPLOYMENT_NAME}},
                    "spec": {
                        "containers": [
                            {
                                "name": "ros-app",
                                "image": f"ros:{self.application.distro}",  # or a custom app image
                                "ports": [{"containerPort": 8080}],
                            },
                        ],
                    },
                },
            },
        }

        # Merge user-specified additional_k8s_params for the main app if any
        final_deployment = deep_merge(base_deployment, self.orch.additional_k8s_params.get("application", {}))

        try:
            self._apps_api.read_namespaced_deployment(DEPLOYMENT_NAME, namespace)
            # Patch or update
            self._apps_api.patch_namespaced_deployment(DEPLOYMENT_NAME, namespace, final_deployment)
            LOGGER.info(f"Patched existing application deployment '{DEPLOYMENT_NAME}'.")
        except ApiException as e:
            if e.status == UNAVAILABLE_STATUS:
                # Create new
                self._apps_api.create_namespaced_deployment(namespace=namespace, body=final_deployment)
                LOGGER.info(f"Created new application deployment '{DEPLOYMENT_NAME}'.")
            else:
                LOGGER.warning(f"Error reading or updating application deployment: {e}")

    def job_configure_observability(self) -> None:
        """Configure the observability stack (Promtail, Loki, Prometheus, Grafana)."""
        if not self.orch.observability or not self.orch.observability.enabled:
            LOGGER.info("Observability is disabled; skipping configuration.")
            return

        LOGGER.info("Configuring observability stack (Promtail, Loki, Prometheus, Grafana)...")

        commands = [
            "helm repo add grafana https://grafana.github.io/helm-charts",
            "helm repo update",
            "helm install prometheus grafana/prometheus",
            "helm install loki grafana/loki-stack",
            "helm install grafana grafana/grafana",
        ]

        for command in commands:
            subprocess.run(command, shell=True, check=True)  # noqa: S602

        # Configure Promtail to send logs to Loki
        promtail_config = """
        server:
          http_listen_port: 9080

        clients:
          - url: http://loki:3100/loki/api/v1/push

        scrape_configs:
          - job_name: system
            static_configs:
              - targets:
                  - localhost
                labels:
                  job: system
                  __path__: /var/log/*log
        """

        with Path("/tmp/promtail-config.yaml").open("w") as f:  # noqa: S108
            f.write(promtail_config)

        subprocess.run("kubectl apply -f /tmp/promtail-config.yaml", shell=True, check=True)  # noqa: S602, S607

        LOGGER.info("Observability stack configured successfully.")

    def job_check_readiness(self) -> bool:
        """Ensure the system is "ready" before any destructive action (like rolling update).

        If a readiness command is provided, we might:
        - Exec into the running container(s) to run a script
        - Or rely on K8s readiness probes
        For simplicity, let's rely on Pod 'Ready' status here, but illustrate how you'd do a custom check.
        """
        if not self.orch.readiness or not self.orch.readiness.command:
            LOGGER.info("No readiness command is defined; assuming ready.")
            return True

        # Example approach: wait for all pods with 'app=<DEPLOYMENT_NAME>' to be in 'Ready' state.
        # The user can further define or override logic as needed.
        namespace = "default"

        # Let's do a simplistic approach: check if all pods from that deployment are ready.
        LOGGER.info("Checking readiness of application pods...")

        for _ in range(30):  # up to e.g. 30 retries
            pods = self._core_api.list_namespaced_pod(namespace, label_selector=f"app={DEPLOYMENT_NAME}").items
            if not pods:
                time.sleep(2)
                continue

            all_ready = True
            for pod in pods:
                cstatuses = pod.status.container_statuses or []
                # A container is "ready" if cstatus.ready == True
                # and the pod overall is also in Running phase
                if pod.status.phase != "Running":
                    all_ready = False
                    break
                for cstatus in cstatuses:
                    if not cstatus.ready:
                        all_ready = False
                        break

            if all_ready:
                LOGGER.info("Pods are ready.")
                return True

            time.sleep(2)

        LOGGER.warning("Timeout: pods never became fully ready.")
        return False

    def job_rolling_update(self) -> None:
        """Execute a rolling update strategy for the app.

        For demonstration, we just patch the Deployment with a new image or strategy.
        (In real usage, you'd do more advanced or user-specified changes here.)
        """
        if not self.orch.rolling_update:
            LOGGER.info("No rolling update config found; skipping.")
            return

        # Check readiness before continuing
        if not self.job_check_readiness():
            LOGGER.info("System is not ready, deferring rolling update.")
            return

        namespace = "default"

        # Example patch: set the strategy
        rolling_update_strategy = {
            "spec": {
                "strategy": {
                    "type": "RollingUpdate",
                    "rollingUpdate": {
                        "maxSurge": self.orch.rolling_update.max_surge,
                        "maxUnavailable": self.orch.rolling_update.max_unavailable,
                    },
                },
            },
        }

        try:
            self._apps_api.patch_namespaced_deployment(DEPLOYMENT_NAME, namespace, rolling_update_strategy)
            LOGGER.info(f"Patched deployment '{DEPLOYMENT_NAME}' with rolling update strategy.")
        except ApiException as e:
            LOGGER.warning(f"Error applying rolling update: {e}")

    def job_distributed_deployment(self) -> None:
        """Apply node selectors / tolerations if 'distributed.enabled' is true (and not forced local).

        TODO: This is a simplified example.
        """
        if not self.orch.distributed or not self.orch.distributed.enabled:
            LOGGER.info("Distributed deployment is disabled; skipping.")
            return

        if self.orch.distributed.force_local_flag:
            LOGGER.info("Distributed deployment is forced to local; skipping remote scheduling.")
            return

        # Example approach: patch the existing Deployment with a nodeSelector.
        namespace = "default"

        # If default_to_remote is true, we might set a nodeSelector to a known remote node label
        node_selector = {
            "spec": {
                "template": {
                    "spec": {
                        "nodeSelector": {
                            "deploymentType": "remote",  # or user-specified
                        },
                    },
                },
            },
        }
        try:
            self._apps_api.patch_namespaced_deployment(DEPLOYMENT_NAME, namespace, node_selector)
            LOGGER.info(f"Patched deployment '{DEPLOYMENT_NAME}' with remote nodeSelector.")
        except ApiException as e:
            LOGGER.warning(f"Error patching for distributed deployment: {e}")

    # ----------------------------------------------------------------
    # Plugin Lifecycle
    # ----------------------------------------------------------------

    def setup(self) -> None:
        """Allocate or initialize resources needed for this plugin."""
        LOGGER.info("[SETUP] OrchestrationPlugin setup.")
        # For example, confirm cluster connectivity or pre-create any resources.

    def start(self) -> None:
        """Orchestrate the main logic of your plugin."""
        LOGGER.info("[START] OrchestrationPlugin start.")

        # 1) Deploy ros-master if configured
        if self.orch.deploy_ros_master:
            self.job_deploy_ros_master()

        # 2) Create persistent volumes if configured
        if self.orch.persistent_storage:
            self.job_create_persistent_storage()

        # 3) Deploy the user application
        self.job_deploy_application()

        # 4) Configure observability if enabled
        self.job_configure_observability()

        # 5) Check readiness
        #    (This also ensures that if the user wants to do a rolling update next,
        #     we confirm the system is stable.)
        is_ready = self.job_check_readiness()

        if not is_ready:
            LOGGER.warning("Application is not ready yet, but continuing plugin flow.")

        # 6) Rolling update if user configured rolling_update
        if self.orch.rolling_update:
            self.job_rolling_update()

        # 7) Distributed deployment if user enabled it
        if self.orch.distributed and self.orch.distributed.enabled:
            self.job_distributed_deployment()

    def process(self) -> None:
        """Perform any post-deployment checks or logic, gather logs, etc."""
        LOGGER.info("[PROCESS] Checking cluster for final status.")
        # Could further monitor readiness, watch logs, etc.
        # For example, re-run readiness or watch events.

    def stop(self) -> None:
        """Gracefully clean up or finalize resources.

        In a real scenario, you might tear down ephemeral test deployments
        or keep them up for production.
        """
        LOGGER.info("[STOP] OrchestrationPlugin stop.")
        # Potentially delete resources or scale them down, etc.
        # Alternatively, do nothing if the user wants the deployment to stay active.
