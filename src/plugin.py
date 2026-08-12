"""Kubernetes orchestration for ROS applications."""

from __future__ import annotations

import shlex
import shutil
import subprocess
import tempfile
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any, Protocol, cast

import yaml
from kubernetes import config
from kubernetes.client import ApiException, AppsV1Api, CoreV1Api, V1Pod
from kubernetes.config.config_exception import ConfigException

from .compat import ModelBuilder, PluginBase, get_logger
from .models import HelmComponentConfig, KubernetesOrchestrationModel, OrchestrationPluginModel, VolumeConfig
from .utils.dict_operations import deep_merge

if TYPE_CHECKING:
    from collections.abc import Callable, Mapping

DEPLOYMENT_NAME = "rigel-k8s-application"
NOT_FOUND_STATUS = 404
LOGGER = get_logger()


class ApplicationLike(Protocol):
    """Application data required from Rigel or the standalone CLI."""

    @property
    def distro(self) -> str:
        """Target ROS distribution."""
        ...


class OrchestrationPlugin(PluginBase):
    """Deploy a ROS application and its supporting resources to Kubernetes."""

    def __init__(  # noqa: PLR0913 - preserves Rigel's required constructor and supports dependency injection
        self,
        raw_data: Mapping[str, Any],
        global_data: Mapping[str, Any] | None,
        application: ApplicationLike,
        providers_data: Mapping[str, Any] | None,
        shared_data: Mapping[str, Any] | None = None,
        *,
        apps_api: AppsV1Api | None = None,
        core_api: CoreV1Api | None = None,
        command_runner: Callable[..., subprocess.CompletedProcess[Any]] = subprocess.run,
    ) -> None:
        """Validate configuration without connecting to a cluster.

        Kubernetes clients can be injected for tests and embedding. Otherwise they
        are initialized lazily by :meth:`setup`.
        """
        base_init = cast("Callable[..., None]", super().__init__)
        base_init(
            raw_data,
            global_data,
            application,
            providers_data,
            shared_data or {},
        )
        self.model: OrchestrationPluginModel = ModelBuilder(OrchestrationPluginModel).build([], self.raw_data)
        self.orch: KubernetesOrchestrationModel = self.model.orchestration
        self._apps_api = apps_api
        self._core_api = core_api
        self._command_runner = command_runner

    @property
    def namespace(self) -> str:
        """Namespace containing the ROS application."""
        return self.orch.namespace

    @property
    def deployment_name(self) -> str:
        """Name of the main application Deployment."""
        return self.orch.deployment_name

    def setup(self) -> None:
        """Initialize Kubernetes clients from local or in-cluster credentials."""
        if self._apps_api is not None and self._core_api is not None:
            return

        try:
            config.load_kube_config()
        except ConfigException as local_error:
            try:
                config.load_incluster_config()
            except ConfigException as cluster_error:
                msg = "No Kubernetes configuration found (tried kubeconfig and in-cluster credentials)"
                raise RuntimeError(msg) from cluster_error
            LOGGER.debug("Local kubeconfig unavailable: %s", local_error)

        self._apps_api = AppsV1Api()
        self._core_api = CoreV1Api()

    def _clients(self) -> tuple[AppsV1Api, CoreV1Api]:
        self.setup()
        if self._apps_api is None or self._core_api is None:  # defensive; setup either creates both or raises
            msg = "Kubernetes clients were not initialized"
            raise RuntimeError(msg)
        return self._apps_api, self._core_api

    def build_ros_master_service(self) -> dict[str, Any]:
        """Build the ROS master Service manifest."""
        return {
            "apiVersion": "v1",
            "kind": "Service",
            "metadata": {"name": "ros-master", "namespace": self.namespace},
            "spec": {
                "selector": {"app.kubernetes.io/name": "ros-master"},
                "ports": [{"name": "ros", "port": 11311, "targetPort": 11311, "protocol": "TCP"}],
            },
        }

    def build_ros_master_deployment(self) -> dict[str, Any]:
        """Build the ROS master Deployment manifest."""
        distro = self.application.distro
        base = {
            "apiVersion": "apps/v1",
            "kind": "Deployment",
            "metadata": {"name": "ros-master", "namespace": self.namespace},
            "spec": {
                "replicas": 1,
                "selector": {"matchLabels": {"app.kubernetes.io/name": "ros-master"}},
                "template": {
                    "metadata": {"labels": {"app.kubernetes.io/name": "ros-master"}},
                    "spec": {
                        "containers": [
                            {
                                "name": "ros-master-container",
                                "image": self.orch.ros_master_image or f"ros:{distro}-ros-core",
                                "command": ["/ros_entrypoint.sh"],
                                "args": ["roscore"],
                                "ports": [{"name": "ros", "containerPort": 11311}],
                                "env": [
                                    {"name": "ROS_HOSTNAME", "value": "ros-master"},
                                    {"name": "ROS_MASTER_URI", "value": "http://ros-master:11311"},
                                ],
                            },
                        ],
                    },
                },
            },
        }
        return deep_merge(base, self.orch.additional_k8s_params.get("ros_master", {}))

    def _rolling_strategy(self) -> dict[str, Any] | None:
        rolling = self.orch.rolling_update
        if rolling is None:
            return None
        if rolling.strategy.lower() == "recreate":
            return {"type": "Recreate"}
        return {
            "type": "RollingUpdate",
            "rollingUpdate": {"maxSurge": rolling.max_surge, "maxUnavailable": rolling.max_unavailable},
        }

    def build_application_deployment(self) -> dict[str, Any]:
        """Build the application Deployment, including storage and health settings."""
        labels = {"app.kubernetes.io/name": self.deployment_name}
        container: dict[str, Any] = {
            "name": "ros-app",
            "image": self.orch.application_image or f"ros:{self.application.distro}-ros-core",
        }

        if self.orch.deploy_ros_master:
            container["env"] = [{"name": "ROS_MASTER_URI", "value": "http://ros-master:11311"}]

        volumes: list[dict[str, Any]] = []
        mounts: list[dict[str, Any]] = []
        if self.orch.persistent_storage:
            for volume in self.orch.persistent_storage.volumes:
                volumes.append(
                    {"name": volume.name, "persistentVolumeClaim": {"claimName": f"{volume.name}-pvc"}},
                )
                mounts.append(
                    {
                        "name": volume.name,
                        "mountPath": volume.mount_path or f"/persistent/{volume.name}",
                    },
                )
        if mounts:
            container["volumeMounts"] = mounts

        if self.orch.readiness:
            readiness = self.orch.readiness
            container["readinessProbe"] = {
                "exec": {"command": shlex.split(readiness.command)},
                "initialDelaySeconds": readiness.initial_delay_seconds,
                "periodSeconds": readiness.period_seconds,
            }

        base: dict[str, Any] = {
            "apiVersion": "apps/v1",
            "kind": "Deployment",
            "metadata": {"name": self.deployment_name, "namespace": self.namespace},
            "spec": {
                "replicas": 1,
                "selector": {"matchLabels": labels},
                "template": {"metadata": {"labels": labels}, "spec": {"containers": [container]}},
            },
        }
        if volumes:
            base["spec"]["template"]["spec"]["volumes"] = volumes
        strategy = self._rolling_strategy()
        if strategy:
            base["spec"]["strategy"] = strategy

        distributed = self.orch.distributed
        if distributed and distributed.enabled and distributed.default_to_remote and not distributed.force_local_flag:
            base["spec"]["template"]["spec"]["nodeSelector"] = {"deploymentType": "remote"}

        return deep_merge(base, self.orch.additional_k8s_params.get("application", {}))

    def _upsert_deployment(self, name: str, body: dict[str, Any]) -> None:
        apps_api, _ = self._clients()
        try:
            apps_api.read_namespaced_deployment(name, self.namespace)
        except ApiException as error:
            if error.status != NOT_FOUND_STATUS:
                raise
            apps_api.create_namespaced_deployment(namespace=self.namespace, body=body)
            LOGGER.info("Created Deployment %s/%s", self.namespace, name)
        else:
            apps_api.patch_namespaced_deployment(name, self.namespace, body)
            LOGGER.info("Updated Deployment %s/%s", self.namespace, name)

    def job_deploy_ros_master(self) -> None:
        """Create or update the ROS master Service and Deployment."""
        _, core_api = self._clients()
        service = self.build_ros_master_service()
        try:
            core_api.read_namespaced_service("ros-master", self.namespace)
        except ApiException as error:
            if error.status != NOT_FOUND_STATUS:
                raise
            core_api.create_namespaced_service(namespace=self.namespace, body=service)
            LOGGER.info("Created Service %s/ros-master", self.namespace)
        else:
            core_api.patch_namespaced_service("ros-master", self.namespace, service)
            LOGGER.info("Updated Service %s/ros-master", self.namespace)

        self._upsert_deployment("ros-master", self.build_ros_master_deployment())

    def _persistent_volume_body(self, volume: VolumeConfig) -> dict[str, Any]:
        spec: dict[str, Any] = {
            "capacity": {"storage": volume.size},
            "accessModes": ["ReadWriteOnce"],
            "hostPath": {"path": volume.host_path},
            "persistentVolumeReclaimPolicy": "Retain",
        }
        if volume.storage_class:
            spec["storageClassName"] = volume.storage_class
        return {
            "apiVersion": "v1",
            "kind": "PersistentVolume",
            "metadata": {"name": f"{volume.name}-pv"},
            "spec": spec,
        }

    def _persistent_volume_claim_body(self, volume: VolumeConfig) -> dict[str, Any]:
        spec: dict[str, Any] = {
            "accessModes": ["ReadWriteOnce"],
            "resources": {"requests": {"storage": volume.size}},
        }
        if volume.storage_class:
            spec["storageClassName"] = volume.storage_class
        if volume.host_path:
            spec["volumeName"] = f"{volume.name}-pv"
        return {
            "apiVersion": "v1",
            "kind": "PersistentVolumeClaim",
            "metadata": {"name": f"{volume.name}-pvc", "namespace": self.namespace},
            "spec": spec,
        }

    def job_create_persistent_storage(self) -> None:
        """Create PVCs, and opt-in static hostPath PVs when explicitly requested."""
        if not self.orch.persistent_storage:
            return
        _, core_api = self._clients()
        for volume in self.orch.persistent_storage.volumes:
            if volume.host_path:
                pv_name = f"{volume.name}-pv"
                pv_body = self._persistent_volume_body(volume)
                try:
                    core_api.read_persistent_volume(pv_name)
                except ApiException as error:
                    if error.status != NOT_FOUND_STATUS:
                        raise
                    core_api.create_persistent_volume(body=pv_body)
                else:
                    core_api.patch_persistent_volume(pv_name, pv_body)

            pvc_name = f"{volume.name}-pvc"
            pvc_body = self._persistent_volume_claim_body(volume)
            try:
                core_api.read_namespaced_persistent_volume_claim(pvc_name, self.namespace)
            except ApiException as error:
                if error.status != NOT_FOUND_STATUS:
                    raise
                core_api.create_namespaced_persistent_volume_claim(namespace=self.namespace, body=pvc_body)
            else:
                core_api.patch_namespaced_persistent_volume_claim(pvc_name, self.namespace, pvc_body)

    def job_deploy_application(self) -> None:
        """Create or update the main ROS application Deployment."""
        self._upsert_deployment(self.deployment_name, self.build_application_deployment())

    def _install_helm_component(
        self,
        component: HelmComponentConfig,
        namespace: str,
        values_dir: Path,
    ) -> None:
        if not component.enabled:
            return
        command = [
            "helm",
            "upgrade",
            "--install",
            component.release,
            component.chart,
            "--namespace",
            namespace,
            "--create-namespace",
            "--wait",
            "--timeout",
            "10m",
        ]
        if component.values:
            values_path = values_dir / f"{component.release}-values.yaml"
            values_path.write_text(yaml.safe_dump(component.values, sort_keys=False), encoding="utf-8")
            command.extend(["--values", str(values_path)])
        self._command_runner(command, check=True)

    def job_configure_observability(self) -> None:
        """Install enabled Prometheus, Loki, and Grafana Helm releases."""
        observability = self.orch.observability
        if observability is None or not observability.enabled:
            return
        if self._command_runner is subprocess.run and shutil.which("helm") is None:
            msg = "Observability is enabled, but the 'helm' executable is not installed"
            raise RuntimeError(msg)

        with tempfile.TemporaryDirectory(prefix="rigel-observability-") as directory:
            values_dir = Path(directory)
            for component in (observability.prometheus, observability.loki, observability.grafana):
                self._install_helm_component(component, observability.namespace, values_dir)

    def _application_pods(self) -> list[V1Pod]:
        """Return pods owned by the configured application Deployment."""
        _, core_api = self._clients()
        return list(
            core_api.list_namespaced_pod(
                self.namespace,
                label_selector=f"app.kubernetes.io/name={self.deployment_name}",
            ).items,
        )

    @staticmethod
    def _pod_is_ready(pod: V1Pod) -> bool:
        """Return whether a pod is running and all of its containers are ready."""
        statuses = pod.status.container_statuses or []
        return pod.status.phase == "Running" and bool(statuses) and all(status.ready for status in statuses)

    def job_check_readiness(self) -> bool:
        """Return whether at least one application pod exists and every container is ready."""
        pods = self._application_pods()
        if not pods:
            return False
        return all(self._pod_is_ready(pod) for pod in pods)

    def collect_results(self, destination: Path) -> Path:
        """Copy configured results from the newest ready application pod."""
        results = self.orch.results
        if results is None:
            msg = "Result collection requires orchestration.results.source_path"
            raise ValueError(msg)
        if self._command_runner is subprocess.run and shutil.which("kubectl") is None:
            msg = "Result collection requires the 'kubectl' executable"
            raise RuntimeError(msg)

        ready_pods = [pod for pod in self._application_pods() if self._pod_is_ready(pod)]
        if not ready_pods:
            msg = f"No ready pods found for Deployment {self.namespace}/{self.deployment_name}"
            raise RuntimeError(msg)
        ready_pods.sort(key=lambda pod: str(pod.metadata.creation_timestamp or ""))
        pod = ready_pods[-1]
        destination.parent.mkdir(parents=True, exist_ok=True)
        command = [
            "kubectl",
            "cp",
            f"{self.namespace}/{pod.metadata.name}:{results.source_path}",
            str(destination),
            "--container",
            results.container,
        ]
        self._command_runner(command, check=True)
        LOGGER.info("Collected results from %s/%s into %s", self.namespace, pod.metadata.name, destination)
        return destination

    def wait_until_ready(self) -> bool:
        """Poll readiness until the configured timeout expires."""
        if self.orch.readiness is None:
            return True
        timeout = self.orch.readiness.timeout_seconds
        deadline = time.monotonic() + timeout
        while True:
            if self.job_check_readiness():
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(min(self.orch.readiness.period_seconds, max(0, deadline - time.monotonic())))

    def job_rolling_update(self) -> None:
        """Apply the configured Deployment update strategy."""
        strategy = self._rolling_strategy()
        if strategy is None:
            return
        apps_api, _ = self._clients()
        apps_api.patch_namespaced_deployment(
            self.deployment_name,
            self.namespace,
            {"spec": {"strategy": strategy}},
        )

    def job_distributed_deployment(self) -> None:
        """Apply remote scheduling when distributed deployment requests it."""
        distributed = self.orch.distributed
        if (
            distributed is None
            or not distributed.enabled
            or not distributed.default_to_remote
            or distributed.force_local_flag
        ):
            return
        apps_api, _ = self._clients()
        apps_api.patch_namespaced_deployment(
            self.deployment_name,
            self.namespace,
            {"spec": {"template": {"spec": {"nodeSelector": {"deploymentType": "remote"}}}}},
        )

    def start(self) -> None:
        """Apply all configured resources and wait once for application readiness."""
        if self.orch.observability_only or self.global_data.get("sequence_name") == "observability":
            self.job_configure_observability()
            return
        self.setup()
        if self.orch.deploy_ros_master:
            self.job_deploy_ros_master()
        self.job_create_persistent_storage()
        self.job_deploy_application()
        self.job_configure_observability()
        if self.orch.readiness and not self.wait_until_ready():
            msg = f"Deployment {self.namespace}/{self.deployment_name} did not become ready in time"
            raise TimeoutError(msg)

    def process(self) -> None:
        """Validate the final application state when readiness is configured."""
        if self.orch.readiness and not self.job_check_readiness():
            msg = f"Deployment {self.namespace}/{self.deployment_name} is not ready"
            raise RuntimeError(msg)

    def _uninstall_observability(self) -> None:
        """Uninstall enabled Helm releases when explicitly requested."""
        observability = self.orch.observability
        if observability is None or not observability.enabled:
            return
        if self._command_runner is subprocess.run and shutil.which("helm") is None:
            msg = "Observability cleanup requires the 'helm' executable"
            raise RuntimeError(msg)
        for component in (observability.prometheus, observability.loki, observability.grafana):
            if component.enabled:
                self._command_runner(
                    [
                        "helm",
                        "uninstall",
                        component.release,
                        "--namespace",
                        observability.namespace,
                        "--ignore-not-found",
                    ],
                    check=True,
                )

    @staticmethod
    def _delete_if_exists(action: Callable[..., Any], *args: str) -> None:
        """Delete a Kubernetes resource while treating absence as success."""
        try:
            action(*args)
        except ApiException as error:
            if error.status != NOT_FOUND_STATUS:
                raise

    def cleanup(self, *, delete_storage: bool = False, uninstall_observability: bool = False) -> None:
        """Delete managed workloads, preserving storage and Helm releases by default."""
        apps_api, core_api = self._clients()
        if not self.orch.observability_only:
            deployments = [self.deployment_name]
            if self.orch.deploy_ros_master:
                deployments.append("ros-master")
            for deployment in deployments:
                self._delete_if_exists(apps_api.delete_namespaced_deployment, deployment, self.namespace)

            if self.orch.deploy_ros_master:
                self._delete_if_exists(core_api.delete_namespaced_service, "ros-master", self.namespace)

            if delete_storage and self.orch.persistent_storage:
                for volume in self.orch.persistent_storage.volumes:
                    self._delete_if_exists(
                        core_api.delete_namespaced_persistent_volume_claim,
                        f"{volume.name}-pvc",
                        self.namespace,
                    )
                    if volume.host_path:
                        self._delete_if_exists(core_api.delete_persistent_volume, f"{volume.name}-pv")

        if uninstall_observability:
            self._uninstall_observability()

    def stop(self) -> None:
        """Leave deployed resources running; teardown is intentionally explicit."""
