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
        """Configure the observability stack (Prometheus, Loki, Grafana)."""
        if not self.orch.observability or not self.orch.observability.enabled:
            LOGGER.info("Observability is disabled; skipping configuration.")
            return

        LOGGER.info("Configuring observability stack (Prometheus, Loki, Grafana)...")

        # Add required Helm repositories
        commands = [
            "helm repo add grafana https://grafana.github.io/helm-charts",
            "helm repo add prometheus-community https://prometheus-community.github.io/helm-charts",
            "helm repo update",
        ]

        for command in commands:
            try:
                subprocess.run(command, shell=True, check=True)  # noqa: S602
            except subprocess.CalledProcessError as e:
                LOGGER.warning(f"Error adding Helm repositories: {e}")
                return

        # Create namespace for observability tools
        try:
            self._core_api.create_namespace(body={"metadata": {"name": "monitoring"}})
            LOGGER.info("Created 'monitoring' namespace.")
        except ApiException as e:
            if e.status != 409:  # 409 means it already exists
                LOGGER.warning(f"Error creating namespace: {e}")
                return
            LOGGER.info("Namespace 'monitoring' already exists.")

        # Configure and install Prometheus
        if self.orch.observability.prometheus.get("enabled", True):
            prom_config = self.orch.observability.prometheus
            prom_values = {
                "server": {
                    "retention": prom_config.get("retention", "24h"),
                    "global": {
                        "scrape_interval": prom_config.get("scrape_interval", "15s"),
                        "evaluation_interval": prom_config.get("evaluation_interval", "15s"),
                    },
                },
                "service": {
                    "type": prom_config.get("service_type", "ClusterIP"),
                },
                "alertmanager": {
                    "enabled": True,
                    "persistence": {
                        "enabled": True,
                        "size": "2Gi",
                    },
                },
                "nodeExporter": {
                    "enabled": self.orch.observability.node_exporter,
                },
                "kubeStateMetrics": {
                    "enabled": True,
                },
            }
            
            # Write Prometheus values to a temporary file
            with Path("/tmp/prometheus-values.yaml").open("w") as f:  # noqa: S108
                import yaml
                yaml.dump(prom_values, f)
            
            try:
                subprocess.run(
                    "helm upgrade --install prometheus prometheus-community/prometheus "
                    "-n monitoring -f /tmp/prometheus-values.yaml",
                    shell=True, check=True  # noqa: S602
                )
                LOGGER.info("Prometheus installed/upgraded successfully.")
            except subprocess.CalledProcessError as e:
                LOGGER.warning(f"Error installing Prometheus: {e}")

        # Configure and install Loki stack
        if self.orch.observability.loki.get("enabled", True):
            loki_config = self.orch.observability.loki
            loki_values = {
                "loki": {
                    "persistence": {
                        "enabled": loki_config.get("persistence", {}).get("enabled", True),
                        "size": loki_config.get("persistence", {}).get("size", "5Gi"),
                    },
                    "config": {
                        "table_manager": {
                            "retention_deletes_enabled": True,
                            "retention_period": loki_config.get("retention", "24h"),
                        },
                    },
                },
                "promtail": {
                    "enabled": True,
                    "config": {
                        "snippets": {
                            "extraScrapeConfigs": """
                            - job_name: ros-logs
                              static_configs:
                              - targets:
                                  - localhost
                                labels:
                                  job: ros-logs
                                  __path__: /var/log/ros/*.log
                            """,
                        },
                    },
                },
            }
            
            # Write Loki values to a temporary file
            with Path("/tmp/loki-values.yaml").open("w") as f:  # noqa: S108
                import yaml
                yaml.dump(loki_values, f)
            
            try:
                subprocess.run(
                    "helm upgrade --install loki grafana/loki-stack "
                    "-n monitoring -f /tmp/loki-values.yaml",
                    shell=True, check=True  # noqa: S602
                )
                LOGGER.info("Loki stack installed/upgraded successfully.")
            except subprocess.CalledProcessError as e:
                LOGGER.warning(f"Error installing Loki: {e}")

        # Configure and install Grafana
        if self.orch.observability.grafana.get("enabled", True):
            grafana_config = self.orch.observability.grafana
            grafana_values = {
                "adminPassword": grafana_config.get("admin_password", "admin"),
                "persistence": {
                    "enabled": grafana_config.get("persistence", {}).get("enabled", True),
                    "size": grafana_config.get("persistence", {}).get("size", "2Gi"),
                },
                "service": {
                    "type": grafana_config.get("service_type", "NodePort"),
                },
                "datasources": {
                    "datasources.yaml": {
                        "apiVersion": 1,
                        "datasources": [
                            {
                                "name": "Prometheus",
                                "type": "prometheus",
                                "url": "http://prometheus-server.monitoring.svc.cluster.local",
                                "access": "proxy",
                                "isDefault": True,
                            },
                            {
                                "name": "Loki",
                                "type": "loki",
                                "url": "http://loki.monitoring.svc.cluster.local:3100",
                                "access": "proxy",
                            },
                        ],
                    },
                },
            }
            
            # Add default ROS dashboards if enabled
            if grafana_config.get("default_dashboards", True):
                grafana_values["dashboardProviders"] = {
                    "dashboardproviders.yaml": {
                        "apiVersion": 1,
                        "providers": [
                            {
                                "name": "default",
                                "orgId": 1,
                                "folder": "",
                                "type": "file",
                                "disableDeletion": False,
                                "editable": True,
                                "options": {
                                    "path": "/var/lib/grafana/dashboards/default",
                                },
                            },
                        ],
                    },
                }
                
                # Create ROS dashboard ConfigMap
                self._create_ros_dashboard_configmap()
                
                grafana_values["dashboardsConfigMaps"] = {
                    "default": "ros-dashboards",
                }
            
            # Write Grafana values to a temporary file
            with Path("/tmp/grafana-values.yaml").open("w") as f:  # noqa: S108
                import yaml
                yaml.dump(grafana_values, f)
            
            try:
                subprocess.run(
                    "helm upgrade --install grafana grafana/grafana "
                    "-n monitoring -f /tmp/grafana-values.yaml",
                    shell=True, check=True  # noqa: S602
                )
                LOGGER.info("Grafana installed/upgraded successfully.")
            except subprocess.CalledProcessError as e:
                LOGGER.warning(f"Error installing Grafana: {e}")
                
        # Deploy ROS metrics exporter if enabled
        if self.orch.observability.ros_metrics:
            self._deploy_ros_metrics_exporter()
                
        LOGGER.info("Observability stack configured successfully.")
        
        # Print access information
        self._print_observability_access_info()

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
        """Apply node selectors / tolerations if 'distributed.enabled' is true (and not forced local)."""
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
            
    def _create_ros_dashboard_configmap(self) -> None:
        """Create a ConfigMap with ROS dashboard definitions for Grafana."""
        namespace = "monitoring"
        
        # Basic ROS system dashboard JSON
        ros_system_dashboard = {
            "annotations": {
                "list": [
                    {
                        "builtIn": 1,
                        "datasource": "-- Grafana --",
                        "enable": True,
                        "hide": True,
                        "iconColor": "rgba(0, 211, 255, 1)",
                        "name": "Annotations & Alerts",
                        "type": "dashboard"
                    }
                ]
            },
            "editable": True,
            "gnetId": None,
            "graphTooltip": 0,
            "id": 1,
            "links": [],
            "panels": [
                {
                    "aliasColors": {},
                    "bars": False,
                    "dashLength": 10,
                    "dashes": False,
                    "datasource": "Prometheus",
                    "fieldConfig": {
                        "defaults": {
                            "custom": {}
                        },
                        "overrides": []
                    },
                    "fill": 1,
                    "fillGradient": 0,
                    "gridPos": {
                        "h": 8,
                        "w": 12,
                        "x": 0,
                        "y": 0
                    },
                    "hiddenSeries": False,
                    "id": 2,
                    "legend": {
                        "avg": False,
                        "current": False,
                        "max": False,
                        "min": False,
                        "show": True,
                        "total": False,
                        "values": False
                    },
                    "lines": True,
                    "linewidth": 1,
                    "nullPointMode": "null",
                    "options": {
                        "alertThreshold": True
                    },
                    "percentage": False,
                    "pluginVersion": "7.2.0",
                    "pointradius": 2,
                    "points": False,
                    "renderer": "flot",
                    "seriesOverrides": [],
                    "spaceLength": 10,
                    "stack": False,
                    "steppedLine": False,
                    "targets": [
                        {
                            "expr": "ros_node_cpu_usage",
                            "interval": "",
                            "legendFormat": "{{node}}",
                            "refId": "A"
                        }
                    ],
                    "thresholds": [],
                    "timeFrom": None,
                    "timeRegions": [],
                    "timeShift": None,
                    "title": "ROS Node CPU Usage",
                    "tooltip": {
                        "shared": True,
                        "sort": 0,
                        "value_type": "individual"
                    },
                    "type": "graph",
                    "xaxis": {
                        "buckets": None,
                        "mode": "time",
                        "name": None,
                        "show": True,
                        "values": []
                    },
                    "yaxes": [
                        {
                            "format": "percent",
                            "label": None,
                            "logBase": 1,
                            "max": None,
                            "min": None,
                            "show": True
                        },
                        {
                            "format": "short",
                            "label": None,
                            "logBase": 1,
                            "max": None,
                            "min": None,
                            "show": True
                        }
                    ],
                    "yaxis": {
                        "align": False,
                        "alignLevel": None
                    }
                },
                {
                    "aliasColors": {},
                    "bars": False,
                    "dashLength": 10,
                    "dashes": False,
                    "datasource": "Prometheus",
                    "fieldConfig": {
                        "defaults": {
                            "custom": {}
                        },
                        "overrides": []
                    },
                    "fill": 1,
                    "fillGradient": 0,
                    "gridPos": {
                        "h": 8,
                        "w": 12,
                        "x": 12,
                        "y": 0
                    },
                    "hiddenSeries": False,
                    "id": 3,
                    "legend": {
                        "avg": False,
                        "current": False,
                        "max": False,
                        "min": False,
                        "show": True,
                        "total": False,
                        "values": False
                    },
                    "lines": True,
                    "linewidth": 1,
                    "nullPointMode": "null",
                    "options": {
                        "alertThreshold": True
                    },
                    "percentage": False,
                    "pluginVersion": "7.2.0",
                    "pointradius": 2,
                    "points": False,
                    "renderer": "flot",
                    "seriesOverrides": [],
                    "spaceLength": 10,
                    "stack": False,
                    "steppedLine": False,
                    "targets": [
                        {
                            "expr": "ros_node_memory_usage_mb",
                            "interval": "",
                            "legendFormat": "{{node}}",
                            "refId": "A"
                        }
                    ],
                    "thresholds": [],
                    "timeFrom": None,
                    "timeRegions": [],
                    "timeShift": None,
                    "title": "ROS Node Memory Usage",
                    "tooltip": {
                        "shared": True,
                        "sort": 0,
                        "value_type": "individual"
                    },
                    "type": "graph",
                    "xaxis": {
                        "buckets": None,
                        "mode": "time",
                        "name": None,
                        "show": True,
                        "values": []
                    },
                    "yaxes": [
                        {
                            "format": "decmbytes",
                            "label": None,
                            "logBase": 1,
                            "max": None,
                            "min": None,
                            "show": True
                        },
                        {
                            "format": "short",
                            "label": None,
                            "logBase": 1,
                            "max": None,
                            "min": None,
                            "show": True
                        }
                    ],
                    "yaxis": {
                        "align": False,
                        "alignLevel": None
                    }
                },
                {
                    "aliasColors": {},
                    "bars": False,
                    "dashLength": 10,
                    "dashes": False,
                    "datasource": "Prometheus",
                    "fieldConfig": {
                        "defaults": {
                            "custom": {}
                        },
                        "overrides": []
                    },
                    "fill": 1,
                    "fillGradient": 0,
                    "gridPos": {
                        "h": 8,
                        "w": 12,
                        "x": 0,
                        "y": 8
                    },
                    "hiddenSeries": False,
                    "id": 4,
                    "legend": {
                        "avg": False,
                        "current": False,
                        "max": False,
                        "min": False,
                        "show": True,
                        "total": False,
                        "values": False
                    },
                    "lines": True,
                    "linewidth": 1,
                    "nullPointMode": "null",
                    "options": {
                        "alertThreshold": True
                    },
                    "percentage": False,
                    "pluginVersion": "7.2.0",
                    "pointradius": 2,
                    "points": False,
                    "renderer": "flot",
                    "seriesOverrides": [],
                    "spaceLength": 10,
                    "stack": False,
                    "steppedLine": False,
                    "targets": [
                        {
                            "expr": "ros_topic_message_frequency",
                            "interval": "",
                            "legendFormat": "{{topic}}",
                            "refId": "A"
                        }
                    ],
                    "thresholds": [],
                    "timeFrom": None,
                    "timeRegions": [],
                    "timeShift": None,
                    "title": "ROS Topic Message Frequency",
                    "tooltip": {
                        "shared": True,
                        "sort": 0,
                        "value_type": "individual"
                    },
                    "type": "graph",
                    "xaxis": {
                        "buckets": None,
                        "mode": "time",
                        "name": None,
                        "show": True,
                        "values": []
                    },
                    "yaxes": [
                        {
                            "format": "hz",
                            "label": None,
                            "logBase": 1,
                            "max": None,
                            "min": None,
                            "show": True
                        },
                        {
                            "format": "short",
                            "label": None,
                            "logBase": 1,
                            "max": None,
                            "min": None,
                            "show": True
                        }
                    ],
                    "yaxis": {
                        "align": False,
                        "alignLevel": None
                    }
                },
                {
                    "datasource": "Loki",
                    "fieldConfig": {
                        "defaults": {
                            "custom": {}
                        },
                        "overrides": []
                    },
                    "gridPos": {
                        "h": 8,
                        "w": 12,
                        "x": 12,
                        "y": 8
                    },
                    "id": 6,
                    "options": {
                        "showLabels": False,
                        "showTime": False,
                        "sortOrder": "Descending",
                        "wrapLogMessage": False
                    },
                    "targets": [
                        {
                            "expr": "{job=\"ros-logs\"}",
                            "refId": "A"
                        }
                    ],
                    "title": "ROS Logs",
                    "type": "logs"
                }
            ],
            "schemaVersion": 26,
            "style": "dark",
            "tags": [],
            "templating": {
                "list": []
            },
            "time": {
                "from": "now-6h",
                "to": "now"
            },
            "timepicker": {},
            "timezone": "",
            "title": "ROS System Dashboard",
            "uid": "ros-system",
            "version": 1
        }
        
        # Convert dashboard to JSON string
        import json
        ros_system_dashboard_json = json.dumps(ros_system_dashboard)
        
        # Create ConfigMap with dashboard
        configmap_data = {
            "metadata": {
                "name": "ros-dashboards",
                "namespace": namespace,
            },
            "data": {
                "ros-system-dashboard.json": ros_system_dashboard_json,
            },
        }
        
        try:
            self._core_api.create_namespaced_config_map(namespace=namespace, body=configmap_data)
            LOGGER.info("Created ROS dashboards ConfigMap.")
        except ApiException as e:
            if e.status != 409:  # 409 means it already exists
                LOGGER.warning(f"Error creating ROS dashboards ConfigMap: {e}")
                return
            
            # Update existing ConfigMap
            try:
                self._core_api.patch_namespaced_config_map(
                    name="ros-dashboards", 
                    namespace=namespace, 
                    body={"data": {"ros-system-dashboard.json": ros_system_dashboard_json}}
                )
                LOGGER.info("Updated ROS dashboards ConfigMap.")
            except ApiException as e:
                LOGGER.warning(f"Error updating ROS dashboards ConfigMap: {e}")
    
    def _deploy_ros_metrics_exporter(self) -> None:
        """Deploy a ROS metrics exporter that collects ROS-specific metrics."""
        namespace = "default"
        
        # Create ConfigMap with ROS metrics exporter script
        ros_metrics_script = """#!/usr/bin/env python3
import rospy
import psutil
import time
import os
import socket
from prometheus_client import start_http_server, Gauge, Counter
from std_msgs.msg import String

# Initialize ROS node
rospy.init_node('ros_metrics_exporter', anonymous=True)

# Create Prometheus metrics
node_cpu_usage = Gauge('ros_node_cpu_usage', 'CPU usage of ROS nodes', ['node'])
node_memory_usage = Gauge('ros_node_memory_usage_mb', 'Memory usage of ROS nodes in MB', ['node'])
topic_message_frequency = Gauge('ros_topic_message_frequency', 'Message frequency on ROS topics', ['topic'])
topic_message_count = Counter('ros_topic_message_count', 'Total messages on ROS topics', ['topic'])

# Track message timestamps for frequency calculation
topic_last_msg_time = {}
topic_frequencies = {}

# Callback for monitoring topic messages
def topic_callback(msg, topic_name):
    now = time.time()
    topic_message_count.labels(topic=topic_name).inc()
    
    if topic_name in topic_last_msg_time:
        # Calculate frequency based on time since last message
        elapsed = now - topic_last_msg_time[topic_name]
        if elapsed > 0:
            freq = 1.0 / elapsed
            topic_frequencies[topic_name] = freq
            topic_message_frequency.labels(topic=topic_name).set(freq)
    
    topic_last_msg_time[topic_name] = now

def get_ros_nodes():
    try:
        return [node for node in rospy.get_node_names() if node != '/ros_metrics_exporter']
    except Exception as e:
        rospy.logwarn(f"Error getting ROS nodes: {e}")
        return []

def get_ros_topics():
    try:
        return rospy.get_published_topics()
    except Exception as e:
        rospy.logwarn(f"Error getting ROS topics: {e}")
        return []

def monitor_nodes():
    # Get all ROS nodes
    nodes = get_ros_nodes()
    
    # For each node, try to get its process info
    for node in nodes:
        try:
            # This is a simplification - in a real system you'd need to map ROS node names to PIDs
            # For demo purposes, we'll just use random values
            cpu_percent = psutil.cpu_percent(interval=None) / len(nodes)
            memory_mb = psutil.virtual_memory().used / (1024 * 1024) / len(nodes)
            
            node_cpu_usage.labels(node=node).set(cpu_percent)
            node_memory_usage.labels(node=node).set(memory_mb)
        except Exception as e:
            rospy.logwarn(f"Error monitoring node {node}: {e}")

def subscribe_to_topics():
    # Get all topics
    topics = get_ros_topics()
    
    # Subscribe to each topic
    for topic_name, topic_type in topics:
        try:
            rospy.Subscriber(topic_name, String, topic_callback, callback_args=topic_name)
            rospy.loginfo(f"Subscribed to {topic_name}")
        except Exception as e:
            rospy.logwarn(f"Error subscribing to {topic_name}: {e}")

if __name__ == '__main__':
    # Start Prometheus HTTP server
    start_http_server(9090)
    rospy.loginfo("Started ROS metrics exporter on port 9090")
    
    # Subscribe to topics
    subscribe_to_topics()
    
    # Main monitoring loop
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        monitor_nodes()
        rate.sleep()
"""
        
        configmap_data = {
            "metadata": {
                "name": "ros-metrics-exporter",
                "namespace": namespace,
            },
            "data": {
                "ros_metrics_exporter.py": ros_metrics_script,
            },
        }
        
        try:
            self._core_api.create_namespaced_config_map(namespace=namespace, body=configmap_data)
            LOGGER.info("Created ROS metrics exporter ConfigMap.")
        except ApiException as e:
            if e.status != 409:  # 409 means it already exists
                LOGGER.warning(f"Error creating ROS metrics exporter ConfigMap: {e}")
                return
            
            # Update existing ConfigMap
            try:
                self._core_api.patch_namespaced_config_map(
                    name="ros-metrics-exporter", 
                    namespace=namespace, 
                    body={"data": {"ros_metrics_exporter.py": ros_metrics_script}}
                )
                LOGGER.info("Updated ROS metrics exporter ConfigMap.")
            except ApiException as e:
                LOGGER.warning(f"Error updating ROS metrics exporter ConfigMap: {e}")
                return
        
        # Create the ROS metrics exporter deployment
        exporter_deployment = {
            "apiVersion": "apps/v1",
            "kind": "Deployment",
            "metadata": {
                "name": "ros-metrics-exporter",
                "namespace": namespace,
            },
            "spec": {
                "replicas": 1,
                "selector": {
                    "matchLabels": {
                        "app": "ros-metrics-exporter",
                    },
                },
                "template": {
                    "metadata": {
                        "labels": {
                            "app": "ros-metrics-exporter",
                        },
                        "annotations": {
                            "prometheus.io/scrape": "true",
                            "prometheus.io/port": "9090",
                        },
                    },
                    "spec": {
                        "containers": [
                            {
                                "name": "ros-metrics-exporter",
                                "image": f"ros:{self.application.distro}-ros-base",
                                "command": ["/bin/bash", "-c"],
                                "args": [
                                    "apt-get update && "
                                    "apt-get install -y python3-pip && "
                                    "pip3 install prometheus-client psutil && "
                                    "chmod +x /scripts/ros_metrics_exporter.py && "
                                    "mkdir -p /var/log/ros && "
                                    "python3 /scripts/ros_metrics_exporter.py"
                                ],
                                "volumeMounts": [
                                    {
                                        "name": "ros-metrics-script",
                                        "mountPath": "/scripts",
                                    },
                                ],
                                "ports": [
                                    {
                                        "containerPort": 9090,
                                    },
                                ],
                                "env": [
                                    {
                                        "name": "ROS_MASTER_URI",
                                        "value": "http://ros-master:11311",
                                    },
                                    {
                                        "name": "ROS_HOSTNAME",
                                        "valueFrom": {
                                            "fieldRef": {
                                                "fieldPath": "status.podIP",
                                            },
                                        },
                                    },
                                ],
                            },
                        ],
                        "volumes": [
                            {
                                "name": "ros-metrics-script",
                                "configMap": {
                                    "name": "ros-metrics-exporter",
                                    "defaultMode": 0o755,
                                },
                            },
                        ],
                    },
                },
            },
        }
        
        try:
            self._apps_api.create_namespaced_deployment(namespace=namespace, body=exporter_deployment)
            LOGGER.info("Created ROS metrics exporter deployment.")
        except ApiException as e:
            if e.status != 409:  # 409 means it already exists
                LOGGER.warning(f"Error creating ROS metrics exporter deployment: {e}")
                return
            
            # Update existing deployment
            try:
                self._apps_api.patch_namespaced_deployment(
                    name="ros-metrics-exporter", 
                    namespace=namespace, 
                    body=exporter_deployment
                )
                LOGGER.info("Updated ROS metrics exporter deployment.")
            except ApiException as e:
                LOGGER.warning(f"Error updating ROS metrics exporter deployment: {e}")
    
    def _print_observability_access_info(self) -> None:
        """Print information on how to access the observability tools."""
        namespace = "monitoring"
        
        try:
            # Get Grafana service info
            grafana_svc = self._core_api.read_namespaced_service("grafana", namespace)
            grafana_type = grafana_svc.spec.type
            
            if grafana_type == "NodePort":
                node_port = None
                for port in grafana_svc.spec.ports:
                    if port.port == 3000:
                        node_port = port.node_port
                        break
                
                if node_port:
                    LOGGER.info(f"\nGrafana dashboard is available at: http://<node-ip>:{node_port}")
                    LOGGER.info("Username: admin")
                    LOGGER.info(f"Password: {self.orch.observability.grafana.get('admin_password', 'admin')}")
            elif grafana_type == "LoadBalancer":
                LOGGER.info("\nGrafana dashboard will be available at the LoadBalancer external IP")
                LOGGER.info("Run 'kubectl get svc -n monitoring grafana' to get the external IP")
            else:
                LOGGER.info("\nTo access Grafana, run: kubectl port-forward -n monitoring svc/grafana 3000:3000")
                LOGGER.info("Then open: http://localhost:3000")
            
            LOGGER.info("\nTo access Prometheus, run: kubectl port-forward -n monitoring svc/prometheus-server 9090:80")
            LOGGER.info("Then open: http://localhost:9090")
            
            LOGGER.info("\nTo access Loki, run: kubectl port-forward -n monitoring svc/loki 3100:3100")
            LOGGER.info("Loki is configured as a datasource in Grafana")
            
        except ApiException as e:
            LOGGER.warning(f"Error getting service information: {e}")

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
