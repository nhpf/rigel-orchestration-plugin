from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field


class ReadinessConfig(BaseModel):
    """Readiness configuration for the application."""

    command: str = Field(..., description="Shell command/script that indicates readiness.")


class ObservabilityConfig(BaseModel):
    """Observability configuration for the application."""

    enabled: bool = False
    grafana: dict[str, Any] = Field(
        default_factory=lambda: {
            "enabled": True,
            "admin_password": "admin",
            "default_dashboards": True,
            "service_type": "NodePort",
            "persistence": {"enabled": True, "size": "2Gi"},
        }
    )
    prometheus: dict[str, Any] = Field(
        default_factory=lambda: {
            "enabled": True,
            "scrape_interval": "15s",
            "evaluation_interval": "15s",
            "retention": "24h",
            "service_type": "ClusterIP",
        }
    )
    loki: dict[str, Any] = Field(
        default_factory=lambda: {
            "enabled": True,
            "retention": "24h",
            "service_type": "ClusterIP",
            "persistence": {"enabled": True, "size": "5Gi"},
        }
    )
    node_exporter: bool = True
    ros_metrics: bool = True


class RollingUpdateConfig(BaseModel):
    """Rolling update configuration for the application."""

    strategy: str = "Rolling"
    max_surge: int = 1
    max_unavailable: int = 0


class VolumeConfig(BaseModel):
    """Volume configuration for the application."""

    name: str
    size: str
    storage_class: str


class PersistentStorageConfig(BaseModel):
    """Persistent storage configuration for the application."""

    volumes: list[VolumeConfig] = []


class DistributedConfig(BaseModel):
    """Distributed configuration for the application."""

    enabled: bool = False
    default_to_remote: bool = False
    force_local_flag: bool = False


class KubernetesOrchestrationModel(BaseModel):
    """Kubernetes Orchestration configuration for the application."""

    deploy_ros_master: bool = True
    readiness: ReadinessConfig | None = None
    observability: ObservabilityConfig | None = None
    rolling_update: RollingUpdateConfig | None = None
    distributed: DistributedConfig | None = None
    persistent_storage: PersistentStorageConfig | None = None
    additional_k8s_params: dict[str, Any] = {}


class OrchestrationPluginModel(BaseModel):
    """The top-level schema for our Orchestration plugin.

    It contains a nested 'KubernetesOrchestrationModel'
    under the key `orchestration`.
    """

    orchestration: KubernetesOrchestrationModel
