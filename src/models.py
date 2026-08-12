from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field, root_validator, validator


def _nested_value(target: dict[str, Any], path: tuple[str, ...], *, value: object) -> None:
    """Set a nested default without overwriting explicit Helm values."""
    current = target
    for key in path[:-1]:
        child = current.setdefault(key, {})
        if not isinstance(child, dict):
            return
        current = child
    current.setdefault(path[-1], value)


class ReadinessConfig(BaseModel):
    """Readiness configuration for the application."""

    command: str = Field(..., min_length=1, description="Executable command that indicates readiness.")
    initial_delay_seconds: int = 5
    period_seconds: int = 5
    timeout_seconds: int = 60

    @validator("initial_delay_seconds", "timeout_seconds")
    def non_negative_seconds(cls, value: int) -> int:  # noqa: N805 - required by Pydantic 1 validators
        """Reject invalid Kubernetes timing values."""
        if value < 0:
            msg = "readiness timing values cannot be negative"
            raise ValueError(msg)
        return value

    @validator("period_seconds")
    def positive_period(cls, value: int) -> int:  # noqa: N805 - required by Pydantic 1 validators
        """Prevent a busy readiness polling loop."""
        if value <= 0:
            msg = "readiness period_seconds must be positive"
            raise ValueError(msg)
        return value


class HelmComponentConfig(BaseModel):
    """A Helm-managed observability component."""

    enabled: bool = True
    release: str
    chart: str
    values: dict[str, Any] = Field(default_factory=dict)


class ObservabilityConfig(BaseModel):
    """Observability configuration for the application."""

    enabled: bool = False
    namespace: str = "monitoring"
    prometheus: HelmComponentConfig = Field(
        default_factory=lambda: HelmComponentConfig(
            release="prometheus",
            chart="oci://ghcr.io/prometheus-community/charts/prometheus",
        ),
    )
    loki: HelmComponentConfig = Field(
        default_factory=lambda: HelmComponentConfig(
            release="loki",
            chart="oci://ghcr.io/grafana-community/helm-charts/loki",
            values={
                "deploymentMode": "Monolithic",
                "loki": {
                    "auth_enabled": False,
                    "commonConfig": {"replication_factor": 1},
                    "useTestSchema": True,
                },
                "singleBinary": {"replicas": 1},
                "backend": {"replicas": 0},
                "read": {"replicas": 0},
                "write": {"replicas": 0},
            },
        ),
    )
    grafana: HelmComponentConfig = Field(
        default_factory=lambda: HelmComponentConfig(
            release="grafana",
            chart="oci://ghcr.io/grafana/helm-charts/grafana",
        ),
    )

    @root_validator(pre=True)
    def legacy_component_settings(cls, values: dict[str, Any]) -> dict[str, Any]:  # noqa: C901, N805, PLR0912
        """Accept and translate the configuration used by the development branch."""
        normalized = dict(values)
        defaults = {
            "prometheus": ("prometheus", "oci://ghcr.io/prometheus-community/charts/prometheus"),
            "loki": ("loki", "oci://ghcr.io/grafana-community/helm-charts/loki"),
            "grafana": ("grafana", "oci://ghcr.io/grafana/helm-charts/grafana"),
        }
        for name, (release, chart) in defaults.items():
            raw_component = normalized.get(name)
            if not isinstance(raw_component, dict):
                continue
            component = dict(raw_component)
            component.setdefault("release", release)
            component.setdefault("chart", chart)
            helm_values = dict(component.get("values", {}))

            if name == "prometheus":
                if "retention" in component:
                    _nested_value(helm_values, ("server", "retention"), value=component["retention"])
                if "scrape_interval" in component:
                    _nested_value(
                        helm_values,
                        ("server", "global", "scrape_interval"),
                        value=component["scrape_interval"],
                    )
                if "evaluation_interval" in component:
                    _nested_value(
                        helm_values,
                        ("server", "global", "evaluation_interval"),
                        value=component["evaluation_interval"],
                    )
            elif name == "loki":
                helm_values.setdefault("deploymentMode", "Monolithic")
                _nested_value(helm_values, ("loki", "auth_enabled"), value=False)
                _nested_value(helm_values, ("loki", "commonConfig", "replication_factor"), value=1)
                _nested_value(helm_values, ("loki", "useTestSchema"), value=True)
                _nested_value(helm_values, ("singleBinary", "replicas"), value=1)
                for workload in ("backend", "read", "write"):
                    _nested_value(helm_values, (workload, "replicas"), value=0)
                if "retention" in component:
                    _nested_value(
                        helm_values,
                        ("loki", "limits_config", "retention_period"),
                        value=component["retention"],
                    )
                if "persistence" in component:
                    _nested_value(
                        helm_values,
                        ("singleBinary", "persistence"),
                        value=component["persistence"],
                    )
            else:
                if "admin_password" in component:
                    helm_values.setdefault("adminPassword", component["admin_password"])
                if "service_type" in component:
                    _nested_value(helm_values, ("service", "type"), value=component["service_type"])
                if "persistence" in component:
                    helm_values.setdefault("persistence", component["persistence"])

            component["values"] = helm_values
            normalized[name] = component
        return normalized


class RollingUpdateConfig(BaseModel):
    """Rolling update configuration for the application."""

    strategy: str = "Rolling"
    max_surge: int = 1
    max_unavailable: int = 0

    @validator("strategy")
    def supported_strategy(cls, value: str) -> str:  # noqa: N805 - required by Pydantic 1 validators
        """Normalize and validate supported Kubernetes update strategies."""
        normalized = value.lower()
        if normalized not in {"rolling", "recreate"}:
            msg = "rolling_update.strategy must be 'Rolling' or 'Recreate'"
            raise ValueError(msg)
        return "Recreate" if normalized == "recreate" else "Rolling"


class VolumeConfig(BaseModel):
    """Volume configuration for the application."""

    name: str
    size: str
    storage_class: str | None = None
    host_path: str | None = None
    mount_path: str | None = None


class PersistentStorageConfig(BaseModel):
    """Persistent storage configuration for the application."""

    volumes: list[VolumeConfig] = Field(default_factory=list)


class ResultsConfig(BaseModel):
    """Location of application results inside the deployed container."""

    source_path: str = Field(..., min_length=1)
    container: str = "ros-app"

    @validator("source_path")
    def absolute_source_path(cls, value: str) -> str:  # noqa: N805 - required by Pydantic 1 validators
        """Require an unambiguous absolute path for Kubernetes copies."""
        if not value.startswith("/"):
            msg = "results.source_path must be an absolute container path"
            raise ValueError(msg)
        return value


class DistributedConfig(BaseModel):
    """Distributed configuration for the application."""

    enabled: bool = False
    default_to_remote: bool = False
    force_local_flag: bool = False


class KubernetesOrchestrationModel(BaseModel):
    """Kubernetes Orchestration configuration for the application."""

    deploy_ros_master: bool = True
    observability_only: bool = False
    namespace: str = "default"
    deployment_name: str = "rigel-k8s-application"
    application_image: str | None = None
    ros_master_image: str | None = None
    readiness: ReadinessConfig | None = None
    observability: ObservabilityConfig | None = None
    rolling_update: RollingUpdateConfig | None = None
    distributed: DistributedConfig | None = None
    persistent_storage: PersistentStorageConfig | None = None
    results: ResultsConfig | None = None
    additional_k8s_params: dict[str, Any] = Field(default_factory=dict)


class OrchestrationPluginModel(BaseModel):
    """The top-level schema for our Orchestration plugin.

    It contains a nested 'KubernetesOrchestrationModel'
    under the key `orchestration`.
    """

    orchestration: KubernetesOrchestrationModel
