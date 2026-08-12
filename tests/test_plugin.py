import subprocess
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from kubernetes.client import ApiException

from src.plugin import OrchestrationPlugin


@pytest.fixture
def apis() -> tuple[MagicMock, MagicMock]:
    return MagicMock(), MagicMock()


@pytest.fixture
def plugin(apis: tuple[MagicMock, MagicMock]) -> OrchestrationPlugin:
    apps_api, core_api = apis
    return OrchestrationPlugin(
        raw_data={
            "orchestration": {
                "deploy_ros_master": True,
                "application_image": "registry.example/robot:v1",
                "readiness": {"command": "/usr/local/bin/readiness_probe.sh", "timeout_seconds": 0},
                "rolling_update": {"max_surge": 1, "max_unavailable": 0},
                "distributed": {"enabled": True, "default_to_remote": True},
                "persistent_storage": {
                    "volumes": [{"name": "logs", "size": "1Gi", "storage_class": "standard"}],
                },
                "additional_k8s_params": {
                    "application": {
                        "spec": {
                            "template": {
                                "spec": {
                                    "containers": [
                                        {
                                            "name": "ros-app",
                                            "env": [{"name": "CUSTOM_ENV", "value": "production"}],
                                        },
                                    ],
                                },
                            },
                        },
                    },
                },
            },
        },
        global_data={},
        application=SimpleNamespace(distro="noetic"),
        providers_data={},
        apps_api=apps_api,
        core_api=core_api,
    )


def test_init_does_not_contact_kubernetes(plugin: OrchestrationPlugin, apis: tuple[MagicMock, MagicMock]) -> None:
    assert plugin.application.distro == "noetic"
    assert plugin.orch.application_image == "registry.example/robot:v1"
    assert not apis[0].method_calls
    assert not apis[1].method_calls


def test_ros_master_is_created_with_service_and_correct_image(
    plugin: OrchestrationPlugin,
    apis: tuple[MagicMock, MagicMock],
) -> None:
    apps_api, core_api = apis
    core_api.read_namespaced_service.side_effect = ApiException(status=404)
    apps_api.read_namespaced_deployment.side_effect = ApiException(status=404)

    plugin.job_deploy_ros_master()

    core_api.create_namespaced_service.assert_called_once()
    apps_api.create_namespaced_deployment.assert_called_once()
    body = apps_api.create_namespaced_deployment.call_args.kwargs["body"]
    container = body["spec"]["template"]["spec"]["containers"][0]
    assert container["image"] == "ros:noetic-ros-core"
    assert container["args"] == ["roscore"]


def test_existing_resources_are_patched(plugin: OrchestrationPlugin, apis: tuple[MagicMock, MagicMock]) -> None:
    apps_api, core_api = apis

    plugin.job_deploy_ros_master()
    plugin.job_deploy_application()

    core_api.patch_namespaced_service.assert_called_once()
    assert apps_api.patch_namespaced_deployment.call_count == 2


def test_non_404_api_errors_are_not_swallowed(plugin: OrchestrationPlugin, apis: tuple[MagicMock, MagicMock]) -> None:
    _, core_api = apis
    core_api.read_namespaced_service.side_effect = ApiException(status=403)

    with pytest.raises(ApiException):
        plugin.job_deploy_ros_master()


def test_application_manifest_preserves_defaults_when_overridden(plugin: OrchestrationPlugin) -> None:
    manifest = plugin.build_application_deployment()
    spec = manifest["spec"]
    pod_spec = spec["template"]["spec"]
    container = pod_spec["containers"][0]

    assert container["name"] == "ros-app"
    assert container["image"] == "registry.example/robot:v1"
    assert container["readinessProbe"]["exec"]["command"] == ["/usr/local/bin/readiness_probe.sh"]
    assert {item["name"] for item in container["env"]} == {"ROS_MASTER_URI", "CUSTOM_ENV"}
    assert container["volumeMounts"] == [{"name": "logs", "mountPath": "/persistent/logs"}]
    assert pod_spec["volumes"][0]["persistentVolumeClaim"]["claimName"] == "logs-pvc"
    assert pod_spec["nodeSelector"] == {"deploymentType": "remote"}
    assert spec["strategy"]["type"] == "RollingUpdate"


def test_dynamic_storage_creates_only_a_claim(plugin: OrchestrationPlugin, apis: tuple[MagicMock, MagicMock]) -> None:
    _, core_api = apis
    core_api.read_namespaced_persistent_volume_claim.side_effect = ApiException(status=404)

    plugin.job_create_persistent_storage()

    core_api.create_namespaced_persistent_volume_claim.assert_called_once()
    core_api.create_persistent_volume.assert_not_called()
    body = core_api.create_namespaced_persistent_volume_claim.call_args.kwargs["body"]
    assert body["spec"]["storageClassName"] == "standard"
    assert "volumeName" not in body["spec"]


def test_readiness_requires_pods_statuses_and_ready_containers(
    plugin: OrchestrationPlugin,
    apis: tuple[MagicMock, MagicMock],
) -> None:
    _, core_api = apis
    core_api.list_namespaced_pod.return_value.items = []
    assert plugin.job_check_readiness() is False

    pod = SimpleNamespace(status=SimpleNamespace(phase="Running", container_statuses=[]))
    core_api.list_namespaced_pod.return_value.items = [pod]
    assert plugin.job_check_readiness() is False

    pod.status.container_statuses = [SimpleNamespace(ready=True)]
    assert plugin.job_check_readiness() is True
    core_api.list_namespaced_pod.assert_called_with(
        "default",
        label_selector="app.kubernetes.io/name=rigel-k8s-application",
    )


def test_rolling_and_distributed_patches(plugin: OrchestrationPlugin, apis: tuple[MagicMock, MagicMock]) -> None:
    apps_api, _ = apis

    plugin.job_rolling_update()
    plugin.job_distributed_deployment()

    assert apps_api.patch_namespaced_deployment.call_args_list[0].args[2] == {
        "spec": {
            "strategy": {
                "type": "RollingUpdate",
                "rollingUpdate": {"maxSurge": 1, "maxUnavailable": 0},
            },
        },
    }
    assert apps_api.patch_namespaced_deployment.call_args_list[1].args[2] == {
        "spec": {"template": {"spec": {"nodeSelector": {"deploymentType": "remote"}}}},
    }


def test_observability_runs_helm_without_a_shell(apis: tuple[MagicMock, MagicMock]) -> None:
    calls: list[list[str]] = []

    def runner(command: list[str], *, check: bool) -> subprocess.CompletedProcess[str]:
        assert check is True
        calls.append(command)
        return subprocess.CompletedProcess(command, 0)

    plugin = OrchestrationPlugin(
        raw_data={
            "orchestration": {
                "observability": {
                    "enabled": True,
                    "prometheus": {"release": "prom", "chart": "example/prom", "values": {"server": {}}},
                    "loki": {"enabled": False, "release": "loki", "chart": "example/loki"},
                    "grafana": {"release": "grafana", "chart": "example/grafana"},
                },
            },
        },
        global_data={},
        application=SimpleNamespace(distro="noetic"),
        providers_data={},
        apps_api=apis[0],
        core_api=apis[1],
        command_runner=runner,
    )

    plugin.job_configure_observability()

    assert len(calls) == 2
    assert calls[0][:5] == ["helm", "upgrade", "--install", "prom", "example/prom"]
    assert all(isinstance(command, list) for command in calls)


def test_development_branch_observability_config_is_translated(apis: tuple[MagicMock, MagicMock]) -> None:
    plugin = OrchestrationPlugin(
        raw_data={
            "orchestration": {
                "observability": {
                    "enabled": True,
                    "grafana": {"enabled": True, "admin_password": "secret", "service_type": "NodePort"},
                    "prometheus": {"enabled": True, "retention": "7d", "scrape_interval": "10s"},
                    "loki": {"enabled": True, "retention": "7d", "persistence": {"enabled": True, "size": "5Gi"}},
                },
            },
        },
        global_data={},
        application=SimpleNamespace(distro="noetic"),
        providers_data={},
        apps_api=apis[0],
        core_api=apis[1],
    )

    observability = plugin.orch.observability
    assert observability is not None
    assert observability.grafana.values["adminPassword"] == "secret"
    assert observability.grafana.values["service"]["type"] == "NodePort"
    assert observability.prometheus.values["server"]["retention"] == "7d"
    assert observability.prometheus.values["server"]["global"]["scrape_interval"] == "10s"
    assert observability.loki.chart == "oci://ghcr.io/grafana-community/helm-charts/loki"
    assert observability.loki.values["deploymentMode"] == "Monolithic"
    assert observability.loki.values["loki"]["limits_config"]["retention_period"] == "7d"
