from collections.abc import Iterator
from unittest.mock import ANY, MagicMock, patch

import pytest

# Import your plugin code
from src.plugin import DEPLOYMENT_NAME, OrchestrationPlugin


@pytest.fixture
def mock_k8s_apis() -> Iterator[tuple[MagicMock, MagicMock]]:
    """Fixture to patch the Kubernetes clients in your plugin."""
    with patch("src.plugin.AppsV1Api") as mock_apps, patch("src.plugin.CoreV1Api") as mock_core:
        yield mock_apps, mock_core


@pytest.fixture
def plugin_instance(mock_k8s_apis: tuple[MagicMock, MagicMock]) -> OrchestrationPlugin:
    """Return an instance of OrchestrationPlugin with minimal config."""
    raw_data = {
        "orchestration": {
            "deploy_ros_master": True,
            "readiness": {"command": "/usr/local/bin/readiness_probe.sh"},
            "observability": {"enabled": True},
            "rolling_update": {"strategy": "Rolling", "max_surge": 1, "max_unavailable": 0},
            "distributed": {"enabled": True, "default_to_remote": True, "force_local_flag": False},
            "persistent_storage": {
                "volumes": [
                    {"name": "logs-volume", "size": "1Gi", "storage_class": "standard"},
                ],
            },
            "additional_k8s_params": {},
        },
    }

    # Create a dummy plugin instance
    from rigel.models.application import Application

    application = Application(distro="noetic")

    # providers_data & shared_data can be empty dict
    return OrchestrationPlugin(
        raw_data=raw_data,
        global_data={},
        application=application,
        providers_data={},
        shared_data={},
    )


def test_plugin_init(plugin_instance: OrchestrationPlugin) -> None:
    """Check if the plugin is initialized correctly."""
    assert plugin_instance.model.orchestration.deploy_ros_master is True
    assert plugin_instance.application.distro == "noetic"


def test_deploy_ros_master(plugin_instance: OrchestrationPlugin, mock_k8s_apis: tuple[MagicMock, MagicMock]) -> None:
    """Ensure job_deploy_ros_master calls the create_namespaced_deployment if not found."""
    mock_apps_api, mock_core_api = mock_k8s_apis  # unpack the mocks

    # By default, read_namespaced_deployment is not set up, so let's simulate a 404
    instance = mock_apps_api.return_value

    from kubernetes.client.rest import ApiException as K8sApiException

    instance.read_namespaced_deployment.side_effect = K8sApiException(status=404, reason="Not Found")

    plugin_instance.job_deploy_ros_master()

    # We expect create_namespaced_deployment to have been called once
    instance.create_namespaced_deployment.assert_called_once()


def test_deploy_application(plugin_instance: OrchestrationPlugin, mock_k8s_apis: tuple[MagicMock, MagicMock]) -> None:
    """Check if job_deploy_application patches or creates the deployment properly."""
    mock_apps_api, _ = mock_k8s_apis
    instance = mock_apps_api.return_value

    # Simulate that reading the deployment fails => triggers a CREATE
    from kubernetes.client.rest import ApiException as K8sApiException

    instance.read_namespaced_deployment.side_effect = K8sApiException(status=404, reason="Not Found")

    plugin_instance.job_deploy_application()
    instance.create_namespaced_deployment.assert_called_once()

    # Now let's simulate a found deployment => triggers a PATCH
    instance.create_namespaced_deployment.reset_mock()
    instance.read_namespaced_deployment.side_effect = None  # no error
    plugin_instance.job_deploy_application()
    instance.patch_namespaced_deployment.assert_called_once_with(
        DEPLOYMENT_NAME,
        "default",
        ANY,
    )


def test_check_readiness(plugin_instance: OrchestrationPlugin, mock_k8s_apis: tuple[MagicMock, MagicMock]) -> None:
    """Validate that readiness returns True if the pods are Running and container ready."""
    mock_apps_api, mock_core_api = mock_k8s_apis
    core_instance = mock_core_api.return_value

    # Mock a single Pod in 'Running' phase with container ready
    fake_pod = MagicMock()
    fake_pod.status.phase = "Running"
    container_status = MagicMock()
    container_status.ready = True
    fake_pod.status.container_statuses = [container_status]
    core_instance.list_namespaced_pod.return_value.items = [fake_pod]

    is_ready = plugin_instance.job_check_readiness()
    assert is_ready is True

    # Now test case: not ready
    container_status.ready = False
    is_ready = plugin_instance.job_check_readiness()
    assert is_ready is False

    # etc.


@pytest.mark.parametrize("observability_enabled", [True, False])
def test_configure_observability(
    plugin_instance: OrchestrationPlugin, mock_k8s_apis: tuple[MagicMock, MagicMock], observability_enabled: bool
) -> None:
    """Check logs or skip logic for job_configure_observability."""
    if plugin_instance.orch.observability is None:
        # Create a default ObservabilityConfig or handle it as needed
        from src.models import ObservabilityConfig

        plugin_instance.orch.observability = ObservabilityConfig()

    plugin_instance.orch.observability.enabled = observability_enabled
    # Just ensure it doesn't crash
    plugin_instance.job_configure_observability()

    # Could assert logger output, or that we do some create_deployment calls if fully implemented


def test_rolling_update(plugin_instance: OrchestrationPlugin, mock_k8s_apis: tuple[MagicMock, MagicMock]) -> None:
    """Ensure rolling update patches the deployment strategy after readiness."""
    mock_apps_api, mock_core_api = mock_k8s_apis

    with patch.object(plugin_instance, "job_check_readiness", return_value=True):
        plugin_instance.job_rolling_update()

    mock_apps_api.return_value.patch_namespaced_deployment.assert_called_once_with(
        DEPLOYMENT_NAME,
        "default",
        {
            "spec": {
                "strategy": {
                    "type": "RollingUpdate",
                    "rollingUpdate": {
                        "maxSurge": 1,
                        "maxUnavailable": 0,
                    },
                },
            },
        },
    )


def test_distributed_deployment(
    plugin_instance: OrchestrationPlugin, mock_k8s_apis: tuple[MagicMock, MagicMock]
) -> None:
    """Check if nodeSelector patch is applied for distributed deployment."""
    mock_apps_api, _ = mock_k8s_apis
    instance = mock_apps_api.return_value

    plugin_instance.job_distributed_deployment()
    instance.patch_namespaced_deployment.assert_called_once_with(
        DEPLOYMENT_NAME,
        "default",
        {
            "spec": {
                "template": {
                    "spec": {
                        "nodeSelector": {
                            "deploymentType": "remote",
                        },
                    },
                },
            },
        },
    )
