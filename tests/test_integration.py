import time

import pytest
from rigel.models.application import Application

from src.plugin import OrchestrationPlugin


@pytest.mark.integration
def test_plugin_end_to_end_in_minikube() -> None:
    """Test requires a local Minikube (or other cluster) running, and a valid kubeconfig."""
    # OPTIONAL: start minikube or ensure it's started.
    # subprocess.run(["minikube", "start"], check=True)  # noqa: ERA001

    # Prepare raw_data that references a real image, readiness script, etc.
    raw_data = {
        "orchestration": {
            "deploy_ros_master": True,
            # ...
        },
    }
    plugin = OrchestrationPlugin(
        raw_data=raw_data,
        global_data={},
        application=Application(distro="noetic"),
        providers_data={},
        shared_data={},
    )

    plugin.setup()
    plugin.start()

    # Wait a bit so pods have time to come up
    time.sleep(30)

    # Possibly poll k8s to confirm the Deployment is up
    # or run plugin.process() again if you do additional logic
    plugin.process()

    # We could do kubectl checks, or direct python k8s-client calls
    # e.g.:
    from kubernetes import client, config

    config.load_kube_config()
    v1 = client.CoreV1Api()
    pods = v1.list_namespaced_pod("default", label_selector="app=rigel-k8s-application")
    assert len(pods.items) > 0
    # ...
    # check if it's Running
    for pod in pods.items:
        assert pod.status.phase == "Running", f"{pod.metadata.name} not running"

    # optional teardown
    plugin.stop()
