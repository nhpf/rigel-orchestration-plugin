import os
from types import SimpleNamespace

import pytest

from src.plugin import OrchestrationPlugin


@pytest.mark.integration
@pytest.mark.skipif(os.environ.get("RUN_K8S_INTEGRATION") != "1", reason="set RUN_K8S_INTEGRATION=1 to enable")
def test_plugin_end_to_end_in_minikube() -> None:
    """Test requires a local Minikube (or other cluster) running, and a valid kubeconfig."""
    # OPTIONAL: start minikube or ensure it's started.
    # subprocess.run(["minikube", "start"], check=True)  # noqa: ERA001

    # Prepare raw_data that references a real image, readiness script, etc.
    raw_data = {
        "orchestration": {
            "deploy_ros_master": True,
            "application_image": "ros:noetic-ros-core",
            "readiness": {"command": "/bin/true", "timeout_seconds": 180},
            "additional_k8s_params": {
                "application": {
                    "spec": {
                        "template": {
                            "spec": {
                                "containers": [
                                    {"name": "ros-app", "command": ["/bin/bash", "-c"], "args": ["sleep infinity"]}
                                ]
                            }
                        }
                    }
                }
            },
        },
    }
    plugin = OrchestrationPlugin(
        raw_data=raw_data,
        global_data={},
        application=SimpleNamespace(distro="noetic"),
        providers_data={},
        shared_data={},
    )

    plugin.start()

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

    plugin.stop()
