import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from src.plugin import OrchestrationPlugin


@pytest.mark.integration
@pytest.mark.skipif(os.environ.get("RUN_K8S_INTEGRATION") != "1", reason="set RUN_K8S_INTEGRATION=1 to enable")
def test_plugin_end_to_end_in_kubernetes(tmp_path: Path) -> None:
    """Deploy, verify persisted output, export it, and clean up in a disposable cluster."""
    raw_data = {
        "orchestration": {
            "deploy_ros_master": True,
            "application_image": "ros:noetic-ros-core",
            "readiness": {"command": "/bin/true", "timeout_seconds": 180},
            "persistent_storage": {
                "volumes": [
                    {
                        "name": "integration-results",
                        "size": "1Mi",
                        "storage_class": "manual",
                        "host_path": "/tmp/rigel-integration-results",  # noqa: S108 - disposable kind node
                        "mount_path": "/results",
                    }
                ]
            },
            "results": {"source_path": "/results/result.txt"},
            "additional_k8s_params": {
                "application": {
                    "spec": {
                        "template": {
                            "spec": {
                                "containers": [
                                    {
                                        "name": "ros-app",
                                        "command": ["/bin/bash", "-c"],
                                        "args": ["printf 'rigel-smoke-ok\\n' > /results/result.txt; sleep infinity"],
                                    }
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

    try:
        plugin.start()
        plugin.process()

        from kubernetes import client, config

        config.load_kube_config()
        v1 = client.CoreV1Api()
        pods = v1.list_namespaced_pod(
            "default",
            label_selector="app.kubernetes.io/name=rigel-k8s-application",
        )
        assert len(pods.items) > 0
        assert all(pod.status.phase == "Running" for pod in pods.items)

        destination = tmp_path / "result.txt"
        plugin.collect_results(destination)
        assert destination.read_text(encoding="utf-8") == "rigel-smoke-ok\n"
    finally:
        plugin.cleanup(delete_storage=True)
