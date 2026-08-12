from pathlib import Path
from unittest.mock import patch

import yaml
from _pytest.capture import CaptureFixture

from src.cli import load_rigelfile, main


def test_load_rigelfile_renders_vars_and_regex_filter(tmp_path: Path) -> None:
    path = tmp_path / "Rigelfile"
    path.write_text(
        'vars:\n  distro: noetic\n  image: example/app:v2\napplication:\n  distro: "{{ vars.distro }}"\n'
        "value: \"{{ vars.image | regex_replace('.*:', '') }}\"\n",
        encoding="utf-8",
    )

    document = load_rigelfile(path)

    assert document["application"]["distro"] == "noetic"
    assert document["value"] == "v2"


def test_dry_run_prints_valid_kubernetes_documents(tmp_path: Path, capsys: CaptureFixture[str]) -> None:
    path = tmp_path / "Rigelfile"
    path.write_text(
        """
vars:
  distro: noetic
  image: example/app:v1
application:
  distro: "{{ vars.distro }}"
jobs:
  deploy:
    plugin: src.plugin.OrchestrationPlugin
    with:
      orchestration:
        application_image: "{{ vars.image }}"
        readiness:
          command: /bin/true
""",
        encoding="utf-8",
    )

    assert main([str(path), "--dry-run"]) == 0

    output = capsys.readouterr().out
    documents = list(yaml.safe_load_all(output))
    assert [document["kind"] for document in documents] == ["Service", "Deployment", "Deployment"]
    assert documents[-1]["spec"]["template"]["spec"]["containers"][0]["image"] == "example/app:v1"


def test_collect_results_mode_does_not_deploy(tmp_path: Path) -> None:
    path = tmp_path / "Rigelfile"
    path.write_text(
        """
application:
  distro: noetic
jobs:
  deploy:
    with:
      orchestration:
        results:
          source_path: /results
""",
        encoding="utf-8",
    )
    destination = tmp_path / "collected"

    with patch("src.cli.OrchestrationPlugin") as plugin_type:
        assert main([str(path), "--collect-results", str(destination)]) == 0

    plugin_type.return_value.collect_results.assert_called_once_with(destination)
    plugin_type.return_value.start.assert_not_called()


def test_cleanup_mode_preserves_storage_by_default(tmp_path: Path) -> None:
    path = tmp_path / "Rigelfile"
    path.write_text(
        """
application:
  distro: noetic
jobs:
  deploy:
    with:
      orchestration: {}
""",
        encoding="utf-8",
    )

    with patch("src.cli.OrchestrationPlugin") as plugin_type:
        assert main([str(path), "--cleanup"]) == 0

    plugin_type.return_value.cleanup.assert_called_once_with(
        delete_storage=False,
        uninstall_observability=False,
    )
