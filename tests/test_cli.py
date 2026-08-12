from pathlib import Path

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
