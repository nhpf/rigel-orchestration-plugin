"""Standalone command-line entry point for the orchestration plugin."""

from __future__ import annotations

import argparse
import logging
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from jinja2 import Environment, StrictUndefined

from .plugin import OrchestrationPlugin


@dataclass(frozen=True)
class Application:
    """Minimal application information required by the orchestrator."""

    distro: str


def _regex_replace(value: object, pattern: str, replacement: str) -> str:
    return re.sub(pattern, replacement, str(value))


def load_rigelfile(path: Path) -> dict[str, Any]:
    """Render the variables in a Rigelfile and parse its YAML content."""
    source = path.read_text(encoding="utf-8")
    unrendered = yaml.safe_load(source) or {}
    variables = unrendered.get("vars", {})
    environment = Environment(undefined=StrictUndefined, autoescape=False)  # noqa: S701 - renders YAML, not HTML
    environment.filters["regex_replace"] = _regex_replace
    rendered = environment.from_string(source).render(vars=variables)
    result = yaml.safe_load(rendered)
    if not isinstance(result, dict):
        msg = f"{path} must contain a YAML mapping"
        raise TypeError(msg)
    return result


def _orchestration_job(document: dict[str, Any], requested: str | None) -> tuple[str, dict[str, Any]]:
    jobs = document.get("jobs", {})
    if requested:
        try:
            job = jobs[requested]
        except KeyError as error:
            msg = f"Job {requested!r} does not exist"
            raise ValueError(msg) from error
        return requested, job
    for name, job in jobs.items():
        if isinstance(job, dict) and "orchestration" in job.get("with", {}):
            return name, job
    msg = "No job with an 'orchestration' configuration was found"
    raise ValueError(msg)


def _dry_run_documents(plugin: OrchestrationPlugin) -> list[dict[str, Any]]:
    documents: list[dict[str, Any]] = []
    if plugin.orch.deploy_ros_master:
        documents.extend([plugin.build_ros_master_service(), plugin.build_ros_master_deployment()])
    if plugin.orch.persistent_storage:
        for volume in plugin.orch.persistent_storage.volumes:
            if volume.host_path:
                documents.append(plugin._persistent_volume_body(volume))  # noqa: SLF001
            documents.append(plugin._persistent_volume_claim_body(volume))  # noqa: SLF001
    documents.append(plugin.build_application_deployment())
    return documents


def main(argv: list[str] | None = None) -> int:
    """Deploy one orchestration job from a Rigelfile."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rigelfile", nargs="?", type=Path, default=Path("Rigelfile"))
    parser.add_argument("--job", help="orchestration job name (auto-detected by default)")
    parser.add_argument("--dry-run", action="store_true", help="print Kubernetes YAML without contacting a cluster")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)
    document = load_rigelfile(args.rigelfile)
    _, job = _orchestration_job(document, args.job)
    application_data = document.get("application", {})
    distro = application_data.get("distro") or document.get("distro")
    if not distro:
        parser.error("the Rigelfile must define application.distro or distro")

    plugin = OrchestrationPlugin(
        raw_data=job.get("with", {}),
        global_data={},
        application=Application(distro=str(distro)),
        providers_data={},
    )
    if args.dry_run:
        sys.stdout.write(yaml.safe_dump_all(_dry_run_documents(plugin), sort_keys=False))
        return 0
    plugin.start()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
