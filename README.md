# Rigel Kubernetes Orchestration

Deploy ROS 1 applications to Kubernetes with a ROS master Service, readiness
checks, persistent storage, rolling updates, optional remote-node scheduling,
result export, and an optional Prometheus/Loki/Grafana stack.

`rigel-k8s` is the supported standalone command. It reads one orchestration job
from a Rigelfile and does not depend on Rigel itself. The
`src.plugin.OrchestrationPlugin` class remains compatible with the unpublished
Rigel 0.3 plugin API for environments that already have it installed.

Rigel 0.3 was never published and its former Git repository is no longer public.
The last public Rigel release (0.2.22) has a different plugin API, so it cannot run
this plugin and is not an installation dependency.

## Requirements

- Python 3.10+
- A Kubernetes cluster and working kubeconfig (`--dry-run` is the exception)
- `kubectl` configured for the same cluster when collecting results
- Helm 3.7+ when installing or uninstalling observability components
- Kubernetes 1.25+ when using the current Loki chart
- Poetry for development or installation from a source checkout

Before deploying, the selected namespace must exist and the caller must be able
to manage Deployments, Services, Pods, and PVCs in it. Static `hostPath` storage
also requires cluster-wide PV permissions. The configured StorageClass and
application image must be available to the target cluster.

## Install and run

From a checkout:

```bash
poetry install

# Render manifests without contacting Kubernetes.
poetry run rigel-k8s Rigelfile.example --dry-run

# Apply the first orchestration job in the file.
poetry run rigel-k8s Rigelfile.example

# Select a job explicitly.
poetry run rigel-k8s Rigelfile.example --job deploy_k8s
```

Tagged GitHub releases contain a wheel and source archive. A wheel can be
installed with `pip install <downloaded-wheel>`. PyPI publication is not
configured.

The command tries local kubeconfig credentials first and then Kubernetes
in-cluster credentials. Configuration, credential, Kubernetes API, Helm, and
copy failures produce a non-zero exit status.

## Configuration

Start with `Rigelfile.example`. Its only job is directly supported by the
standalone command. A minimal job is:

```yaml
vars:
  distro: noetic
  image: registry.example/robot:v1

application:
  distro: "{{ vars.distro }}"

jobs:
  deploy_k8s:
    plugin: src.plugin.OrchestrationPlugin
    with:
      orchestration:
        application_image: "{{ vars.image }}"
        deploy_ros_master: true
        readiness:
          command: /usr/local/bin/readiness_probe.sh
          timeout_seconds: 120
```

`additional_k8s_params.application` and `additional_k8s_params.ros_master`
overlay their respective generated manifests. Kubernetes lists whose entries
have a `name` field—containers, environment variables, volumes, and mounts—are
merged by name. Other lists are replaced.

### Persistent storage and results

The application owns result production: it must write files beneath the mounted
path. The orchestrator owns persistence and explicit export:

```yaml
persistent_storage:
  volumes:
    - name: results
      size: 10Gi
      storage_class: standard
      mount_path: /results

results:
  source_path: /results
  container: ros-app
```

Export from the newest ready application pod without redeploying:

```bash
poetry run rigel-k8s Rigelfile --collect-results ./collected-results
```

Collection uses `kubectl cp`, so the application image must contain `tar`. Use a
new destination per run to avoid mixing outputs. Export important results before
deleting storage.

Dynamic provisioning is used when `host_path` is absent. Set `host_path` only
for a trusted, single-node development cluster; it creates a static PV whose
data is tied to that node.

### Rolling updates and distributed scheduling

```yaml
rolling_update:
  strategy: Rolling
  max_surge: 1
  max_unavailable: 0

distributed:
  enabled: true
  default_to_remote: true
  force_local_flag: false
```

Remote scheduling adds `nodeSelector: {deploymentType: remote}`. Eligible nodes
must carry that label. `force_local_flag` suppresses the selector.

### Observability

Observability is opt-in and uses Helm OCI charts. Each component accepts a
release, chart, enabled flag, and arbitrary Helm values:

```yaml
observability:
  enabled: true
  namespace: monitoring
  prometheus:
    release: prometheus
    chart: oci://ghcr.io/prometheus-community/charts/prometheus
    values:
      server:
        retention: 7d
  loki:
    enabled: false
    release: loki
    chart: oci://ghcr.io/grafana-community/helm-charts/loki
  grafana:
    release: grafana
    chart: oci://ghcr.io/grafana/helm-charts/grafana
```

Set `observability_only: true` for a job that installs only this stack. Existing
Rigel 0.3 sequences named `observability` receive the same behavior. Legacy
development-branch observability keys are translated to Helm values. Helm is
invoked without a shell, waits for enabled releases, and propagates failures.

## Application contract

The application image must:

- remain running for as long as the workload is active;
- include the executable configured as the readiness command;
- include `tar` when result collection is used;
- write outputs under `results.source_path` when results are expected; and
- be pullable by the target cluster.

The included Dockerfile is a ROS Noetic example. Its readiness script succeeds
when it can query the configured ROS master.

## Cleanup

Cleanup is explicit. By default it removes the application and ROS master
workloads but preserves PVCs, PVs, and observability releases:

```bash
poetry run rigel-k8s Rigelfile --cleanup
```

After exporting results, storage and observability can also be removed:

```bash
poetry run rigel-k8s Rigelfile --cleanup --delete-storage --uninstall-observability
```

`--delete-storage` permanently deletes managed PVCs and any managed hostPath PVs.
The plugin never deletes the namespace.

## Production handover

Before enabling unattended runs:

1. Pin the application image to an immutable tag or digest.
2. Confirm namespace, RBAC, registry credentials, StorageClass, capacity, and
   retention policy in the target cluster.
3. Run a deployment, verify ROS behavior, export a known result, and perform a
   storage-preserving cleanup.
4. Restore into or inspect the retained storage as required by the operating
   procedure, then test destructive cleanup in a disposable namespace.
5. Configure GitHub branch protection to require all `CI` jobs before merge.
6. Select a project license before distributing the package outside its current
   ownership boundary.

See `docs/HANDOVER.md` for the operator checklist and failure recovery commands.

## Development

```bash
poetry run ruff check src tests
poetry run mypy src tests --pretty
poetry run pytest
poetry build
```

The Kubernetes integration test is opt-in:

```bash
RUN_K8S_INTEGRATION=1 poetry run pytest tests/test_integration.py -v
```

It expects a disposable, configured cluster plus `kubectl`. CI creates a kind
cluster and verifies deployment, readiness, persistent output export, and cleanup.
