# Rigel Kubernetes Orchestration

Deploy ROS 1 applications to Kubernetes with a ROS master Service, application
readiness checks, persistent storage, rolling updates, optional remote-node
scheduling, and an optional Prometheus/Loki/Grafana stack.

The project can be used in two ways:

- `rigel-k8s` is the supported standalone command. It reads the orchestration job
  from a Rigelfile and does not depend on Rigel itself.
- `src.plugin.OrchestrationPlugin` remains compatible with the unpublished Rigel
  0.3 plugin API for environments that already have it installed.

Rigel 0.3 was never published and its former Git repository is no longer public.
The last public Rigel release (0.2.22) has a different plugin API, so it cannot run
this plugin. For that reason Rigel is no longer an installation dependency.

## Requirements

- Python 3.10+
- A Kubernetes cluster and working kubeconfig (not needed for `--dry-run`)
- Helm 3.7+ only when observability is enabled (current Loki charts also
  require Kubernetes 1.25+)
- Poetry for the development workflow

## Install and run

```bash
poetry install

# Render manifests without contacting Kubernetes
poetry run rigel-k8s Rigelfile.example --dry-run

# Apply the first orchestration job in the file
poetry run rigel-k8s Rigelfile.example

# Select a job explicitly
poetry run rigel-k8s Rigelfile.example --job deploy_k8s
```

The command tries local kubeconfig credentials first and then Kubernetes
in-cluster credentials. Missing credentials and Kubernetes API errors cause a
non-zero exit instead of being logged and ignored.

## Configuration

A minimal Rigelfile is:

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

### Storage

Storage uses dynamic provisioning by default:

```yaml
persistent_storage:
  volumes:
    - name: logs
      size: 1Gi
      storage_class: standard
      mount_path: /var/log/ros
```

This creates a PVC and mounts it in the application. Set `host_path` only for a
trusted single-node development cluster; doing so also creates a static PV.

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

Remote scheduling adds `nodeSelector: {deploymentType: remote}`. Nodes must carry
that label. `force_local_flag` suppresses the selector.

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

Set `observability_only: true` for a job that should install only this stack.
Existing Rigel 0.3 sequences named `observability` receive the same behavior.
The older development-branch keys such as `admin_password`, `retention`, and
`scrape_interval` are translated to their corresponding Helm values.

Helm is invoked without a shell, waits for each enabled release, and propagates
installation failures.

## Application image

The application image must stay running and include any executable named by the
readiness command. The included Dockerfile is a ROS Noetic example. Its readiness
script succeeds when it can query the configured ROS master.

The plugin creates these resources:

```text
ros-master Service -> ros-master Deployment
                              ^
                              | ROS_MASTER_URI
PVC(s) ------------> application Deployment
```

## Development

```bash
poetry run ruff check src tests
poetry run mypy src tests --pretty
poetry run pytest
```

The Kubernetes integration test is opt-in:

```bash
RUN_K8S_INTEGRATION=1 poetry run pytest tests/test_integration.py -v
```

It expects an already configured cluster. CI creates a kind cluster before
enabling it.
