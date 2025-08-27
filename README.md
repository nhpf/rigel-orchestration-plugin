# Kubernetes Orchestration Plugin for Rigel

Rigel plugin that orchestrates ROS applications on Kubernetes with automatic ROS Master deployment, readiness probes, persistent storage, rolling updates, and observability.

[![Tests](https://img.shields.io/badge/tests-11%2F11%20passing-brightgreen)](tests/)
[![Coverage](https://img.shields.io/badge/coverage-76%25-yellow)](#)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue)](#)

## Features

- 🚀 **Automatic ROS Master Deployment**: Deploys and manages ROS Master service for multi-node communication
- 🔧 **Flexible Container Management**: Supports any ROS Docker image via Rigelfile configuration
- 🏥 **Health Monitoring**: Built-in readiness probes with init container pattern for reliable startup
- 💾 **Persistent Storage**: Configurable persistent volumes for logs and data
- 🔄 **Rolling Updates**: Zero-downtime updates with configurable strategies
- 📊 **Observability Ready**: Integration points for Prometheus, Loki, and Grafana
- 🌐 **Distributed Deployment**: Optional multi-node deployment capabilities
- 🛡️ **Production Hardened**: Kubernetes-native resource management with proper error handling

## Quick Start

### Prerequisites

- Python 3.10+ with [Poetry](https://python-poetry.org/docs/)
- A running Kubernetes cluster ([Minikube](https://minikube.sigs.k8s.io/docs/), [kind](https://kind.sigs.k8s.io/), or cloud provider)
- `kubectl` configured to access your cluster
- Docker for building images

### Installation

1. **Clone and setup the plugin:**

```bash
git clone https://github.com/your-org/rigel-orchestration-plugin.git
cd rigel-orchestration-plugin
poetry install
```

2. **Create your project's Rigelfile:**

```bash
cp Rigelfile.example Rigelfile
# Edit Rigelfile to set your image name and configuration
```

3. **Configure your Docker image:**

```yaml
# In your Rigelfile
vars:
  distro: "noetic"
  base_image: "your-registry/your-ros-app:latest"
```

4. **Deploy to Kubernetes:**

```bash
poetry run rigel run sequence demo
```

### Verification

Check your deployment:

```bash
# Check pods
kubectl get pods -l app=rigel-k8s-application
kubectl get pods -l app=ros-master

# Check logs
kubectl logs deployment/ros-master
kubectl logs deployment/rigel-k8s-application

# Test ROS connectivity (optional)
kubectl exec -it deployment/rigel-k8s-application -- rostopic list
```

## Configuration

### Basic Configuration

The plugin is configured entirely through your `Rigelfile`. Here's a minimal example:

```yaml
vars:
  distro: "noetic"
  base_image: "my-registry/my-ros-app:v1.0.0"

jobs:
  build:
    plugin: "rigel.plugins.core.BuildXPlugin"
    with:
      image: "{{ vars.base_image }}"
      push: true # Push to registry for K8s access

  deploy_k8s:
    plugin: "src.plugin.OrchestrationPlugin"
    with:
      orchestration:
        deploy_ros_master: true
        readiness:
          command: "/usr/local/bin/readiness_probe.sh"
        persistent_storage:
          volumes:
            - name: "data-volume"
              size: "5Gi"
              storage_class: "fast-ssd"

sequences:
  deploy:
    stages:
      - jobs: ["build", "deploy_k8s"]
```

### Advanced Configuration Options

#### Persistent Storage

```yaml
persistent_storage:
  volumes:
    - name: "logs-volume"
      size: "1Gi"
      storage_class: "standard"
    - name: "data-volume"
      size: "10Gi"
      storage_class: "fast-ssd"
```

#### Rolling Update Strategy

```yaml
rolling_update:
  strategy: "Rolling"
  max_surge: 1
  max_unavailable: 0
```

#### Custom Environment Variables

```yaml
additional_k8s_params:
  application:
    spec:
      template:
        spec:
          containers:
            - name: "ros-app"
              env:
                - name: ROS_DOMAIN_ID
                  value: "42"
                - name: CUSTOM_CONFIG
                  value: "production"
```

#### Resource Limits

```yaml
additional_k8s_params:
  application:
    spec:
      template:
        spec:
          containers:
            - name: "ros-app"
              resources:
                requests:
                  cpu: "100m"
                  memory: "256Mi"
                limits:
                  cpu: "1000m"
                  memory: "1Gi"
```

## Docker Image Requirements

Your ROS Docker image must include:

1. **Readiness Probe Script**: Create `/usr/local/bin/readiness_probe.sh`
2. **Entry Point**: Executable entry point script
3. **ROS Environment**: Proper ROS workspace setup

### Example Dockerfile additions:

```dockerfile
# Copy readiness probe script
COPY readiness_probe.sh /usr/local/bin/readiness_probe.sh
RUN chmod +x /usr/local/bin/readiness_probe.sh

# Entry point for your application
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
```

### Example readiness_probe.sh:

```bash
#!/bin/bash
# Check if the ready file exists (created by init container)
if [ -f /tmp/ready ]; then
    exit 0
else
    exit 1
fi
```

## Architecture

The plugin creates the following Kubernetes resources:

```
┌─────────────────┐    ┌──────────────────┐
│   ROS Master    │    │   Application    │
│   Deployment    │    │   Deployment     │
│   + Service     │    │                  │
│                 │    │ ┌──────────────┐ │
│ ros:noetic-core │    │ │ Init Container│ │
│ roscore         │◄───┤ │ (readiness)   │ │
└─────────────────┘    │ └──────────────┘ │
                       │ ┌──────────────┐ │
                       │ │ Main Container│ │
                       │ │ (your app)   │ │
                       │ └──────────────┘ │
                       └──────────────────┘
                              │
                       ┌──────────────────┐
                       │ Persistent Volume│
                       │ Claims (PVCs)    │
                       └──────────────────┘
```

### Key Components

1. **ROS Master Service**: Provides stable endpoint for ROS communication
2. **Init Container Pattern**: Ensures reliable readiness probe setup
3. **Main Application Container**: Your ROS application with health checking
4. **Persistent Storage**: Configurable volumes for data persistence
5. **Rolling Update Support**: Zero-downtime deployments

## Operations

### Updating Applications

```bash
# Update your image version in Rigelfile
# Then run update sequence
poetry run rigel run sequence update
```

### Monitoring

```bash
# Watch pod status
kubectl get pods -w

# Check readiness probe status
kubectl describe pod <pod-name>

# View application logs
kubectl logs -f deployment/rigel-k8s-application
kubectl logs -f deployment/ros-master
```

### Troubleshooting

#### Pod Not Ready

```bash
# Check readiness probe
kubectl describe pod <pod-name>
kubectl exec <pod-name> -- /usr/local/bin/readiness_probe.sh

# Check init container logs
kubectl logs <pod-name> -c readiness-init
```

#### ROS Master Connection Issues

```bash
# Verify ROS master service
kubectl get svc ros-master
kubectl exec deployment/rigel-k8s-application -- rostopic list
```

#### Storage Issues

```bash
# Check PVCs
kubectl get pvc
kubectl describe pvc logs-volume-pvc
```

## Development

### Project Structure

```
├── src/
│   ├── plugin.py              # Main plugin implementation
│   ├── models.py              # Pydantic models for configuration
│   └── utils/
│       └── dict_operations.py # Utility functions
├── tests/                     # Comprehensive test suite
├── examples/
│   └── deploy_and_update.py   # Example deployment script
├── app_testing/               # Testing applications and examples
├── Dockerfile                 # Production-ready container image
├── dockerfile_entrypoint.sh   # Container entry point
├── readiness_probe.sh         # Readiness check script
├── Rigelfile                  # Test configuration
├── Rigelfile.example          # Template for users
└── README.md                  # This file
```

### Quality Assurance

This plugin maintains high code quality with:

- **✅ 100% Test Success Rate**: All 11 tests passing consistently
- **📊 76% Code Coverage**: Comprehensive test coverage across core functionality
- **🧪 Integration Tests**: End-to-end testing with Minikube
- **🔍 Type Safety**: Full type annotations with mypy compatibility
- **📝 Code Quality**: Linting with ruff and proper error handling

### Running Tests

```bash
# Run full test suite
poetry run pytest tests/ -v

# Run with coverage report
poetry run pytest tests/ --cov=src --cov-report=html

# Run only integration tests
poetry run pytest tests/test_integration.py -v
```

### Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass (11/11 passing required)
5. Maintain or improve code coverage
6. Submit a pull request

## Examples

### Basic Deployment Script

The `examples/deploy_and_update.py` script demonstrates how to:

- Deploy a ROS application to Kubernetes
- Perform rolling updates
- Monitor deployment status
- Handle common deployment scenarios

```bash
python examples/deploy_and_update.py --help
```

### Sample Applications

The `app_testing/` directory contains complete examples:

- **action/**: ROS action server/client example
- **pubsub/**: ROS publisher/subscriber example
- **service/**: ROS service with Docker Compose setup

## Configuration Reference

### OrchestrationPlugin Parameters

| Parameter                        | Type | Default     | Description                   |
| -------------------------------- | ---- | ----------- | ----------------------------- |
| `deploy_ros_master`              | bool | `false`     | Deploy ROS Master service     |
| `readiness.command`              | str  | -           | Readiness probe command       |
| `observability.enabled`          | bool | `false`     | Enable observability stack    |
| `rolling_update.strategy`        | str  | `"Rolling"` | Update strategy               |
| `rolling_update.max_surge`       | int  | `1`         | Max pods above desired        |
| `rolling_update.max_unavailable` | int  | `0`         | Max unavailable pods          |
| `persistent_storage.volumes`     | list | `[]`        | Volume configurations         |
| `distributed.enabled`            | bool | `false`     | Enable distributed deployment |

### Example Complete Configuration

See `Rigelfile.example` for a complete configuration template.
