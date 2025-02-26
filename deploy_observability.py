#!/usr/bin/env python3
"""
Deploy the observability stack (Prometheus, Loki, Grafana) for ROS applications.
This script can be run directly without using Rigel.
"""

import logging
import subprocess
import time
import json
import yaml
import os
from pathlib import Path
from kubernetes import config, client
from kubernetes.client import ApiException, AppsV1Api, CoreV1Api

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
LOGGER = logging.getLogger("deploy-observability")

def create_namespace(api, name="monitoring"):
    """Create a Kubernetes namespace if it doesn't exist."""
    try:
        api.create_namespace(body={"metadata": {"name": name}})
        LOGGER.info(f"Created '{name}' namespace.")
    except ApiException as e:
        if e.status != 409:  # 409 means it already exists
            LOGGER.warning(f"Error creating namespace: {e}")
            return False
        LOGGER.info(f"Namespace '{name}' already exists.")
    return True

def add_helm_repos():
    """Add required Helm repositories."""
    commands = [
        "helm repo add grafana https://grafana.github.io/helm-charts",
        "helm repo add prometheus-community https://prometheus-community.github.io/helm-charts",
        "helm repo update",
    ]

    for command in commands:
        try:
            subprocess.run(command, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            LOGGER.warning(f"Error adding Helm repositories: {e}")
            return False
    return True

def deploy_prometheus(namespace="monitoring"):
    """Deploy Prometheus using Helm."""
    prom_values = {
        "server": {
            "retention": "7d",
            "global": {
                "scrape_interval": "10s",
                "evaluation_interval": "10s",
            },
        },
        "service": {
            "type": "ClusterIP",
        },
        "alertmanager": {
            "enabled": True,
            "persistence": {
                "enabled": True,
                "size": "2Gi",
            },
        },
        "nodeExporter": {
            "enabled": True,
        },
        "kubeStateMetrics": {
            "enabled": True,
        },
    }
    
    # Write Prometheus values to a temporary file
    with Path("/tmp/prometheus-values.yaml").open("w") as f:
        yaml.dump(prom_values, f)
    
    try:
        subprocess.run(
            f"helm upgrade --install prometheus prometheus-community/prometheus "
            f"-n {namespace} -f /tmp/prometheus-values.yaml",
            shell=True, check=True
        )
        LOGGER.info("Prometheus installed/upgraded successfully.")
        return True
    except subprocess.CalledProcessError as e:
        LOGGER.warning(f"Error installing Prometheus: {e}")
        return False

def deploy_loki(namespace="monitoring"):
    """Deploy Loki stack using Helm."""
    loki_values = {
        "loki": {
            "persistence": {
                "enabled": True,
                "size": "5Gi",
            },
            "config": {
                "table_manager": {
                    "retention_deletes_enabled": True,
                    "retention_period": "7d",
                },
            },
        },
        "promtail": {
            "enabled": True,
            "config": {
                "snippets": {
                    "extraScrapeConfigs": """
                    - job_name: ros-logs
                      static_configs:
                      - targets:
                          - localhost
                        labels:
                          job: ros-logs
                          __path__: /var/log/ros/*.log
                    """,
                },
            },
        },
    }
    
    # Write Loki values to a temporary file
    with Path("/tmp/loki-values.yaml").open("w") as f:
        yaml.dump(loki_values, f)
    
    try:
        subprocess.run(
            f"helm upgrade --install loki grafana/loki-stack "
            f"-n {namespace} -f /tmp/loki-values.yaml",
            shell=True, check=True
        )
        LOGGER.info("Loki stack installed/upgraded successfully.")
        return True
    except subprocess.CalledProcessError as e:
        LOGGER.warning(f"Error installing Loki: {e}")
        return False

def create_ros_dashboard_configmap(api, namespace="monitoring"):
    """Create a ConfigMap with ROS dashboard definitions for Grafana."""
    # Basic ROS system dashboard JSON
    ros_system_dashboard = {
        "annotations": {
            "list": [
                {
                    "builtIn": 1,
                    "datasource": "-- Grafana --",
                    "enable": True,
                    "hide": True,
                    "iconColor": "rgba(0, 211, 255, 1)",
                    "name": "Annotations & Alerts",
                    "type": "dashboard"
                }
            ]
        },
        "editable": True,
        "gnetId": None,
        "graphTooltip": 0,
        "id": 1,
        "links": [],
        "panels": [
            {
                "aliasColors": {},
                "bars": False,
                "dashLength": 10,
                "dashes": False,
                "datasource": "Prometheus",
                "fieldConfig": {
                    "defaults": {
                        "custom": {}
                    },
                    "overrides": []
                },
                "fill": 1,
                "fillGradient": 0,
                "gridPos": {
                    "h": 8,
                    "w": 12,
                    "x": 0,
                    "y": 0
                },
                "hiddenSeries": False,
                "id": 2,
                "legend": {
                    "avg": False,
                    "current": False,
                    "max": False,
                    "min": False,
                    "show": True,
                    "total": False,
                    "values": False
                },
                "lines": True,
                "linewidth": 1,
                "nullPointMode": "null",
                "options": {
                    "alertThreshold": True
                },
                "percentage": False,
                "pluginVersion": "7.2.0",
                "pointradius": 2,
                "points": False,
                "renderer": "flot",
                "seriesOverrides": [],
                "spaceLength": 10,
                "stack": False,
                "steppedLine": False,
                "targets": [
                    {
                        "expr": "ros_node_cpu_usage",
                        "interval": "",
                        "legendFormat": "{{node}}",
                        "refId": "A"
                    }
                ],
                "thresholds": [],
                "timeFrom": None,
                "timeRegions": [],
                "timeShift": None,
                "title": "ROS Node CPU Usage",
                "tooltip": {
                    "shared": True,
                    "sort": 0,
                    "value_type": "individual"
                },
                "type": "graph",
                "xaxis": {
                    "buckets": None,
                    "mode": "time",
                    "name": None,
                    "show": True,
                    "values": []
                },
                "yaxes": [
                    {
                        "format": "percent",
                        "label": None,
                        "logBase": 1,
                        "max": None,
                        "min": None,
                        "show": True
                    },
                    {
                        "format": "short",
                        "label": None,
                        "logBase": 1,
                        "max": None,
                        "min": None,
                        "show": True
                    }
                ],
                "yaxis": {
                    "align": False,
                    "alignLevel": None
                }
            },
            {
                "aliasColors": {},
                "bars": False,
                "dashLength": 10,
                "dashes": False,
                "datasource": "Prometheus",
                "fieldConfig": {
                    "defaults": {
                        "custom": {}
                    },
                    "overrides": []
                },
                "fill": 1,
                "fillGradient": 0,
                "gridPos": {
                    "h": 8,
                    "w": 12,
                    "x": 12,
                    "y": 0
                },
                "hiddenSeries": False,
                "id": 3,
                "legend": {
                    "avg": False,
                    "current": False,
                    "max": False,
                    "min": False,
                    "show": True,
                    "total": False,
                    "values": False
                },
                "lines": True,
                "linewidth": 1,
                "nullPointMode": "null",
                "options": {
                    "alertThreshold": True
                },
                "percentage": False,
                "pluginVersion": "7.2.0",
                "pointradius": 2,
                "points": False,
                "renderer": "flot",
                "seriesOverrides": [],
                "spaceLength": 10,
                "stack": False,
                "steppedLine": False,
                "targets": [
                    {
                        "expr": "ros_node_memory_usage_mb",
                        "interval": "",
                        "legendFormat": "{{node}}",
                        "refId": "A"
                    }
                ],
                "thresholds": [],
                "timeFrom": None,
                "timeRegions": [],
                "timeShift": None,
                "title": "ROS Node Memory Usage",
                "tooltip": {
                    "shared": True,
                    "sort": 0,
                    "value_type": "individual"
                },
                "type": "graph",
                "xaxis": {
                    "buckets": None,
                    "mode": "time",
                    "name": None,
                    "show": True,
                    "values": []
                },
                "yaxes": [
                    {
                        "format": "decmbytes",
                        "label": None,
                        "logBase": 1,
                        "max": None,
                        "min": None,
                        "show": True
                    },
                    {
                        "format": "short",
                        "label": None,
                        "logBase": 1,
                        "max": None,
                        "min": None,
                        "show": True
                    }
                ],
                "yaxis": {
                    "align": False,
                    "alignLevel": None
                }
            },
            {
                "aliasColors": {},
                "bars": False,
                "dashLength": 10,
                "dashes": False,
                "datasource": "Prometheus",
                "fieldConfig": {
                    "defaults": {
                        "custom": {}
                    },
                    "overrides": []
                },
                "fill": 1,
                "fillGradient": 0,
                "gridPos": {
                    "h": 8,
                    "w": 12,
                    "x": 0,
                    "y": 8
                },
                "hiddenSeries": False,
                "id": 4,
                "legend": {
                    "avg": False,
                    "current": False,
                    "max": False,
                    "min": False,
                    "show": True,
                    "total": False,
                    "values": False
                },
                "lines": True,
                "linewidth": 1,
                "nullPointMode": "null",
                "options": {
                    "alertThreshold": True
                },
                "percentage": False,
                "pluginVersion": "7.2.0",
                "pointradius": 2,
                "points": False,
                "renderer": "flot",
                "seriesOverrides": [],
                "spaceLength": 10,
                "stack": False,
                "steppedLine": False,
                "targets": [
                    {
                        "expr": "ros_topic_message_frequency",
                        "interval": "",
                        "legendFormat": "{{topic}}",
                        "refId": "A"
                    }
                ],
                "thresholds": [],
                "timeFrom": None,
                "timeRegions": [],
                "timeShift": None,
                "title": "ROS Topic Message Frequency",
                "tooltip": {
                    "shared": True,
                    "sort": 0,
                    "value_type": "individual"
                },
                "type": "graph",
                "xaxis": {
                    "buckets": None,
                    "mode": "time",
                    "name": None,
                    "show": True,
                    "values": []
                },
                "yaxes": [
                    {
                        "format": "hz",
                        "label": None,
                        "logBase": 1,
                        "max": None,
                        "min": None,
                        "show": True
                    },
                    {
                        "format": "short",
                        "label": None,
                        "logBase": 1,
                        "max": None,
                        "min": None,
                        "show": True
                    }
                ],
                "yaxis": {
                    "align": False,
                    "alignLevel": None
                }
            },
            {
                "datasource": "Loki",
                "fieldConfig": {
                    "defaults": {
                        "custom": {}
                    },
                    "overrides": []
                },
                "gridPos": {
                    "h": 8,
                    "w": 12,
                    "x": 12,
                    "y": 8
                },
                "id": 6,
                "options": {
                    "showLabels": False,
                    "showTime": False,
                    "sortOrder": "Descending",
                    "wrapLogMessage": False
                },
                "targets": [
                    {
                        "expr": "{job=\"ros-logs\"}",
                        "refId": "A"
                    }
                ],
                "title": "ROS Logs",
                "type": "logs"
            }
        ],
        "schemaVersion": 26,
        "style": "dark",
        "tags": [],
        "templating": {
            "list": []
        },
        "time": {
            "from": "now-6h",
            "to": "now"
        },
        "timepicker": {},
        "timezone": "",
        "title": "ROS System Dashboard",
        "uid": "ros-system",
        "version": 1
    }
    
    # Convert dashboard to JSON string
    ros_system_dashboard_json = json.dumps(ros_system_dashboard)
    
    # Create ConfigMap with dashboard
    configmap_data = {
        "metadata": {
            "name": "ros-dashboards",
            "namespace": namespace,
        },
        "data": {
            "ros-system-dashboard.json": ros_system_dashboard_json,
        },
    }
    
    try:
        api.create_namespaced_config_map(namespace=namespace, body=configmap_data)
        LOGGER.info("Created ROS dashboards ConfigMap.")
        return True
    except ApiException as e:
        if e.status != 409:  # 409 means it already exists
            LOGGER.warning(f"Error creating ROS dashboards ConfigMap: {e}")
            return False
        
        # Update existing ConfigMap
        try:
            api.patch_namespaced_config_map(
                name="ros-dashboards", 
                namespace=namespace, 
                body={"data": {"ros-system-dashboard.json": ros_system_dashboard_json}}
            )
            LOGGER.info("Updated ROS dashboards ConfigMap.")
            return True
        except ApiException as e:
            LOGGER.warning(f"Error updating ROS dashboards ConfigMap: {e}")
            return False

def deploy_grafana(namespace="monitoring"):
    """Deploy Grafana using Helm."""
    grafana_values = {
        "adminPassword": "rigel-admin",
        "persistence": {
            "enabled": True,
            "size": "2Gi",
        },
        "service": {
            "type": "NodePort",
        },
        "datasources": {
            "datasources.yaml": {
                "apiVersion": 1,
                "datasources": [
                    {
                        "name": "Prometheus",
                        "type": "prometheus",
                        "url": "http://prometheus-server.monitoring.svc.cluster.local",
                        "access": "proxy",
                        "isDefault": True,
                    },
                    {
                        "name": "Loki",
                        "type": "loki",
                        "url": "http://loki.monitoring.svc.cluster.local:3100",
                        "access": "proxy",
                    },
                ],
            },
        },
        "dashboardProviders": {
            "dashboardproviders.yaml": {
                "apiVersion": 1,
                "providers": [
                    {
                        "name": "default",
                        "orgId": 1,
                        "folder": "",
                        "type": "file",
                        "disableDeletion": False,
                        "editable": True,
                        "options": {
                            "path": "/var/lib/grafana/dashboards/default",
                        },
                    },
                ],
            },
        },
        "dashboardsConfigMaps": {
            "default": "ros-dashboards",
        },
    }
    
    # Write Grafana values to a temporary file
    with Path("/tmp/grafana-values.yaml").open("w") as f:
        yaml.dump(grafana_values, f)
    
    try:
        subprocess.run(
            f"helm upgrade --install grafana grafana/grafana "
            f"-n {namespace} -f /tmp/grafana-values.yaml",
            shell=True, check=True
        )
        LOGGER.info("Grafana installed/upgraded successfully.")
        return True
    except subprocess.CalledProcessError as e:
        LOGGER.warning(f"Error installing Grafana: {e}")
        return False

def deploy_ros_metrics_exporter(api, apps_api, ros_distro="noetic", namespace="default"):
    """Deploy a ROS metrics exporter that collects ROS-specific metrics."""
    # Create ConfigMap with ROS metrics exporter script
    ros_metrics_script = """#!/usr/bin/env python3
import rospy
import psutil
import time
import os
import socket
from prometheus_client import start_http_server, Gauge, Counter
from std_msgs.msg import String

# Initialize ROS node
rospy.init_node('ros_metrics_exporter', anonymous=True)

# Create Prometheus metrics
node_cpu_usage = Gauge('ros_node_cpu_usage', 'CPU usage of ROS nodes', ['node'])
node_memory_usage = Gauge('ros_node_memory_usage_mb', 'Memory usage of ROS nodes in MB', ['node'])
topic_message_frequency = Gauge('ros_topic_message_frequency', 'Message frequency on ROS topics', ['topic'])
topic_message_count = Counter('ros_topic_message_count', 'Total messages on ROS topics', ['topic'])

# Track message timestamps for frequency calculation
topic_last_msg_time = {}
topic_frequencies = {}

# Callback for monitoring topic messages
def topic_callback(msg, topic_name):
    now = time.time()
    topic_message_count.labels(topic=topic_name).inc()
    
    if topic_name in topic_last_msg_time:
        # Calculate frequency based on time since last message
        elapsed = now - topic_last_msg_time[topic_name]
        if elapsed > 0:
            freq = 1.0 / elapsed
            topic_frequencies[topic_name] = freq
            topic_message_frequency.labels(topic=topic_name).set(freq)
    
    topic_last_msg_time[topic_name] = now

def get_ros_nodes():
    try:
        return [node for node in rospy.get_node_names() if node != '/ros_metrics_exporter']
    except Exception as e:
        rospy.logwarn(f"Error getting ROS nodes: {e}")
        return []

def get_ros_topics():
    try:
        return rospy.get_published_topics()
    except Exception as e:
        rospy.logwarn(f"Error getting ROS topics: {e}")
        return []

def monitor_nodes():
    # Get all ROS nodes
    nodes = get_ros_nodes()
    
    # For each node, try to get its process info
    for node in nodes:
        try:
            # This is a simplification - in a real system you'd need to map ROS node names to PIDs
            # For demo purposes, we'll just use random values
            cpu_percent = psutil.cpu_percent(interval=None) / len(nodes) if nodes else 0
            memory_mb = psutil.virtual_memory().used / (1024 * 1024) / len(nodes) if nodes else 0
            
            node_cpu_usage.labels(node=node).set(cpu_percent)
            node_memory_usage.labels(node=node).set(memory_mb)
        except Exception as e:
            rospy.logwarn(f"Error monitoring node {node}: {e}")

def subscribe_to_topics():
    # Get all topics
    topics = get_ros_topics()
    
    # Subscribe to each topic
    for topic_name, topic_type in topics:
        try:
            rospy.Subscriber(topic_name, String, topic_callback, callback_args=topic_name)
            rospy.loginfo(f"Subscribed to {topic_name}")
        except Exception as e:
            rospy.logwarn(f"Error subscribing to {topic_name}: {e}")

if __name__ == '__main__':
    # Start Prometheus HTTP server
    start_http_server(9090)
    rospy.loginfo("Started ROS metrics exporter on port 9090")
    
    # Subscribe to topics
    subscribe_to_topics()
    
    # Main monitoring loop
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        monitor_nodes()
        rate.sleep()
"""
    
    configmap_data = {
        "metadata": {
            "name": "ros-metrics-exporter",
            "namespace": namespace,
        },
        "data": {
            "ros_metrics_exporter.py": ros_metrics_script,
        },
    }
    
    try:
        api.create_namespaced_config_map(namespace=namespace, body=configmap_data)
        LOGGER.info("Created ROS metrics exporter ConfigMap.")
    except ApiException as e:
        if e.status != 409:  # 409 means it already exists
            LOGGER.warning(f"Error creating ROS metrics exporter ConfigMap: {e}")
            return False
        
        # Update existing ConfigMap
        try:
            api.patch_namespaced_config_map(
                name="ros-metrics-exporter", 
                namespace=namespace, 
                body={"data": {"ros_metrics_exporter.py": ros_metrics_script}}
            )
            LOGGER.info("Updated ROS metrics exporter ConfigMap.")
        except ApiException as e:
            LOGGER.warning(f"Error updating ROS metrics exporter ConfigMap: {e}")
            return False
    
    # Create the ROS metrics exporter deployment
    exporter_deployment = {
        "apiVersion": "apps/v1",
        "kind": "Deployment",
        "metadata": {
            "name": "ros-metrics-exporter",
            "namespace": namespace,
        },
        "spec": {
            "replicas": 1,
            "selector": {
                "matchLabels": {
                    "app": "ros-metrics-exporter",
                },
            },
            "template": {
                "metadata": {
                    "labels": {
                        "app": "ros-metrics-exporter",
                    },
                    "annotations": {
                        "prometheus.io/scrape": "true",
                        "prometheus.io/port": "9090",
                    },
                },
                "spec": {
                    "containers": [
                        {
                            "name": "ros-metrics-exporter",
                            "image": f"ros:{ros_distro}-ros-base",
                            "command": ["/bin/bash", "-c"],
                            "args": [
                                "apt-get update && "
                                "apt-get install -y python3-pip && "
                                "pip3 install prometheus-client psutil && "
                                "chmod +x /scripts/ros_metrics_exporter.py && "
                                "mkdir -p /var/log/ros && "
                                "python3 /scripts/ros_metrics_exporter.py"
                            ],
                            "volumeMounts": [
                                {
                                    "name": "ros-metrics-script",
                                    "mountPath": "/scripts",
                                },
                            ],
                            "ports": [
                                {
                                    "containerPort": 9090,
                                },
                            ],
                            "env": [
                                {
                                    "name": "ROS_MASTER_URI",
                                    "value": "http://ros-master:11311",
                                },
                                {
                                    "name": "ROS_HOSTNAME",
                                    "valueFrom": {
                                        "fieldRef": {
                                            "fieldPath": "status.podIP",
                                        },
                                    },
                                },
                            ],
                        },
                    ],
                    "volumes": [
                        {
                            "name": "ros-metrics-script",
                            "configMap": {
                                "name": "ros-metrics-exporter",
                                "defaultMode": 0o755,
                            },
                        },
                    ],
                },
            },
        },
    }
    
    try:
        apps_api.create_namespaced_deployment(namespace=namespace, body=exporter_deployment)
        LOGGER.info("Created ROS metrics exporter deployment.")
        return True
    except ApiException as e:
        if e.status != 409:  # 409 means it already exists
            LOGGER.warning(f"Error creating ROS metrics exporter deployment: {e}")
            return False
        
        # Update existing deployment
        try:
            apps_api.patch_namespaced_deployment(
                name="ros-metrics-exporter", 
                namespace=namespace, 
                body=exporter_deployment
            )
            LOGGER.info("Updated ROS metrics exporter deployment.")
            return True
        except ApiException as e:
            LOGGER.warning(f"Error updating ROS metrics exporter deployment: {e}")
            return False

def print_access_info(api, namespace="monitoring"):
    """Print information on how to access the observability tools."""
    try:
        # Get Grafana service info
        grafana_svc = api.read_namespaced_service("grafana", namespace)
        grafana_type = grafana_svc.spec.type
        
        if grafana_type == "NodePort":
            node_port = None
            for port in grafana_svc.spec.ports:
                if port.port == 3000:
                    node_port = port.node_port
                    break
            
            if node_port:
                LOGGER.info(f"\nGrafana dashboard is available at: http://<node-ip>:{node_port}")
                LOGGER.info("Username: admin")
                LOGGER.info("Password: rigel-admin")
        elif grafana_type == "LoadBalancer":
            LOGGER.info("\nGrafana dashboard will be available at the LoadBalancer external IP")
            LOGGER.info("Run 'kubectl get svc -n monitoring grafana' to get the external IP")
        else:
            LOGGER.info("\nTo access Grafana, run: kubectl port-forward -n monitoring svc/grafana 3000:3000")
            LOGGER.info("Then open: http://localhost:3000")
        
        LOGGER.info("\nTo access Prometheus, run: kubectl port-forward -n monitoring svc/prometheus-server 9090:80")
        LOGGER.info("Then open: http://localhost:9090")
        
        LOGGER.info("\nTo access Loki, run: kubectl port-forward -n monitoring svc/loki 3100:3100")
        LOGGER.info("Loki is configured as a datasource in Grafana")
        
    except ApiException as e:
        LOGGER.warning(f"Error getting service information: {e}")

def main():
    """Main function to deploy the observability stack."""
    # Set up Kubernetes client
    try:
        # Try to detect if we're running in minikube
        if subprocess.run(["which", "minikube"], stdout=subprocess.PIPE, stderr=subprocess.PIPE).returncode == 0:
            try:
                # Get minikube docker env and apply it
                LOGGER.info("Detected minikube, trying to use minikube context")
                subprocess.run(["minikube", "start"], check=False)
                config.load_kube_config(context="minikube")
                LOGGER.info("Using minikube context")
            except Exception as e:
                LOGGER.warning(f"Failed to use minikube context: {e}, falling back to default kubeconfig")
                config.load_kube_config()
        else:
            config.load_kube_config()
            LOGGER.info("Using local kubeconfig")
    except Exception:
        try:
            config.load_incluster_config()
            LOGGER.info("Using in-cluster config")
        except Exception as e:
            LOGGER.error(f"Could not configure Kubernetes client: {e}")
            LOGGER.error("Make sure you have a running Kubernetes cluster and kubectl is configured correctly")
            LOGGER.error("You might need to run: minikube start")
            return False

    core_api = CoreV1Api()
    apps_api = AppsV1Api()
    
    # Create monitoring namespace
    if not create_namespace(core_api):
        return False
    
    # Add Helm repositories
    if not add_helm_repos():
        return False
    
    # Deploy Prometheus
    if not deploy_prometheus():
        return False
    
    # Deploy Loki
    if not deploy_loki():
        return False
    
    # Create ROS dashboard ConfigMap
    if not create_ros_dashboard_configmap(core_api):
        return False
    
    # Deploy Grafana
    if not deploy_grafana():
        return False
    
    # Deploy ROS metrics exporter
    if not deploy_ros_metrics_exporter(core_api, apps_api):
        return False
    
    # Print access information
    print_access_info(core_api)
    
    return True

if __name__ == "__main__":
    success = main()
    if success:
        LOGGER.info("Observability stack deployed successfully!")
    else:
        LOGGER.error("Failed to deploy observability stack")
