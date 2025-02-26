#!/bin/bash
# Script to run the observability sequence with Rigel

echo "Starting Kubernetes cluster if not running..."
if which minikube > /dev/null; then
    minikube status || minikube start
else
    echo "Minikube not found. Make sure you have a running Kubernetes cluster."
fi

echo "Running Rigel observability sequence..."
rigel run sequence observability

if [ $? -ne 0 ]; then
    echo "Rigel sequence failed. Trying standalone script..."
    ./deploy_observability.py
fi

echo "Setting up port forwarding for Grafana..."
kubectl port-forward -n monitoring svc/grafana 3000:3000 &
PF_PID=$!

echo "Grafana should be available at: http://localhost:3000"
echo "Username: admin"
echo "Password: rigel-admin"
echo ""
echo "Press Ctrl+C to stop port forwarding when done"

# Wait for user to press Ctrl+C
trap "kill $PF_PID; echo 'Port forwarding stopped'" INT
wait
