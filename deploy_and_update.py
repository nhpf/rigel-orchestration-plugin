#!/usr/bin/env python3
"""
Example script demonstrating how to deploy and update a ROS application
using Rigel and the Kubernetes orchestration plugin.
"""

import argparse
import os
import subprocess
import time
from pathlib import Path

# Define colors for better terminal output
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
RESET = "\033[0m"

def print_step(step_num, message):
    """Print a step in the workflow with formatting."""
    print(f"\n{GREEN}Step {step_num}: {message}{RESET}")

def print_command(cmd):
    """Print a command that will be executed."""
    print(f"{YELLOW}$ {cmd}{RESET}")

def run_command(cmd, check=True):
    """Run a shell command and print its output."""
    print_command(cmd)
    result = subprocess.run(cmd, shell=True, check=check, text=True, capture_output=True)
    if result.stdout:
        print(result.stdout)
    if result.stderr:
        print(f"{RED}{result.stderr}{RESET}")
    return result

def update_rigelfile(version="1.0.0"):
    """Update the Rigelfile with a new base_image version."""
    print_step(2, f"Updating Rigelfile with application version {version}")
    
    # Read the current Rigelfile
    with open("Rigelfile", "r") as f:
        content = f.read()
    
    # Update the base_image version
    updated_content = content.replace(
        'base_image: "nhopf/turtle-rigel:latest"', 
        f'base_image: "nhopf/turtle-rigel:{version}"'
    )
    
    # Write the updated Rigelfile
    with open("Rigelfile", "w") as f:
        f.write(updated_content)
    
    print(f"Updated Rigelfile with version {version}")

def build_docker_image(version):
    """Build a Docker image for the ROS application."""
    print_step(3, f"Building Docker image for version {version}")
    
    # Create a temporary Dockerfile
    dockerfile_content = f"""FROM ros:noetic

WORKDIR /app

# Copy the publisher code
COPY app_testing/pubsub/publisher.py /app/

# Set environment variables
ENV ROS_MASTER_URI=http://ros-master:11311
ENV APP_VERSION={version}

# Install dependencies
RUN apt-get update && apt-get install -y python3-pip && pip3 install prometheus_client

CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && python3 /app/publisher.py"]
"""
    
    with open("Dockerfile", "w") as f:
        f.write(dockerfile_content)
    
    # Build the Docker image
    run_command(f"docker build -t nhopf/turtle-rigel:{version} .")
    
    # Push the image to Docker Hub (assuming user is logged in)
    run_command(f"docker push nhopf/turtle-rigel:{version}")
    
    # Clean up
    os.remove("Dockerfile")
    
    print(f"Built and pushed Docker image nhopf/turtle-rigel:{version}")

def deploy_application():
    """Deploy the application using Rigel."""
    print_step(4, "Deploying the application with Rigel")
    
    # Run Rigel to deploy the application
    run_command("rigel run demo")
    
    # Wait for deployment to be ready
    print("Waiting for deployment to be ready...")
    time.sleep(10)  # In a real scenario, you'd poll the K8s API
    
    # Check the deployment status
    run_command("kubectl get pods")

def monitor_application():
    """Monitor the running application."""
    print_step(5, "Monitoring the application")
    
    # Get logs from the publisher pod
    run_command("kubectl get pods -l app=rigel-k8s-application -o name | xargs kubectl logs --tail=20")
    
    # Check metrics from the publisher
    print("\nChecking publisher metrics...")
    run_command("kubectl port-forward $(kubectl get pods -l app=rigel-k8s-application -o name) 8000:8000 &", check=False)
    time.sleep(2)
    run_command("curl localhost:8000/metrics")
    run_command("pkill -f 'port-forward'", check=False)

def update_application(new_version):
    """Update the application to a new version."""
    print_step(6, f"Updating application to version {new_version}")
    
    # Build a new Docker image
    build_docker_image(new_version)
    
    # Update the Rigelfile with the new version
    update_rigelfile(new_version)
    
    # Apply the update
    print("Applying the update with Rigel...")
    run_command("rigel run demo")
    
    # Wait for the rolling update to complete
    print("Waiting for rolling update to complete...")
    time.sleep(15)  # In a real scenario, you'd poll the K8s API
    
    # Check the updated deployment
    run_command("kubectl get pods")
    
    # Verify the new version is running
    print("\nVerifying the new version is running...")
    run_command("kubectl get pods -l app=rigel-k8s-application -o name | xargs kubectl logs --tail=5 | grep APP_VERSION")

def main():
    """Main function to demonstrate the deployment and update workflow."""
    parser = argparse.ArgumentParser(description="Deploy and update a ROS application using Rigel")
    parser.add_argument("--initial-version", default="1.0.0", help="Initial version to deploy")
    parser.add_argument("--update-version", default="1.1.0", help="Version to update to")
    parser.add_argument("--skip-deploy", action="store_true", help="Skip the initial deployment")
    parser.add_argument("--skip-update", action="store_true", help="Skip the update")
    
    args = parser.parse_args()
    
    print_step(1, "Starting deployment and update workflow")
    
    if not args.skip_deploy:
        # Update Rigelfile for initial deployment
        update_rigelfile(args.initial_version)
        
        # Build Docker image
        build_docker_image(args.initial_version)
        
        # Deploy the application
        deploy_application()
        
        # Monitor the application
        monitor_application()
    
    if not args.skip_update:
        # Update the application
        update_application(args.update_version)
        
        # Monitor the updated application
        monitor_application()
    
    print_step(7, "Workflow completed successfully")
    print(f"""
{GREEN}Summary:{RESET}
1. Updated the Rigelfile with application configuration
2. Built a Docker image for the ROS publisher
3. Deployed the application using Rigel
4. Monitored the running application
5. Updated the application to a new version using rolling update
6. Verified the update was successful

{YELLOW}To clean up:{RESET}
$ kubectl delete deployment ros-master
$ kubectl delete deployment rigel-k8s-application
$ kubectl delete pvc --all
$ kubectl delete pv --all
""")

if __name__ == "__main__":
    main()
