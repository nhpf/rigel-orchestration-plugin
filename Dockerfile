# File: Dockerfile
FROM ros:noetic-ros-core

# 1. Install a small package to test communication (turtlesim).
RUN apt-get update && apt-get install -y \
    ros-noetic-turtlesim && \
    rm -rf /var/lib/apt/lists/*

# 2. Copy your readiness script (the one checking /tmp/ready).
COPY readiness_probe.sh /usr/local/bin/readiness_probe.sh
RUN chmod +x /usr/local/bin/readiness_probe.sh

# 3. By default, start roscore and a talker, then mark readiness
#    so your plugin can tell K8s that the container is "Ready."
CMD ["/bin/bash", "-c", "\
  source /opt/ros/noetic/setup.bash && \
  echo 'Starting roscore...' && \
  roscore & \
  sleep 5 && \
  touch /tmp/ready && \
  echo 'Launching simple talker...' && \
  rostopic pub /hello std_msgs/String 'Hello from K8s' -r 1 \
"]
