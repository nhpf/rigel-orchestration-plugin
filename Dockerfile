FROM ros:noetic-ros-core

# Install dependencies.
RUN rm -f /etc/apt/sources.list.d/ros*.list && \
    apt-get clean && apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Create default user 'rigeluser'.
ARG USERNAME=rigeluser
RUN groupadd $USERNAME
RUN useradd -ms /bin/bash -g $USERNAME $USERNAME

# Create ROS workspace folder
RUN mkdir -p /home/$USERNAME/ros_workspace/src

# Set user as ROS workspace owner
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME

# Copy this repository into the ROS workspace.
COPY . /home/rigeluser/ros_workspace/src/rigel-orchestration-plugin

# Copy bringup script.
COPY dockerfile_entrypoint.sh /home/rigeluser/robot-entrypoint.sh

# Copy readiness probe script.
COPY readiness_probe.sh /usr/local/bin/readiness_probe.sh
RUN chmod +x /usr/local/bin/readiness_probe.sh /home/rigeluser/robot-entrypoint.sh && \
    chown $USERNAME:$USERNAME /home/rigeluser/robot-entrypoint.sh

USER $USERNAME

# Compile ROS workspace.
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
    && cd /home/rigeluser/ros_workspace \
    && catkin_make "
    
# Launch ROS application.
CMD ["rostopic", "pub", "/hello", "std_msgs/String", "Hello from K8s", "-r", "1"]
ENTRYPOINT [ "/home/rigeluser/robot-entrypoint.sh" ]
