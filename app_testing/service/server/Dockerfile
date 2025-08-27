# Use a base ROS image that includes the core runtime and Python
FROM ros:noetic-ros-core

RUN apt-get update && apt-get install -y \
    python3 python3-pip ros-noetic-rospy-tutorials && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install prometheus_client

# Create a working directory
WORKDIR /app

# Copy the server script into the container
COPY server.py /app/

# Make it executable
RUN chmod +x server.py

# We'll just run the server node, but note we need a running roscore somewhere else
CMD ["bash", "-c", "sleep 5 && python3 /app/server.py"]
