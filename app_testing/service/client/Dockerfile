FROM ros:noetic-ros-core

RUN apt-get update && apt-get install -y \
    python3 python3-pip ros-noetic-rospy-tutorials && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install prometheus_client

WORKDIR /app

COPY client.py /app/
RUN chmod +x client.py

# This client by default will call the service with two integers, but you can override
# the CMD at runtime or in docker-compose
CMD ["bash", "-c", "sleep 5 && python3 /app/client.py"]
