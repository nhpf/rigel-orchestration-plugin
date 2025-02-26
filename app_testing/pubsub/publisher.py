#!/usr/bin/env python

import os
import socket

import rospy
from prometheus_client import Counter, Gauge, start_http_server
from std_msgs.msg import Int32

# Create metrics to track publishing activity
messages_published = Counter("messages_published_total", "Number of messages published to number_sequence topic")
current_number = Gauge("current_number", "The current number being published")
app_info = Gauge("app_info", "Application information", ["version"])

def publisher():
    start_http_server(8000)

    # Get version from environment variable or default to "unknown"
    version = os.environ.get("APP_VERSION", "unknown")
    app_info.labels(version=version).set(1)
    
    rospy.init_node("number_publisher", anonymous=True)
    pub = rospy.Publisher("/number_sequence", Int32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    number = 0
    hostname = socket.gethostname()
    ip_address = socket.gethostbyname(hostname)
    rospy.loginfo(f"Publisher IP: {ip_address}")
    rospy.loginfo(f"Publisher version: {version}")

    while not rospy.is_shutdown():
        number += 1
        rospy.loginfo(f"Publishing: {number}")
        pub.publish(number)
        current_number.set(number)
        messages_published.inc()
        rate.sleep()


if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
