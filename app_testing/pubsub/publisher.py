#!/usr/bin/env python

import socket

import rospy
from prometheus_client import Counter, start_http_server
from std_msgs.msg import Int32

# Create a metric to track time spent and requests made.
messages_published = Counter("messages_published_total", "Number of requests for number_sequence.")


def publisher():
    start_http_server(8000)

    rospy.init_node("number_publisher", anonymous=True)
    pub = rospy.Publisher("/number_sequence", Int32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    number = 0
    hostname = socket.gethostname()
    ip_address = socket.gethostbyname(hostname)
    rospy.loginfo(f"Publisher IP: {ip_address}")  # Python2 does not allow f-strings

    while not rospy.is_shutdown():
        number += 1
        rospy.loginfo(f"Publishing: {number}")  # Python2 does not allow f-strings
        pub.publish(number)
        # Publish current unix timestamp miliseconds
        # pub.publish(int(datetime.now().timestamp() * 1000))
        messages_published.inc()
        rate.sleep()


if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
