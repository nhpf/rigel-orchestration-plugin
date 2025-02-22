#!/usr/bin/env python

import socket
import time

import rospy
from prometheus_client import Counter, start_http_server
from std_msgs.msg import Int32

messages_received = Counter("messages_received_total", "Number of messages received.")


class NumberSubscriber:
    def __init__(self):
        start_http_server(8000)
        self.received_numbers = []
        self.last_received_time = time.time()
        self.expected_number = 1
        self.missed_numbers = []
        self.hostname = socket.gethostname()
        self.ip_address = socket.gethostbyname(self.hostname)
        rospy.loginfo(f"Subscriber IP: {self.ip_address}")
        rospy.Subscriber("/number_sequence", Int32, self.callback)

    def callback(self, msg):
        current_time = time.time()
        self.received_numbers.append(msg.data)
        rospy.loginfo(f"Received: {msg.data}")

        # Detect missed numbers
        if msg.data != self.expected_number:
            missed = list(range(self.expected_number, msg.data))
            self.missed_numbers.extend(missed)
            rospy.logwarn(f"Missed Numbers: {missed}")

        self.expected_number = msg.data + 1
        self.last_received_time = current_time
        messages_received.inc()

    def run(self):
        rospy.spin()
        # After shutdown, log the results
        rospy.loginfo(f"Total Numbers Received: {len(self.received_numbers)}")
        rospy.loginfo(f"Missed Numbers: {self.missed_numbers}")


if __name__ == "__main__":
    try:
        rospy.init_node("number_subscriber", anonymous=True)
        subscriber = NumberSubscriber()
        subscriber.run()
    except Exception as e:
        rospy.logerr(f"Unhandled Exception: {e}")
    except rospy.ROSInterruptException:
        pass
