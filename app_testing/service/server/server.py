#!/usr/bin/env python3

import time
import rospy
from rospy_tutorials.srv import AddTwoInts, AddTwoIntsResponse

# Prometheus imports
from prometheus_client import start_http_server, Counter, Histogram

# Define Prometheus metrics
REQUEST_COUNTER = Counter(
    'arithmetic_server_requests_total',
    'Total number of requests received by the arithmetic server'
)

REQUEST_LATENCY = Histogram(
    'arithmetic_server_request_latency_seconds',
    'Time spent handling arithmetic requests'
)

def handle_add_two_ints(req):
    """
    Callback for handling the AddTwoInts service.
    Returns the sum of req.a and req.b.
    """
    start_time = time.time()
    REQUEST_COUNTER.inc()

    result = req.a + req.b
    rospy.loginfo(f"Received request: a={req.a}, b={req.b}; Responding with sum={result}")

    # Simulate some processing time or logic if needed
    time.sleep(0.01)

    latency = time.time() - start_time
    REQUEST_LATENCY.observe(latency)

    return AddTwoIntsResponse(result)

def arithmetic_server():
    """
    Initializes the arithmetic_server node and starts the 'add_two_ints' service.
    Also starts a Prometheus metrics endpoint on port 8000.
    """
    rospy.init_node('arithmetic_server')

    # Start Prometheus HTTP server on port 8000
    start_http_server(8000)
    rospy.loginfo("Prometheus metrics for arithmetic_server exposed on port 8000")

    service = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    rospy.loginfo("Arithmetic server is ready to add two integers.")
    rospy.spin()

if __name__ == "__main__":
    arithmetic_server()
