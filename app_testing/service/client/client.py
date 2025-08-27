#!/usr/bin/env python3

import os
import random
import time
import rospy
from rospy_tutorials.srv import AddTwoInts

# Prometheus imports
from prometheus_client import start_http_server, Counter, Gauge, Histogram

# Metrics: how many requests the client sent, how many succeeded, last sum value, etc.
REQUEST_COUNTER = Counter(
    'arithmetic_client_requests_total',
    'Total number of requests sent by the arithmetic client'
)

SUCCESS_COUNTER = Counter(
    'arithmetic_client_success_total',
    'Number of successful responses received by the arithmetic client'
)

REQUEST_LATENCY = Histogram(
    'arithmetic_client_request_latency_seconds',
    'Time spent waiting for the service to return'
)

LAST_SUM_GAUGE = Gauge(
    'arithmetic_client_last_sum',
    'Tracks the most recent sum value returned by the server'
)

def call_add_two_ints(a, b):
    """
    Client utility to request the add_two_ints service.
    """
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        start_time = time.time()
        response = add_two_ints(a, b)
        latency = time.time() - start_time
        REQUEST_LATENCY.observe(latency)
        return response.sum
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

if __name__ == "__main__":
    rospy.init_node('arithmetic_client')

    # Start Prometheus metrics on port 8001
    start_http_server(8001)
    rospy.loginfo("Prometheus metrics for arithmetic_client exposed on port 8001")

    log_file_path = "/persistent_volume/arithmetic_logs.txt"
    os.makedirs(os.path.dirname(log_file_path), exist_ok=True)

    rospy.loginfo("Continuous random-addition client started.")

    while not rospy.is_shutdown():
        # Generate random integers
        a = random.randint(0, 100)
        b = random.randint(0, 100)

        # Increment the total request counter
        REQUEST_COUNTER.inc()

        # Call the service
        result = call_add_two_ints(a, b)
        if result is None:
            rospy.logerr("No valid result returned from the service.")
        else:
            # If success, increment success counter, record last sum
            SUCCESS_COUNTER.inc()
            LAST_SUM_GAUGE.set(result)

            # Log request/response to persistent volume
            with open(log_file_path, 'a') as log_file:
                log_entry = f"Request: a={a}, b={b} | Response: sum={result}\n"
                log_file.write(log_entry)

            rospy.loginfo(f"Result of {a} + {b} = {result}")

        # Sleep for 1 second
        time.sleep(1)
