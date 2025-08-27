#!/usr/bin/env python3

import os
import random
import time
import rospy
import actionlib

from prometheus_client import start_http_server, Counter

from my_package.msg import (
    PrimeFactorizationAction,
    PrimeFactorizationGoal,
    PrimeFactorizationFeedback,
    PrimeFactorizationResult
)

# Prometheus counters
GOALS_SENT = Counter('prime_factor_client_goals_total', 'Number of prime factor goals sent')
FACTORS_RECEIVED = Counter('prime_factor_client_factors_received_total', 'Number of factors received in feedback')
GOALS_SUCCEEDED = Counter('prime_factor_client_goals_succeeded_total', 'Number of goals that succeeded')
GOALS_PREEMPTED = Counter('prime_factor_client_goals_preempted_total', 'Number of goals that were preempted')

LOG_FILE_PATH = "/persistent_volume/prime_factors.log"

def feedback_cb(feedback):
    """
    Called whenever the server publishes feedback: we log each prime factor found.
    """
    rospy.loginfo(f"[Feedback] Current number: {feedback.current_number}, factor: {feedback.factor}")
    FACTORS_RECEIVED.inc()
    with open(LOG_FILE_PATH, 'a') as f:
        f.write(f"FEEDBACK => number={feedback.current_number}, factor={feedback.factor}\n")

def done_cb(state, result):
    """
    Called when the server finishes or the action is canceled.
    """
    import actionlib  # for GoalStatus constants
    if state == actionlib.GoalStatus.SUCCEEDED:
        GOALS_SUCCEEDED.inc()
        rospy.loginfo("[Result] Succeeded. Final factorization:")
        for line in result.factorization:
            rospy.loginfo(f"  {line}")
            with open(LOG_FILE_PATH, 'a') as f:
                f.write(f"RESULT => {line}\n")
    elif state == actionlib.GoalStatus.PREEMPTED:
        GOALS_PREEMPTED.inc()
        rospy.logwarn("[Result] Goal preempted by client or server.")
        with open(LOG_FILE_PATH, 'a') as f:
            f.write("RESULT => preempted\n")
    else:
        rospy.logwarn(f"[Result] Goal ended with state={state}.")
        with open(LOG_FILE_PATH, 'a') as f:
            f.write(f"RESULT => ended with state={state}\n")

def active_cb():
    rospy.loginfo("Goal just went active. The server is factoring numbers.")
    with open(LOG_FILE_PATH, 'a') as f:
        f.write("STATUS => goal active\n")

def main():
    rospy.init_node("prime_factor_client")

    # Start Prometheus on port 8001
    start_http_server(8001)
    rospy.loginfo("Prometheus metrics for prime_factor_client on port 8001")

    os.makedirs(os.path.dirname(LOG_FILE_PATH), exist_ok=True)

    # Create action client and wait for server
    client = actionlib.SimpleActionClient('prime_factor', PrimeFactorizationAction)
    rospy.loginfo("Waiting for prime_factor action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to prime_factor action server.")

    # Continuous loop: generate random integers and send them to the server
    rate = rospy.Rate(0.1)  # e.g. one goal every 10 seconds (0.1 Hz)
    while not rospy.is_shutdown():
        # Generate random integers (e.g. 2-5 random numbers in the range [50..250])
        num_count = random.randint(2, 5)
        numbers = [random.randint(50, 250) for _ in range(num_count)]

        rospy.loginfo(f"Sending new goal with {numbers}...")
        with open(LOG_FILE_PATH, 'a') as f:
            f.write(f"\nGOAL => {numbers}\n")

        GOALS_SENT.inc()

        # Populate goal
        goal = PrimeFactorizationGoal()
        goal.input_numbers = numbers

        # Send goal (callbacks handle feedback and done)
        client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)

        # Optionally wait for result, or just let them run concurrently.
        # For a continuous scenario, you might just wait some time or check isDone.
        # We'll demonstrate waiting a certain time before sending next goal:
        # e.g. If you want to see partial overlap, you can skip waiting for result
        # or you can do client.wait_for_result(rospy.Duration(20)) etc.

        # Let's do a short wait here so we don't spam the server too aggressively.
        rate.sleep()

if __name__ == "__main__":
    main()
