#!/usr/bin/env python3

import time
import math
import rospy
import actionlib

from prometheus_client import start_http_server, Counter, Histogram, Gauge

# Import the generated action message classes:
# Adjust the import path to match your ROS package name, e.g. `my_package.msg`
from my_package.msg import (
    PrimeFactorizationAction,
    PrimeFactorizationFeedback,
    PrimeFactorizationResult
)

# ---------------------------
# Prometheus Metrics
# ---------------------------
# Count how many goals (requests) have been received
GOALS_RECEIVED = Counter('prime_factor_server_goals_total', 'Total number of goals received')

# Count how many prime factors have been published as feedback
FACTORS_FOUND = Counter('prime_factor_server_factors_found_total', 'Count of prime factors found')

# Time how long each goal takes
GOAL_PROCESSING_TIME = Histogram('prime_factor_server_goal_duration_seconds',
                                 'Time spent factoring numbers in one goal')

# Track how many goals are currently being processed
ACTIVE_GOALS = Gauge('prime_factor_server_active_goals', 'Number of active goals being processed')

def prime_factors_slow(n):
    """
    Generator function that yields prime factors of n one-by-one,
    simulating a long-running task with a small sleep each time.
    """
    num = n
    # Handle negative numbers; factor only the absolute value
    num_abs = abs(num)

    # Factor out 2's
    while num_abs % 2 == 0 and not rospy.is_shutdown():
        yield 2
        num_abs //= 2
        time.sleep(1)  # simulate slow work

    # Factor out odd numbers
    factor = 3
    max_factor = int(math.sqrt(num_abs)) + 1
    while factor <= max_factor and num_abs > 1 and not rospy.is_shutdown():
        while num_abs % factor == 0:
            yield factor
            num_abs //= factor
            time.sleep(1)  # simulate slow work
        factor += 2
        max_factor = int(math.sqrt(num_abs)) + 1

    # If remainder is prime
    if num_abs > 1:
        yield num_abs

    # If original number was negative, let's signal that somehow (optional)
    # but typically the prime factors of negative are the same as positive
    # except for a sign factor (-1), which we won't do here.

def compute_factorization(numbers, as_server, feedback, result):
    """
    For each number in the 'numbers' list, factor it slowly,
    publishing feedback, and building the final result strings.
    """
    factor_strings = []
    for number in numbers:
        if as_server.is_preempt_requested():
            rospy.logwarn("Goal preempted before finishing.")
            as_server.set_preempted()
            return

        # Keep track of the factors for this specific number
        factors_for_this = []
        # We send feedback for each factor
        for f in prime_factors_slow(number):
            if as_server.is_preempt_requested():
                rospy.logwarn("Goal preempted mid-factorization.")
                as_server.set_preempted()
                return

            # Update counters, feedback
            FACTORS_FOUND.inc()
            feedback.current_number = number
            feedback.factor = f
            as_server.publish_feedback(feedback)
            factors_for_this.append(f)

        # Build a readable string "X = f1 x f2 x ..."
        if len(factors_for_this) == 0:
            # no prime factors found (e.g. input was 0 or 1)
            factor_strings.append(f"{number} = (none)")
        else:
            factor_str = " x ".join(str(x) for x in factors_for_this)
            factor_strings.append(f"{number} = {factor_str}")

    result.factorization = factor_strings

class PrimeFactorServer:
    def __init__(self):
        # Start Prometheus metric server on port 8000
        start_http_server(8000)
        rospy.loginfo("Prometheus metrics available on port 8000")

        # Create the action server
        self._as = actionlib.SimpleActionServer('prime_factor',
                                                PrimeFactorizationAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()
        rospy.loginfo("PrimeFactorServer started, action name: 'prime_factor'")

    def execute_cb(self, goal):
        GOALS_RECEIVED.inc()
        ACTIVE_GOALS.inc()

        start_time = time.time()
        feedback = PrimeFactorizationFeedback()
        result = PrimeFactorizationResult()

        rospy.loginfo(f"Received goal with {len(goal.input_numbers)} number(s).")

        # Factor each number, checking for preemption
        compute_factorization(goal.input_numbers, self._as, feedback, result)

        if not self._as.is_preempt_requested() and not rospy.is_shutdown():
            # If we weren't preempted or shutdown, set success
            self._as.set_succeeded(result)
            rospy.loginfo("Action succeeded. Final factorization:")
            for line in result.factorization:
                rospy.loginfo(f"  {line}")

        # Record how long it took
        duration = time.time() - start_time
        GOAL_PROCESSING_TIME.observe(duration)
        ACTIVE_GOALS.dec()

def main():
    rospy.init_node('prime_factor_server')
    server = PrimeFactorServer()
    rospy.spin()

if __name__ == '__main__':
    main()
