import redis
import time
import statistics
from prometheus_client import start_http_server, Counter, Gauge, Summary

# Metrics setup
messages_received = Counter('messages_received_total', 'Total received messages')
temp_min = Gauge('temperature_min', 'Minimum temperature')
temp_max = Gauge('temperature_max', 'Maximum temperature')
temp_avg = Gauge('temperature_avg', 'Average temperature')
temp_std = Gauge('temperature_std', 'Temperature standard deviation')
processing_time = Summary('message_processing_seconds', 'Time spent processing messages')

r = redis.Redis(host='redis', port=6379)
start_http_server(8000)  # Expose metrics on port 8000

window_size = 500  # 1 second window at 500Hz
readings = []

@processing_time.time()
def process_message(message):
    global readings
    readings.append(float(message['temperature']))

    # Keep only the last second of data
    if len(readings) > window_size:
        readings = readings[-window_size:]

    # Update metrics
    if readings:
        temp_min.set(min(readings))
        temp_max.set(max(readings))
        temp_avg.set(statistics.mean(readings))
        if len(readings) >= 2:
            temp_std.set(statistics.stdev(readings))

pubsub = r.pubsub()
pubsub.subscribe('sensor-data')

while True:
    message = pubsub.get_message()
    if message and message['type'] == 'message':
        messages_received.inc()
        try:
            data = eval(message['data'])
            process_message(data)
        except:
            pass
    time.sleep(0.001)
