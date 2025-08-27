import redis
import time
import random
from prometheus_client import start_http_server, Counter, Gauge

# Metrics setup
messages_published = Counter('messages_published_total', 'Total published messages')
temperature_gauge = Gauge('temperature_celsius', 'Current temperature measurement')

r = redis.Redis(host='redis', port=6379)
start_http_server(8000)  # Expose metrics on port 8000

def generate_sensor_data():
    """Generate realistic temperature readings with some noise"""
    base_temp = 25.0
    noise = random.uniform(-0.5, 0.5)
    return round(base_temp + noise, 2)

while True:
    # Generate and publish sensor data at 500Hz (every 0.002s)
    timestamp = time.time()
    temperature = generate_sensor_data()

    message = {
        'timestamp': timestamp,
        'temperature': temperature,
        'sensor_id': 'sensor-001'
    }

    r.publish('sensor-data', str(message))
    messages_published.inc()
    temperature_gauge.set(temperature)

    time.sleep(0.002)  # 500 Hz
