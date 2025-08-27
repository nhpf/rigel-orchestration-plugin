FROM python:3.9-alpine
RUN pip install redis prometheus-client
COPY subscriber.py .
CMD ["python", "subscriber.py"]
