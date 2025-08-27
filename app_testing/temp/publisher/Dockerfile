FROM python:3.9-alpine
RUN pip install redis prometheus-client
COPY publisher.py .
CMD ["python", "publisher.py"]
