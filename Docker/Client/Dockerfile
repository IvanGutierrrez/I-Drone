FROM python:3.9-slim

WORKDIR /app

# Install protobuf
RUN pip install --no-cache-dir protobuf

# Keep container running
CMD ["tail", "-f", "/dev/null"]
