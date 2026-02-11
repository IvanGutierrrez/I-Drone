FROM python:3.9-slim

WORKDIR /app

# Install dependencies
RUN pip install --no-cache-dir protobuf pyyaml

# Copy Client files
COPY Client/ /app/

# Keep container running
CMD ["tail", "-f", "/dev/null"]
