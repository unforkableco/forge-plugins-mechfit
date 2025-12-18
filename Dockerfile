FROM python:3.11-slim

# Install system dependencies
# libfcl-dev: for collision checking (optional, python-fcl wheels might have it, but safer to have)
# libgl1/libglib2.0: for trimesh visual/rendering deps
RUN apt-get update && apt-get install -y \
    libfcl-dev \
    libgl1 \
    libglib2.0-0 \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Install Python dependencies
RUN pip install --no-cache-dir \
    fastapi \
    uvicorn \
    trimesh \
    numpy \
    scipy \
    requests \
    pydantic \
    python-fcl \
    networkx \
    matplotlib \
    pytest \
    httpx

# Copy source code
COPY src /app/src
COPY tests /app/tests

# Set environment
ENV PYTHONPATH=/app

# Expose port
EXPOSE 8080

# Run server
CMD ["uvicorn", "src.server:app", "--host", "0.0.0.0", "--port", "8080"]
