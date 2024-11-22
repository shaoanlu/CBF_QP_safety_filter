# Use an official Python image with version 3.6+
FROM python:3.10-slim

# Set the working directory inside the container
WORKDIR /app

# Copy the application code into the container
COPY . /app

# Install required dependencies
RUN apt-get update && apt-get install -y \
    fontconfig \
    fonts-dejavu-core \
    x11-apps \
    libx11-6 \
    libxext6 \
    libsm6 \
    libxrender1 \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --no-cache-dir numpy==1.26.4 scipy==1.13.1 osqp==0.6.7 pygame==2.6.1

# Expose any required ports (if needed by pygame or other services)
EXPOSE 8000

# Set the default command to run the demo
CMD ["python3", "demo.py"]
