# TEKNOFEST ROV - BlueOS Docker Container
FROM python:3.9-slim-buster

# Sistem paketleri
RUN apt-get update && apt-get install -y \
    python3-tk \
    python3-pil \
    python3-matplotlib \
    python3-numpy \
    python3-scipy \
    libopenblas-dev \
    libatlas-base-dev \
    build-essential \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Working directory
WORKDIR /app

# Python dependencies
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# App files
COPY . .

# Make scripts executable
RUN chmod +x start_web_gui.sh setup_raspberry_pi.sh

# Create necessary directories
RUN mkdir -p templates static logs config

# Environment
ENV PYTHONPATH="/usr/lib/python3/dist-packages"
ENV FLASK_APP=web_gui.py
ENV FLASK_ENV=production

# Port expose
EXPOSE 5000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:5000/api/status || exit 1

# Default command
CMD ["python3", "web_gui.py"] 