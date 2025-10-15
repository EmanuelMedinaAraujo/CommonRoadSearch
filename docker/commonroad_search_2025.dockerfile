# syntax=docker/dockerfile:1
FROM python:3.11-slim

ENV DEBIAN_FRONTEND=noninteractive

# --- System dependencies ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget pkg-config \
    libboost-all-dev libeigen3-dev libomp-dev libssl-dev \
    libgl1-mesa-dev libglu1-mesa-dev libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /commonroad

# --- Core build tools ---
RUN pip install --no-cache-dir --upgrade pip setuptools wheel scikit-build-core cmake ninja pybind11 nanobind

# --- Basic Python stack ---
RUN pip install --no-cache-dir numpy scipy matplotlib jupyterlab notebook ipywidgets tqdm networkx shapely lxml

# --- Install CommonRoad packages (safe fallback sequence) ---

# Step 1: Pure Python packages
RUN pip install --no-cache-dir commonroad-io commonroad-vehicle-models

# Step 2: CLCS (C++ extension, smaller)
RUN pip install --no-cache-dir --use-pep517 commonroad-clcs || echo "CLCS build failed (non-fatal)"

# Step 3: Route planner (depends on CLCS)
RUN pip install --no-cache-dir commonroad-route-planner || echo "Route planner partially installed"

# Step 4: Drivability checker (fails on ARM / optional)
RUN pip install --no-cache-dir --use-pep517 commonroad-drivability-checker || echo "Drivability checker unavailable on ARM / skipping"

# Install Triangle fork compatible with Python 3.11+
RUN pip install --no-cache-dir git+https://github.com/drufat/triangle.git@master

# --- Configure Jupyter ---
RUN mkdir -p /root/.jupyter && echo "\
c.ServerApp.token = ''\n\
c.ServerApp.password = ''\n\
c.ServerApp.ip = '0.0.0.0'\n\
c.ServerApp.open_browser = False\n\
c.ServerApp.port = 8888\n" > /root/.jupyter/jupyter_server_config.py

EXPOSE 8888

CMD ["bash", "-c", "cd /commonroad/commonroad-search && jupyter lab --allow-root --ip=0.0.0.0 --port=8888"]