FROM python:3.11-slim

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    imagemagick \
    tk-dev \
    make \
    m4 \
    socat \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /commonroad

# Install Poetry and configure it
RUN pip install --upgrade pip setuptools wheel \
    && pip install poetry==1.8.3 \
    && poetry config virtualenvs.create false

# Clone and install CommonRoad-Search
RUN git clone https://gitlab.lrz.de/tum-cps/commonroad-search.git
WORKDIR /commonroad/commonroad-search
RUN poetry install --no-root --no-interaction && pip install jupyter ipywidgets

# Launch Jupyter by default
ENTRYPOINT bash -c "\
    jupyter notebook --ip 0.0.0.0 --no-browser --allow-root \
    --NotebookApp.token='' --NotebookApp.password='' --port=8888"
