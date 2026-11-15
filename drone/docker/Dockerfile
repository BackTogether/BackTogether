# Drone web app (hoverctl + drone.sdk), the lost-child tools and the patrols in containers.
# Build context: the drone/ folder (see compose.yaml). Targets:
#   app     web UI + HTTP API + simulator, and the vision tools that need no depth model
#           (drone/find, drone/land, drone/dock, drone.identify): bleak, ffmpeg, numpy, OpenCV
#   patrol  app + the patrols (bounce, dvd, clear, sweep, so also `drone/find --sweep`) with the
#           depth model baked in (works offline on the drone's Wi-Fi)
# Heavy layers come before the code, so editing code only rebuilds the last COPY.
# Photos of people and the identify models are never copied (.dockerignore): compose mounts them.

FROM python:3.11-slim AS base
RUN apt-get update \
    && apt-get install -y --no-install-recommends ffmpeg curl \
    && rm -rf /var/lib/apt/lists/*
# The venue Wi-Fi drops connections: pip's cache is kept between builds (never in the image), so
# a failed build goes on from the wheels it already has.
ENV PIP_RETRIES=10 PIP_DEFAULT_TIMEOUT=60
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install bleak numpy pillow opencv-contrib-python-headless
# The launchers (drone/find, drone/land, drone/dock, drone/patrol/live) start with
# `uv run [options] python ...` to fetch their dependencies. Here those are installed already,
# so `uv` only runs what follows its options and the launchers work unchanged.
COPY --chmod=755 <<'EOF' /usr/local/bin/uv
#!/bin/sh
while [ $# -gt 0 ] && [ "$1" != python ]; do shift; done
exec "$@"
EOF
WORKDIR /app
ENV PYTHONUNBUFFERED=1
RUN mkdir -p recordings

FROM base AS app
COPY . drone/
EXPOSE 8077
CMD ["python", "-m", "drone.sdk", "sim", "--bind", "0.0.0.0", "--http-port", "8077", "--no-browser", "--room", "4"]

FROM base AS patrol-deps
# CPU-only torch: the default PyPI wheel on Linux pulls ~2 GB of CUDA libraries.
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --index-url https://download.pytorch.org/whl/cpu torch
RUN --mount=type=cache,target=/root/.cache/pip pip install transformers av
ARG DEPTH_MODEL=depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf
ENV HF_HOME=/opt/hf
RUN python -c "from transformers import AutoImageProcessor as P, AutoModelForDepthEstimation as M; \
P.from_pretrained('${DEPTH_MODEL}'); M.from_pretrained('${DEPTH_MODEL}')"
ENV HF_HUB_OFFLINE=1

FROM patrol-deps AS patrol
COPY . drone/
CMD ["python", "-m", "drone.patrol.bounce", "--help"]
