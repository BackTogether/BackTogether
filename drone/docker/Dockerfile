# Drone web app (hoverctl + drone.sdk) and the bounce patrol in containers.
# Build context: the drone/ folder (see compose.yaml). Targets:
#   app     web UI + HTTP API + simulator: stdlib, bleak, ffmpeg
#   patrol  bounce patrol with the depth model baked in (works offline on the drone's Wi-Fi)
# Heavy layers come before the code, so editing code only rebuilds the last COPY.

FROM python:3.11-slim AS base
RUN apt-get update \
    && apt-get install -y --no-install-recommends ffmpeg \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --no-cache-dir bleak
WORKDIR /app
ENV PYTHONUNBUFFERED=1
RUN mkdir -p recordings

FROM base AS app
COPY . drone/
EXPOSE 8077
CMD ["python", "-m", "drone.sdk", "sim", "--bind", "0.0.0.0", "--http-port", "8077", "--no-browser", "--room", "4"]

FROM base AS patrol-deps
# CPU-only torch: the default PyPI wheel on Linux pulls ~2 GB of CUDA libraries.
RUN pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu torch \
    && pip install --no-cache-dir numpy pillow transformers
ARG DEPTH_MODEL=depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf
ENV HF_HOME=/opt/hf
RUN python -c "from transformers import AutoImageProcessor as P, AutoModelForDepthEstimation as M; \
P.from_pretrained('${DEPTH_MODEL}'); M.from_pretrained('${DEPTH_MODEL}')"
ENV HF_HUB_OFFLINE=1

FROM patrol-deps AS patrol
COPY . drone/
CMD ["python", "-m", "drone.patrol.bounce", "--help"]
