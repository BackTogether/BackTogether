"""Distance to the nearest obstacle in front of the drone, from its single camera.

1. Metric depth per pixel with Depth Anything V2 Metric Indoor Small (Apache-2.0, CPU).
2. Scale check with geometry: with the camera tilted down by `pitch` at `height` above a flat
   floor, floor pixels have a known depth. The median ratio expected/model on the bottom band
   of the image corrects the model's scale (typically off by ~10 %).
3. Every pixel becomes a 3D point (pinhole camera, `hfov`). Points higher than OBSTACLE_MIN_M
   above the floor inside a corridor of +-CORRIDOR_HALF_WIDTH_M in front are obstacles; the
   front distance is a low percentile of their horizontal distance.

Run from the repo root, e.g. on a phone-screen recording (drone.telemetry.record):
  uv run --no-project --with numpy --with pillow --with torch --with transformers --with av \
      python -m drone.patrol.front_distance --video <video.mkv> --crop-preview --height 1.8
or live from the hoverctl web app (altitude from its odometry):
  ... python -m drone.patrol.front_distance --web http://127.0.0.1:8077 --takeoff-height 1.0
"""

import argparse
import io
import os
import threading
import time
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image

MODEL_ID = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"
PATCH = 14  # the model needs sides that are multiples of 14
DEFAULT_INPUT_WIDTH = 392
DEFAULT_PITCH_DEG = 45.0
DEFAULT_HFOV_DEG = 80.0  # unknown for the X1 preview: calibrate (see the patrol plan, T1)
FLOOR_BAND = 0.25  # bottom fraction of the image assumed to be floor for the scale check
MIN_RAY_DOWN = 0.05  # skip rays that barely point down (floor depth explodes)
OBSTACLE_MIN_M = 0.3  # higher than this above the floor = obstacle
CORRIDOR_HALF_WIDTH_M = 0.5
FRONT_PERCENTILE = 5
MIN_OBSTACLE_FRACTION = 0.005  # of corridor pixels; fewer = noise, front is clear
# Phone screen recordings: the camera preview is this vertical band of the app screen.
PREVIEW_TOP, PREVIEW_BOTTOM = 0.14, 0.395
NOT_FLYING_WAIT_S = 1.0


@dataclass(frozen=True)
class FrontReading:
    front_m: float | None  # None = nothing in front within view
    front_raw_m: float | None  # same, with the model's own scale
    floor_scale: float  # expected / model depth on the floor band (1.0 = model agrees)
    visible_m: float | None  # floor distance at the top of the image (None = sees horizon)


class DepthModel:
    """Metric depth in metres, on CPU. Heavy imports stay here."""

    def __init__(self, input_width: int = DEFAULT_INPUT_WIDTH) -> None:
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation

        torch.set_num_threads(os.cpu_count() or 1)
        self._torch = torch
        self.input_width = input_width
        self.processor = AutoImageProcessor.from_pretrained(MODEL_ID)
        self.model = AutoModelForDepthEstimation.from_pretrained(MODEL_ID).eval()

    def predict(self, image: Image.Image) -> np.ndarray:
        width = _multiple_of_patch(self.input_width)
        height = _multiple_of_patch(round(image.height * self.input_width / image.width))
        resized = image.convert("RGB").resize((width, height), Image.BILINEAR)
        inputs = self.processor(images=resized, return_tensors="pt", do_resize=False)
        with self._torch.inference_mode():
            depth = self.model(**inputs).predicted_depth
        return depth[0].numpy()


def _multiple_of_patch(value: int) -> int:
    return max(PATCH, round(value / PATCH) * PATCH)


def camera_rays(width: int, height: int, hfov_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """Normalised image coordinates (x right, y down) of every pixel for a pinhole camera."""
    focal = (width / 2) / np.tan(np.radians(hfov_deg) / 2)
    xs = (np.arange(width) + 0.5 - width / 2) / focal
    ys = (np.arange(height) + 0.5 - height / 2) / focal
    return np.meshgrid(xs, ys)


def measure_front(
    depth: np.ndarray, height_m: float, pitch_deg: float, hfov_deg: float
) -> FrontReading:
    """Front obstacle distance from a metric depth map; camera tilted down by pitch_deg."""
    rows, cols = depth.shape
    x, y = camera_rays(cols, rows, hfov_deg)
    pitch = np.radians(pitch_deg)
    ray_down = np.sin(pitch) + np.cos(pitch) * y  # downward component of each ray (per unit z)

    floor_rows = slice(int(rows * (1 - FLOOR_BAND)), rows)
    usable = ray_down[floor_rows] > MIN_RAY_DOWN
    expected = height_m / ray_down[floor_rows][usable]
    model = depth[floor_rows][usable]
    floor_scale = float(np.median(expected / np.maximum(model, 1e-3))) if usable.any() else 1.0

    raw = _front_from_depth(depth, x, y, height_m, pitch)
    scaled = _front_from_depth(depth * floor_scale, x, y, height_m, pitch)
    return FrontReading(scaled, raw, floor_scale, _floor_forward(height_m, pitch, y[0, 0]))


def _floor_forward(height_m: float, pitch: float, y: float) -> float | None:
    """Horizontal distance where the ray of image row `y` meets the floor (None: it doesn't)."""
    down = np.sin(pitch) + np.cos(pitch) * y
    if down <= MIN_RAY_DOWN:
        return None
    return float(height_m * (np.cos(pitch) - np.sin(pitch) * y) / down)


def _front_from_depth(
    depth: np.ndarray, x: np.ndarray, y: np.ndarray, height_m: float, pitch: float
) -> float | None:
    forward = depth * (np.cos(pitch) - np.sin(pitch) * y)
    right = depth * x
    above_floor = height_m - depth * (np.sin(pitch) + np.cos(pitch) * y)
    corridor = (np.abs(right) < CORRIDOR_HALF_WIDTH_M) & (forward > 0)
    obstacle = corridor & (above_floor > OBSTACLE_MIN_M)
    if obstacle.sum() < MIN_OBSTACLE_FRACTION * max(corridor.sum(), 1):
        return None
    return float(np.percentile(forward[obstacle], FRONT_PERCENTILE))


def format_reading(reading: FrontReading) -> str:
    def metres(value: float | None) -> str:
        return "clear" if value is None else f"{value:5.2f} m"

    visible = "horizon" if reading.visible_m is None else f"{reading.visible_m:.1f} m"
    return (
        f"front={metres(reading.front_m)}  (model {metres(reading.front_raw_m)},"
        f" floor scale x{reading.floor_scale:.2f}, sees up to {visible})"
    )


class LatestFrame:
    """Keeps only the newest JPEG of a live stream, so a slow model never lags behind."""

    def __init__(self, frames: Iterator[bytes]) -> None:
        self._jpeg: bytes | None = None
        self._count = 0
        self._lock = threading.Lock()
        threading.Thread(target=self._run, args=(frames,), daemon=True).start()

    def _run(self, frames: Iterator[bytes]) -> None:
        for jpeg in frames:
            with self._lock:
                self._jpeg, self._count = jpeg, self._count + 1

    def wait_new(self, seen: int) -> tuple[int, Image.Image]:
        while True:
            with self._lock:
                if self._count > seen and self._jpeg is not None:
                    return self._count, Image.open(io.BytesIO(self._jpeg))
            time.sleep(0.01)


def video_frames(path: Path, crop_preview: bool, every: int) -> Iterator[tuple[float, Image.Image]]:
    """(video time, frame) from a recorded file, every Nth frame."""
    import av

    with av.open(str(path)) as container:
        stream = container.streams.video[0]
        for index, frame in enumerate(container.decode(stream)):
            if index % every:
                continue
            image = frame.to_image()
            if crop_preview:
                top, bottom = int(image.height * PREVIEW_TOP), int(image.height * PREVIEW_BOTTOM)
                image = image.crop((0, top, image.width, bottom))
            yield float(frame.pts * stream.time_base), image


def run_video(args: argparse.Namespace, model: DepthModel) -> None:
    for t, image in video_frames(args.video, args.crop_preview, args.every):
        started = time.monotonic()
        reading = measure_front(model.predict(image), args.height, args.pitch, args.hfov)
        rate = 1 / (time.monotonic() - started)
        print(f"t={t:6.1f}s  {format_reading(reading)}  h={args.height:.2f} m  {rate:.1f} fps")


def run_live(args: argparse.Namespace, model: DepthModel) -> None:
    from drone.patrol.hoverweb import HoverWeb

    web = HoverWeb(args.web)
    latest = LatestFrame(web.frames())
    seen, started = 0, time.monotonic()
    while True:
        seen, image = latest.wait_new(seen)
        state = web.state()
        height = (state.altitude_m or 0.0) + args.takeoff_height if state.in_flight else None
        if height is None:
            print(f"{state.flight_state}: not flying, no height for the geometry", flush=True)
            time.sleep(NOT_FLYING_WAIT_S)
            continue
        tick = time.monotonic()
        reading = measure_front(model.predict(image), height, args.pitch, args.hfov)
        rate = 1 / (time.monotonic() - tick)
        t = time.monotonic() - started
        print(f"t={t:6.1f}s  {format_reading(reading)}  h={height:.2f} m  {rate:.1f} fps")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--video", type=Path, help="recorded video file")
    source.add_argument("--web", help="hoverctl web app URL, e.g. http://127.0.0.1:8077")
    parser.add_argument("--crop-preview", action="store_true", help="phone-screen recording")
    parser.add_argument("--every", type=int, default=10, help="video: use every Nth frame")
    parser.add_argument("--height", type=float, default=1.8, help="video: camera height (m)")
    parser.add_argument("--takeoff-height", type=float, default=1.0, help="live: m above floor")
    parser.add_argument("--pitch", type=float, default=DEFAULT_PITCH_DEG, help="camera down, deg")
    parser.add_argument("--hfov", type=float, default=DEFAULT_HFOV_DEG, help="horizontal FOV")
    parser.add_argument("--input-width", type=int, default=DEFAULT_INPUT_WIDTH)
    args = parser.parse_args()

    print(f"Loading {MODEL_ID} ...", flush=True)
    model = DepthModel(args.input_width)
    try:
        if args.video:
            run_video(args, model)
        else:
            run_live(args, model)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
