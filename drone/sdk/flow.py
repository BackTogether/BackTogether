"""Yaw measured from the live video (optical flow), so the heading does not rest on the stick
model alone. The X1 never reports its attitude or yaw.

Method: the decoder's ffmpeg also writes a 320x180 grayscale copy of every frame. Every
FRAME_GAP frames (0.2 s), each of STRIPS horizontal strips is phase-correlated with the same strip
FRAME_GAP frames earlier (FFT, sub-pixel peak). A yaw turn shifts the image sideways, and with
the camera tilted the shift depends on the row (pinhole rotational flow, x right, y down,
normalised): seen rotation m(y) = -atan(dx / f) = w * (cos(tilt) * (1 + x^2) + y * sin(tilt)).
A straight-line fit m = A + B*y over the strips gives (A / (1 + x^2), B) = w (cos t, sin t), so
w = its length, signed by the side of the gimbal's travel the vector lies on: no tilt needed.
(Projecting on the open-loop tilt estimate instead read 13 % low on the spin flight, where the
estimate said -30 deg and the rows showed a level camera.) Translation also shifts the image
(more for near scenery), so the estimator ignores steps while the drone flies sideways, moves
the camera or flies fast.

Checked against exact 360-degree spins (camera.py): one revolution integrates to 360 deg at
sticks 100 / 80 / 60, and steady stick-60 turns on two other flights (camera -32 and -50 deg)
read within +1..+8 % of the spin rate.

Confidence comes from the phase-correlation peaks (low on blank or blurred images) and the
straight-line fit (scattered strips: something else moved). Measured on our recordings: hover
peaks 0.6-0.9, steady turns 0.12-0.3, the drone handled or landing 0.05.

numpy is optional: without it make_video_yaw() returns None and the heading uses the sticks.
Enable it with `uv run --with numpy ...`. CPU: ~0.6 ms of numpy per step (5 per second) plus a
scale in the decoder's ffmpeg.
"""

import importlib.util
import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from .camera import CAMERA_WIDTH_PX, FLOW_FOCAL_PX
from .tilt import GIMBAL_BOTTOM_DEG, GIMBAL_TOP_DEG

FLOW_WIDTH = 320
FLOW_HEIGHT = 180
FOCAL_PX = FLOW_FOCAL_PX * FLOW_WIDTH / CAMERA_WIDTH_PX
FRAME_GAP = 6  # adjacent frames move < 1 px in a slow turn: compare frames 0.2 s apart
MAX_STEP_S = 0.6  # a longer gap (video dropout) starts over instead of measuring across it
STRIPS = 6
STRIP_HEIGHT = FLOW_HEIGHT // STRIPS
SPECTRUM_FLOOR = 1e-6  # keeps the phase-only normalisation finite
PEAK_FLOOR = 0.07  # median strip peak at or below: confidence 0
PEAK_GOOD = 0.15  # at or above: confidence 1
MAX_RESIDUAL_DEG = 0.5  # strips this far off the straight-line fit: not a rotation
# The sign of the turn: which side of the middle of the gimbal's travel (105 deg < 180) the
# fitted direction lies on
MID_TILT_RAD = math.radians((GIMBAL_TOP_DEG + GIMBAL_BOTTOM_DEG) / 2)
NO_NUMPY_NOTE = "numpy not installed: heading from the sticks only (uv run --with numpy ...)"


@dataclass(frozen=True)
class YawStep:
    """The yaw between two frames, measured from the video."""

    start: float  # unix time of the earlier frame (when it was decoded)
    end: float  # unix time of the later frame
    yaw_deg: float  # + = turned right
    confidence: float  # 0..1


def make_video_yaw() -> "VideoYaw | None":
    """A VideoYaw, or None when numpy is missing (the heading then uses the sticks only)."""
    if importlib.util.find_spec("numpy") is None:
        return None
    return VideoYaw()


class VideoYaw:
    """Feed it the gray frames (`attach` does it from the decoder); it returns a YawStep every
    FRAME_GAP frames. Not thread-safe: one feeder (the decoder's reader thread)."""

    def __init__(self) -> None:
        import numpy  # optional dependency: see make_video_yaw

        self._np = numpy
        columns = (numpy.arange(FLOW_WIDTH) + 0.5 - FLOW_WIDTH / 2) / FOCAL_PX
        column_weights = numpy.hanning(FLOW_WIDTH)
        # Off-centre columns move (1 + x^2) times more in a turn; the window weights them
        self._width_gain = 1 + float(
            numpy.sum(column_weights * columns**2) / numpy.sum(column_weights)
        )
        self._window = numpy.outer(numpy.hanning(STRIP_HEIGHT), column_weights)
        self._rows = (numpy.arange(STRIPS) + 0.5) * STRIP_HEIGHT / FOCAL_PX - (
            FLOW_HEIGHT / 2 / FOCAL_PX
        )
        self._reference: Any = None  # strip spectra of the reference frame
        self._reference_time = 0.0
        self._frames_since = 0

    def attach(self, decoder: Any, on_step: Callable[[YawStep], None]) -> bool:
        """Ask `decoder` (an MjpegDecoder) for gray frames and call `on_step(step)` with every
        step, on its reader thread. False if the decoder cannot (Windows)."""

        def on_frame(frame: bytes, at: float) -> None:
            step = self.feed(frame, at)
            if step is not None:
                on_step(step)

        return decoder.set_gray_output(FLOW_WIDTH, FLOW_HEIGHT, on_frame)

    def feed(self, frame: bytes, at: float) -> YawStep | None:
        """One FLOW_WIDTH x FLOW_HEIGHT gray frame decoded at unix time `at`."""
        self._frames_since += 1
        stale = self._reference is None or at - self._reference_time > MAX_STEP_S
        if not stale and self._frames_since < FRAME_GAP:
            return None
        spectra = self._spectra(frame)
        step = None if stale else self._step(spectra, at)
        self._reference, self._reference_time, self._frames_since = spectra, at, 0
        return step

    def _spectra(self, frame: bytes) -> Any:
        np = self._np
        image = np.frombuffer(frame, np.uint8).reshape(FLOW_HEIGHT, FLOW_WIDTH)
        strips = image[: STRIPS * STRIP_HEIGHT].astype(np.float32)
        strips = strips.reshape(STRIPS, STRIP_HEIGHT, FLOW_WIDTH)
        strips = (strips - strips.mean(axis=(1, 2), keepdims=True)) * self._window
        return np.fft.rfft2(strips)

    def _step(self, spectra: Any, at: float) -> YawStep:
        np = self._np
        shifts, peaks = self._shifts(self._reference, spectra)
        seen = -np.arctan(shifts / FOCAL_PX)  # rad per strip, + = turned right
        slope, offset = np.polyfit(self._rows, seen, 1)
        residual = float(np.sqrt(np.mean((seen - (offset + slope * self._rows)) ** 2)))
        along, across = offset / self._width_gain, slope  # w cos(tilt), w sin(tilt)
        side = along * math.cos(MID_TILT_RAD) + across * math.sin(MID_TILT_RAD)
        yaw = math.degrees(math.copysign(math.hypot(along, across), side))
        peak = float(np.median(peaks))
        confidence = min(1.0, max(0.0, (peak - PEAK_FLOOR) / (PEAK_GOOD - PEAK_FLOOR)))
        if math.degrees(residual) > MAX_RESIDUAL_DEG:
            confidence = 0.0
        return YawStep(self._reference_time, at, yaw, confidence)

    def _shifts(self, before: Any, after: Any) -> tuple[Any, Any]:
        """Horizontal shift (px, sub-pixel) and correlation peak of each strip."""
        np = self._np
        cross = after * np.conj(before)
        cross /= np.abs(cross) + SPECTRUM_FLOOR
        surface = np.fft.irfft2(cross, s=(STRIP_HEIGHT, FLOW_WIDTH), axes=(1, 2))
        flat = surface.reshape(STRIPS, -1)
        row, column = np.divmod(flat.argmax(axis=1), FLOW_WIDTH)
        strip = np.arange(STRIPS)
        peak = surface[strip, row, column]
        left = surface[strip, row, (column - 1) % FLOW_WIDTH]
        right = surface[strip, row, (column + 1) % FLOW_WIDTH]
        curvature = left - 2 * peak + right
        safe = np.where(curvature == 0, 1.0, curvature)
        fraction = np.where(curvature == 0, 0.0, 0.5 * (left - right) / safe)
        shift = column + fraction
        return np.where(shift > FLOW_WIDTH / 2, shift - FLOW_WIDTH, shift), peak
