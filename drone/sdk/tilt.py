"""Camera tilt estimate. The X1 never reports its gimbal angle, so we integrate the gimbal stick
we actually send (after the ramp, cap and interlock) with a model measured on our drone.

Calibration (2026-11-14 test: stick log recordings/sticks-2026.jsonl + video motion analysis;
how to redo it: drone/sdk/README.md, "Camera tilt"):
- bottom stop -90 deg (camera straight down, confirmed visually); top stop +15 deg (+-4) above
  level (a frame at the top stop, camera on the floor: vanishing point of two vertical column
  edges vs the horizon, with the intrinsics in camera.py): about 105 deg of travel;
- stop to stop: 10.0 s at gimbal stick 100 (ramp included), 32 s at stick 40, same up and down;
- fit: rate = 0.124 deg/s per stick % above a 13.5 % dead zone.

At start the tilt is unknown: the estimator keeps the range of tilts that fits the motion so
far, [low, high], both clamped at the stops. Holding the stick toward a stop for the full travel
squeezes the range onto that stop: the estimate is then "anchored" and exact up to model error,
which every later push into a stop removes again.
"""

import math
import threading

from drone.hoverctl.protocol import Axes
from drone.hoverctl.sticks import RAMP_START, RAMP_STEP, TICK_S

from .telemetry import Telemetry, Tilt

GIMBAL_TOP_DEG = 15.0
GIMBAL_BOTTOM_DEG = -90.0
GIMBAL_RATE_DEG_S_PER_PCT = 0.124
GIMBAL_DEADZONE_PCT = 13.5
ANCHORED_WITHIN_DEG = 0.5  # range narrower than this counts as known


def gimbal_rate(stick: float) -> float:
    """Tilt rate in deg/s for a gimbal stick value (-100..100, + = camera up)."""
    magnitude = max(0.0, abs(stick) - GIMBAL_DEADZONE_PCT)
    return math.copysign(GIMBAL_RATE_DEG_S_PER_PCT * magnitude, stick)


def hold_time(degrees: float, stick_cap: float) -> float:
    """Seconds of full gimbal input that move the camera `degrees` at speed cap `stick_cap`
    (stick %), stepping the sender's real ramp (RAMP_START, then +RAMP_STEP per tick).
    Full sweep: 10.2 s at cap 100, 32 s at cap 40, like the measurement."""
    if gimbal_rate(stick_cap) <= 0:
        raise ValueError(
            f"speed {stick_cap:.0f} is inside the gimbal dead zone ({GIMBAL_DEADZONE_PCT} %): "
            "raise the speed"
        )
    moved, seconds, stick = 0.0, 0.0, min(RAMP_START, stick_cap)
    while moved < degrees:
        moved += gimbal_rate(stick) * TICK_S
        seconds += TICK_S
        stick = min(stick_cap, stick + RAMP_STEP)
    return seconds


def _clamp(deg: float) -> float:
    return min(max(deg, GIMBAL_BOTTOM_DEG), GIMBAL_TOP_DEG)


class TiltEstimator:
    """Same style as the Estimator: on_command() and update() from the Drone's control thread,
    tilt() from any thread. Replace it with Drone(tilt_estimator=...)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._low = GIMBAL_BOTTOM_DEG
        self._high = GIMBAL_TOP_DEG
        self._reported: float | None = None

    def on_command(self, sticks: Axes, dt: float) -> None:
        change = gimbal_rate(sticks.gimbal) * dt
        if not change:
            return
        with self._lock:
            self._low = _clamp(self._low + change)
            self._high = _clamp(self._high + change)

    def update(self, telemetry: Telemetry) -> None:
        """A tilt the drone reports (never seen on the X1) replaces the estimate when it
        changes."""
        reported = telemetry.camera_tilt_deg
        if reported is None or reported == self._reported:
            return
        with self._lock:
            self._reported = reported
            self._low = self._high = _clamp(reported)

    def tilt(self) -> Tilt:
        with self._lock:
            low, high, reported = self._low, self._high, self._reported
        anchored = high - low <= ANCHORED_WITHIN_DEG
        if not anchored:
            return Tilt(None, low, high, False, "unknown")
        return Tilt((low + high) / 2, low, high, True, "reported" if reported else "estimated")
