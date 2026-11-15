"""Camera height above the floor, robust to bad altitude readings and a wrong takeoff height.

The floor geometry (floor_boundary.py) scales every distance with the camera height, so a
1 m error there was most of our 2x over-read (2026-11-14: --takeoff-height 1.0 after a floor
takeoff). Two layers:

1. `CameraHeight`: SDK altitude (above the takeoff point) + the takeoff height (0 = took off
   from the floor). Readings that are missing, out of range or jump faster than the drone can
   climb are dropped (the odometry z jumps by metres around takeoffs and landings); a new jump
   level is only believed once it has held for a few readings.
2. `HeightCalibration`: while the drone flies straight at a wall, the floor-geometry distance
   must shrink exactly as fast as the odometry says the drone moves. A line fit of distance vs
   travel gives the factor between the assumed and the true geometry (it absorbs a wrong height
   and most of a camera tilt error): on the recorded approach, from 2.72 m assumed (1 m wrong)
   to 1.57 m, error 1.67 -> 0.15 m. The odometry's travel is the only scale it trusts.
"""

import math
from dataclasses import dataclass, field

import numpy as np

MIN_ALT_M = 0.1  # above the takeoff point; below this the drone is landing or on the ground
MAX_ALT_M = 6.0  # above this an altitude reading is taken as a glitch
MAX_STEP_M = 0.6  # between two readings (the patrol reads ~5/s; climbs are < 1 m/s)
CONFIRM_READINGS = 5  # a jump is believed after holding this many readings
# Self-calibration
MIN_TRAVEL_M = 0.8  # straight travel toward the same wall before fitting
MIN_SAMPLES = 6
MIN_SAMPLE_SPACING_M = 0.1
MIN_FIT_R2 = 0.9
SCALE_LIMITS = (0.4, 2.5)  # beyond this the fit, not the height, is wrong
SCALE_BLEND = 0.5  # weight of the current leg's fit against the earlier legs'


@dataclass
class CameraHeight:
    takeoff_height_m: float = 0.0
    _alt: float | None = None
    _pending: list[float] = field(default_factory=list)

    def update(self, altitude_m: float | None, in_flight: bool) -> None:
        if not in_flight:
            self._alt, self._pending = None, []  # a new takeoff re-zeroes the altitude
            return
        if altitude_m is None or not MIN_ALT_M <= altitude_m <= MAX_ALT_M:
            return
        if self._alt is None or abs(altitude_m - self._alt) <= MAX_STEP_M:
            self._alt, self._pending = altitude_m, []
            return
        self._pending.append(altitude_m)
        if len(self._pending) >= CONFIRM_READINGS and np.ptp(self._pending) <= MAX_STEP_M:
            self._alt, self._pending = float(np.median(self._pending)), []

    @property
    def metres(self) -> float | None:
        """Camera height above the floor, None until a plausible altitude arrived."""
        return None if self._alt is None else self._alt + self.takeoff_height_m


@dataclass
class HeightCalibration:
    """Scale for the assumed camera height, learnt from straight approaches to a wall."""

    legs: int = 0  # approaches that gave a good fit
    _committed: float = 1.0  # from the legs flown so far
    _leg_scale: float | None = None  # from the current leg
    _samples: list[tuple[float, float, float]] = field(default_factory=list)  # x, y, front

    @property
    def scale(self) -> float:
        if self._leg_scale is None:
            return self._committed
        if self.legs == 0:
            return self._leg_scale
        return (1 - SCALE_BLEND) * self._committed + SCALE_BLEND * self._leg_scale

    def end_leg(self) -> None:
        """A turn: the next approach is toward another wall."""
        if self._leg_scale is not None:
            self._committed, self.legs = self.scale, self.legs + 1
        self._leg_scale, self._samples = None, []

    def add(self, xy: tuple[float, float], front_m: float) -> None:
        """One reading of a straight approach: odometry position and the distance measured
        with the *uncalibrated* height."""
        if self._samples:
            last_x, last_y, _ = self._samples[-1]
            if math.hypot(xy[0] - last_x, xy[1] - last_y) < MIN_SAMPLE_SPACING_M:
                return
        self._samples.append((xy[0], xy[1], front_m))
        fitted = _fit_scale(np.array(self._samples))
        if fitted is not None:
            self._leg_scale = fitted


def _fit_scale(points: np.ndarray) -> float | None:
    """Rows (x, y, front). Distance vs travel along the leg: the slope must be -1."""
    if len(points) < MIN_SAMPLES:
        return None
    direction = points[-1, :2] - points[0, :2]
    travel = float(np.hypot(*direction))
    if travel < MIN_TRAVEL_M:
        return None
    along = (points[:, :2] - points[0, :2]) @ (direction / travel)
    slope, intercept = np.polyfit(along, points[:, 2], 1)
    residual = np.sum((points[:, 2] - (intercept + slope * along)) ** 2)
    total = np.sum((points[:, 2] - points[:, 2].mean()) ** 2)
    if slope >= 0 or total == 0 or 1 - residual / total < MIN_FIT_R2:
        return None
    scale = -1 / slope
    return scale if SCALE_LIMITS[0] <= scale <= SCALE_LIMITS[1] else None
