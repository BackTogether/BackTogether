"""State estimation: the `Estimator` interface and the default dead-reckoning estimator.

The X1 gives us position and velocity at a fixed 1 Hz in a frame that resets at takeoff, and
no heading at all. `DeadReckoner` makes the best of that:
- heading: integrates the yaw stick that was actually sent x a calibrated yaw rate, and corrects
  it from the direction of travel whenever the drone flies a steady horizontal command without
  turning (velocity direction - command direction = heading);
- position: the last odometry sample, extrapolated with its velocity for up to
  EXTRAPOLATE_MAX_S;
- takeoff: a new flight starts a new frame, so heading and uncertainty are reset.

A better estimator (optical flow, visual localisation, teach-and-repeat) implements the same
three methods and is passed as `Drone(estimator=...)`; see drone/sdk/README.md.
"""

import math
import threading
import time
from typing import Protocol

from drone.hoverctl.protocol import AXIS_LIMIT, Axes

from .telemetry import Pose, Telemetry

# Unknown for the X1 until calibrate_yaw() runs on it: deg/s at full yaw stick (100).
# Measured 2026-11-14 from video optical flow (f = 850 px): 8.3 deg/s at stick 60, scaled
# linearly to full stick. A full-stick turn would confirm whether there is a dead zone.
DEFAULT_YAW_RATE_DPS = 13.8
YAW_RATE_ERROR = 0.15  # a turn of D degrees adds D x this to the heading sigma
HEADING_DRIFT_DPS = 0.5  # the drone may creep in yaw on its own while hovering
INITIAL_HEADING_SIGMA_DEG = 45.0  # the takeoff frame's axes vs the camera are unknown
MAX_HEADING_SIGMA_DEG = 180.0
# Heading fix from the direction of travel
FIX_MIN_STICK = 15.0  # horizontal stick (of 100) needed to trust the direction
FIX_MAX_YAW_STICK = 3.0  # no fix while turning
FIX_STEADY_S = 1.5  # the command must hold its direction this long (odometry lags)
FIX_DIRECTION_TOLERANCE_DEG = 15.0
FIX_MIN_SPEED_MPS = 0.25
FIX_SIGMA_DEG = 15.0  # 1-sigma of one velocity-direction measurement
# Position
EXTRAPOLATE_MAX_S = 1.0  # odometry is 1 Hz: never extrapolate further than one period
ODOMETRY_SIGMA_M = 0.1


class Estimator(Protocol):
    """What the Drone feeds and the Navigator reads. Implementations must be thread-safe:
    update() and on_command() run on the Drone's control thread, pose() on any thread."""

    def update(self, telemetry: Telemetry) -> None:
        """A telemetry reading (10 Hz; odometry inside changes at 1 Hz)."""

    def on_command(self, sticks: Axes, dt: float) -> None:
        """The stick values sent for the last `dt` seconds (-100..100, our signs)."""

    def pose(self) -> Pose:
        """The current belief, in the takeoff frame."""


def wrap_deg(angle: float) -> float:
    """Angle in (-180, 180]."""
    wrapped = math.fmod(angle + 180.0, 360.0)
    return (wrapped + 360.0 if wrapped <= 0 else wrapped) - 180.0


class DeadReckoner:
    """The default estimator: yaw-stick integration + travel-direction fixes + 1 Hz odometry."""

    def __init__(self, yaw_rate_dps: float = DEFAULT_YAW_RATE_DPS) -> None:
        self.yaw_rate_dps = yaw_rate_dps
        self._lock = threading.Lock()
        self._in_flight = False
        self._new_frame()

    def _new_frame(self) -> None:
        self._heading = 0.0
        self._heading_sigma = INITIAL_HEADING_SIGMA_DEG
        self._position = (0.0, 0.0, 0.0)
        self._velocity = (0.0, 0.0, 0.0)
        self._odometry_time: float | None = None
        self._steady_direction: float | None = None  # body-frame direction of the command
        self._steady_since = 0.0

    def update(self, telemetry: Telemetry) -> None:
        with self._lock:
            if telemetry.in_flight and not self._in_flight:
                self._new_frame()  # takeoff: the odometry frame starts over
            self._in_flight = telemetry.in_flight
            fresh = telemetry.odometry_time != self._odometry_time
            if not telemetry.in_flight or telemetry.position is None or not fresh:
                return
            self._position = telemetry.position
            self._velocity = telemetry.velocity or (0.0, 0.0, 0.0)
            self._odometry_time = telemetry.odometry_time
            self._fix_heading_from_travel()

    def on_command(self, sticks: Axes, dt: float) -> None:
        with self._lock:
            if not self._in_flight:
                return
            turn = sticks.yaw / AXIS_LIMIT * self.yaw_rate_dps * dt
            self._heading = wrap_deg(self._heading + turn)
            self._heading_sigma = min(
                MAX_HEADING_SIGMA_DEG,
                self._heading_sigma + abs(turn) * YAW_RATE_ERROR + HEADING_DRIFT_DPS * dt,
            )
            self._track_steady_command(sticks)

    def pose(self) -> Pose:
        with self._lock:
            age = 0.0
            if self._odometry_time is not None:
                age = min(max(time.time() - self._odometry_time, 0.0), EXTRAPOLATE_MAX_S)
            x, y, z = (p + v * age for p, v in zip(self._position, self._velocity, strict=True))
            speed = math.hypot(self._velocity[0], self._velocity[1])
            return Pose(
                x=x,
                y=y,
                z=z,
                heading_deg=self._heading,
                heading_sigma_deg=self._heading_sigma,
                position_sigma_m=ODOMETRY_SIGMA_M + speed * age,
                velocity=self._velocity,
                in_flight=self._in_flight,
            )

    def _track_steady_command(self, sticks: Axes) -> None:
        if abs(sticks.yaw) > FIX_MAX_YAW_STICK or math.hypot(sticks.pitch, sticks.roll) < (
            FIX_MIN_STICK
        ):
            self._steady_direction = None
            return
        direction = math.degrees(math.atan2(sticks.roll, sticks.pitch))  # 0 = forward, 90 = right
        changed = (
            self._steady_direction is None
            or abs(wrap_deg(direction - self._steady_direction)) > FIX_DIRECTION_TOLERANCE_DEG
        )
        if changed:
            self._steady_direction = direction
            self._steady_since = time.monotonic()

    def _fix_heading_from_travel(self) -> None:
        """Flying a steady command, velocity direction = heading + command direction."""
        vx, vy, _ = self._velocity
        steady = (
            self._steady_direction is not None
            and time.monotonic() - self._steady_since >= FIX_STEADY_S
        )
        if not steady or math.hypot(vx, vy) < FIX_MIN_SPEED_MPS:
            return
        measured = math.degrees(math.atan2(vy, vx)) - self._steady_direction
        # 1-D Kalman update on the heading
        prior = self._heading_sigma**2
        gain = prior / (prior + FIX_SIGMA_DEG**2)
        self._heading = wrap_deg(self._heading + gain * wrap_deg(measured - self._heading))
        self._heading_sigma = math.sqrt((1 - gain) * prior)
