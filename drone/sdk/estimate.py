"""State estimation: the `Estimator` interface and the default dead-reckoning estimator.

The X1 gives us position and velocity at a fixed 1 Hz in a frame that resets at takeoff, and
no heading at all. `DeadReckoner` makes the best of that:
- heading: integrates the yaw stick that was actually sent x a calibrated yaw rate; where the
  video measures the yaw (flow.py, `on_yaw_step`) the measured turn replaces the stick model's
  over the same interval, and steady turns re-calibrate the stick model online. It also corrects
  the heading from the direction of travel whenever the drone flies a steady horizontal command
  without turning (velocity direction - command direction = heading);
- position: the last odometry sample, extrapolated with its velocity for up to
  EXTRAPOLATE_MAX_S;
- takeoff: a new flight starts a new frame, so heading and uncertainty are reset.

A better estimator (optical flow, visual localisation, teach-and-repeat) implements the same
three methods and is passed as `Drone(estimator=...)`; see drone/sdk/README.md.
"""

import math
import threading
import time
from collections import deque
from typing import Protocol

from drone.hoverctl.protocol import AXIS_LIMIT, Axes

from .flow import YawStep
from .telemetry import Pose, Telemetry
from .tilt import GIMBAL_DEADZONE_PCT

# Unknown for the X1 until calibrate_yaw() runs on it: deg/s at full yaw stick (100).
# Measured 2026-11-14 with exact 360-degree spins (drone.calibration.spin, view repeat in the
# video): one turn in 14.9 s at stick 100, 18.7 s at 80, 25.2 s at 60 = 24.2 / 19.3 / 14.3 deg/s,
# i.e. ~0.24 deg/s per stick % with almost no dead zone.
DEFAULT_YAW_RATE_DPS = 24.2
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
# Yaw measured from the video (flow.py)
FLOW_MIN_CONFIDENCE = 0.3  # steps below are ignored (blank, blurred or disturbed images)
FLOW_STALE_S = 1.0  # no trusted step for this long: the heading is back on the sticks alone
FLOW_YAW_ERROR = 0.05  # a measured turn of D degrees adds D x this to the heading sigma
FLOW_DRIFT_DPS = 0.05  # the video sees the slow creep that HEADING_DRIFT_DPS allows for
FLOW_RATE_WINDOW = 5  # steps (~1 s) averaged for the displayed rate
STICK_HISTORY = 150  # stick-model turn samples kept (~5 s at 30 Hz), to match a step's interval
# Image motion that is not yaw: moving the camera, flying sideways or fast
FLOW_GATE_ROLL_STICK = 10.0
FLOW_GATE_SPEED_MPS = 1.0
FLOW_GATE_HOLD_S = 1.0  # the drone keeps drifting after the stick is released
# Online calibration of yaw_rate_dps from steady measured turns
CALIBRATE_MIN_STICK = 20.0
CALIBRATE_STEADY_S = 1.0  # skip the drone's spin-up after the stick settles
CALIBRATE_GAIN = 0.05  # per step (5 per second): ~4 s of steady turning to mostly converge
CALIBRATE_MAX_RATIO = 2.0  # one step moves the model at most this far (glitches)
MIN_YAW_RATE_DPS = 3.0
MAX_YAW_RATE_DPS = 120.0
# Position
EXTRAPOLATE_MAX_S = 1.0  # odometry is 1 Hz: never extrapolate further than one period
ODOMETRY_SIGMA_M = 0.1


class Estimator(Protocol):
    """What the Drone feeds and the Navigator reads. Implementations must be thread-safe:
    update() and on_command() run on the Drone's control thread, pose() on any thread.
    Optional: on_yaw_step(step: YawStep), called on the video thread when the video measures
    the yaw (DeadReckoner has it); estimators without it simply do not get the measurement."""

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
    """The default estimator: yaw from the video where trusted, else yaw-stick integration,
    + travel-direction fixes + 1 Hz odometry."""

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
        self._stick_total = 0.0  # stick-model turn since takeoff, deg
        self._stick_history: deque[tuple[float, float]] = deque(maxlen=STICK_HISTORY)
        self._flow_gated_until = 0.0  # unix time
        self._flow_at: float | None = None  # unix time of the last applied step
        self._recent_steps: deque[YawStep] = deque(maxlen=FLOW_RATE_WINDOW)
        self._waiting_steps: deque[YawStep] = deque()  # until the stick history covers them
        self._yaw_stick: float | None = None  # a steady yaw stick for calibration, since:
        self._yaw_stick_since = 0.0

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
            if math.hypot(self._velocity[0], self._velocity[1]) > FLOW_GATE_SPEED_MPS:
                self._flow_gated_until = time.time() + FLOW_GATE_HOLD_S
            self._fix_heading_from_travel()

    def on_command(self, sticks: Axes, dt: float) -> None:
        with self._lock:
            if not self._in_flight:
                return
            now = time.time()
            turn = sticks.yaw / AXIS_LIMIT * self.yaw_rate_dps * dt
            self._stick_total += turn
            self._stick_history.append((now, self._stick_total))
            self._heading = wrap_deg(self._heading + turn)
            measured = self._heading_source(now) == "flow"
            turn_error = FLOW_YAW_ERROR if measured else YAW_RATE_ERROR
            drift = FLOW_DRIFT_DPS if measured else HEADING_DRIFT_DPS
            self._heading_sigma = min(
                MAX_HEADING_SIGMA_DEG, self._heading_sigma + abs(turn) * turn_error + drift * dt
            )
            if abs(sticks.gimbal) > GIMBAL_DEADZONE_PCT or abs(sticks.roll) > FLOW_GATE_ROLL_STICK:
                self._flow_gated_until = now + FLOW_GATE_HOLD_S
            self._track_yaw_stick(sticks.yaw, now)
            self._track_steady_command(sticks)
            self._apply_waiting_steps()

    def on_yaw_step(self, step: YawStep) -> None:
        """A yaw measured from the video: it replaces the stick model's turn over the same
        interval, so the decoding delay does not matter and nothing is counted twice."""
        with self._lock:
            self._recent_steps.append(step)
            if self._in_flight and step.confidence >= FLOW_MIN_CONFIDENCE:
                self._waiting_steps.append(step)
                self._apply_waiting_steps()

    def _apply_waiting_steps(self) -> None:
        """Apply the steps the stick history now covers: a frame usually comes between two
        control ticks, and the stick turn up to its time is only known at the next tick
        (clamping there instead counted ~8 % of every turn twice)."""
        history = self._stick_history
        while self._waiting_steps and history and history[-1][0] >= self._waiting_steps[0].end:
            step = self._waiting_steps.popleft()
            if step.start < history[0][0] or step.start < self._flow_gated_until:
                continue
            stick_turn = self._stick_turn(step.end) - self._stick_turn(step.start)
            self._heading = wrap_deg(self._heading + step.yaw_deg - stick_turn)
            self._flow_at = step.end
            self._calibrate(step, stick_turn)

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
                heading_source=self._heading_source(time.time()),
                **self._flow_reading(),
            )

    def _heading_source(self, now: float) -> str:
        fresh = self._flow_at is not None and now - self._flow_at < FLOW_STALE_S
        return "flow" if fresh else "sticks"

    def _flow_reading(self) -> dict[str, float | None]:
        """The video's yaw rate and confidence over the last ~1 s, None when it is silent."""
        steps = [s for s in self._recent_steps if time.time() - s.end < FLOW_STALE_S]
        if not steps:
            return {"flow_yaw_rate_dps": None, "flow_confidence": None}
        good = [s for s in steps if s.confidence >= FLOW_MIN_CONFIDENCE]
        span = sum(s.end - s.start for s in good)
        return {
            "flow_yaw_rate_dps": sum(s.yaw_deg for s in good) / span if span > 0 else None,
            "flow_confidence": sum(s.confidence for s in steps) / len(steps),
        }

    def _stick_turn(self, at: float) -> float:
        """The stick model's total turn at unix time `at`, interpolated in the history."""
        history = self._stick_history
        if at <= history[0][0]:
            return history[0][1]
        for (t0, turn0), (t1, turn1) in zip(history, list(history)[1:], strict=False):
            if t0 <= at <= t1:
                return turn0 + (turn1 - turn0) * (at - t0) / max(t1 - t0, 1e-6)
        return history[-1][1]

    def _track_yaw_stick(self, yaw: float, now: float) -> None:
        if abs(yaw) < CALIBRATE_MIN_STICK:
            self._yaw_stick = None
        elif self._yaw_stick != yaw:
            self._yaw_stick, self._yaw_stick_since = yaw, now

    def _calibrate(self, step: YawStep, stick_turn: float) -> None:
        """Steady turn measured: nudge yaw_rate_dps toward what the video saw."""
        steady = (
            self._yaw_stick is not None and step.start - self._yaw_stick_since >= CALIBRATE_STEADY_S
        )
        if not steady or stick_turn * step.yaw_deg <= 0:
            return
        ratio = min(CALIBRATE_MAX_RATIO, max(1 / CALIBRATE_MAX_RATIO, step.yaw_deg / stick_turn))
        rate = self.yaw_rate_dps * (1 + CALIBRATE_GAIN * (ratio - 1))
        self.yaw_rate_dps = min(MAX_YAW_RATE_DPS, max(MIN_YAW_RATE_DPS, rate))

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
