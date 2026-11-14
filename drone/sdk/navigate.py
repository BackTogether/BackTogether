"""`Navigator`: slow closed-loop flight on top of the estimator.

Position only arrives at 1 Hz, so far from a target the navigator flies a proportional command
on the extrapolated pose ("cruise"), and near it switches to stop-and-correct: neutral, wait for
a fresh odometry sample, then a short pulse toward the target, until it is within tolerance.
Commands are computed in the takeoff frame and turned into body-frame sticks with the
estimated heading; a heading error bends the path, and the loop corrects it.

Every tick runs the safety rules (safety.py). STOP (Drone.stop) raises Interrupted.
"""

import math
import time
from collections.abc import Callable
from typing import TypeVar

from .drone import Drone
from .estimate import wrap_deg
from .safety import (
    HAZARD_RETURN,
    Interrupted,
    NotReady,
    ReturnHome,
    SafetyLanding,
    SafetyLimits,
)
from .telemetry import Pose

TICK_S = 0.1
COMMAND_HOLD_S = 0.4  # each tick renews the command; a stuck loop goes neutral by itself
POSITION_TOLERANCE_M = 0.4
ALT_TOLERANCE_M = 0.2
HEADING_TOLERANCE_DEG = 5.0
APPROACH_RADIUS_M = 1.5  # inside it: stop-and-correct
CRUISE_GAIN = 0.4  # stick fraction per metre of error
CRUISE_MAX = 0.6
APPROACH_FRACTION = 0.35
PULSE_S_PER_M = 1.2
MIN_PULSE_S = 0.3
MAX_PULSE_S = 1.5
ALT_GAIN = 0.8
MAX_VERTICAL = 0.5
YAW_GAIN = 1 / 45  # stick fraction per degree of heading error
MAX_TURN = 0.8
MIN_TURN = 0.15
SETTLE_S = 0.5  # after going neutral, before a sample is trusted
ODOMETRY_WAIT_S = 2.5
GOTO_TIMEOUT_S = 90.0
TURN_TIMEOUT_S = 30.0
LAND_RETRY_S = 1.0
SUPERVISE_IDLE_S = 2.0  # supervise() ends this long after the last outside command
HOME = (0.0, 0.0)  # the takeoff point
FLYING_PREFIX = "flying"
MODE_MANUAL = "MANUAL"
QUARTER_TURN_DEG = 90.0
# calibrate_yaw
CALIBRATION_TURN_DEG = 90.0  # well under 180, so a yaw rate off by up to 2x is not ambiguous
CALIBRATION_LEG_S = 3.5
CALIBRATION_LEG_SPEED = 0.5
CALIBRATION_SETTLE_S = 1.5  # odometry velocity lags the stick
CALIBRATION_MIN_TURN_DEG = 20.0

T = TypeVar("T")


class Navigator:
    """turn_to, climb_to, goto and hover, with the safety rules. One action at a time."""

    def __init__(self, drone: Drone, limits: SafetyLimits | None = None) -> None:
        self.drone = drone
        self.limits = limits or SafetyLimits()
        self.target: tuple[float, float, float] | None = None  # (x, y, alt) being flown to
        self._returning = False
        self._holding: str | None = None  # a loss being ridden out

    # --- actions ------------------------------------------------------------------------------

    def goto(
        self,
        x: float,
        y: float,
        alt: float | None = None,
        tolerance_m: float = POSITION_TOLERANCE_M,
        timeout_s: float = GOTO_TIMEOUT_S,
        max_fraction: float = CRUISE_MAX,
    ) -> Pose:
        """Fly to (x, y) in the takeoff frame at `alt` (default: the current altitude), with
        horizontal sticks up to `max_fraction` (lower it for a slow approach)."""
        epoch = self._begin()
        alt = self.drone.pose().alt if alt is None else alt
        self.limits.check_target(x, y, alt)
        return self._fly_to(epoch, (x, y), alt, tolerance_m, timeout_s, max_fraction)

    def climb_to(self, alt: float, timeout_s: float = GOTO_TIMEOUT_S) -> Pose:
        """Change altitude; also holds the horizontal position."""
        epoch = self._begin()
        pose = self.drone.pose()
        self.limits.check_target(pose.x, pose.y, alt)
        return self._fly_to(epoch, (pose.x, pose.y), alt, POSITION_TOLERANCE_M, timeout_s)

    def turn_to(
        self,
        heading_deg: float,
        tolerance_deg: float = HEADING_TOLERANCE_DEG,
        timeout_s: float = TURN_TIMEOUT_S,
    ) -> Pose:
        """Turn to an estimated heading (degrees from the takeoff frame's +x, + = right)."""
        epoch = self._begin()
        deadline = time.monotonic() + timeout_s
        try:
            while True:
                self._check_time(deadline, "turn")
                self.guard(epoch)
                pose = self.drone.pose()
                error = wrap_deg(heading_deg - pose.heading_deg)
                if abs(error) <= tolerance_deg:
                    return pose
                turn = math.copysign(min(MAX_TURN, max(MIN_TURN, abs(error) * YAW_GAIN)), error)
                self.send(epoch, turn=turn)
                self.drone.sleep(TICK_S, epoch)
        finally:
            self.drone.neutral()

    def turn_by(self, degrees: float) -> Pose:
        """Turn relative to the current estimated heading (|degrees| < 180)."""
        return self.turn_to(wrap_deg(self.drone.pose().heading_deg + degrees))

    def scan(self) -> Pose:
        """A full turn on the spot, in quarter turns."""
        pose = self.drone.pose()
        for _ in range(round(360 / QUARTER_TURN_DEG)):
            pose = self.turn_by(QUARTER_TURN_DEG)
        return pose

    def hover(self, seconds: float) -> None:
        """Neutral sticks for `seconds`, with the safety rules running."""
        epoch = self._begin()
        self.drone.neutral()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.guard(epoch)
            self.drone.sleep(min(TICK_S, max(deadline - time.monotonic(), 0.0)), epoch)

    def supervise(self, idle_s: float = SUPERVISE_IDLE_S) -> str:
        """Apply the safety rules to commands from outside (HTTP /move, a client-side patrol):
        runs until no command came for `idle_s`. Use with run_guarded()."""
        epoch = self._begin()
        idle_since: float | None = None
        while True:
            self.guard(epoch)
            if self.drone.command_active():
                idle_since = None
            elif idle_since is None:
                idle_since = time.monotonic()
            elif time.monotonic() - idle_since > idle_s:
                return "remote control ended"
            self.drone.sleep(TICK_S, epoch)

    def blocking_reason(self) -> str | None:
        """Why outside commands must wait now (returning home, riding out a loss), or None."""
        if self._returning:
            return "returning home"
        holding = self._holding
        return f"holding: {holding}" if holding else None

    def run_guarded(self, action: Callable[[], T]) -> T | str:
        """Run an action; if a safety rule asks for it, return home and land instead."""
        try:
            return action()
        except ReturnHome as why:
            return self.return_home_and_land(str(why))

    def return_home_and_land(self, reason: str) -> str:
        """Fly back to the takeoff point at a safe altitude, then land. STOP still wins."""
        print(f"navigator: {reason}: returning to the takeoff point to land", flush=True)
        self._returning = True
        try:
            epoch = self.drone.epoch
            alt = self.limits.clamp_alt(self.drone.pose().alt)
            try:
                self._fly_to(epoch, HOME, alt, POSITION_TOLERANCE_M, GOTO_TIMEOUT_S)
            except TimeoutError:
                print("navigator: home not reached in time, landing here", flush=True)
            self._land(f"returned home: {reason}")
        finally:
            self._returning = False
        return f"returned home and landed: {reason}"

    # --- control ------------------------------------------------------------------------------

    def require_ready(self) -> None:
        """Raise NotReady unless the drone is flying in MANUAL."""
        telemetry = self.drone.telemetry()
        state = telemetry.flight_state or "no state"
        if not state.startswith(FLYING_PREFIX) or telemetry.mode != MODE_MANUAL:
            raise NotReady(f"needs flying in MANUAL, the drone is {state} in {telemetry.mode}")

    def _begin(self) -> int:
        """The epoch of a new action, after require_ready()."""
        self.require_ready()
        return self.drone.epoch

    def _fly_to(
        self,
        epoch: int,
        xy: tuple[float, float],
        alt: float,
        tolerance_m: float,
        timeout_s: float,
        max_fraction: float = CRUISE_MAX,
    ) -> Pose:
        self.target = (*xy, alt)
        deadline = time.monotonic() + timeout_s
        try:
            while True:
                self._check_time(deadline, "goto")
                self.guard(epoch)
                pose = self.drone.pose()
                if math.hypot(xy[0] - pose.x, xy[1] - pose.y) > APPROACH_RADIUS_M:
                    self._cruise(epoch, pose, xy, alt, max_fraction)
                    self.drone.sleep(TICK_S, epoch)
                    continue
                pose = self._settle(epoch)
                distance = math.hypot(xy[0] - pose.x, xy[1] - pose.y)
                if distance <= tolerance_m and abs(alt - pose.alt) <= ALT_TOLERANCE_M:
                    return pose
                self._pulse(epoch, pose, xy, alt, max_fraction)
        finally:
            self.target = None
            self.drone.neutral()

    def _cruise(
        self, epoch: int, pose: Pose, xy: tuple[float, float], alt: float, max_fraction: float
    ) -> None:
        fwd, right, distance = _body_direction(pose, xy)
        fraction = min(max_fraction, CRUISE_GAIN * distance)
        up = _vertical(alt - pose.alt)
        self.send(epoch, fwd=fwd * fraction, right=right * fraction, up=up)

    def _pulse(
        self, epoch: int, pose: Pose, xy: tuple[float, float], alt: float, max_fraction: float
    ) -> None:
        """One short push toward the target, then neutral."""
        fwd, right, distance = _body_direction(pose, xy)
        alt_error = alt - pose.alt
        seconds = max(distance, abs(alt_error)) * PULSE_S_PER_M
        seconds = min(MAX_PULSE_S, max(MIN_PULSE_S, seconds))
        fraction = min(APPROACH_FRACTION, max_fraction)
        fraction = fraction if distance > POSITION_TOLERANCE_M / 2 else 0.0
        up = _vertical(alt_error)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.guard(epoch)
            self.send(epoch, fwd=fwd * fraction, right=right * fraction, up=up)
            self.drone.sleep(TICK_S, epoch)
        self.drone.neutral()

    def _settle(self, epoch: int) -> Pose:
        """Neutral, then the pose from the next odometry sample (stop-and-correct)."""
        self.drone.neutral()
        self.drone.sleep(SETTLE_S, epoch)
        while self.drone.wait_odometry(time.time(), ODOMETRY_WAIT_S, epoch) is None:
            self.guard(epoch)  # telemetry loss: the rules decide (hover, then land)
        return self.drone.pose()

    def send(
        self, epoch: int, fwd: float = 0.0, right: float = 0.0, up: float = 0.0, turn: float = 0.0
    ) -> None:
        """Command scaled so no axis goes over limits.max_speed (stick %)."""
        scale = min(1.0, self.limits.max_speed / max(self.drone.speed_cap, 1.0))
        self.drone.command(
            fwd * scale, right * scale, up * scale, turn * scale, COMMAND_HOLD_S, epoch
        )

    def guard(self, epoch: int) -> None:
        """STOP, then the safety rules; see safety.py."""
        self.drone.check(epoch)
        telemetry = self.drone.telemetry()
        hazard = self.limits.hazard(telemetry, telemetry.pose or self.drone.pose())
        if hazard is None:
            return
        if hazard.kind == HAZARD_RETURN:
            if not self._returning:
                raise ReturnHome(hazard.reason)
            return
        self._ride_out_loss(epoch, hazard.reason)

    def _ride_out_loss(self, epoch: int, reason: str) -> None:
        """Hover through a loss; resume when it ends, land when it lasts too long."""
        print(f"navigator: {reason}: hovering", flush=True)
        self._holding = reason
        since = time.monotonic()
        try:
            while True:
                self.drone.neutral()
                self.drone.sleep(TICK_S, epoch)
                telemetry = self.drone.telemetry()
                hazard = self.limits.hazard(telemetry, telemetry.pose or self.drone.pose())
                if hazard is None or hazard.kind == HAZARD_RETURN:
                    print(f"navigator: {reason} is over, resuming", flush=True)
                    return
                if time.monotonic() - since > self.limits.loss_land_s:
                    self._land(f"{reason} for {self.limits.loss_land_s:.0f} s")
        finally:
            self._holding = None

    def _land(self, reason: str) -> None:
        """Land, retrying while the link is down. STOP during the retries cancels."""
        while True:
            result = self.drone.land()
            if result["ok"]:
                raise SafetyLanding(reason)
            epoch = self.drone.epoch
            print(f"navigator: land failed ({result['error']}), retrying", flush=True)
            time.sleep(LAND_RETRY_S)
            if self.drone.epoch != epoch:
                raise Interrupted("stopped while retrying to land")

    @staticmethod
    def _check_time(deadline: float, what: str) -> None:
        if time.monotonic() > deadline:
            raise TimeoutError(f"{what}: target not reached in time")


def _body_direction(pose: Pose, xy: tuple[float, float]) -> tuple[float, float, float]:
    """Unit (fwd, right) toward `xy` with the estimated heading, and the distance."""
    ex, ey = xy[0] - pose.x, xy[1] - pose.y
    distance = math.hypot(ex, ey)
    if distance == 0:
        return 0.0, 0.0, 0.0
    heading = math.radians(pose.heading_deg)
    fwd = ex * math.cos(heading) + ey * math.sin(heading)
    right = -ex * math.sin(heading) + ey * math.cos(heading)
    return fwd / distance, right / distance, distance


def _vertical(alt_error: float) -> float:
    return max(-MAX_VERTICAL, min(MAX_VERTICAL, ALT_GAIN * alt_error))


def calibrate_yaw(navigator: Navigator) -> float:
    """Measure the yaw rate at full stick and store it in the DeadReckoner.

    Flies forward (heading from the direction of travel), turns right about 90 degrees by the
    current estimate, flies forward again: the change in travel direction over the yaw stick
    actually sent gives the true rate. Needs about 4 m of open space ahead, flying in MANUAL.
    Returns the new deg/s at full stick; put it in DEFAULT_YAW_RATE_DPS (estimate.py).
    """
    estimator = navigator.drone.estimator
    if not hasattr(estimator, "yaw_rate_dps"):
        raise TypeError("calibrate_yaw needs an estimator with a yaw_rate_dps (DeadReckoner)")
    epoch = navigator._begin()
    try:
        before = _travel_direction(navigator, epoch)
        start = estimator.pose().heading_deg
        scale = min(1.0, navigator.limits.max_speed / max(navigator.drone.speed_cap, 1.0))
        rate_now = estimator.yaw_rate_dps * navigator.drone.speed_cap / 100 * scale
        deadline = time.monotonic() + CALIBRATION_TURN_DEG / rate_now
        while time.monotonic() < deadline:
            navigator.guard(epoch)
            navigator.send(epoch, turn=1.0)
            navigator.drone.sleep(TICK_S, epoch)
        navigator.drone.neutral()
        navigator.drone.sleep(SETTLE_S, epoch)
        predicted = wrap_deg(estimator.pose().heading_deg - start)
        after = _travel_direction(navigator, epoch)
    finally:
        navigator.drone.neutral()
    measured = wrap_deg(after - before)
    if predicted < CALIBRATION_MIN_TURN_DEG or measured <= 0:
        raise ValueError(f"calibration failed: estimated {predicted:.0f}, measured {measured:.0f}")
    estimator.yaw_rate_dps *= measured / predicted
    print(f"calibrate_yaw: {estimator.yaw_rate_dps:.1f} deg/s at full stick", flush=True)
    return estimator.yaw_rate_dps


def _travel_direction(navigator: Navigator, epoch: int) -> float:
    """Fly forward for a leg; the mean odometry velocity direction (frame degrees)."""
    drone = navigator.drone
    start = time.monotonic()
    sum_x = sum_y = 0.0
    last_sample = drone.telemetry().odometry_time
    while time.monotonic() - start < CALIBRATION_LEG_S:
        navigator.guard(epoch)
        navigator.send(epoch, fwd=CALIBRATION_LEG_SPEED)
        drone.sleep(TICK_S, epoch)
        telemetry = drone.telemetry()
        settled = time.monotonic() - start >= CALIBRATION_SETTLE_S
        if settled and telemetry.velocity and telemetry.odometry_time != last_sample:
            last_sample = telemetry.odometry_time
            sum_x += telemetry.velocity[0]
            sum_y += telemetry.velocity[1]
    drone.neutral()
    if math.hypot(sum_x, sum_y) == 0:
        raise ValueError("calibration failed: no odometry velocity during the leg")
    return math.degrees(math.atan2(sum_y, sum_x))
