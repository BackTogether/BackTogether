"""Clear patrol: fly straight to a wall, hover, turn always the same way until the way is clear.

A deterministic alternative to dvd.py's reflection turns, with the same perception
(floor-boundary front distance, camera height, tilt), SDK plumbing, safety holds, panel and
simulator support:
1. FORWARD at --speed (slower near walls or when unsure) until the nearest reading of the
   last WINDOW_S is closer than --stop-at.
2. HOVER --hover-s with neutral sticks: stop and stabilise.
3. TURN in place, always --turn (right or left), at --turn-input, until the front distance
   is beyond --clear-at (a clear view counts too) on every fresh reading for --clear-for-s.
   Unreadable or low-confidence readings count as not clear. Then FORWARD again.
4. A full 360 deg without that: hover, then turn on (same way) to the heading with the
   largest front distance seen, and go forward if it is beyond --stop-at + BEST_MARGIN_M;
   else hover there, "boxed in", until the patrol is stopped.

Degrees turned come from the heading (the SDK's measured yaw, else its estimator), or from the
calibrated yaw rate x the time spent turning when there is no heading.

Run from the repo root:
  uv run --offline --no-project --with numpy --with pillow \\
      python -m drone.patrol.clear --sim --duration 300 --pillars --plot /tmp/clear.png
  drone/patrol/live clear-dry      # real drone: decisions only;  drone/patrol/live clear  to fly
Stop it with Ctrl+C (neutral + STOP), or STOP / LAND in the web page.
"""

import argparse
import math
import time
from collections.abc import Callable
from enum import Enum

from drone.patrol import dvd
from drone.patrol.dvd import Perception, Sample, SimLog, TurnChoice
from drone.patrol.floor_boundary import Status
from drone.patrol.sdkdrone import DroneState, PatrolDrone
from drone.sdk.estimate import wrap_deg

FULL_TURN_DEG = 360.0
BEST_MARGIN_M = 0.5  # after a full turn: the best heading must beat --stop-at by this
MIN_CLEAR_READINGS = 3  # a "clear" streak needs at least this many readings
MIN_KNOWN_FRACTION = 0.8  # of the corridor columns: below this a reading is low-confidence
MAX_TIMED_STEP_S = 0.3  # without a heading: longer gaps between steps were holds, not turning
LEGEND = "green: turned until clear, grey: 360 without clear -> best heading, red: boxed in"
TURN_SIGNS = {"right": 1.0, "left": -1.0}


class Phase(Enum):
    FORWARD = "FORWARD"
    HOVER = "HOVER"
    TURN = "TURN"
    ALIGN = "ALIGN"  # after a full turn: on to the best heading seen
    BOXED = "BOXED"


class TurnTracker:
    """Degrees turned in one direction since the start (negative if the drone turned back)."""

    def __init__(self, sign: float, yaw_rate: float) -> None:
        self.sign, self.yaw_rate = sign, yaw_rate
        self.degrees = 0.0
        self._heading: float | None = None
        self._at: float | None = None

    def update(self, state: DroneState, now: float) -> float:
        heading = dvd.current_heading(state)
        if heading is not None and self._heading is not None:
            self.degrees += self.sign * wrap_deg(heading - self._heading)
        elif self._at is not None:
            self.degrees += self.yaw_rate * min(now - self._at, MAX_TIMED_STEP_S)
        self._heading, self._at = heading, now
        return self.degrees


def clear_value(sample: Sample) -> float | None:
    """Front distance for the turn's decisions; None when unreadable or low-confidence."""
    if sample.wall.status is Status.UNKNOWN or sample.wall.known_fraction < MIN_KNOWN_FRACTION:
        return None
    return dvd.front_value(sample.wall)


def metres(value: float | None) -> str:
    if value is None:
        return "?"
    return "clear" if math.isinf(value) else f"{value:.2f} m"


class ClearPatrol:
    def __init__(
        self,
        args: argparse.Namespace,
        perception: Perception,
        yaw_rate: float,
        on_turn: Callable[[TurnChoice], None],
    ) -> None:
        """`yaw_rate`: deg/s at --turn-input (the SDK's calibration); `on_turn`: for the log."""
        self.args, self.perception, self.yaw_rate = args, perception, yaw_rate
        self.on_turn = on_turn
        self.sign = TURN_SIGNS[args.turn]
        self.phase = Phase.FORWARD
        self.since = time.monotonic()
        self.until = 0.0
        self.after_hover = Phase.TURN
        self.tracker = TurnTracker(self.sign, yaw_rate)
        self.target_deg = 0.0  # ALIGN: degrees to turn
        self.seen_at = 0.0  # TURN: readings up to this time are used
        self.clear_since: float | None = None  # first reading of the current clear streak
        self.clear_count = 0
        self.best: tuple[float, float] = (-1.0, 0.0)  # (front m, degrees turned) of this turn
        self.decision = "starting"
        self.counts = {"clear": 0, "best": 0, "boxed": 0}

    def _enter(self, phase: Phase, seconds: float = 0.0, note: str = "") -> None:
        self.phase, self.since = phase, time.monotonic()
        self.until = self.since + seconds
        print(f"-> {phase.value} {note}".rstrip(), flush=True)

    def step(self, state: DroneState) -> dict[str, float]:
        """Advance the state machine; the stick axes for this tick."""
        now = time.monotonic()
        if self.phase is Phase.FORWARD:
            return self._forward(state, now)
        if self.phase is Phase.HOVER:
            if now >= self.until:
                self._start_turn(state, now)
            return {}
        if self.phase is Phase.TURN:
            return self._turning(state, now)
        if self.phase is Phase.ALIGN:
            return self._aligning(state, now)
        return {}  # BOXED: hover until the patrol is stopped

    def panel_line(self, latest: Sample) -> str:
        front = metres(dvd.front_value(latest.wall))
        turning = self.phase in (Phase.TURN, Phase.ALIGN)
        turned = f"{self.sign * self.tracker.degrees:+4.0f}deg" if turning else "    -"
        return f"{self.phase.value:7s} front={front:>7s} turned={turned} | {self.decision}"

    def _forward(self, state: DroneState, now: float) -> dict[str, float]:
        # Only this leg's readings: the turn's last ones looked at other headings.
        window = self.perception.readings.since(max(now - dvd.WINDOW_S, self.since))
        front, unsure = dvd.nearest_front(window)
        if front is not None and front < self.args.stop_at:
            self.decision = f"front {front:.2f} < stop-at {self.args.stop_at} m: hover"
            self.after_hover = Phase.TURN
            self._enter(Phase.HOVER, self.args.hover_s, f"({window[-1].wall.summary()})")
            return {}
        dvd.calibrate(self.perception, state, window)
        if unsure:
            self.decision = "unsure view: slow"
        elif front < self.args.slow_at:
            self.decision = f"front < slow-at {self.args.slow_at} m: slow"
        else:
            self.decision = "forward"
        slow = unsure or front < self.args.slow_at
        return {"pitch": self.args.slow_speed if slow else self.args.speed}

    def _start_turn(self, state: DroneState, now: float) -> None:
        self.tracker = TurnTracker(self.sign, self.yaw_rate)
        self.tracker.update(state, now)
        if self.after_hover is Phase.ALIGN:
            self._enter(Phase.ALIGN, note=f"{self.args.turn} {self.target_deg:.0f} deg")
            return
        calibration = self.perception.calibration
        calibration.end_leg()
        print(f"camera height scale x{calibration.scale:.2f} ({calibration.legs} legs)", flush=True)
        self.seen_at, self.clear_since, self.clear_count = now, None, 0
        self.best = (-1.0, 0.0)
        self.decision = f"turn {self.args.turn} until > {self.args.clear_at} m"
        self._enter(Phase.TURN, note=self.args.turn)

    def _turning(self, state: DroneState, now: float) -> dict[str, float]:
        turned = self.tracker.update(state, now)
        for sample in self.perception.readings.since(self.seen_at):
            if sample.at <= self.seen_at:
                continue
            self.seen_at = sample.at
            if self._clear_after(sample, turned):
                return self._cleared(turned)
        if turned >= FULL_TURN_DEG:
            return self._full_turn(turned)
        streak = 0.0 if self.clear_since is None else self.seen_at - self.clear_since
        self.decision = (
            f"turn {self.args.turn} until > {self.args.clear_at} m for "
            f"{self.args.clear_for_s} s: clear {streak:.1f} s, best {metres(self.best[0])}"
        )
        return {"yaw": self.sign * self.args.turn_input}

    def _clear_after(self, sample: Sample, turned: float) -> bool:
        """Count one fresh reading; True once the clear streak is long enough."""
        value = clear_value(sample)
        if value is not None and value > self.best[0]:
            self.best = (value, turned)
        if value is None or value <= self.args.clear_at:
            self.clear_since, self.clear_count = None, 0
            return False
        if self.clear_since is None:
            self.clear_since = sample.at
        self.clear_count += 1
        long_enough = sample.at - self.clear_since >= self.args.clear_for_s
        return long_enough and self.clear_count >= MIN_CLEAR_READINGS

    def _cleared(self, turned: float) -> dict[str, float]:
        self.counts["clear"] += 1
        self.on_turn(TurnChoice(self.sign * turned, "clear", ""))
        self.decision = f"clear after {turned:.0f} deg: forward"
        self._enter(Phase.FORWARD, note=f"(clear after {self.args.turn} {turned:.0f} deg)")
        return {}

    def _full_turn(self, turned: float) -> dict[str, float]:
        """360 without a clear reading: to the best heading seen, or boxed in."""
        front, at = self.best
        if front <= self.args.stop_at + BEST_MARGIN_M:
            self.counts["boxed"] += 1
            self.on_turn(TurnChoice(self.sign * turned, "boxed", ""))
            self.decision = f"BOXED IN: best {metres(front)} after a full turn: hovering"
            self._enter(Phase.BOXED, note=self.decision)
            return {}
        self.target_deg = (at - turned) % FULL_TURN_DEG
        self.after_hover = Phase.ALIGN
        self.decision = f"360 without clear: best {metres(front)} at {at:.0f} deg, back to it"
        self._enter(Phase.HOVER, self.args.hover_s, f"({self.decision})")
        return {}

    def _aligning(self, state: DroneState, now: float) -> dict[str, float]:
        turned = self.tracker.update(state, now)
        if turned < self.target_deg:
            self.decision = f"to the best heading: {turned:.0f}/{self.target_deg:.0f} deg"
            return {"yaw": self.sign * self.args.turn_input}
        self.counts["best"] += 1
        self.on_turn(TurnChoice(self.sign * turned, "best", ""))
        self.decision = "at the best heading: forward"
        self._enter(Phase.FORWARD, note="(best heading)")
        return {}


def run(args: argparse.Namespace, drone: PatrolDrone, log: SimLog | None) -> str:
    dvd.setup_camera(args, drone)
    yaw_rate = drone.yaw_rate_dps(args.turn_input)
    print(f"turn {args.turn} at {yaw_rate:.1f} deg/s (yaw input {args.turn_input}, SDK rate)")
    perception = Perception(drone, args)
    patrol = ClearPatrol(args, perception, yaw_rate, log.turned if log else lambda _: None)
    outcome = drone.run(lambda: dvd.loop(args, drone, perception, patrol))
    counts = ", ".join(f"{count} {how}" for how, count in patrol.counts.items())
    scale = perception.calibration
    return f"{outcome}; turns: {counts}; height scale x{scale.scale:.2f} ({scale.legs} legs)"


def parse_args() -> argparse.Namespace:
    parser, flight = dvd.new_parser(__doc__.splitlines()[0])
    flight.add_argument("--stop-at", type=float, default=1.0, help="hover and turn below (m)")
    flight.add_argument("--hover-s", type=float, default=1.0, help="hover before turning, s")
    flight.add_argument("--turn", choices=tuple(TURN_SIGNS), default="right", help="always")
    flight.add_argument("--turn-input", type=float, default=1.0, help="yaw input 0..1 in turns")
    flight.add_argument("--clear-at", type=float, default=4.0, help="way clear beyond this (m)")
    flight.add_argument(
        "--clear-for-s", type=float, default=0.5, help="clear on every reading for this long"
    )
    return dvd.parse_patrol_args(parser)


def main() -> None:
    dvd.fly(parse_args(), "clear", run, LEGEND)


if __name__ == "__main__":
    main()
