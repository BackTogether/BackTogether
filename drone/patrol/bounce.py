"""Bounce patrol: fly straight until a wall is close, turn a random angle, repeat.

A random bounce ("random billiard") keeps covering a room without a map or a heading, and its
path is unpredictable for a target that moves. The front distance comes from the camera
(drone.patrol.front_distance); the sticks go through the hoverctl web app, which keeps its own
--max cap and watchdog. Safety: any stale input, wrong flight state, altitude out of range or
leaving the geofence makes the drone hover; low battery lands it.

Stop it with Ctrl+C (neutral + STOP) or LAND in the web app. The web app's STOP alone is not
enough: the next patrol tick would override it.

Run from the repo root, with the hoverctl web app already flying in MANUAL:
  uv run --no-project --with numpy --with pillow --with torch --with transformers \
      python -m drone.patrol.bounce --dry-run
"""

import argparse
import contextlib
import math
import random
import threading
import time
from dataclasses import dataclass
from enum import Enum

from drone.patrol.front_distance import (
    DEFAULT_HFOV_DEG,
    DEFAULT_PITCH_DEG,
    DepthModel,
    FrontReading,
    LatestFrame,
    format_reading,
    measure_front,
)
from drone.patrol.hoverweb import DEFAULT_URL, DroneState, HoverWeb

TICK_S = 0.1  # sticks every 100 ms; the web app goes neutral after 300 ms without input
HOVER_S = 1.0  # settle before turning and before checking
EXTRA_TURN_DEG = 30.0  # when the way is still blocked after a turn
MAX_STATE_AGE_S = 2.0
MAX_READING_AGE_S = 1.0
FLYING_STATE = "flying_manual"


class Phase(Enum):
    FORWARD = "FORWARD"
    HOVER = "HOVER"
    TURN = "TURN"
    CHECK = "CHECK"


@dataclass
class Reading:
    front: FrontReading
    at: float  # time.monotonic()


class Perception:
    """Runs the depth model on the newest frame in a thread; keeps the last reading."""

    def __init__(self, web: HoverWeb, args: argparse.Namespace) -> None:
        self.latest: Reading | None = None
        self._web, self._args = web, args
        self._model = DepthModel()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        frames = LatestFrame(self._web.frames())
        seen = 0
        while True:
            seen, image = frames.wait_new(seen)
            try:
                state = self._web.state()
            except OSError:
                continue  # the control loop holds on stale readings
            if not state.in_flight or state.altitude_m is None:
                continue
            height = state.altitude_m + self._args.takeoff_height
            front = measure_front(
                self._model.predict(image), height, self._args.pitch, self._args.hfov
            )
            self.latest = Reading(front, time.monotonic())


def hold_reason(state: DroneState, reading: Reading | None, args: argparse.Namespace) -> str | None:
    """Why the patrol must hover instead of moving, or None if it may move."""
    if state.state_age_s is None or state.state_age_s > MAX_STATE_AGE_S:
        return "flight state stale"
    if state.flight_state != FLYING_STATE:
        return f"state is {state.flight_state}, not {FLYING_STATE}"
    if state.altitude_m is None or not args.min_alt <= state.altitude_m <= args.max_alt:
        return f"altitude {state.altitude_m} outside [{args.min_alt}, {args.max_alt}] m"
    if state.position and math.hypot(*state.position[:2]) > args.geofence:
        return f"outside geofence ({args.geofence} m from takeoff)"
    if reading is None or time.monotonic() - reading.at > MAX_READING_AGE_S:
        return "no fresh front distance"
    return None


class BouncePatrol:
    def __init__(self, args: argparse.Namespace, rng: random.Random) -> None:
        self.args, self.rng = args, rng
        self.phase = Phase.FORWARD
        self.until = 0.0  # end of the current timed phase
        self.turn_sign = 1.0

    def _enter(self, phase: Phase, seconds: float = 0.0, note: str = "") -> None:
        self.phase, self.until = phase, time.monotonic() + seconds
        print(f"-> {phase.value} {note}".rstrip(), flush=True)

    def _turn(self, degrees: float) -> None:
        note = f"{self.turn_sign * degrees:+.0f} deg"
        self._enter(Phase.TURN, degrees / self.args.yaw_rate, note)

    def step(self, reading: Reading) -> dict[str, float]:
        """Advance the state machine; return the stick axes for this tick."""
        now, front = time.monotonic(), reading.front.front_m
        blocked = front is not None and front < self.args.stop_at
        if self.phase is Phase.FORWARD:
            if blocked:
                self._enter(Phase.HOVER, HOVER_S, f"(wall at {front:.2f} m)")
                return {}
            return {"pitch": self.args.speed}
        if now < self.until:
            if self.phase is Phase.TURN:
                return {"yaw": self.turn_sign * self.args.yaw_input}
            return {}
        if self.phase is Phase.HOVER:
            self.turn_sign = self.rng.choice((-1.0, 1.0))
            self._turn(self.rng.uniform(self.args.turn_min, self.args.turn_max))
        elif self.phase is Phase.TURN:
            self._enter(Phase.CHECK, HOVER_S)
        elif reading.at < self.until:
            return {}  # CHECK: wait for a reading taken after settling
        elif front is None or front > self.args.clear_at:
            self._enter(Phase.FORWARD)
        else:
            self._turn(EXTRA_TURN_DEG)
        return {}


def run(args: argparse.Namespace) -> None:
    web = HoverWeb(args.web)
    perception = Perception(web, args)
    patrol = BouncePatrol(args, random.Random(args.seed))
    last_hold, last_print, landing = "", 0.0, False
    while True:
        tick = time.monotonic()
        reading = perception.latest
        try:
            state = web.state()
            if state.battery_pct is not None and state.battery_pct < args.min_battery:
                landing = landing or start_landing(web, state, args.dry_run)
            reason = "landing" if landing else hold_reason(state, reading, args)
        except OSError as error:
            reason = f"web app unreachable: {error}"
        axes = {} if reason else patrol.step(reading)
        if reason and reason != last_hold:
            print(f"HOLD: {reason}", flush=True)
        last_hold = reason or ""
        if reading and reading.at > last_print:
            last_print = reading.at
            print(f"{patrol.phase.value:7s} {format_reading(reading.front)}", flush=True)
        if not args.dry_run:
            # On a network error the web app's watchdog goes neutral by itself; next tick retries.
            with contextlib.suppress(OSError):
                web.send_sticks(**axes)
        time.sleep(max(0.0, TICK_S - (time.monotonic() - tick)))


def start_landing(web: HoverWeb, state: DroneState, dry_run: bool) -> bool:
    print(f"battery {state.battery_pct} %: LANDING", flush=True)
    if not dry_run:
        web.land()
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--web", default=DEFAULT_URL, help="hoverctl web app URL")
    parser.add_argument("--dry-run", action="store_true", help="decide and print, send nothing")
    parser.add_argument("--speed", type=float, default=0.5, help="forward input 0..1 of --max")
    parser.add_argument("--stop-at", type=float, default=1.0, help="wall distance to stop (m)")
    parser.add_argument("--clear-at", type=float, default=1.8, help="free distance to go (m)")
    parser.add_argument("--turn-min", type=float, default=100.0, help="random turn, deg")
    parser.add_argument("--turn-max", type=float, default=170.0, help="random turn, deg")
    parser.add_argument("--yaw-input", type=float, default=0.5, help="yaw input 0..1 for turns")
    parser.add_argument("--yaw-rate", type=float, default=45.0, help="deg/s at --yaw-input (T1)")
    parser.add_argument("--takeoff-height", type=float, default=1.0, help="takeoff m above floor")
    parser.add_argument("--pitch", type=float, default=DEFAULT_PITCH_DEG, help="camera down, deg")
    parser.add_argument("--hfov", type=float, default=DEFAULT_HFOV_DEG, help="horizontal FOV")
    parser.add_argument("--geofence", type=float, default=6.0, help="max m from takeoff")
    parser.add_argument("--min-alt", type=float, default=0.5, help="m above takeoff")
    parser.add_argument("--max-alt", type=float, default=2.5, help="m above takeoff")
    parser.add_argument("--min-battery", type=int, default=30, help="% to land")
    parser.add_argument("--seed", type=int, help="fix the random turns (tests)")
    args = parser.parse_args()

    web = HoverWeb(args.web)
    try:
        run(args)
    except KeyboardInterrupt:
        if not args.dry_run:
            web.send_sticks()
            web.stop()
        print("\nstopped: sticks neutral", flush=True)


if __name__ == "__main__":
    main()
