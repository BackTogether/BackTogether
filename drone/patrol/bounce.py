"""Bounce patrol: fly straight until a wall is close, turn a random angle, repeat.

A random bounce ("random billiard") keeps covering a room without a map or a heading, and its
path is unpredictable for a target that moves. The front distance comes from the camera
(drone.patrol.front_distance), or with --fake-depth from the simulator's walls. Everything goes
through drone.sdk (drone.patrol.sdkdrone): frames, state, altitude, the camera tilt estimate and
the sticks, so the SDK's safety applies: STOP (web page, API, a pilot key) ends the patrol,
link/telemetry/video loss hovers then lands, leaving the geofence or a low battery returns to
the takeoff point and lands. The patrol itself hovers while the drone is not flying in MANUAL,
is below --min-alt, the camera tilt is unknown or the front distance is stale.

Where the drone is:
  --sdk URL   the SDK server (drone/fly, `python -m drone.sdk serve|sim`), here or elsewhere
  --sim       an in-process simulator: takes off by itself; use with --fake-depth
  --local     the real drone in this process (needs `--with bleak`; press its button)

Run from the repo root, e.g.
  uv run python -m drone.patrol.bounce --sim --fake-depth --duration 120      # no model, no drone
  (over HTTP: `python -m drone.sdk sim --room 4`, MANUAL + /sim/button, then --sdk URL)
  uv run --no-project --with numpy --with pillow --with torch --with transformers \\
      python -m drone.patrol.bounce --sdk http://127.0.0.1:8077 --dry-run    # real drone, drone/fly
Stop it with Ctrl+C (neutral + STOP), or STOP / LAND in the web page.
"""

import argparse
import contextlib
import random
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum

from drone.patrol import console
from drone.patrol.sdkdrone import (
    DEFAULT_URL,
    DroneState,
    LocalDrone,
    PatrolDrone,
    Refused,
    RemoteDrone,
    Stopped,
)
from drone.sdk import Interrupted, SafetyLanding, SafetyLimits
from drone.sdk.camera import CAMERA_HFOV_DEG

TICK_S = 0.1  # sticks every 100 ms; each command lapses after 300 ms (sdkdrone.STICK_HOLD_S)
HOVER_S = 1.0  # settle before turning and before checking
EXTRA_TURN_DEG = 30.0  # when the way is still blocked after a turn
MAX_READING_AGE_S = 1.0
FLYING_STATE = "flying_manual"
DEFAULT_TILT_DEG = -30.0  # floor in the lower part, room ahead in the upper (-45 sees mostly floor)
FAKE_DEPTH_PERIOD_S = 0.2  # --fake-depth readings, about as often as the model on a laptop
PRINT_PERIOD_S = 1.0
TAKEOFF_TIMEOUT_S = 30.0
SIM_ROOM_HALF_M = 4.0  # --sim: walls at +-4 m, inside the default --geofence


class Phase(Enum):
    FORWARD = "FORWARD"
    HOVER = "HOVER"
    TURN = "TURN"
    CHECK = "CHECK"


@dataclass
class Reading:
    front_m: float | None  # None = nothing in front within view
    summary: str  # for the log
    at: float  # time.monotonic()


class DepthPerception:
    """Runs the depth model on the newest frame in a thread; keeps the last reading.

    Camera height = SDK altitude + --takeoff-height; camera pitch = the SDK's live tilt
    estimate (or --pitch); HFOV = the measured one (drone.sdk.camera).
    """

    def __init__(self, drone: PatrolDrone, args: argparse.Namespace) -> None:
        from drone.patrol.front_distance import DepthModel  # numpy, pillow, torch: only here

        self.latest: Reading | None = None
        self._drone, self._args = drone, args
        self._model = DepthModel()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        from drone.patrol.front_distance import LatestFrame, format_reading, measure_front

        frames = LatestFrame(self._drone.frames())
        seen = 0
        while True:
            seen, image = frames.wait_new(seen)
            try:
                state = self._drone.state()
            except OSError:
                continue  # the control loop holds on stale readings
            pitch = self._args.pitch if self._args.pitch is not None else _down(state.tilt_deg)
            if not state.in_flight or state.altitude_m is None or pitch is None:
                continue
            height = state.altitude_m + self._args.takeoff_height
            front = measure_front(self._model.predict(image), height, pitch, self._args.hfov)
            self.latest = Reading(front.front_m, format_reading(front), time.monotonic())


class FakePerception:
    """The simulator's true distance to the wall ahead, instead of the depth model."""

    def __init__(self, drone: PatrolDrone) -> None:
        self.latest: Reading | None = None
        self._drone = drone
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        while True:
            with contextlib.suppress(OSError):
                front = self._drone.fake_front_m()
                summary = "front=no simulator" if front is None else f"front={front:5.2f} m (sim)"
                self.latest = Reading(front, summary, time.monotonic())
            time.sleep(FAKE_DEPTH_PERIOD_S)


def _down(tilt_deg: float | None) -> float | None:
    """front_distance's pitch (deg below level) from the SDK's tilt (deg, down negative)."""
    return None if tilt_deg is None else -tilt_deg


def hold_reason(state: DroneState, reading: Reading | None, args: argparse.Namespace) -> str | None:
    """Why the patrol must hover instead of moving, or None if it may move. The SDK checks the
    rest (link, telemetry and video loss, geofence, ceiling, battery, STOP)."""
    if state.flight_state != FLYING_STATE:
        return f"state is {state.flight_state}, not {FLYING_STATE}"
    if state.altitude_m is None or state.altitude_m < args.min_alt:
        return f"altitude {state.altitude_m} below {args.min_alt} m"
    if not args.fake_depth and args.pitch is None and state.tilt_deg is None:
        return "camera tilt unknown (hold U or O to a stop, or restart to re-anchor it)"
    if reading is None or time.monotonic() - reading.at > MAX_READING_AGE_S:
        return "no fresh front distance"
    return None


class BouncePatrol:
    def __init__(self, args: argparse.Namespace, rng: random.Random, yaw_rate: float) -> None:
        """`yaw_rate`: deg/s at --yaw-input, from the SDK estimator's calibration."""
        self.args, self.rng, self.yaw_rate = args, rng, yaw_rate
        self.phase = Phase.FORWARD
        self.until = 0.0  # end of the current timed phase
        self.turn_sign = 1.0
        self.turns = 0

    def _enter(self, phase: Phase, seconds: float = 0.0, note: str = "") -> None:
        self.phase, self.until = phase, time.monotonic() + seconds
        print(f"-> {phase.value} {note}".rstrip(), flush=True)

    def _turn(self, degrees: float) -> None:
        self.turns += 1
        note = f"{self.turn_sign * degrees:+.0f} deg"
        self._enter(Phase.TURN, degrees / self.yaw_rate, note)

    def step(self, reading: Reading) -> dict[str, float]:
        """Advance the state machine; return the stick axes for this tick."""
        now, front = time.monotonic(), reading.front_m
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


def run(args: argparse.Namespace, drone: PatrolDrone) -> str:
    """Set the camera tilt, then bounce until --duration, STOP or a safety rule."""
    if not args.fake_depth and args.pitch is None:
        print(f"camera tilt -> {args.tilt:.0f} deg (anchors the estimate first if unknown)")
        print(f"camera tilt is {drone.set_tilt(args.tilt)} deg (estimated)", flush=True)
    yaw_rate = drone.yaw_rate_dps(args.yaw_input)
    print(f"turn rate {yaw_rate:.1f} deg/s at yaw input {args.yaw_input} (SDK calibration)")
    perception = FakePerception(drone) if args.fake_depth else DepthPerception(drone, args)
    patrol = BouncePatrol(args, random.Random(args.seed), yaw_rate)
    outcome = drone.run(lambda: _loop(args, drone, perception, patrol))
    return f"{outcome}; {patrol.turns} turns"


def _loop(
    args: argparse.Namespace,
    drone: PatrolDrone,
    perception: DepthPerception | FakePerception,
    patrol: BouncePatrol,
) -> str:
    started = time.monotonic()
    last_hold, last_print = "", 0.0
    while not args.duration or time.monotonic() - started < args.duration:
        tick = time.monotonic()
        reading = perception.latest
        try:
            reason = hold_reason(drone.state(), reading, args)
        except OSError as error:
            reason = f"SDK unreachable: {error}"
        axes = {} if reason else patrol.step(reading)
        if not args.dry_run:
            try:
                drone.send_sticks(**axes)
            except Stopped:
                raise
            except Refused as why:
                reason = f"SDK: {why}"
            except OSError:
                pass  # the SDK's watchdog goes neutral by itself; next tick retries
        if reason and reason != last_hold:
            print(f"HOLD: {reason}", flush=True)
        last_hold = reason or ""
        if reading and reading.at - last_print >= PRINT_PERIOD_S:
            last_print = reading.at
            print(f"{patrol.phase.value:7s} {reading.summary}", flush=True)
        time.sleep(max(0.0, TICK_S - (time.monotonic() - tick)))
    return f"{args.duration:.0f} s done"


def connect(args: argparse.Namespace) -> tuple[PatrolDrone, Callable[[], None]]:
    """The drone access and what to do at exit (in-process drones land: nothing else would
    fly them after this program)."""
    if not (args.sim or args.local):
        return RemoteDrone(args.sdk), lambda: None
    limits = SafetyLimits(
        x_min=-args.geofence,
        x_max=args.geofence,
        y_min=-args.geofence,
        y_max=args.geofence,
        max_alt_m=args.max_alt,
        low_battery_pct=args.min_battery,
    )
    if args.sim:
        from drone.sdk.sim import connect_sim

        sdk_drone = connect_sim(max_speed=args.max, seed=args.seed, room_half_m=SIM_ROOM_HALF_M)
        sdk_drone.enter_manual()
        sdk_drone.link.press_button()
    else:
        from drone.sdk import Drone

        sdk_drone = Drone.connect(max_speed=args.max)
        if not sdk_drone.wait_connected():
            raise SystemExit("drone not found over Bluetooth")
        print(f"manual: {sdk_drone.enter_manual()}; press the drone's button to take off")
    if not sdk_drone.wait_for_takeoff(TAKEOFF_TIMEOUT_S if args.sim else None):
        raise SystemExit("no takeoff")

    def close() -> None:
        print(f"landing: {sdk_drone.land()}", flush=True)
        sdk_drone.close()

    return LocalDrone(sdk_drone, limits), close


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    where = parser.add_mutually_exclusive_group()
    where.add_argument("--sdk", default=DEFAULT_URL, help="SDK server URL (default %(default)s)")
    where.add_argument("--sim", action="store_true", help="in-process simulator")
    where.add_argument("--local", action="store_true", help="real drone in this process")
    parser.add_argument("--fake-depth", action="store_true", help="simulator walls, no model")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds, 0 = until stopped")
    parser.add_argument("--dry-run", action="store_true", help="decide and print, send nothing")
    parser.add_argument("--speed", type=float, default=0.5, help="forward input 0..1 of the cap")
    parser.add_argument("--stop-at", type=float, default=1.0, help="wall distance to stop (m)")
    parser.add_argument("--clear-at", type=float, default=1.8, help="free distance to go (m)")
    parser.add_argument("--turn-min", type=float, default=100.0, help="random turn, deg")
    parser.add_argument("--turn-max", type=float, default=170.0, help="random turn, deg")
    parser.add_argument("--yaw-input", type=float, default=0.5, help="yaw input 0..1 for turns")
    parser.add_argument("--takeoff-height", type=float, default=1.0, help="takeoff m above floor")
    parser.add_argument(
        "--tilt", type=float, default=DEFAULT_TILT_DEG, help="camera tilt to set first, deg"
    )
    parser.add_argument(
        "--pitch", type=float, help="camera down, deg: overrides the SDK's tilt estimate"
    )
    parser.add_argument("--hfov", type=float, default=CAMERA_HFOV_DEG, help="horizontal FOV")
    parser.add_argument("--min-alt", type=float, default=0.5, help="m above takeoff: hover below")
    local = parser.add_argument_group("--sim / --local only (with --sdk the server's limits apply)")
    local.add_argument("--max", type=float, default=60.0, help="speed cap, stick %%")
    local.add_argument("--geofence", type=float, default=6.0, help="box half-size, m")
    local.add_argument("--max-alt", type=float, default=2.5, help="m above takeoff")
    local.add_argument("--min-battery", type=int, default=30, help="%% to return home and land")
    parser.add_argument("--seed", type=int, help="fix the random turns (tests)")
    args = parser.parse_args()
    console.install("bounce_dry" if args.dry_run else "bounce")

    drone, close = connect(args)
    try:
        print(f"patrol ended: {run(args, drone)}", flush=True)
    except (KeyboardInterrupt, Stopped, Interrupted) as reason:
        if not args.dry_run:
            with contextlib.suppress(OSError):
                drone.stop()  # neutral, and the SDK's STOP
        print(f"\nstopped ({type(reason).__name__}): sticks neutral", flush=True)
    except SafetyLanding as reason:
        print(f"safety landing: {reason}", flush=True)
    finally:
        close()


if __name__ == "__main__":
    main()
