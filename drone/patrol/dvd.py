"""DVD patrol: fly straight, and at a wall turn by the reflection angle, like the DVD logo.

Angle of incidence = angle of reflection keeps the path sweeping the whole room instead of
pacing one line. Perception (floor_boundary.py): where the floor ends in the depth map gives
the front distance by geometry (camera height and tilt, no learned scale) and the wall's
direction; the camera height is filtered and self-calibrated against the odometry
(camera_height.py). Corners (two walls) reflect off both; when the wall's direction is not
clear, a random turn like bounce.py. Any obstacle standing on the floor counts (walls, people,
tables); the patrol slows down near one and when the view is unreadable.

Everything goes through drone.sdk (sdkdrone.py), so its safety rules apply: STOP / LAND in the
page or the API ends the patrol, link/telemetry/video loss hovers then lands, the geofence and a
low battery bring it home. The patrol hovers by itself when not flying in MANUAL, below
--min-alt, without a camera height or tilt, or without a fresh reading.

Turns: closed loop on the heading: the telemetry's measured yaw (`attitude.yaw_deg`) when the
SDK has one, else the estimator's heading (the video's yaw where trusted, else the sticks x the
calibrated yaw rate); timed with that rate if neither is there.

Run from the repo root:
  uv run --offline --no-project --with numpy --with pillow \\
      python -m drone.patrol.dvd --sim --duration 300 --plot /tmp/dvd.png   # simulated room
  (over HTTP: `python -m drone.sdk sim --frame-yaw-offset 0 --room 10`, MANUAL + /sim/button,
   then --sdk URL --sim-depth --sim-room 10)
  HF_HUB_OFFLINE=1 uv run --offline --no-project --with numpy --with pillow --with torch \\
      --with transformers python -m drone.patrol.dvd --sdk http://127.0.0.1:8077 --dry-run
Stop it with Ctrl+C (neutral + STOP), or STOP / LAND in the web page.
"""

import argparse
import contextlib
import json
import math
import random
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

import numpy as np

from drone.patrol import bounce, console
from drone.patrol.camera_height import CameraHeight, HeightCalibration
from drone.patrol.floor_boundary import (
    GEOMETRY_HFOV_DEG,
    Status,
    Wall,
    WallReading,
    measure_wall,
    reflection_turn_deg,
)
from drone.patrol.sdkdrone import DroneState, LocalDrone, PatrolDrone, Refused, Stopped
from drone.sdk import Interrupted, SafetyLanding, SafetyLimits
from drone.sdk.estimate import wrap_deg

TICK_S = bounce.TICK_S
HOVER_S = 1.5  # stop, settle and look at the wall before turning
SETTLE_S = 0.5  # readings in the first part of a hover are taken while still braking
CHECK_S = 1.0  # after a turn: readings that must all say "clear"
MIN_CHECK_READINGS = 3
CHECK_TIMEOUT_S = 4.0
EXTRA_TURN_DEG = 35.0  # still blocked after a turn: keep turning the same way
WINDOW_S = 0.6  # FORWARD stops on the nearest reading of this window (one bad frame is noise)
MAX_READING_AGE_S = 1.0
UNKNOWN_SLOW_FRACTION = 0.5  # of the window: slow down when the view is this unreadable
MIN_WALL_READINGS = 3  # in the hover, to trust the wall's direction
MAX_INCIDENCE_SPREAD_DEG = 15.0  # interquartile range of those readings
HEAD_ON_DEG = 2.0  # incidence below this: the side to turn to is a coin flip
CORNER_DEPTH_M = 1.0  # a second wall line crossing the heading this close behind the first
CALIBRATION_MAX_M = 3.5  # height calibration uses readings nearer than this
YAW_TOLERANCE_DEG = 4.0  # closed-loop turns (measured heading)
TURN_TIMEOUT_FACTOR = 2.0  # closed-loop turn gives up after this x the timed duration
DEFAULT_TILT_DEG = bounce.DEFAULT_TILT_DEG
PERCEPTION_PERIOD_S = 0.1  # simulated depth: about the model's rate on a laptop GPU
TRUTH_PERIOD_S = 0.2
TAKEOFF_TIMEOUT_S = 30.0
DEFAULT_SIM_ROOM_M = 10.0  # +-10 m: a 20 x 20 m hall


class Phase(Enum):
    FORWARD = "FORWARD"
    HOVER = "HOVER"
    TURN = "TURN"
    CHECK = "CHECK"


@dataclass(frozen=True)
class Sample:
    wall: WallReading
    at: float  # time.monotonic()
    scale: float  # height calibration applied to this reading


def front_value(wall: WallReading) -> float | None:
    """Distance to stop on: 0 when something is closer than the camera can see the floor,
    inf when clear, None when unreadable."""
    if wall.status is Status.NEAR:
        return 0.0
    if wall.status is Status.CLEAR:
        return math.inf
    return wall.front_m


@dataclass
class Readings:
    """The perception's recent results (thread-safe enough: append/read of a deque)."""

    samples: deque = field(default_factory=lambda: deque(maxlen=200))
    problem: str = "starting"  # why there is no fresh reading

    def add(self, sample: Sample) -> None:
        self.samples.append(sample)
        self.problem = ""

    def since(self, at: float) -> list[Sample]:
        return [sample for sample in list(self.samples) if sample.at >= at]

    @property
    def latest(self) -> Sample | None:
        return self.samples[-1] if self.samples else None


class Perception:
    """Depth maps in, Samples out, in a thread. The depth comes from the model on the newest
    camera frame, or (--sim-depth) is rendered from the simulator's true pose."""

    def __init__(self, drone: PatrolDrone, args: argparse.Namespace) -> None:
        self.readings = Readings()
        self.height = CameraHeight(args.takeoff_height)
        self.calibration = HeightCalibration()
        self._drone, self._args = drone, args
        self._depth = (
            _sim_depth_source(drone, args) if args.sim_depth else _model_source(drone, args)
        )
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        while True:
            depth = self._depth()
            try:
                state = self._drone.state()
            except OSError as error:
                self.readings.problem = f"SDK unreachable: {error}"
                continue
            self.height.update(state.altitude_m, state.in_flight)
            pitch = self._pitch(state)
            height = self.height.metres
            if height is None or pitch is None:
                why = "camera height unknown" if height is None else "camera tilt unknown"
                self.readings.problem = why
                continue
            scale = 1.0 if self._args.no_calibrate else self.calibration.scale
            wall = measure_wall(depth, height * scale, pitch, self._args.hfov)
            self.readings.add(Sample(wall, time.monotonic(), scale))

    def _pitch(self, state: DroneState) -> float | None:
        """Degrees below level: --pitch, the simulated camera, or the SDK's tilt estimate."""
        if self._args.pitch is not None:
            return self._args.pitch
        if self._args.sim_depth:
            return -self._args.tilt + self._args.sim_tilt_error
        return None if state.tilt_deg is None else -state.tilt_deg


def _model_source(drone: PatrolDrone, args: argparse.Namespace) -> Callable[[], np.ndarray]:
    from drone.patrol.front_distance import DepthModel, LatestFrame

    model = DepthModel(args.input_width, args.model)
    print(f"depth model {args.model} on {model.device}", flush=True)
    frames = LatestFrame(drone.frames())
    seen = [0]

    def depth() -> np.ndarray:
        seen[0], image = frames.wait_new(seen[0])
        return model.predict(image)

    return depth


def _sim_depth_source(drone: PatrolDrone, args: argparse.Namespace) -> Callable[[], np.ndarray]:
    from drone.patrol.sim_room import render_depth

    rng = np.random.default_rng(args.seed)

    def depth() -> np.ndarray:
        time.sleep(PERCEPTION_PERIOD_S)
        truth = drone.sim_truth()
        if truth is None:
            raise SystemExit("--sim-depth needs the simulator")
        pitch = -args.tilt
        return render_depth(
            args.room, truth["x"], truth["y"], truth["heading_deg"], truth["alt"], pitch,
            args.hfov, rng,
        )  # fmt: skip

    return depth


@dataclass(frozen=True)
class TurnChoice:
    degrees: float  # + = right
    how: str  # "reflect", "corner", "random" or "extra" (still blocked after a turn)
    note: str


def choose_turn(samples: list[Sample], args: argparse.Namespace, rng: random.Random) -> TurnChoice:
    """Reflect off the wall(s) seen while hovering, or a random turn if they are unclear."""
    seen = [s.wall.walls for s in samples if s.wall.walls and s.wall.status is Status.WALL]
    firsts = [walls[0].incidence_deg for walls in seen]
    if len(firsts) < MIN_WALL_READINGS:
        return _random_turn(args, rng, f"wall direction unseen ({len(firsts)} readings)")
    spread = float(np.subtract(*np.percentile(firsts, [75, 25])))
    if spread > MAX_INCIDENCE_SPREAD_DEG:
        return _random_turn(args, rng, f"wall direction unsure (spread {spread:.0f} deg)")
    walls = [Wall(float(np.median(firsts)), 0.0, len(firsts))]
    seconds = [w[1].incidence_deg for w in seen if _corner(w)]
    if len(seconds) * 2 >= len(seen):
        walls.append(Wall(float(np.median(seconds)), 0.0, len(seconds)))
    incidence = walls[0].incidence_deg
    raw = reflection_turn_deg(tuple(walls))
    side = math.copysign(1.0, raw)
    magnitude = abs(raw)
    if len(walls) == 1:
        if abs(incidence) < HEAD_ON_DEG:
            side = rng.choice((-1.0, 1.0))
        # Mirror: leave at the incidence angle off the wall's normal, but at least --min-exit,
        # or a head-on wall would send it back and forth along one line.
        exit_deg = max(abs(incidence), args.min_exit)
        magnitude = 180.0 - abs(incidence) - exit_deg
    magnitude = max(magnitude, args.min_turn)
    magnitude += rng.uniform(-args.jitter, args.jitter)
    how = "corner" if len(walls) > 1 else "reflect"
    angles = "/".join(f"{w.incidence_deg:+.0f}" for w in walls)
    return TurnChoice(side * magnitude, how, f"incidence {angles} deg -> reflection {raw:+.0f}")


def _corner(walls: tuple[Wall, ...]) -> bool:
    """A second wall right behind the first one along the heading: we are in its corner."""
    return len(walls) > 1 and walls[1].distance_m - walls[0].distance_m <= CORNER_DEPTH_M


def _random_turn(args: argparse.Namespace, rng: random.Random, why: str) -> TurnChoice:
    degrees = rng.choice((-1.0, 1.0)) * rng.uniform(args.turn_min, args.turn_max)
    return TurnChoice(degrees, "random", why)


class DvdPatrol:
    def __init__(
        self,
        args: argparse.Namespace,
        perception: Perception,
        yaw_rate: float,
        rng: random.Random,
        on_turn: Callable[[TurnChoice], None],
    ) -> None:
        """`yaw_rate`: deg/s at --yaw-input (the SDK's calibration); `on_turn`: for the log."""
        self.args, self.perception, self.yaw_rate, self.rng = args, perception, yaw_rate, rng
        self.on_turn = on_turn
        self.phase = Phase.FORWARD
        self.since = time.monotonic()
        self.until = 0.0
        self.turn: TurnChoice | None = None
        self.target_yaw: float | None = None
        self.timed_end = 0.0  # when a turn at the calibrated rate would end
        self.counts = {"reflect": 0, "corner": 0, "random": 0, "extra": 0}

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
                self._start_turn(choose_turn(self._settled(), self.args, self.rng), state)
            return {}
        if self.phase is Phase.TURN:
            return self._turning(state, now)
        return self._check(state, now)

    def _settled(self) -> list[Sample]:
        return self.perception.readings.since(self.since + SETTLE_S)

    def _forward(self, state: DroneState, now: float) -> dict[str, float]:
        window = self.perception.readings.since(now - WINDOW_S)
        values = [front_value(s.wall) for s in window]
        known = [v for v in values if v is not None]
        front = min(known) if known else None
        if front is not None and front < self.args.stop_at:
            latest = self.perception.readings.latest.wall.summary()
            self._enter(Phase.HOVER, HOVER_S, f"(front {front:.2f} m: {latest})")
            return {}
        self._calibrate(state, window)
        unsure = not known or (len(values) - len(known)) > UNKNOWN_SLOW_FRACTION * len(values)
        slow = unsure or front < self.args.slow_at
        return {"pitch": self.args.slow_speed if slow else self.args.speed}

    def _calibrate(self, state: DroneState, window: list[Sample]) -> None:
        """Straight flight at a wall: distance vs odometry travel fixes the height scale."""
        latest = window[-1] if window else None
        if latest is None or latest.wall.status is not Status.WALL or state.pose_xy is None:
            return
        if latest.wall.front_m > CALIBRATION_MAX_M:
            return  # far away a small tilt error bends the geometry: fit where stopping happens
        self.perception.calibration.add(state.pose_xy, latest.wall.front_m / latest.scale)

    def _start_turn(self, turn: TurnChoice, state: DroneState) -> None:
        self.turn = turn
        self.counts[turn.how] += 1
        calibration = self.perception.calibration
        calibration.end_leg()
        print(f"camera height scale x{calibration.scale:.2f} ({calibration.legs} legs)", flush=True)
        self.on_turn(turn)
        seconds = abs(turn.degrees) / self.yaw_rate
        heading = _heading(state)
        self.target_yaw = None if heading is None else wrap_deg(heading + turn.degrees)
        self.timed_end = time.monotonic() + seconds
        timeout = seconds * (1 if heading is None else TURN_TIMEOUT_FACTOR)
        self._enter(Phase.TURN, timeout, f"{turn.degrees:+.0f} deg ({turn.how}: {turn.note})")

    def _turning(self, state: DroneState, now: float) -> dict[str, float]:
        heading = _heading(state)
        if self.target_yaw is None or heading is None:
            done = now >= self.timed_end
        else:
            error = wrap_deg(self.target_yaw - heading)
            done = abs(error) <= YAW_TOLERANCE_DEG or error * self.turn.degrees < 0
        if done or now >= self.until:
            self._enter(Phase.CHECK, CHECK_TIMEOUT_S)
            return {}
        return {"yaw": math.copysign(self.args.yaw_input, self.turn.degrees)}

    def _check(self, state: DroneState, now: float) -> dict[str, float]:
        if now < self.since + SETTLE_S:
            return {}
        recent = self.perception.readings.since(max(self.since + SETTLE_S, now - CHECK_S))
        values = [front_value(s.wall) for s in recent]
        clear = [v is not None and v >= self.args.clear_at for v in values]
        if len(values) >= MIN_CHECK_READINGS and all(clear):
            self._enter(Phase.FORWARD)
            return {}
        blocked = any(v is not None and v < self.args.clear_at for v in values)
        if (blocked and len(values) >= MIN_CHECK_READINGS) or now >= self.until:
            side = math.copysign(1.0, self.turn.degrees) if self.turn else 1.0
            why = "still blocked" if blocked else "no clear reading"
            self._start_turn(TurnChoice(side * EXTRA_TURN_DEG, "extra", why), state)
        return {}


def _heading(state: DroneState) -> float | None:
    """Measured yaw if the telemetry has it, else the estimator's heading."""
    return state.yaw_deg if state.yaw_deg is not None else state.heading_deg


@dataclass
class SimLog:
    """Simulator runs: the true path and the turns, for the plot and the numbers."""

    path: list[tuple[float, float]] = field(default_factory=list)
    turns: list[tuple[float, float, float, str]] = field(default_factory=list)
    wall_hits: int = 0
    pillar_hits: int = 0
    _touching: bool = False

    def follow(self, drone: PatrolDrone, room: object) -> None:
        from drone.patrol.sim_room import pillar_hit

        def run() -> None:
            while True:
                with contextlib.suppress(OSError):
                    truth = drone.sim_truth()
                    if truth and truth["alt"] > 0.2:
                        self.path.append((truth["x"], truth["y"]))
                        self.wall_hits = int(truth["wall_hits"])
                        touching = pillar_hit(room, truth["x"], truth["y"])
                        self.pillar_hits += touching and not self._touching
                        self._touching = touching
                time.sleep(TRUTH_PERIOD_S)

        threading.Thread(target=run, daemon=True).start()

    def turned(self, turn: TurnChoice) -> None:
        if self.path:
            self.turns.append((*self.path[-1], turn.degrees, turn.how))


def hold_reason(state: DroneState, perception: Perception, args: argparse.Namespace) -> str | None:
    """Why the patrol must hover now; the SDK checks the rest (link, geofence, battery, STOP)."""
    if state.flight_state != bounce.FLYING_STATE:
        return f"state is {state.flight_state}, not {bounce.FLYING_STATE}"
    if state.altitude_m is None or state.altitude_m < args.min_alt:
        return f"altitude {state.altitude_m} below {args.min_alt} m"
    latest = perception.readings.latest
    if latest is None or time.monotonic() - latest.at > MAX_READING_AGE_S:
        return f"no fresh front distance ({perception.readings.problem or 'waiting'})"
    return None


def run(args: argparse.Namespace, drone: PatrolDrone, log: SimLog | None) -> str:
    if not args.sim_depth and args.pitch is None:
        print(f"camera tilt -> {args.tilt:.0f} deg (anchors the estimate first if unknown)")
        print(f"camera tilt is {drone.set_tilt(args.tilt)} deg (estimated)", flush=True)
    yaw_rate = drone.yaw_rate_dps(args.yaw_input)
    print(f"turn rate {yaw_rate:.1f} deg/s at yaw input {args.yaw_input} (SDK calibration)")
    perception = Perception(drone, args)
    rng = random.Random(args.seed)
    patrol = DvdPatrol(args, perception, yaw_rate, rng, log.turned if log else lambda _: None)
    outcome = drone.run(lambda: _loop(args, drone, perception, patrol))
    counts = ", ".join(f"{count} {how}" for how, count in patrol.counts.items())
    scale = perception.calibration
    return f"{outcome}; turns: {counts}; height scale x{scale.scale:.2f} ({scale.legs} legs)"


def _loop(
    args: argparse.Namespace, drone: PatrolDrone, perception: Perception, patrol: DvdPatrol
) -> str:
    started = time.monotonic()
    last_hold, last_print = "", 0.0
    while not args.duration or time.monotonic() - started < args.duration:
        tick = time.monotonic()
        try:
            state = drone.state()
            reason = hold_reason(state, perception, args)
        except OSError as error:
            state, reason = None, f"SDK unreachable: {error}"
        axes = {} if reason else patrol.step(state)
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
        latest = perception.readings.latest
        if latest and latest.at - last_print >= bounce.PRINT_PERIOD_S:
            last_print = latest.at
            height = perception.height.metres
            camera = "?" if height is None else f"{height * latest.scale:.2f}"
            print(f"{patrol.phase.value:7s} {latest.wall.summary()} h={camera} m", flush=True)
        time.sleep(max(0.0, TICK_S - (time.monotonic() - tick)))
    return f"{args.duration:.0f} s done"


def connect(args: argparse.Namespace) -> tuple[PatrolDrone, Callable[[], None]]:
    """bounce's connections, except the in-process simulator: a room of --sim-room with its
    takeoff frame aligned to the walls (so --sim-depth can render it)."""
    if not args.sim:
        return bounce.connect(args)
    from drone.sdk.sim import connect_sim

    limits = SafetyLimits(
        x_min=-args.geofence,
        x_max=args.geofence,
        y_min=-args.geofence,
        y_max=args.geofence,
        max_alt_m=args.max_alt,
        low_battery_pct=args.min_battery,
    )
    sim = connect_sim(
        max_speed=args.max, seed=args.seed, room_half_m=args.sim_room, frame_yaw_offset_deg=0.0
    )
    sim.enter_manual()
    sim.link.press_button()
    if not sim.wait_for_takeoff(TAKEOFF_TIMEOUT_S):
        raise SystemExit("no takeoff")

    def close() -> None:
        print(f"landing: {sim.land()}", flush=True)
        sim.close()

    return LocalDrone(sim, limits), close


def report(args: argparse.Namespace, log: SimLog) -> None:
    from drone.patrol.path_plot import coverage, draw, path_length

    covered = coverage(args.room, log.path)
    summary = (
        f"{len(log.path) * TRUTH_PERIOD_S:.0f} s, {path_length(log.path):.0f} m flown, "
        f"{len(log.turns)} turns, coverage {covered:.0%}, wall hits {log.wall_hits}, "
        f"pillar hits {log.pillar_hits}"
    )
    print(f"sim: {summary}", flush=True)
    if args.log:
        args.log.write_text(json.dumps({"path": log.path, "turns": log.turns, "summary": summary}))
    if args.plot:
        title = f"dvd patrol, {2 * args.sim_room:.0f} m room: {summary}"
        draw(args.room, log.path, log.turns, args.plot, title)
        print(f"plot: {args.plot}", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    where = parser.add_mutually_exclusive_group()
    where.add_argument("--sdk", default=bounce.DEFAULT_URL, help="SDK server (%(default)s)")
    where.add_argument("--sim", action="store_true", help="in-process simulator (+ --sim-depth)")
    where.add_argument("--local", action="store_true", help="real drone in this process")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds, 0 = until stopped")
    parser.add_argument("--dry-run", action="store_true", help="decide and print, send nothing")
    flight = parser.add_argument_group("flight")
    flight.add_argument("--speed", type=float, default=0.6, help="forward input 0..1 of the cap")
    flight.add_argument("--slow-speed", type=float, default=0.3, help="near walls / unsure")
    flight.add_argument("--slow-at", type=float, default=3.0, help="slow down below this (m)")
    flight.add_argument("--stop-at", type=float, default=1.5, help="stop and turn below (m)")
    flight.add_argument("--clear-at", type=float, default=2.5, help="go on above this (m)")
    flight.add_argument("--yaw-input", type=float, default=1.0, help="yaw input 0..1 in turns")
    flight.add_argument("--min-turn", type=float, default=60.0, help="smallest turn, deg")
    flight.add_argument(
        "--min-exit", type=float, default=40.0, help="leave at least this far off the wall normal"
    )
    flight.add_argument("--jitter", type=float, default=5.0, help="random +- deg on each turn")
    flight.add_argument("--turn-min", type=float, default=100.0, help="fallback random turn")
    flight.add_argument("--turn-max", type=float, default=170.0, help="fallback random turn")
    flight.add_argument("--min-alt", type=float, default=0.5, help="m above takeoff: hover below")
    camera = parser.add_argument_group("camera and perception")
    camera.add_argument(
        "--takeoff-height",
        type=float,
        default=0.0,
        help="m above the floor where the drone took off (0 = floor, ~1 from a hand or table)",
    )
    camera.add_argument("--no-calibrate", action="store_true", help="trust the height as given")
    camera.add_argument("--tilt", type=float, default=DEFAULT_TILT_DEG, help="camera tilt, deg")
    camera.add_argument("--pitch", type=float, help="camera down, deg: overrides the estimate")
    camera.add_argument("--hfov", type=float, default=GEOMETRY_HFOV_DEG, help="horizontal FOV")
    camera.add_argument("--model", choices=("small", "base"), default="small", help="depth model")
    camera.add_argument("--input-width", type=int, default=392, help="model input, px")
    local = parser.add_argument_group("--sim / --local only (with --sdk the server's limits apply)")
    local.add_argument("--max", type=float, default=60.0, help="speed cap, stick %%")
    local.add_argument("--geofence", type=float, default=14.0, help="box half-size, m")
    local.add_argument("--max-alt", type=float, default=2.5, help="m above takeoff")
    local.add_argument("--min-battery", type=int, default=30, help="%% to return home and land")
    sim = parser.add_argument_group("simulator")
    sim.add_argument("--sim-depth", action="store_true", help="render the sim room (on with --sim)")
    sim.add_argument("--sim-room", type=float, default=DEFAULT_SIM_ROOM_M, help="walls at +-m")
    sim.add_argument("--pillars", action="store_true", help="add people-sized pillars")
    sim.add_argument("--sim-tilt-error", type=float, default=0.0, help="deg, perception's tilt")
    sim.add_argument("--log", type=Path, help="write the sim path and turns as JSON")
    sim.add_argument("--plot", type=Path, help="draw the sim path as a PNG")
    parser.add_argument("--seed", type=int, help="repeatable runs")
    args = parser.parse_args()
    args.sim_depth = args.sim_depth or args.sim
    return args


def main() -> None:
    args = parse_args()
    console.install("dvd_dry" if args.dry_run else "dvd")
    log = None
    if args.sim_depth:
        from drone.patrol.sim_room import Room, demo_pillars

        pillars = demo_pillars(args.sim_room) if args.pillars else ()
        args.room = Room(args.sim_room, pillars)
        log = SimLog()
    drone, close = connect(args)
    if log:
        log.follow(drone, args.room)
    try:
        print(f"patrol ended: {run(args, drone, log)}", flush=True)
    except (KeyboardInterrupt, Stopped, Interrupted) as reason:
        if not args.dry_run:
            with contextlib.suppress(OSError):
                drone.stop()  # neutral, and the SDK's STOP
        print(f"\nstopped ({type(reason).__name__}): sticks neutral", flush=True)
    except SafetyLanding as reason:
        print(f"safety landing: {reason}", flush=True)
    finally:
        if log:
            report(args, log)
        close()


if __name__ == "__main__":
    main()
