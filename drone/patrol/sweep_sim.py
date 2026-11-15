"""Run the sweep (sweep.py): connections, Ctrl+C / STOP handling, and simulator runs with the
numbers that matter in a hall: coverage vs time, distance, overlap, wall and pillar hits, lane
straightness and spacing, plus a plot of the path over the coverage map.

--sim: the in-process simulator in a rectangular hall (--sim-hall LENGTH WIDTH, the start in its
middle facing along the length), the depth rendered from the true pose (sim_room.py). Heading
errors to test with: --sim-frame-offset (the odometry frame turned from the start heading: the
SDK heading starts that far off) and --sim-yaw-error (the stick model's yaw rate off by that
fraction; the sim has no video yaw, so turns use the stick model).
"""

import argparse
import contextlib
import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from drone.patrol import bounce, console
from drone.patrol.coverage import SEEN_MAX_M, CameraView, CoverageMap
from drone.patrol.sdkdrone import LocalDrone, PatrolDrone, Stopped
from drone.patrol.sim_room import Room, hall_pillars, pillar_hit
from drone.patrol.sweep_patrol import SweepPatrol
from drone.sdk import Drone, Interrupted, SafetyLanding, SafetyLimits

TRUTH_PERIOD_S = 0.2
COVERAGE_EVERY_S = 60.0  # coverage vs time, in the report
LANE_TRIM_M = 1.5  # lane straightness: leave out the ends (turn, braking)
MIN_LANE_M = 4.0  # shorter lanes (blocked early) are not measured
SAME_LANE_M = 1.0  # lanes closer than this across are the same lane flown twice
MIN_FLYING_ALT_M = 0.2
FENCE_MARGIN_M = 2.0  # --sim: the geofence just outside the hall's corners
TAKEOFF_TIMEOUT_S = 30.0
LEGEND = (
    "cells: red unseen, light green 1 lane, green 2+ lanes; dots (lane no.): green wall, "
    "red obstacle, grey revisit; path blue->orange; +y (the drone's right) is up"
)
RunSweep = Callable[[argparse.Namespace, PatrolDrone, Callable[[SweepPatrol], None] | None], str]


class HallDrone(LocalDrone):
    """The in-process simulator, with sim_truth() in hall coordinates (start point at 0, 0,
    facing +x): its odometry frame is turned by --sim-frame-offset from them."""

    def __init__(self, drone: Drone, limits: SafetyLimits, frame_offset_deg: float) -> None:
        super().__init__(drone, limits)
        self.frame_offset_deg = frame_offset_deg

    def sim_truth(self) -> dict[str, float] | None:
        truth = super().sim_truth()
        if truth is None:
            return None
        turn = math.radians(self.frame_offset_deg)
        x, y = truth["x"], truth["y"]
        return {
            **truth,
            "x": x * math.cos(turn) - y * math.sin(turn),
            "y": x * math.sin(turn) + y * math.cos(turn),
            "heading_deg": truth["heading_deg"] + self.frame_offset_deg,
        }


def hall(args: argparse.Namespace) -> Room:
    length, width = args.sim_hall or (2 * args.sim_room, 2 * args.sim_room)
    pillars = hall_pillars(length / 2, width / 2) if args.pillars else ()
    return Room(length / 2, pillars, width / 2)


def connect(args: argparse.Namespace) -> tuple[PatrolDrone, Callable[[], None]]:
    """bounce's connections (--sdk, --local), or the simulated hall (--sim)."""
    if not args.sim:
        return bounce.connect(args)
    from drone.sdk.estimate import DeadReckoner
    from drone.sdk.sim import TRUE_YAW_RATE_DPS, connect_sim

    room: Room = args.room
    fence = max(args.geofence, math.hypot(room.half_m, room.half_y) + FENCE_MARGIN_M)
    limits = SafetyLimits(
        x_min=-fence,
        x_max=fence,
        y_min=-fence,
        y_max=fence,
        max_alt_m=args.max_alt,
        low_battery_pct=args.min_battery,
    )
    estimator = DeadReckoner(TRUE_YAW_RATE_DPS / (1 + args.sim_yaw_error))
    sim = connect_sim(
        max_speed=args.max,
        estimator=estimator,
        frame_yaw_offset_deg=args.sim_frame_offset,
        seed=args.seed,
        room_half_m=room.half_m,
        room_half_y_m=room.half_y,
    )
    sim.link.set_battery(100.0)  # a long sweep; the drain is not what is tested here
    sim.enter_manual()
    sim.link.press_button()
    if not sim.wait_for_takeoff(TAKEOFF_TIMEOUT_S):
        raise SystemExit("no takeoff")

    def close() -> None:
        print(f"landing: {sim.land()}", flush=True)
        sim.close()

    return HallDrone(sim, limits, args.sim_frame_offset), close


@dataclass
class Sample:
    t: float
    x: float
    y: float
    phase: str
    leg: int


@dataclass
class Tracker:
    """The true flight (hall coordinates), sampled in a thread while the sweep runs."""

    drone: PatrolDrone
    room: Room
    args: argparse.Namespace
    cover: CoverageMap = field(default_factory=CoverageMap)
    samples: list[Sample] = field(default_factory=list)
    marks: list[tuple[float, float, float, str]] = field(default_factory=list)
    coverage_at: list[tuple[float, float]] = field(default_factory=list)
    wall_hits: int = 0
    pillar_hits: int = 0
    _touching: bool = False

    @property
    def box(self) -> tuple[float, float, float, float]:
        return -self.room.half_m, self.room.half_m, -self.room.half_y, self.room.half_y

    def watch(self, patrol: SweepPatrol) -> None:
        threading.Thread(target=self._run, args=(patrol,), daemon=True).start()

    def _run(self, patrol: SweepPatrol) -> None:
        started = time.monotonic()
        next_report, last_phase = COVERAGE_EVERY_S, ""
        while True:
            time.sleep(TRUTH_PERIOD_S)
            truth = self.drone.sim_truth()
            if truth is None or truth["alt"] < MIN_FLYING_ALT_M:
                continue
            t = time.monotonic() - started
            phase = patrol.plan[0].phase.value if patrol.plan else "DONE"
            x, y = truth["x"], truth["y"]
            self.samples.append(Sample(t, x, y, phase, patrol.legs))  # legs: one per lane
            self._mark_event(last_phase, phase, x, y, patrol.lanes)
            last_phase = phase
            view = CameraView(
                truth["alt"] + self.args.takeoff_height, -self.args.tilt, self.args.hfov
            )
            self.cover.mark(x, y, truth["heading_deg"], view, SEEN_MAX_M, patrol.lanes)
            self.wall_hits = int(truth["wall_hits"])
            touching = pillar_hit(self.room, x, y)
            if touching and not self._touching:
                self.pillar_hits += 1
                nearest = min(math.hypot(x - p.x, y - p.y) for p in self.room.pillars)
                print(f"sim: pillar contact at ({x:+.1f}, {y:+.1f}) in {phase}, {nearest:.2f} m")
            self._touching = touching
            if t >= next_report:
                self.coverage_at.append((t, self.cover.fraction(self.box)))
                next_report += COVERAGE_EVERY_S

    def _mark_event(self, before: str, now: str, x: float, y: float, lane: int) -> None:
        how = {("LANE", "HOVER"): "wall", ("LANE", "WAIT"): "obstacle", ("TURN", "REVISIT"): "side"}
        if (before, now) in how:
            self.marks.append((x, y, lane, how[(before, now)]))

    def report(self, out: Path | None) -> None:
        if not self.samples:
            print("sim: no flight recorded", flush=True)
            return
        path = [(s.x, s.y) for s in self.samples]
        flown = sum(math.dist(a, b) for a, b in zip(path, path[1:], strict=False))
        lanes = lane_lines(self.samples)
        rms = [lane.rms_m for lane in lanes]
        spacings = np.diff(_distinct(sorted(lane.mean_y for lane in lanes)))
        timeline = ", ".join(f"{t / 60:.0f} min {c:.0%}" for t, c in self.coverage_at)
        summary = (
            f"{self.samples[-1].t:.0f} s, {flown:.0f} m flown, coverage "
            f"{self.cover.fraction(self.box):.0%}, overlap {self.cover.overlap(self.box):.0%}, "
            f"wall hits {self.wall_hits}, pillar hits {self.pillar_hits}"
        )
        straight = (
            f"{len(lanes)} lanes >= {MIN_LANE_M:.0f} m: off-line rms median "
            f"{np.median(rms):.2f} m, max {max(lane.max_m for lane in lanes):.2f} m; "
            f"direction error max {max(abs(lane.angle_deg) for lane in lanes):.1f} deg; "
            f"spacing {', '.join(f'{d:.1f}' for d in spacings) or '-'} m"
            if lanes
            else "no lane long enough to measure"
        )
        print(f"sim: {summary}", flush=True)
        print(f"sim: coverage vs time: {timeline}", flush=True)
        print(f"sim: {straight}", flush=True)
        if out:
            from drone.patrol.path_plot import draw

            hall = f"sweep, {2 * self.room.half_m:.0f} x {2 * self.room.half_y:.0f} m hall"
            title = f"{hall}, start (square) facing +x (right): {summary}\n{straight}"
            draw(self.room, path, self.marks, out, title, LEGEND, self.cover)
            print(f"plot: {out}", flush=True)


@dataclass(frozen=True)
class LaneLine:
    mean_y: float
    angle_deg: float  # off the hall's length axis
    rms_m: float  # true path off its own straight line
    max_m: float


def lane_lines(samples: list[Sample]) -> list[LaneLine]:
    """Straightness of every lane long enough, from the true path."""
    lines = []
    for leg in sorted({s.leg for s in samples if s.phase == "LANE"}):
        points = np.array([(s.x, s.y) for s in samples if s.leg == leg and s.phase == "LANE"])
        if len(points) < 2 or math.dist(points[0], points[-1]) < MIN_LANE_M:
            continue
        direction = (points[-1] - points[0]) / math.dist(points[0], points[-1])
        along = (points - points[0]) @ direction
        keep = (along > LANE_TRIM_M) & (along < along[-1] - LANE_TRIM_M)
        part = points[keep]
        if len(part) < 2:
            continue
        slope, intercept = np.polyfit(part[:, 0], part[:, 1], 1)
        off = (part[:, 1] - (slope * part[:, 0] + intercept)) / math.hypot(1, slope)
        lines.append(
            LaneLine(
                float(part[:, 1].mean()),
                math.degrees(math.atan(slope)),
                float(np.sqrt(np.mean(off**2))),
                float(np.abs(off).max()),
            )
        )
    return lines


def _distinct(values: list[float], tolerance: float = SAME_LANE_M) -> list[float]:
    """Sorted values, merging those within `tolerance` (the start lane is flown twice)."""
    merged: list[list[float]] = []
    for value in values:
        if merged and value - merged[-1][-1] <= tolerance:
            merged[-1].append(value)
        else:
            merged.append([value])
    return [float(np.mean(group)) for group in merged]


def fly(args: argparse.Namespace, run_sweep: RunSweep) -> None:
    """Connect, run the sweep under Ctrl+C / STOP / safety landings, report a sim run."""
    console.install("sweep_dry" if args.dry_run else "sweep")
    if args.sim_depth:
        args.room = hall(args)
    drone, close = connect(args)
    tracker = Tracker(drone, args.room, args) if args.sim else None
    try:
        outcome = run_sweep(args, drone, tracker.watch if tracker else None)
        print(f"patrol ended: {outcome}", flush=True)
    except KeyboardInterrupt:
        if not args.dry_run:
            with contextlib.suppress(OSError):
                drone.stop()  # neutral, and the SDK's STOP
        print("\nstopped (Ctrl+C): sticks neutral, STOP sent", flush=True)
    except (Stopped, Interrupted) as reason:
        # STOP was pressed, or the mission took the drone after a detection: no STOP from
        # here, it would cancel what the mission does now
        print(f"\nstopped by the SDK: {reason}", flush=True)
    except SafetyLanding as reason:
        print(f"safety landing: {reason}", flush=True)
    finally:
        if tracker:
            tracker.report(args.plot)
        close()
