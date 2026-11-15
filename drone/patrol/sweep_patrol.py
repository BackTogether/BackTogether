"""The sweep's state machine (sweep.py has the overview, the CLI and the mission hook).

`SweepPatrol.step()` runs once per control tick (dvd.loop): it advances a short plan of steps
(CLIMB, ALIGN, LANE, WAIT, HOVER, TURN, PROBE, CROSS, REVISIT, DONE) and returns the stick
axes. Decisions (end of a lane, end of a side, the next revisit) replace the rest of the plan.
Legs are straight lines in the lane frame, held with the SDK heading (yaw) and the odometry
position (roll); turns are relative, measured with the best heading (dvd.current_heading).
"""

import argparse
import math
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum

from drone.patrol.coverage import SEEN_MAX_M, Box, CameraView, CoverageMap, Patch
from drone.patrol.dvd import (
    SETTLE_S,
    WINDOW_S,
    Perception,
    Sample,
    calibrate,
    current_heading,
    front_value,
    nearest_front,
)
from drone.patrol.floor_boundary import Status
from drone.patrol.sdkdrone import DroneState
from drone.sdk.estimate import wrap_deg
from drone.sdk.mission_geometry import Alignment

# Align
ALIGN_M = 3.0  # straight, no yaw, before the lane axis is fixed
ALIGN_MAX_M = 6.0  # ...or longer while the SDK heading is still unsure
ALIGN_SIGMA_DEG = 10.0  # SDK heading uncertainty that ends the align leg at ALIGN_M
ALIGN_MIN_M = 1.0  # shorter (blocked at once): the SDK heading is the axis
MISSION_AXIS_TOLERANCE_DEG = 15.0  # heading this close to the mission's leg: it goes on
# Lanes and crossings
HOVER_S = 0.5  # at the end of every leg
HEADING_GAIN = 1 / 40  # yaw input per degree of heading error
MAX_HOLD_YAW = 0.25  # yaw input while holding a heading (small: keeps the SDK's travel fix on)
CROSS_TRACK_GAIN = 0.12  # roll input per metre off the line
MAX_ROLL = 0.12  # stays under the SDK's 10 % roll gate for the video yaw at cap <= 80
CROSS_BRAKE_M = 0.25  # end a crossing this short of its target: the drone coasts on
NEAR_EDGE_MARGIN_M = 0.15  # stop when a contact is this close to the image's bottom edge
REJOIN_M = 1.0  # resumed this far off the leg's line: fly back to it first
STICKS_SLOW_AFTER_S = 10.0  # heading on the stick model this long: --sticks-speed
WIDTH_TOLERANCE_M = 0.3
MAX_LEGS = 60  # a guard against a sweep that never ends
# Turns (relative, measured with the best heading: dvd.current_heading)
TURN_GAIN = 1 / 15  # yaw input per degree still to turn: full input until the last 15 deg
MIN_TURN_YAW = 0.35
TURN_TOLERANCE_DEG = 3.0
TURN_TIMEOUT_S = 25.0
# Probing the next lane's side, obstacles
PROBE_S = 1.0
MIN_PROBE_READINGS = 3
PROBE_TIMEOUT_S = 4.0
SIDE_MARGIN_M = 0.5
SIDE_WALL_CLEARANCE_M = 1.5  # a last, closer lane stays this far from the side wall
WALL_LOOKBACK_S = 3.0  # readings before a stop that may show the wall's floor-contact line
WALL_MIN_SPAN_M = 2.0  # a contact line this wide is a wall (a person is ~0.5 m)
WALL_MATCH_M = 0.6  # ...if it crosses the heading this close to the front reading
MIN_WALL_READINGS = 2
KNOWN_WALL_TOLERANCE_M = 2.0  # a stop this much short of a wall seen before: an obstacle
CLEAR_READINGS = 3  # in a row at --clear-at or farther: an obstacle has gone
# Altitude
ALT_GAIN = 0.5  # thrust input per metre (holding)
CLIMB_GAIN = 1.0  # (climbing to --alt at the start)
MAX_THRUST = 0.3
ALT_DEADBAND_M = 0.1
CLIMB_TOLERANCE_M = 0.15
CLIMB_TIMEOUT_S = 15.0
# Revisit
REVISIT_MIN_M2 = 2.0
REVISIT_MAX = 3
REVISIT_STOP_M = 0.5  # stop this far before the patch's floor reaches the bottom of the image
ALONG, BACK = 0.0, 180.0  # lane headings in the lane frame


class Phase(Enum):
    CLIMB = "CLIMB"
    ALIGN = "ALIGN"
    LANE = "LANE"
    WAIT = "WAIT"  # an obstacle ahead: hovering until it goes
    HOVER = "HOVER"
    TURN = "TURN"
    PROBE = "PROBE"  # reading the distance toward the next lane
    CROSS = "CROSS"  # one lane spacing over (or back to the start lane)
    REVISIT = "REVISIT"
    DONE = "DONE"


class SweepFinished(Exception):
    """The sweep is complete; the message is the summary."""


@dataclass(frozen=True)
class LaneFrame:
    origin: tuple[float, float]  # odometry frame
    axis_deg: float  # the lanes' direction in the odometry frame

    def point(self, xy: tuple[float, float]) -> tuple[float, float]:
        dx, dy = xy[0] - self.origin[0], xy[1] - self.origin[1]
        axis = math.radians(self.axis_deg)
        return dx * math.cos(axis) + dy * math.sin(axis), -dx * math.sin(axis) + dy * math.cos(axis)

    def heading(self, odometry_heading_deg: float) -> float:
        return wrap_deg(odometry_heading_deg - self.axis_deg)


@dataclass(frozen=True)
class Leg:
    """A straight line in the lane frame."""

    start: tuple[float, float]
    heading_deg: float
    length_m: float | None  # None: until something is in the way

    def track(self, point: tuple[float, float]) -> tuple[float, float]:
        """(progress along the leg, offset to its right) of a lane-frame point."""
        heading = math.radians(self.heading_deg)
        dx, dy = point[0] - self.start[0], point[1] - self.start[1]
        return (
            dx * math.cos(heading) + dy * math.sin(heading),
            -dx * math.sin(heading) + dy * math.cos(heading),
        )


@dataclass
class Step:
    phase: Phase
    heading_deg: float = 0.0  # TURN: to face (lane frame); LANE / REVISIT: to fly
    seconds: float = 0.0  # HOVER
    length_m: float | None = None  # REVISIT
    transit: bool = False  # CROSS: back to the start lane (never ends a side)
    leg: Leg | None = None  # set when a LANE / CROSS / REVISIT starts
    started: bool = False
    began: float = 0.0  # time.monotonic() when it started


@dataclass
class Turn:
    amount_deg: float  # + = right
    turned_deg: float = 0.0
    last_deg: float | None = None


@dataclass
class Track:
    """What the sweep found, in the lane frame."""

    end_walls: dict[float, float] = field(default_factory=dict)  # lane heading -> wall x
    side_walls: dict[int, float] = field(default_factory=dict)  # +1 / -1 -> wall y
    covered: dict[int, float] = field(default_factory=dict)  # side -> |y| covered
    blocked: list[tuple[float, float]] = field(default_factory=list)
    revisited: list[Patch] = field(default_factory=list)
    flown_m: float = 0.0  # between 1 Hz odometry samples (the extrapolated pose jitters)
    last_odometry: tuple[float, ...] | None = None


def camera_pitch(args: argparse.Namespace, state: DroneState) -> float | None:
    """Degrees below level: --pitch, the simulated camera, or the SDK's tilt estimate."""
    if args.pitch is not None:
        return args.pitch
    if args.sim_depth:
        return -args.tilt
    return None if state.tilt_deg is None else -state.tilt_deg


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class SweepPatrol:
    """The state machine: a plan of steps, run one tick at a time by dvd.loop."""

    def __init__(
        self,
        args: argparse.Namespace,
        perception: Perception,
        view: CameraView,
        spacing_m: float,
        alt_target_m: float | None,
        alignment: Alignment | None = None,
    ) -> None:
        """`alignment`: the mission's aligning leg, flown just before: ALIGN goes on from it."""
        self.args, self.perception, self.view, self.spacing = args, perception, view, spacing_m
        self.alt_target = alt_target_m
        self.alignment = alignment
        # the floor boundary cannot measure nearer than the bottom of the image: an obstacle
        # still seen there, in the corridor, is about to pass under it
        self.stop_m = max(args.stop_at, view.near_m() + NEAR_EDGE_MARGIN_M)
        print(
            f"stops {self.stop_m:.2f} m before obstacles (floor visible from "
            f"{view.near_m():.2f} m), corridor +-{args.corridor:.1f} m",
            flush=True,
        )
        self.frame: LaneFrame | None = None
        self.cover = CoverageMap()
        self.track = Track()
        self.plan: deque[Step] = deque([Step(Phase.CLIMB), Step(Phase.ALIGN)])
        self.side = 1 if args.side == "right" else -1  # toward +y (right of the axis) or -y
        self.second_side = False
        self.offset = 0.0  # y of the lane being flown
        self.next_offset = 0.0
        self.lane_heading = ALONG
        self.lanes = 0  # lanes started
        self.legs = 0  # lanes + crossings + revisits started
        self.turn: Turn | None = None
        self.since = time.monotonic()
        self.align_start: tuple[float, float] | None = None
        self.align_m = ALIGN_M  # straight travel before the axis is fixed (and the heading sure)
        self.sticks_since: float | None = None
        self.heading_error = 0.0
        self.cross_track = 0.0
        self.first_lane_end: float | None = None  # x: --length lanes span from there
        self.lane_blocked = False  # the last lane ended at an obstacle, not at its end
        self.attempted: list[Patch] = []
        self._state: DroneState | None = None

    # --- the loop's interface -------------------------------------------------------------------

    def step(self, state: DroneState) -> dict[str, float]:
        """The stick axes for this tick."""
        if state.pose_xy is None or state.heading_deg is None:
            return {}
        self._state = state
        now = time.monotonic()
        self._follow(state, now)
        current = self.plan[0]
        if not current.started:
            self._begin(current, state)
        axes = self._handlers()[current.phase](current, state, now)
        if "thrust" not in axes:
            axes["thrust"] = self._thrust(state)
        return axes

    def panel_line(self, latest: Sample) -> str:
        phase = self.plan[0].phase.value if self.plan else Phase.DONE.value
        value = front_value(latest.wall)
        front = "?" if value is None else "clear" if math.isinf(value) else f"{value:.1f}m"
        total = self._lanes_estimate()
        lanes = f"lane {max(self.lanes, 1)}" + (f"/~{total}" if total else "")
        side = "R" if self.side > 0 else "L"
        box = self.room_box()
        cover = f"{self._coverage_note(box)} |{self.cover.strip(box)}|" if box else ""
        return (
            f"{phase:7s} {lanes} {side} front {front:>5s} hdg {self.heading_error:+4.0f} "
            f"xtrk {self.cross_track:+.2f}m {cover}"
        )

    def resume(self, state: DroneState) -> None:
        """After someone else flew the drone (the mission's approach): restart a turn in
        progress from the heading now; back to the line of a lane or crossing in progress."""
        if not self.plan or self.frame is None or state.pose_xy is None:
            return
        if self.plan[0].phase is Phase.WAIT:
            self._next()  # the obstacle is long gone or still there: the leg will tell
        step = self.plan[0]
        if step.phase is Phase.TURN:
            step.started = False
            return
        if step.phase not in (Phase.LANE, Phase.CROSS) or step.leg is None:
            return
        here = self.frame.point(state.pose_xy)
        progress, off = step.leg.track(here)
        if abs(off) < REJOIN_M:
            return
        heading = math.radians(step.leg.heading_deg)
        start = step.leg.start
        target = (start[0] + progress * math.cos(heading), start[1] + progress * math.sin(heading))
        bearing = math.degrees(math.atan2(target[1] - here[1], target[0] - here[0]))
        back = Leg(here, bearing, math.dist(here, target))
        print(f"back to the {step.phase.value.lower()} line, {abs(off):.1f} m away", flush=True)
        self.plan.extendleft(
            reversed(
                [
                    Step(Phase.TURN, bearing),
                    Step(Phase.CROSS, transit=True, leg=back),
                    Step(Phase.TURN, step.leg.heading_deg),
                ]
            )
        )

    # --- steps --------------------------------------------------------------------------------

    def _handlers(self) -> dict[Phase, Callable[[Step, DroneState, float], dict[str, float]]]:
        return {
            Phase.CLIMB: self._climb,
            Phase.ALIGN: self._align,
            Phase.LANE: self._fly,
            Phase.CROSS: self._fly,
            Phase.REVISIT: self._fly,
            Phase.WAIT: self._wait,
            Phase.HOVER: self._hover,
            Phase.TURN: self._turning,
            Phase.PROBE: self._probe,
            Phase.DONE: self._done,
        }

    def _begin(self, step: Step, state: DroneState) -> None:
        step.started = True
        self.since = step.began = time.monotonic()
        note = ""
        if step.phase is Phase.ALIGN:
            self.align_start, self.align_m = state.pose_xy, ALIGN_M
            if self.alignment is not None:
                self._continue_mission_leg(state)
        elif step.phase is Phase.TURN:
            amount = wrap_deg(self.frame.axis_deg + step.heading_deg - state.heading_deg)
            self.turn = Turn(amount)
            note = f"{amount:+.0f} deg to {step.heading_deg:+.0f}"
        elif step.phase in (Phase.LANE, Phase.CROSS, Phase.REVISIT):
            step.leg = step.leg or self._new_leg(step, state)
            self.legs += 1
            note = self._leg_note(step)
        elif step.phase is Phase.HOVER:
            note = f"{step.seconds:.1f} s"
        print(f"-> {step.phase.value} {note}".rstrip(), flush=True)

    def _next(self) -> None:
        self.plan.popleft()

    def _new_leg(self, step: Step, state: DroneState) -> Leg:
        x, y = self.frame.point(state.pose_xy)
        if step.phase is Phase.LANE:
            self.lanes += 1
            self.offset = self.next_offset
            self.lane_heading = step.heading_deg
            return Leg((x, self.offset), step.heading_deg, self._lane_length(step.heading_deg, x))
        if step.phase is Phase.CROSS:
            heading = 90.0 if self.next_offset > y else -90.0
            return Leg((x, y), heading, abs(self.next_offset - y))
        return Leg((x, y), step.heading_deg, step.length_m)

    def _leg_note(self, step: Step) -> str:
        leg = step.leg
        length = "to the wall" if leg.length_m is None else f"{leg.length_m:.1f} m"
        if step.phase is Phase.LANE:
            heading = f"heading {leg.heading_deg:.0f}"
            return f"lane {self.lanes} at y={self.offset:+.1f} m, {heading}, {length}"
        return f"heading {leg.heading_deg:+.0f}, {length}"

    def _lane_length(self, heading_deg: float, x: float) -> float | None:
        """--length: lanes span the hall from where the first lane ended."""
        if self.args.length is None:
            return None
        far = self.first_lane_end
        if far is None:
            return max(0.0, self.args.length - x)
        if heading_deg == ALONG:
            return max(0.0, far - x)
        return max(0.0, x - (far - self.args.length))

    def _climb(self, step: Step, state: DroneState, now: float) -> dict[str, float]:
        error = self._altitude_error()
        if error is None or abs(error) <= CLIMB_TOLERANCE_M or now - self.since > CLIMB_TIMEOUT_S:
            self._next()
            return {}
        return {"thrust": _clip(CLIMB_GAIN * error, MAX_THRUST)}

    def _align(self, step: Step, state: DroneState, now: float) -> dict[str, float]:
        dx = state.pose_xy[0] - self.align_start[0]
        dy = state.pose_xy[1] - self.align_start[1]
        travel = math.hypot(dx, dy)
        sigma = state.heading_sigma_deg
        sure = sigma is None or sigma <= ALIGN_SIGMA_DEG
        front, unsure = nearest_front(self.perception.readings.since(now - WINDOW_S))
        blocked = front is not None and front < self.stop_m
        if blocked or (travel >= self.align_m and sure) or travel >= ALIGN_MAX_M:
            axis = math.degrees(math.atan2(dy, dx)) if travel >= ALIGN_MIN_M else state.heading_deg
            self.frame = LaneFrame(self.align_start, axis)
            print(
                f"lane axis {axis:+.1f} deg in the odometry frame after {travel:.1f} m "
                f"(SDK heading {state.heading_deg:+.1f}, sigma {sigma or 0:.0f} deg, "
                f"{state.heading_source})",
                flush=True,
            )
            self._next()
            self.plan.appendleft(Step(Phase.LANE, ALONG))
            return {}
        return {"pitch": self._speed(front, unsure, now)}

    def _continue_mission_leg(self, state: DroneState) -> None:
        """The mission's aligning leg (odometry displacement, atan2(dy, dx) like ALIGN's) ended
        here: if the drone still faces along it, ALIGN goes on from that leg's start (about one
        leg back), so it ends at once when the SDK heading is sure, else flies on straight.
        Turned since (a look-around, an approach) or too short: ALIGN's own leg from here."""
        axis, leg = self.alignment.forward_deg, self.alignment.leg_m
        off = wrap_deg(state.heading_deg - axis)
        if leg < ALIGN_MIN_M or abs(off) > MISSION_AXIS_TOLERANCE_DEG:
            print(
                f"mission's aligning leg not used ({leg:.1f} m at {axis:+.1f} deg, heading "
                f"{off:+.0f} deg off it): aligning from here",
                flush=True,
            )
            return
        self.align_start = (
            state.pose_xy[0] - leg * math.cos(math.radians(axis)),
            state.pose_xy[1] - leg * math.sin(math.radians(axis)),
        )
        self.align_m = ALIGN_MIN_M  # the mission's leg is the distance flown
        print(
            f"continuing the mission's {leg:.1f} m aligning leg ({axis:+.1f} deg, heading "
            f"{off:+.0f} deg off it)",
            flush=True,
        )

    def _fly(self, step: Step, state: DroneState, now: float) -> dict[str, float]:
        leg = step.leg
        progress, off = leg.track(self.frame.point(state.pose_xy))
        self.cross_track = off
        window = self.perception.readings.since(now - WINDOW_S)
        front, unsure = nearest_front(window)
        if front is not None and front < self.stop_m:
            self._stop_for(step, front, progress, now)
            return {}
        brake = CROSS_BRAKE_M if step.phase is not Phase.LANE else 0.0
        if leg.length_m is not None and progress >= leg.length_m - brake:
            self._leg_done(step, "length" if step.phase is Phase.LANE else "reached")
            return {}
        if step.phase is Phase.LANE:
            calibrate(self.perception, state, window)
            speed = self._speed(front, unsure, now)
        else:
            speed = self.args.slow_speed
        return {
            "pitch": speed,
            "roll": _clip(-CROSS_TRACK_GAIN * off, MAX_ROLL),
            "yaw": self._hold(leg.heading_deg, state),
        }

    def _stop_for(self, step: Step, front: float, progress: float, now: float) -> None:
        """Something closer than --stop-at: the end wall, or an obstacle to wait for."""
        if self._wall_ahead(step, now):
            print(f"wall at {front:.1f} m after {progress:.1f} m", flush=True)
            self._note_wall(step)
            self._leg_done(step, "wall")
            return
        print(f"obstacle at {front:.1f} m after {progress:.1f} m: waiting", flush=True)
        self.plan.appendleft(Step(Phase.WAIT))

    def _wait(self, step: Step, state: DroneState, now: float) -> dict[str, float]:
        recent = self.perception.readings.since(self.since + SETTLE_S)[-CLEAR_READINGS:]
        values = [front_value(sample.wall) for sample in recent]
        if len(values) == CLEAR_READINGS and all(
            v is not None and v >= self.args.clear_at for v in values
        ):
            print("obstacle gone: on", flush=True)
            self._next()
            return {}
        if now - self.since >= self.args.wait:
            self._next()
            leg_step = self.plan[0]
            x, y = self.frame.point(state.pose_xy)
            self.track.blocked.append((x, y))
            print(f"still blocked at ({x:+.1f}, {y:+.1f}): leg ends here", flush=True)
            self._leg_done(leg_step, "obstacle")
        return {}

    def _hover(self, step: Step, state: DroneState, now: float) -> dict[str, float]:
        if now - self.since >= step.seconds:
            self._next()
        return {}

    def _turning(self, step: Step, state: DroneState, now: float) -> dict[str, float]:
        turn = self.turn
        heading = current_heading(state)
        if turn.last_deg is not None:
            turn.turned_deg += wrap_deg(heading - turn.last_deg)
        turn.last_deg = heading
        left = turn.amount_deg - turn.turned_deg
        if abs(left) <= TURN_TOLERANCE_DEG or left * turn.amount_deg < 0:
            self._next()
            return {}
        if now - self.since > TURN_TIMEOUT_S:
            print(f"turn timed out with {left:+.0f} deg to go", flush=True)
            self._next()
            return {}
        magnitude = min(self.args.yaw_input, max(MIN_TURN_YAW, abs(left) * TURN_GAIN))
        return {"yaw": math.copysign(magnitude, left)}

    def _probe(self, step: Step, state: DroneState, now: float) -> dict[str, float]:
        readings = self.perception.readings.since(self.since + SETTLE_S)
        values = [v for v in (front_value(s.wall) for s in readings) if v is not None]
        waited = now - self.since
        if waited < SETTLE_S + PROBE_S or (
            len(values) < MIN_PROBE_READINGS and waited < PROBE_TIMEOUT_S
        ):
            return {}
        self._next()
        x, y = self.frame.point(state.pose_xy)
        front = min(values) if values else 0.0  # unreadable: do not fly blind that way
        obstacle = front >= self.view.near_m() and not _wide_contact(readings)
        if front < self.spacing + SIDE_MARGIN_M and obstacle:
            print(f"obstacle at {front:.1f} m that way, not a wall: crossing", flush=True)
        elif front < self.spacing + SIDE_MARGIN_M:
            wall = max(front, self.view.near_m())
            self.track.side_walls[self.side] = y + self.side * wall
            last_step = wall - SIDE_WALL_CLEARANCE_M
            if last_step < self.spacing / 2:
                print(f"side wall at {front:.1f} m: this side is done", flush=True)
                self._side_done(abs(y) + wall)
                return {}
            print(f"side wall at {front:.1f} m: a last lane {last_step:.1f} m over", flush=True)
            self.next_offset = y + self.side * last_step
        self.plan.extendleft(
            reversed(
                [
                    Step(Phase.CROSS),
                    Step(Phase.HOVER, seconds=HOVER_S),
                    Step(Phase.TURN, self._next_lane_heading()),
                    Step(Phase.LANE, self._next_lane_heading()),
                ]
            )
        )
        return {}

    def _done(self, step: Step, state: DroneState, now: float) -> dict[str, float]:
        raise SweepFinished(self.summary())

    # --- decisions ----------------------------------------------------------------------------

    def _leg_done(self, step: Step, why: str) -> None:
        """A LANE / CROSS / REVISIT ended (why: length, reached, wall, obstacle)."""
        self.plan.remove(step)
        if step.phase is Phase.LANE:
            self._lane_ended(why)
        elif step.phase is Phase.CROSS:
            self._cross_ended(step, why)
        else:
            self._next_revisit()

    def _lane_ended(self, why: str) -> None:
        self.lane_blocked = why == "obstacle"
        self.perception.calibration.end_leg()
        x, y = self.frame.point(self._state.pose_xy)
        if self.first_lane_end is None:
            self.first_lane_end = x
        print(f"lane {self.lanes} done ({why}) at ({x:+.1f}, {y:+.1f})", flush=True)
        if self.legs >= MAX_LEGS:
            self._finish("too many legs")
            return
        next_offset = self.offset + self.side * self.spacing
        if not self._width_left(self.side):
            self._side_done(abs(self.offset) + self.spacing / 2)
            return
        self.next_offset = next_offset
        self.plan.extendleft(
            reversed(
                [
                    Step(Phase.HOVER, seconds=HOVER_S),
                    Step(Phase.TURN, 90.0 * self.side),
                    Step(Phase.PROBE),
                ]
            )
        )

    def _cross_ended(self, step: Step, why: str) -> None:
        x, y = self.frame.point(self._state.pose_xy)
        if why == "reached":
            return
        progress, _ = step.leg.track((x, y))
        if why == "wall" and progress < self.spacing / 3 and not step.transit:
            self._side_done(abs(y) + self.view.near_m())
            return
        self.next_offset = y  # the next lane where the crossing had to stop

    def _side_done(self, covered_m: float) -> None:
        self.track.covered[self.side] = covered_m
        self.plan.clear()
        if not self.second_side and self._other_side_left():
            self.second_side = True
            self.side = -self.side
            self.next_offset = 0.0
            x, y = self.frame.point(self._state.pose_xy)
            print(f"back to the start lane ({abs(y):.1f} m), then the other side", flush=True)
            self.plan.append(Step(Phase.HOVER, seconds=HOVER_S))
            if abs(y) > self.spacing / 3:
                self.plan.extend(
                    [
                        Step(Phase.TURN, 90.0 if y < 0 else -90.0),
                        Step(Phase.CROSS, transit=True),
                        Step(Phase.HOVER, seconds=HOVER_S),
                    ]
                )
            heading = self._flip(self.lane_heading)
            self.plan.extend([Step(Phase.TURN, heading), Step(Phase.LANE, heading)])
            return
        if self.args.revisit:
            self._next_revisit()
        else:
            self._finish("both sides done")

    def _next_revisit(self) -> None:
        """Fly a straight leg toward the largest unseen patch not tried yet."""
        box = self.room_box()
        patches = [
            patch
            for patch in self.cover.patches(box, REVISIT_MIN_M2)
            if not any(math.dist((p.x, p.y), (patch.x, patch.y)) < 1.0 for p in self.attempted)
        ]
        if not patches or len(self.attempted) >= REVISIT_MAX:
            self._finish("sweep and revisits done")
            return
        x, y = self.frame.point(self._state.pose_xy)
        patch = patches[0]  # the largest
        self.attempted.append(patch)
        self.track.revisited.append(patch)
        bearing = math.degrees(math.atan2(patch.y - y, patch.x - x))
        length = max(
            0.0, math.dist((x, y), (patch.x, patch.y)) - self.view.near_m() - REVISIT_STOP_M
        )
        print(
            f"revisit: {patch.area_m2:.0f} m2 unseen at ({patch.x:+.1f}, {patch.y:+.1f}), "
            f"{length:.1f} m at {bearing:+.0f} deg",
            flush=True,
        )
        self.plan.extend(
            [
                Step(Phase.HOVER, seconds=HOVER_S),
                Step(Phase.TURN, bearing),
                Step(Phase.REVISIT, bearing, length_m=length),
            ]
        )

    def _finish(self, why: str) -> None:
        print(f"sweep: {why}", flush=True)
        self.plan.clear()
        self.plan.extend([Step(Phase.HOVER, seconds=HOVER_S), Step(Phase.DONE)])

    # --- checks -------------------------------------------------------------------------------

    def _width_left(self, side: int) -> bool:
        """--width: is there width left for another lane on `side`?"""
        if self.args.width is None:
            return True
        this = abs(self.offset) + self.spacing / 2
        other = self.track.covered.get(-side, self.spacing / 2)
        return this + other < self.args.width - WIDTH_TOLERANCE_M

    def _other_side_left(self) -> bool:
        if self.args.width is None:
            return True
        return (
            self.track.covered[self.side] + self.spacing / 2 < self.args.width - WIDTH_TOLERANCE_M
        )

    def _wall_ahead(self, step: Step, now: float) -> bool:
        """Is what stopped us a wall? Where an end wall was seen before, by its position;
        else by a wide floor-contact line in the readings before the stop."""
        if step.phase is Phase.LANE and step.leg.heading_deg in self.track.end_walls:
            known = self.track.end_walls[step.leg.heading_deg]
            x, _ = self.frame.point(self._state.pose_xy)
            remaining = (known - x) * (1 if step.leg.heading_deg == ALONG else -1)
            return remaining <= self.view.near_m() + KNOWN_WALL_TOLERANCE_M
        # only this leg's readings: before a turn the camera looked elsewhere
        return _wide_contact(self.perception.readings.since(max(now - WALL_LOOKBACK_S, step.began)))

    def _note_wall(self, step: Step) -> None:
        """An end wall's x, for the next lanes and the hall's extent."""
        if step.phase is not Phase.LANE:
            return
        latest = self.perception.readings.latest
        value = front_value(latest.wall) if latest else None
        distance = max(value or 0.0, self.view.near_m())
        x, _ = self.frame.point(self._state.pose_xy)
        direction = 1 if step.leg.heading_deg == ALONG else -1
        self.track.end_walls.setdefault(step.leg.heading_deg, x + direction * distance)

    # --- control ------------------------------------------------------------------------------

    def _hold(self, heading_deg: float, state: DroneState) -> float:
        self.heading_error = wrap_deg(self.frame.axis_deg + heading_deg - state.heading_deg)
        return _clip(self.heading_error * HEADING_GAIN, MAX_HOLD_YAW)

    def _speed(self, front: float | None, unsure: bool, now: float) -> float:
        slow = unsure or front is None or front < self.args.slow_at
        speed = self.args.slow_speed if slow else self.args.speed
        if self.sticks_since is not None and now - self.sticks_since > STICKS_SLOW_AFTER_S:
            speed *= self.args.sticks_speed
        return speed

    def _thrust(self, state: DroneState) -> float:
        error = self._altitude_error()
        if error is None or abs(error) < ALT_DEADBAND_M:
            return 0.0
        return _clip(ALT_GAIN * error, MAX_THRUST)

    def _altitude_error(self) -> float | None:
        """--alt minus the altitude, filtered like the camera height (camera_height.py drops
        the odometry's jumps), None when either is unknown."""
        height = self.perception.height.metres
        if self.alt_target is None or height is None:
            return None
        return self.alt_target - (height - self.args.takeoff_height)

    def _follow(self, state: DroneState, now: float) -> None:
        """Heading source, distance flown and the floor the camera sees."""
        if state.heading_source == "sticks":
            self.sticks_since = self.sticks_since or now
        else:
            self.sticks_since = None
        last, position = self.track.last_odometry, state.position
        if position is not None and position != last:
            if last is not None:
                self.track.flown_m += math.dist(last[:2], position[:2])
            self.track.last_odometry = position
        if self.frame is None:
            return
        height = self.perception.height.metres
        pitch = camera_pitch(self.args, state)
        latest = self.perception.readings.latest
        if height is None or pitch is None or latest is None:
            return
        view = CameraView(height * latest.scale, pitch, self.args.hfov)
        x, y = self.frame.point(state.pose_xy)
        heading = self.frame.heading(state.heading_deg)
        # by lane (a crossing counts with the lane before it): the overlap is between lanes
        self.cover.mark(x, y, heading, view, _seen_range(latest), self.lanes)

    # --- reporting ----------------------------------------------------------------------------

    def room_box(self) -> Box | None:
        """The hall in the lane frame as far as known: walls seen, else the floor seen."""
        seen = self.cover.seen_box()
        if seen is None:
            return None
        walls = self.track
        return (
            walls.end_walls.get(BACK, seen[0]),
            walls.end_walls.get(ALONG, seen[1]),
            walls.side_walls.get(-1, seen[2]),
            walls.side_walls.get(1, seen[3]),
        )

    def _coverage_note(self, box: Box) -> str:
        """Seen floor, and its share of the hall as far as it is known yet."""
        fraction = self.cover.fraction(box)
        length, width = box[1] - box[0], box[3] - box[2]
        return (
            f"seen {fraction * length * width:.0f}m2 = {fraction:.0%} of {length:.0f}x{width:.0f}m"
        )

    def _lanes_estimate(self) -> int | None:
        if self.args.width is None:
            return None
        return math.ceil(self.args.width / self.spacing) + 1  # + the start lane flown twice

    def summary(self) -> str:
        box = self.room_box()
        if box is None:
            return f"{self.lanes} lanes, nothing seen"
        patches = self.cover.patches(box)
        holes = ", ".join(f"{p.area_m2:.0f} m2 at ({p.x:+.1f}, {p.y:+.1f})" for p in patches[:3])
        blocked = ", ".join(f"({x:+.1f}, {y:+.1f})" for x, y in self.track.blocked)
        return (
            f"{self.lanes} lanes, {self.track.flown_m:.0f} m flown, coverage "
            f"{self.cover.fraction(box):.0%} of {box[1] - box[0]:.0f} x {box[3] - box[2]:.0f} m "
            f"(lane frame), overlap {self.cover.overlap(box):.0%}; unseen: {holes or 'none'}; "
            f"blocked at: {blocked or 'none'}"
        )

    def _next_lane_heading(self) -> float:
        """The other way, except after a lane blocked by an obstacle: then on the same way past
        it, so the stretch behind the obstacle gets flown on both its sides."""
        return self.lane_heading if self.lane_blocked else self._flip(self.lane_heading)

    @staticmethod
    def _flip(heading_deg: float) -> float:
        return BACK if heading_deg == ALONG else ALONG


def _wide_contact(samples: list[Sample]) -> bool:
    """Do the readings show a wall: a wide floor-contact line where the front reading is (a
    person or a pillar is ~0.5 m wide)?"""
    wide = 0
    for sample in samples:
        wall = sample.wall
        if wall.status is not Status.WALL:
            continue
        wide += any(
            line.span_m >= WALL_MIN_SPAN_M and abs(line.distance_m - wall.front_m) <= WALL_MATCH_M
            for line in wall.walls
        )
    return wide >= MIN_WALL_READINGS


def _seen_range(latest: Sample) -> float:
    """How far ahead the floor is visible: up to the obstacle in front, at most SEEN_MAX_M."""
    wall = latest.wall
    if wall.status is Status.WALL and wall.front_m is not None:
        return min(SEEN_MAX_M, wall.front_m)
    if wall.status is Status.NEAR:
        return wall.near_m
    return SEEN_MAX_M
