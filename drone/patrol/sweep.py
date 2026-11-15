"""Lawnmower (boustrophedon) sweep: parallel lanes over a big hall, to search it from the video.

The predictable way to cover a sports hall: every lane is a straight line along the hall, the
next one a lane spacing over, so the camera footprints overlap by --overlap. Start anywhere,
facing along the hall (no corner start needed). Lane frame: x along the lanes from the start
point, y to the right of them (heading 0 = along, 90 = right).

1. ALIGN: a few metres straight ahead, no yaw: the odometry displacement is the lane axis in the
   odometry frame (whose axes are not the drone's), and the SDK's heading learns the same angle
   from the direction of travel (estimate.py) before any heading is held.
2. LANE: fly the lane, holding the heading (the SDK heading) and the cross-track offset
   (odometry, gentle roll), until the floor boundary (floor_boundary.py) shows something within
   --corridor of the path closer than --stop-at (at least the image's near edge + 0.15 m: the
   floor boundary cannot see nearer), or --length. A wall (a wide floor-contact line, or where
   the wall was seen before) ends the lane; anything else (a person, a pillar) is waited for up
   to --wait s, then the lane ends there, the spot is reported and the next lane goes on the
   same way past it.
3. Hover, TURN 90 deg toward the sweep side, PROBE that way: a wall closer than one lane spacing
   + margin ends the side (or leaves room for one last, closer lane); else CROSS one spacing
   (odometry), TURN 90 deg the same way, and fly the next lane back (turns right, right, then
   left, left...).
4. The other side: back across to the start lane, which is flown again in full (the first time
   it only covered what was ahead of the start point), then lanes on the other side until its
   wall, or until --width is covered.
5. Coverage (coverage.py): the floor cells the camera saw (footprint from the camera height,
   tilt, HFOV, heading and position, up to 4 m ahead), as % of the hall seen so far; at the end
   the largest unseen patches are revisited with straight legs (--revisit) and reported.

Mission (drone/sdk/mission.py): in-process as MissionConfig(patrol=MissionSweep([...])), or
over HTTP with strategy "external": a detection hands the drone to the mission, and the sweep
resumes where it was if the mission patrols again.

Safety: everything goes through drone.sdk (sdkdrone.py): STOP / LAND ends the sweep (no
auto-resume), link/telemetry/video loss hovers then lands, geofence and low battery bring it
home. The patrol hovers by itself when not flying in MANUAL, below --min-alt, without a camera
height or tilt, or without a fresh depth reading (dvd.hold_reason); it slows down near walls,
when the view is unreadable and when the heading has been on the stick model for long.

Run from the repo root (helper: drone/patrol/live sweep-dry | sweep):
  uv run --offline --no-project --with numpy --with pillow python -m drone.patrol.sweep \\
      --sim --sim-hall 30 20 --pillars --plot /tmp/sweep.png      # in-process simulated hall
  HF_HUB_OFFLINE=1 uv run --offline --no-project --with numpy --with pillow --with torch \\
      --with transformers python -m drone.patrol.sweep --sdk http://127.0.0.1:8077 --dry-run
Stop it with Ctrl+C (neutral + STOP), or STOP / LAND in the web page.
"""

import argparse
import time
from collections.abc import Callable, Sequence

from drone.patrol.coverage import DEFAULT_OVERLAP, CameraView, lane_spacing
from drone.patrol.dvd import Perception, loop, new_parser, parse_patrol_args, setup_camera
from drone.patrol.sdkdrone import DroneState, LocalDrone, PatrolDrone, RemoteDrone, Stopped
from drone.patrol.sweep_patrol import SweepFinished, SweepPatrol, camera_pitch
from drone.sdk import Navigator
from drone.sdk.mission import END_STATES

MISSION_POLL_S = 1.0


def plan_view(
    args: argparse.Namespace, state: DroneState
) -> tuple[CameraView, float, float | None]:
    """The camera footprint at the patrol altitude and tilt, the lane spacing, and the altitude
    to hold (None: whatever it is)."""
    alt = args.alt if args.alt > 0 else state.altitude_m
    if alt is None and not args.lane:
        raise SystemExit("no altitude yet for the lane spacing: take off, or give --alt / --lane")
    pitch = camera_pitch(args, state)
    view = CameraView(
        (alt or 0.0) + args.takeoff_height, -args.tilt if pitch is None else pitch, args.hfov
    )
    spacing = args.lane or lane_spacing(view, args.overlap)
    print(
        f"lane spacing {spacing:.2f} m: camera footprint {view.swath_m():.2f} m wide at its near "
        f"edge ({view.near_m():.1f} m ahead) at {view.height_m:.1f} m, {view.pitch_deg:.0f} deg "
        f"down, {args.overlap:.0%} overlap{' (--lane)' if args.lane else ''}",
        flush=True,
    )
    return view, spacing, (args.alt if args.alt > 0 else None)


def run(
    args: argparse.Namespace,
    drone: PatrolDrone,
    on_start: Callable[[SweepPatrol], None] | None = None,
) -> str:
    """Fly the sweep until it is done, --duration, STOP or a safety rule."""
    setup_camera(args, drone)
    view, spacing, alt = plan_view(args, drone.state())
    perception = Perception(drone, args, args.corridor)
    patrol = SweepPatrol(args, perception, view, spacing, alt)
    if on_start:
        on_start(patrol)
    while True:
        try:
            return drone.run(lambda: loop(args, drone, perception, patrol))
        except SweepFinished as done:
            return f"sweep done: {done}"
        except Stopped:
            if not wait_for_mission(drone):
                print(f"sweep so far: {patrol.summary()}", flush=True)
                raise
            patrol.resume(drone.state())
        except BaseException:
            print(f"sweep so far: {patrol.summary()}", flush=True)
            raise


def wait_for_mission(drone: PatrolDrone) -> bool:
    """The SDK said stop: if a mission (strategy "external") took the drone for an approach,
    wait until it hands it back (True); False for a real STOP or the mission's end."""
    if not isinstance(drone, RemoteDrone):
        return False
    said = ""
    while True:
        mission = drone.client.mission()
        if mission.get("accepts_moves"):
            print("the mission hands the drone back: sweeping on", flush=True)
            return True
        state = mission.get("state")
        if state in END_STATES:
            return False
        if state != said:
            print(f"the mission flies the drone ({state}): waiting", flush=True)
            said = state
        time.sleep(MISSION_POLL_S)


class MissionSweep:
    """The sweep as the mission's patrol (drone/sdk/mission.py, MissionConfig.patrol):

        MissionConfig(patrol=MissionSweep(["--alt", "2"]), patrol_tilt_deg=-30, ...)

    The mission flies the drone in this process; a detection interrupts the sweep (STOP), and
    the next call picks it up where it was. Safety rules (ReturnHome, STOP) go to the mission."""

    __name__ = "sweep"

    def __init__(self, argv: Sequence[str] = ()) -> None:
        self.args = parse_args(list(argv))
        if self.args.sim_depth:  # testing with the in-process simulator: render its hall
            from drone.patrol.sweep_sim import hall

            self.args.room = hall(self.args)
        self._patrol: SweepPatrol | None = None
        self._perception: Perception | None = None

    def __call__(self, navigator: Navigator) -> str:
        drone = LocalDrone(navigator.drone, navigator.limits)
        if self._patrol is None:
            view, spacing, alt = plan_view(self.args, drone.state())
            self._perception = Perception(drone, self.args, self.args.corridor)
            self._patrol = SweepPatrol(self.args, self._perception, view, spacing, alt)
        else:
            self._patrol.resume(drone.state())
        try:
            return loop(self.args, drone, self._perception, self._patrol)
        except SweepFinished as done:
            return f"sweep done: {done}"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser, flight = new_parser(__doc__.splitlines()[0])
    flight.add_argument("--side", choices=("right", "left"), default="right", help="first side")
    flight.add_argument("--lane", type=float, help="lane spacing, m (default: camera footprint)")
    flight.add_argument("--overlap", type=float, default=DEFAULT_OVERLAP, help="of the footprint")
    flight.add_argument("--alt", type=float, default=2.0, help="m above takeoff, 0 = keep")
    flight.add_argument(
        "--stop-at", type=float, default=1.0, help="m, at least the floor's near edge + 0.15"
    )
    flight.add_argument("--clear-at", type=float, default=2.0, help="an obstacle has gone (m)")
    flight.add_argument("--wait", type=float, default=3.0, help="s to wait for an obstacle")
    flight.add_argument(
        "--corridor", type=float, default=0.8, help="m each side of the path that must be free"
    )
    flight.add_argument("--length", type=float, help="m: lanes no longer than this")
    flight.add_argument("--width", type=float, help="m: sweep no wider than this")
    flight.add_argument("--yaw-input", type=float, default=1.0, help="yaw input 0..1 in turns")
    flight.add_argument(
        "--sticks-speed", type=float, default=0.7, help="speed factor while heading=sticks"
    )
    flight.add_argument(
        "--revisit", action=argparse.BooleanOptionalAction, default=True, help="unseen patches"
    )
    sim = parser.add_argument_group("sweep simulator (--sim)")
    sim.add_argument("--sim-hall", type=float, nargs=2, metavar=("LENGTH", "WIDTH"), help="m")
    sim.add_argument("--sim-frame-offset", type=float, default=0.0, help="odometry vs heading")
    sim.add_argument("--sim-yaw-error", type=float, default=0.0, help="stick model rate error")
    return parse_patrol_args(parser, argv)


def main() -> None:
    from drone.patrol import sweep_sim

    sweep_sim.fly(parse_args(), run)


if __name__ == "__main__":
    main()
