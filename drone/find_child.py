"""Find the child and stop right over them: the first real-world run of the lost-child search.

The drone climbs to the patrol altitude and patrols (the lawnmower sweep with --sweep, else a
square of waypoints ahead of the takeoff point). A glimpse of the child's face (2 frames within
1 s) makes it stop and turn toward it on the spot (the mission's look-around), then patrol on
if the face does not come back. Enough face frames lock on the child; from there the body is
tracked, the drone turns to it, flies to it and puts the camera straight down over it (the
position fix; the child may be a little off centre), then orbits the child facing them (--orbit
turns at --radius, at the patrol altitude unless --orbit-alt; the circle follows them if they
step aside) and hovers there while the robot drives to the child. drone/dock then lands it on
the robot's pad (or LAND in the web page).

    drone/fly 100 sd --fence 15 --max-alt 4   # terminal 1: drone + page, http://127.0.0.1:8077
    drone/find jorge --sweep         # terminal 2, then the drone's button (--side left: first
                                     #   lane turn to the left; --length/--width bound the hall)
    drone/find jorge                 # the square of waypoints instead of the sweep
    drone/find jorge --watch         # no flight command: recognise and track while you fly by
                                     #   hand from the page; records each flight, takeoff to landing
    drone/find jorge --collect       # also save frames for the body model (identify/collect.py)

The child is a folder of face photos (copied into drone/identify/people/<folder name>/, which
git ignores) or the name of a folder already there. Other people's folders stay in the database:
a face that looks more like someone else is never taken for the child.

Before takeoff the camera is set to the patrol tilt on the ground (the gimbal needs a full
sweep to find its stops; in the air that would cost ~15 s): watch the video tilt up, then down.

How it works: the mission (drone/sdk/mission.py) flies the patrol, the look-around and the
approach; this script is its detector (the tracker of drone/identify/feed.py: face first, then
the body; glimpses to POST /mission/glimpse) and presses STOP as soon as the mission reports
the child (or, with an orbit, once it is done), so the drone stays there. If that STOP is
missed, the mission flies back to the takeoff point and lands; any other return home (patrol
timeout, abort, failure, a safety rule) is left alone. With --sweep it also runs the sweep
(drone/patrol/live sweep, its output in recordings/sweep-<time>.log). Ctrl+C or Space / STOP
in the page: the drone hovers where it is.
"""

import argparse
import logging
import os
import shutil
import subprocess
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any

from drone.identify.__main__ import load_people
from drone.identify.body_track import LOCK_FACE_FRAMES, SEARCHING
from drone.identify.boxes_video import BoxesVideo
from drone.identify.collect import Collector
from drone.identify.feed import (
    DroneServer,
    fractions,
    make_tracker_for,
    mission_body,
    run_live,
    vision_body,
)
from drone.identify.people import PEOPLE_DIR, People, is_photo
from drone.report_child import ROBOT_URL, child_report, deliver, finish, follow, mission_id
from drone.sdk.client import DroneApiError, DroneClient, tilt_on_ground
from drone.sdk.mission import END_STATES, IDLE, REPORTING, RETURNING

DEFAULT_URL = "http://127.0.0.1:8077"
# Frame "launch": x ahead of the camera at takeoff, y to its left, metres
DEFAULT_WAYPOINTS = "2,-1.5 4,-1.5 4,1.5 2,1.5"
SWEEP_WAYPOINTS = [{"x": 2.0, "y": 0.0}]  # unused by an external patrol, but required
STATUS_EVERY_S = 0.2  # the mission hovers 3 s when it reports: plenty of polls to catch it
IDLE_S = 0.05  # no new frame yet
# A glimpse (the mission turns toward it) needs this many frames with the child's face...
GLIMPSE_FRAMES = 2
GLIMPSE_WINDOW_S = 1.0  # ...this close together (frame times)
SWEEP_ALIGN_M = 2.5  # the mission's aligning leg is the sweep's lane axis: longer, steadier
SWEEP_EXIT_S = 5.0  # after terminate(), before kill()
CHILD_REPORTED = "child reported"  # the mission's step flying home after the orbit (mission.py)
REPO = Path(__file__).resolve().parent.parent
LIVE_HELPER = REPO / "drone" / "patrol" / ("live.cmd" if sys.platform == "win32" else "live")
RECORDINGS = REPO / "recordings"
# Exit codes, so drone/demo stops instead of going on to land on the robot
INTERRUPTED = 130  # Ctrl+C (the shell's own code for it)
NOT_FOUND = 1  # the mission ended without the child
FLIGHT_POLL_S = 0.5  # --watch: how often it checks whether the drone took off or landed

log = logging.getLogger("find_child")


def add_child(face: str) -> str:
    """The child's name in the database. A folder of photos is copied in under its own name."""
    folder = Path(face).expanduser()
    if not folder.is_dir():
        return face
    photos = [path for path in folder.iterdir() if is_photo(path)]
    if not photos:
        raise SystemExit(f"no .jpg, .jpeg or .png photo in {folder}")
    target = PEOPLE_DIR / folder.name
    if folder.resolve() != target.resolve():
        target.mkdir(parents=True, exist_ok=True)
        for photo in photos:
            shutil.copy2(photo, target / photo.name)
        log.info("%d photos of %s copied to %s", len(photos), folder.name, target)
    return folder.name


def parse_waypoints(text: str) -> list[dict[str, float]]:
    """ "x,y x,y ..." -> [{"x": x, "y": y}, ...]"""
    points = []
    for pair in text.split():
        x, y = pair.split(",")
        points.append({"x": float(x), "y": float(y)})
    return points


def start_sweep(args: argparse.Namespace) -> subprocess.Popen:
    """drone/patrol/live sweep, flying for the mission (strategy "external")."""
    command = [str(LIVE_HELPER), "sweep", "--pitch", f"{-args.tilt:g}", "--side", args.side]
    command += ["--alt", f"{args.alt:g}"]
    if args.length:
        command += ["--length", f"{args.length:g}"]
    if args.width:
        command += ["--width", f"{args.width:g}"]
    RECORDINGS.mkdir(exist_ok=True)
    path = RECORDINGS / f"sweep-{time.strftime('%Y%m%d-%H%M%S')}.log"
    env = {"DVD_TAKEOFF_HEIGHT": f"{args.takeoff_height:g}", "SDK": args.url}
    log.info("sweep: %s (output in %s)", " ".join(command[1:]), path)
    with path.open("w") as output:  # the subprocess has its own copy
        return subprocess.Popen(
            command, stdout=output, stderr=subprocess.STDOUT, env={**os.environ, **env}, cwd=REPO
        )


def end_sweep(sweep: subprocess.Popen) -> None:
    if sweep.poll() is not None:
        return
    if sys.platform == "win32":
        # terminate() would end only the launcher (cmd), and the sweep under it would fly on
        tree = ["taskkill", "/T", "/F", "/PID", str(sweep.pid)]
        subprocess.run(tree, capture_output=True, check=False)
    else:
        sweep.terminate()
    try:
        sweep.wait(SWEEP_EXIT_S)
    except subprocess.TimeoutExpired:
        sweep.kill()


def load_child(name: str) -> People:
    """The face database, checked to have the child (before any flight)."""
    people = load_people(PEOPLE_DIR)
    if name not in people.names:
        raise SystemExit(f"{name} has no usable face photo in {PEOPLE_DIR} ({people.names})")
    return people


def earliest_report(mission: dict[str, Any]) -> dict[str, Any]:
    """The mission with its child report, or, before it, the approach's estimate in its place
    (same id: deliver() sends one of the two, whichever comes first)."""
    if mission.get("child") is None and mission.get("child_estimate") is not None:
        return {**mission, "child": mission["child_estimate"]}
    return mission


def done_with_child(mission: dict[str, Any], args: argparse.Namespace) -> bool:
    """Over the child (no orbit), or flying home after the orbit around them: time to STOP.
    Not a return for a patrol timeout, an abort, a failure or a safety rule (those fly home
    and land)."""
    if args.orbit <= 0:
        return mission["state"] == REPORTING
    return (
        mission["state"] == RETURNING
        and mission.get("step") == CHILD_REPORTED
        and mission.get("child") is not None
    )


class FlightRecording:
    """The watch's recordings, from each takeoff to its landing (nothing while on the ground):
    the camera as it is (drone/fly's MP4 of the stream), the tracker's view with the boxes
    (recordings/boxes-<child>-<time>.mp4) and, with --collect, frames for the body model. A
    frame sink of feed.run_live: it checks the drone's flight state every FLIGHT_POLL_S."""

    def __init__(self, client: DroneClient, name: str, collect: bool) -> None:
        self._client, self._name, self._collect = client, name, collect
        self._next_poll = 0.0
        self._video: BoxesVideo | None = None
        self._collector: Collector | None = None
        self._camera = False  # drone/fly is recording the camera for us

    def add(self, image: Any, result: Any, frame_time: float) -> None:
        if time.monotonic() >= self._next_poll:
            self._next_poll = time.monotonic() + FLIGHT_POLL_S
            flying = self._in_flight()
            if flying and self._video is None:
                self._start()
            elif flying is False and self._video is not None:
                log.info("landed")
                self.stop()
        if self._video is not None:
            self._video.add(image, result, frame_time)
        if self._collector is not None:
            self._collector.add(image, result, frame_time)

    def stop(self) -> None:
        """End the recordings of this flight, if any, and print where they are."""
        if self._video is None:
            return
        if self._camera:
            try:
                print(f"camera video: {self._client.record(False).get('path')}")
            except DroneApiError as error:
                log.warning("camera recording: %s", error)
        path = self._video.close()
        print(f"video with the boxes: {path}" if path else "video with the boxes: no frame")
        report_collected(self._collector)
        self._video, self._collector, self._camera = None, None, False

    def _in_flight(self) -> bool | None:
        """None when drone/fly does not answer: keep recording (or not) as before."""
        try:
            return bool(self._client.status()["telemetry"]["in_flight"])
        except DroneApiError:
            return None

    def _start(self) -> None:
        self._video = BoxesVideo(RECORDINGS, self._name)
        self._collector = Collector(RECORDINGS) if self._collect else None
        try:
            self._camera = bool(self._client.record(True).get("ok"))
        except DroneApiError as error:  # e.g. already recording from the page: keep that one
            log.warning("camera recording: %s", error)
        log.info("takeoff: recording (boxes in %s)", self._video.path)


def search(
    client: DroneClient,
    server: DroneServer,
    name: str,
    people: People,
    args: argparse.Namespace,
    started_at: float,
    recording: FlightRecording,
) -> dict[str, Any]:
    """Feed the mission until it is done with the child (then STOP: the drone hovers there) or
    ends. Returns the mission's last status. With --robot the child goes to the robot as soon as
    the mission reports them (in background threads: the flight loop never waits for it)."""
    tracker = make_tracker_for(name, people)
    # at once: it loads its depth model during the takeoff and waits for the mission's patrol
    sweep = start_sweep(args) if args.sweep else None
    face_times: deque[float] = deque(maxlen=GLIMPSE_FRAMES)
    started = False
    last_mission = last_vision = None
    next_status = 0.0
    try:
        while True:
            if time.monotonic() >= next_status:
                next_status = time.monotonic() + STATUS_EVERY_S
                mission = client.mission()
                state = mission["state"]
                if args.robot:
                    deliver(
                        earliest_report(mission),
                        [args.robot],
                        started_at,
                        args.heading_offset,
                        args.launch_ahead,
                    )
                if done_with_child(mission, args):
                    client.stop()  # hover there
                    return mission
                started = started or state != IDLE
                if started and state in END_STATES:
                    return mission
                now = f"{state}: {mission.get('step')}"
                if now != last_mission:
                    log.info("mission %s", now)
                    last_mission = now
            frame = server.next()
            if frame is None:
                time.sleep(IDLE_S)
                continue
            frame_time, image = frame
            result = tracker.update(image, frame_time)
            recording.add(image, result, frame_time)
            server.post("/vision", vision_body(result, frame_time, image.shape, name))
            report = mission_body(result, frame_time, image.shape, name)
            if report:
                server.post("/mission/target", report)
            face = max(
                (f for f in result.faces if f.target), key=lambda f: f.similarity, default=None
            )
            glimpsed = result.state == SEARCHING and face is not None
            if glimpsed and _confirmed(face_times, frame_time):
                glimpse = {"frame_time": frame_time, "bbox": fractions(face.box, image.shape)}
                server.post("/mission/glimpse", glimpse)
            now = _vision_line(result)
            if now != last_vision:
                log.info("vision %s", now)
                last_vision = now
    finally:
        if sweep is not None:
            end_sweep(sweep)


def _confirmed(face_times: deque[float], frame_time: float) -> bool:
    """Note a frame with the child's face: True once the last GLIMPSE_FRAMES of them fall
    within GLIMPSE_WINDOW_S (one alone is often someone else matched for a frame)."""
    face_times.append(frame_time)
    return (
        len(face_times) == face_times.maxlen and face_times[-1] - face_times[0] <= GLIMPSE_WINDOW_S
    )


def _vision_line(result: Any) -> str:
    if result.target is not None:
        return f"{result.state}: {result.target.kind}"  # not the source: it flips every frame
    if result.state != SEARCHING:
        return f"{result.state}: out of sight, lock kept"
    return f"{result.state}: face {result.face_frames}/{LOCK_FACE_FRAMES}"


def main() -> None:
    parser = argparse.ArgumentParser(description="patrol, find the child, hover over them")
    parser.add_argument("child", help="folder of face photos, or a name in drone/identify/people")
    parser.add_argument("--url", default=DEFAULT_URL, help="drone/fly (or the simulator)")
    parser.add_argument("--alt", type=float, default=3.0, help="patrol altitude above takeoff, m")
    parser.add_argument("--tilt", type=float, default=-40.0, help="camera tilt while patrolling")
    parser.add_argument("--sweep", action="store_true", help="lawnmower sweep, not waypoints")
    parser.add_argument(
        "--side", choices=("right", "left"), default="right", help="--sweep: first lane turn"
    )
    parser.add_argument("--length", type=float, help="--sweep: lanes no longer than this, m")
    parser.add_argument("--width", type=float, help="--sweep: sweep no wider than this, m")
    parser.add_argument(
        "--waypoints", default=DEFAULT_WAYPOINTS, help='patrol, "x,y x,y ...": x ahead, y left'
    )
    parser.add_argument(
        "--look", type=float, default=180.0, help="max turn after a glimpse, deg (0 = none)"
    )
    parser.add_argument(
        "--orbit", type=float, default=1.0, help="turns around the child, facing them (0 = none)"
    )
    parser.add_argument(
        "--orbit-alt", type=float, help="orbit altitude, m (default: stay at the patrol altitude)"
    )
    parser.add_argument("--radius", type=float, default=1.5, help="orbit radius, m")
    parser.add_argument("--takeoff-height", type=float, default=0.0, help="m above the floor")
    parser.add_argument(
        "--robot",
        nargs="?",
        const=ROBOT_URL,
        metavar="TARGET",
        help=f"send the child to the robot and follow it there (alone: {ROBOT_URL})",
    )
    parser.add_argument(
        "--heading-offset",
        type=float,
        default=0.0,
        help="--robot: degrees the camera pointed left of the robot's front at takeoff",
    )
    parser.add_argument(
        "--launch-ahead",
        type=float,
        default=0.0,
        help="--robot: metres in front of the robot the drone took off",
    )
    parser.add_argument(
        "--watch",
        action="store_true",
        help="fly by hand: recognise, track and record the boxes (no flight command)",
    )
    parser.add_argument(
        "--collect", action="store_true", help="save frames for the body model (collect.py)"
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    name = add_child(args.child)
    people = load_child(name)
    client = DroneClient(args.url)
    try:
        telemetry = client.status()["telemetry"]
    except DroneApiError as error:
        raise SystemExit(f"no drone at {args.url} ({error}): start drone/fly first") from error
    if not telemetry["in_flight"] and not telemetry["manual_session"]:
        try:  # Manual Control also starts the video
            client.manual()
        except DroneApiError as error:
            log.warning("manual control: %s (press MANUAL CONTROL in the page)", error)
    if args.watch:
        watch(client, name, people, args)
        return
    recording = FlightRecording(client, name, args.collect)
    if not telemetry["in_flight"]:
        tilt_on_ground(client, args.tilt)
    patrol_config = (
        {"waypoints": SWEEP_WAYPOINTS, "strategy": "external", "align_m": SWEEP_ALIGN_M}
        if args.sweep
        else {"waypoints": parse_waypoints(args.waypoints), "strategy": "waypoints"}
    )
    started_at = time.time()  # a child reported before this is an earlier mission's
    client.mission_start(
        **patrol_config,
        patrol_alt_m=args.alt,
        patrol_tilt_deg=args.tilt,
        orbit_turns=args.orbit,
        orbit_alt_m=args.orbit_alt,
        orbit_radius_m=args.radius,
        look_max_deg=args.look,
        launch_height_m=args.takeoff_height,
    )
    patrol = "the sweep" if args.sweep else "the waypoints"
    log.info("looking for %s at %.1f m with %s: press the drone's button", name, args.alt, patrol)
    try:
        mission = search(client, DroneServer(args.url), name, people, args, started_at, recording)
    except KeyboardInterrupt:
        client.stop()
        print("stopped: the drone hovers where it is")
        recording.stop()
        raise SystemExit(INTERRUPTED) from None
    child = mission.get("child")
    state, message = mission["state"], mission.get("message")
    if done_with_child(mission, args):
        where = f"orbited {name} {args.orbit:g} times" if args.orbit > 0 else f"over {name}"
        print(f"{where}: hovering (drone/dock lands it on the robot, or LAND in the page)")
    elif child:
        print(f"mission ended after reporting {name}: {state}: {message}")
    else:
        print(f"mission ended without the child: {state}: {message}")
    if child:
        print(f"child at {child.get('drone')} m from the takeoff point (drone frame)")
        print_for_robot(mission, name, args)
    try:
        if args.robot and child:
            follow_robot(mission, args, started_at)
    except KeyboardInterrupt:
        print("stopped following the robot: the drone hovers where it is")
        raise SystemExit(INTERRUPTED) from None
    finally:
        recording.stop()
    if not child:
        raise SystemExit(NOT_FOUND)


def print_for_robot(mission: dict[str, Any], name: str, args: argparse.Namespace) -> None:
    """The child in the robot's frame (what --robot sends), as the command that sends it by
    hand: x metres ahead of where the robot stood at takeoff, y to its left."""
    try:
        report = child_report(mission, args.heading_offset, args.launch_ahead)
    except ValueError as error:
        print(f"no robot-frame position for {name}: {error}")
        return
    x, y = report["x"], report["y"]
    print(f"{name} for the robot: {x:+.2f} m ahead, {y:+.2f} m left -> drone/robot {x:.2f} {y:.2f}")


def watch(client: DroneClient, name: str, people: People, args: argparse.Namespace) -> None:
    """Recognise and track on the live video while the drone is flown by hand (the page), no
    flight command; record each flight (FlightRecording)."""
    recording = FlightRecording(client, name, args.collect)
    log.info("watching for %s on %s: boxes on the page, recording from takeoff", name, args.url)
    try:
        run_live(name, args.url, people, [recording])
    except KeyboardInterrupt:
        print("stopped watching")
    recording.stop()


def report_collected(collector: Collector | None) -> None:
    if collector is not None:
        print(f"{collector.saved} frames for the body model in {collector.folder}")


def follow_robot(mission: dict[str, Any], args: argparse.Namespace, started_at: float) -> None:
    """Wait for the robot's acknowledgement of the report, then show it driving to the child
    until it arrives (the drone keeps hovering)."""
    # the delivery the search started (deliver() starts each report once), None if not sent
    sent = deliver(mission, [args.robot], started_at, args.heading_offset, args.launch_ahead)
    if sent is None or not finish():
        print("the robot did NOT get the child (see above): by hand, drone/robot <x> <y>")
        return
    arrived = follow(args.robot, mission_id(mission))
    print("the robot is at the child" if arrived else "the robot did NOT reach the child")


if __name__ == "__main__":
    main()
