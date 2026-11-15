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
    drone/find jorge --watch         # no flight: recognise and track on the live video only

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
import time
from collections import deque
from pathlib import Path
from typing import Any

from drone.identify.__main__ import load_people
from drone.identify.body_track import LOCK_FACE_FRAMES, SEARCHING
from drone.identify.feed import (
    DroneServer,
    fractions,
    make_tracker_for,
    mission_body,
    run_live,
    vision_body,
)
from drone.identify.people import PEOPLE_DIR, People, is_photo
from drone.sdk.client import DroneApiError, DroneClient
from drone.sdk.mission import END_STATES, IDLE, REPORTING, RETURNING

DEFAULT_URL = "http://127.0.0.1:8077"
# Frame "launch": x ahead of the camera at takeoff, y to its left, metres
DEFAULT_WAYPOINTS = "2,-1.5 4,-1.5 4,1.5 2,1.5"
SWEEP_WAYPOINTS = [{"x": 2.0, "y": 0.0}]  # unused by an external patrol, but required
STATUS_EVERY_S = 0.2  # the mission hovers 3 s when it reports: plenty of polls to catch it
IDLE_S = 0.05  # no new frame yet
TILT_POLL_S = 0.5
MANUAL_WAIT_S = 10.0  # for the sticks to unlock after Manual Control
# A glimpse (the mission turns toward it) needs this many frames with the child's face...
GLIMPSE_FRAMES = 2
GLIMPSE_WINDOW_S = 1.0  # ...this close together (frame times)
SWEEP_ALIGN_M = 2.5  # the mission's aligning leg is the sweep's lane axis: longer, steadier
SWEEP_EXIT_S = 5.0  # after terminate(), before kill()
CHILD_REPORTED = "child reported"  # the mission's step flying home after the orbit (mission.py)
REPO = Path(__file__).resolve().parent.parent
LIVE_HELPER = REPO / "drone" / "patrol" / "live"
RECORDINGS = REPO / "recordings"

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


def tilt_on_ground(client: DroneClient, deg: float) -> None:
    """Find the gimbal's stops and set `deg` before takeoff (the drone's own tilt task). Right
    after Manual Control the sticks stay locked until the drone reports MANUAL: retry a while."""
    deadline = time.monotonic() + MANUAL_WAIT_S
    while True:
        try:
            client.tilt(deg)
            break
        except DroneApiError as error:
            if time.monotonic() < deadline:
                time.sleep(TILT_POLL_S)
                continue
            log.warning(
                "camera not tilted on the ground (%s): the mission tilts it in the air", error
            )
            return
    log.info("camera to %.0f deg on the ground: watch the video tilt up, then down", deg)
    while client.status()["state"] == "tilt":
        time.sleep(TILT_POLL_S)
    log.info("camera tilt %s", client.status()["telemetry"].get("tilt"))


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


def search(
    client: DroneClient, server: DroneServer, name: str, people: People, args: argparse.Namespace
) -> dict[str, Any]:
    """Feed the mission until it is done with the child (then STOP: the drone hovers there) or
    ends. Returns the mission's last status."""
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
        "--watch", action="store_true", help="no flight: recognise and track on the video only"
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
        log.info("watching for %s on %s: boxes on the page, no flight", name, args.url)
        try:
            run_live(name, args.url, people)
        except KeyboardInterrupt:
            print("stopped watching")
        return
    if not telemetry["in_flight"]:
        tilt_on_ground(client, args.tilt)
    patrol_config = (
        {"waypoints": SWEEP_WAYPOINTS, "strategy": "external", "align_m": SWEEP_ALIGN_M}
        if args.sweep
        else {"waypoints": parse_waypoints(args.waypoints), "strategy": "waypoints"}
    )
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
        mission = search(client, DroneServer(args.url), name, people, args)
    except KeyboardInterrupt:
        client.stop()
        print("stopped: the drone hovers where it is")
        return
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


if __name__ == "__main__":
    main()
