"""Bring the drone back to the robot and land on its pad, when it is time.

    drone/fly 100 sd --fence 15 --max-alt 4      # terminal 1: drone + page
    drone/find jorge --sweep                     # terminal 2: find the child, hover over them
    drone/dock --arrived                         # terminal 3: waits, then lands on the robot
                                                 # (robot through ngrok; or --arrived <target>)
    drone/dock --care http://<robot>:8090        # also/or: once the parent hung up (8090:
                                                 # demo.launch's care port)
    drone/dock --now                             # land on the robot right away
    drone/dock --pad-wait 120                    # land 2 min after the robot's pad came in view
    drone/dock --now --sim --sim-robot 3 1       # simulator: fake robot standing at (3, 1)

It waits for the first of:
- the robot's pad (or the robot) in view for --pad-wait seconds: it drove to the child near the
  hovering drone, which stays over the child until then, turned toward where the robot should be
  with the camera at PAD_WAIT_TILT_DEG (PAD_CONFIRM checks in a row start the clock: one frame
  could be a false sighting). The landing then starts from that view;
- the robot stands still at the child: its child_goal_server (--arrived, GET /status: the
  http://<robot>:8001 or ntfy:<topic> of drone/report_child.py) says "arrived" for this
  mission's report and its pose has not changed for STILL_S (on arrival it may still turn to
  look for the child's face). The pose is the server's last tf reading, without a time: if tf
  stops while the robot moves, it looks still;
- the parent hung up: the robot's care server (bobac/robot/care_server.py, GET /status) went
  from "video_call" to anything else (--care; the parent is with the child now);
- low battery: at or below --battery % (above the server's own return-home rule, which would
  land at the takeoff point, blind);
- the mission flying home by itself (patrol over without the child, or the child reported and
  drone/find's STOP missed): it would land at the takeoff point, blind, where the robot may no
  longer be. A return home of the server's safety rules is left alone.
Then STOP (the drone hovers, the mission ends) and drone/land_on_pad.py takes over with where the
robot should be, best first: --robot; the robot's live pose (ROS, GET /map/state, with the
mission's map <-> takeoff transform); where it drove to (the child reported: the mission's
goal, standoff_m short of the child on the line from the takeoff point); the takeoff point. It
flies near it, finds the robot and then its pad, and lands on it.
"""

import argparse
import json
import logging
import math
import time
import urllib.error
import urllib.request
from dataclasses import replace
from typing import Any

from drone.identify.feed import DroneServer
from drone.land_on_pad import (
    DEFAULT_URL,
    PAD_HEIGHT_M,
    Detect,
    DetectRobot,
    LandSettings,
    add_sim_arguments,
    detect_shown,
    detectors,
    land_on_pad,
    newest_frame,
    stop,
)
from drone.report_child import ROBOT_URL, HttpTarget, LinkError, mission_id, parse_target
from drone.sdk.client import DroneApiError, DroneClient, standoff_goal

POLL_S = 1.0
HTTP_TIMEOUT_S = 2.0
BATTERY_PCT = 35  # the server's own rule returns home at 25 % (SafetyLimits.low_battery_pct)
CALL_STATE = "video_call"  # bobac/robot/care.py
RETURNING = "returning"  # drone/sdk/mission.py
OWN_RETURNS = ("child not found", "child reported")  # the mission's go-home reasons we take over
DEFAULT_STANDOFF_M = 0.8  # MissionConfig.standoff_m
ARRIVED = "arrived"  # child_goal_server's state at the child
STILL_S = 3.0  # the robot's pose unchanged this long: it stands still
STILL_M = 0.05  # ...within this (tf jitter)
STILL_DEG = 5.0
STILL_READINGS = 2  # ...in this many readings in a row after the first
# Readings further apart than this are not compared: the robot's search for the child's face
# turns it once round in ~21 s (face_check, 0.3 rad/s), back to the very same pose
MAX_GAP_S = 15.0
PAD_CONFIRM = 3  # --pad-wait: the pad (or the robot) in this many checks in a row starts the clock
# --pad-wait: camera tilt while waiting, inside the pad detector's range (25-75 deg down, pad.py
# TILT_DEG). From 3 m it sees the floor from right below the drone to ~2 m ahead, where the robot
# stops (0.8 m short of the child, under the drone).
PAD_WAIT_TILT_DEG = -70.0
ACTION_POLL_S = 0.2
NTFY_POLL_S = 10.0  # each poll is a listen and a publish; ntfy.sh refills one request per 5 s

log = logging.getLogger("dock")


class RobotArrival:
    """Whether the robot reached the child and stands still, from its child_goal_server's
    GET /status (reached like drone/report_child.py reaches it: directly or through ntfy)."""

    def __init__(self, target: str) -> None:
        self.target = parse_target(target)
        self.poll_s = POLL_S if isinstance(self.target, HttpTarget) else NTFY_POLL_S
        self._next_poll = 0.0
        self._was_away = False  # seen not arrived since we started: an "arrived" is this run's
        self._announced = False
        self._pose: dict[str, float] | None = None  # where it has been standing...
        self._still_since = 0.0  # ...since when...
        self._still_readings = 0  # ...and how many readings in a row found it there again
        self._read_at = 0.0  # the last reading of its pose

    def stopped(self, mission: dict[str, Any]) -> bool:
        """True once the robot arrived for this mission's report (not a leftover "arrived" of
        an earlier run) and STILL_READINGS readings in a row, over STILL_S or more, found it
        where it stood. A missed reading, one without a usable pose, or one more than MAX_GAP_S
        after the last starts that over."""
        asked = time.monotonic()
        if asked < self._next_poll:
            return False
        self._next_poll = asked + self.poll_s  # from the asking: a slow answer adds no gap
        try:
            status = self.target.ping()
        except LinkError as error:
            self._start_over(f"no answer from the robot ({error})")
            return False
        arrived = status.get("state") == ARRIVED
        self._was_away = self._was_away or not arrived
        ours = status.get("mission_id") == mission_id(mission)
        pose = _pose(status)
        if not arrived or pose is None or not (ours or self._was_away):
            self._pose = None
            return False
        if not self._announced:
            log.info("the robot arrived at the child: waiting for it to stand still")
            self._announced = True
        now = time.monotonic()
        if now - self._read_at > MAX_GAP_S:
            self._start_over(f"its last two answers came {now - self._read_at:.0f} s apart")
        if self._pose is not None and not _moved(self._pose, pose):
            self._still_readings += 1
        else:
            self._pose, self._still_since, self._still_readings = pose, now, 0
        self._read_at = now
        return self._still_readings >= STILL_READINGS and now - self._still_since >= STILL_S

    def _start_over(self, why: str) -> None:
        if self._pose is not None:
            log.info("checking again that the robot stands still: %s", why)
        self._pose = None


class PadWait:
    """Whether the robot's pad (else the robot) has been in view for `wait_s`, from the video:
    each check looks at the newest frame (its box goes on the page's video)."""

    def __init__(
        self, client: DroneClient, detect: Detect, detect_robot: DetectRobot | None, wait_s: float
    ) -> None:
        self.client, self.detect, self.detect_robot, self.wait_s = (
            client,
            detect,
            detect_robot,
            wait_s,
        )
        self._overlay = DroneServer(client.base_url)
        self._frame_time = 0.0
        self._in_a_row = 0
        self._seen_at: float | None = None  # monotonic time of the confirmed sighting
        self.fired = False

    def look_at_robot(self) -> None:
        """Turn on the spot toward where the robot should be, camera to PAD_WAIT_TILT_DEG: the
        drone stays over the child and the pad comes into view when the robot gets there."""
        robot, source = robot_position(self.client)
        pose = self.client.status()["telemetry"]["pose"]
        bearing = math.degrees(math.atan2(robot[1] - pose["y"], robot[0] - pose["x"]))
        log.info(
            "waiting for the robot's pad: facing %s, camera %.0f deg", source, PAD_WAIT_TILT_DEG
        )
        self.client.turn(heading=bearing)
        _wait_action(self.client, "turn")
        self.client.tilt(PAD_WAIT_TILT_DEG)
        _wait_action(self.client, "tilt")

    def due(self) -> bool:
        try:
            frame = newest_frame(self.client, self._frame_time)
        except DroneApiError as error:  # no frame yet, wifi
            log.debug("no frame: %s", error)
            frame = None
        if frame is not None:
            self._frame_time = frame.time
            seen = detect_shown(self.detect, self._overlay, frame, self.detect_robot)
            self._in_a_row = self._in_a_row + 1 if seen.pad or seen.robot else 0
            if self._seen_at is None and self._in_a_row >= PAD_CONFIRM:
                self._seen_at = time.monotonic()
                what = "pad" if seen.pad else "robot"
                log.info("the robot's %s is in view: landing on it in %.0f s", what, self.wait_s)
        self.fired = self._seen_at is not None and time.monotonic() - self._seen_at >= self.wait_s
        return self.fired


def _wait_action(client: DroneClient, action: str) -> None:
    """Until the server's action (turn, tilt) ends."""
    while client.status()["state"] == action:
        time.sleep(ACTION_POLL_S)


def _pose(status: dict[str, Any]) -> dict[str, float] | None:
    """The status's pose when it is a whole {x, y, yaw} of finite numbers, else None."""
    pose = status.get("pose")
    try:
        values = {key: float(pose[key]) for key in ("x", "y", "yaw")}
    except (TypeError, KeyError, ValueError, OverflowError):
        return None
    return values if all(math.isfinite(value) for value in values.values()) else None


def _moved(before: dict[str, float], after: dict[str, float]) -> bool:
    turned = math.degrees(abs(math.remainder(after["yaw"] - before["yaw"], math.tau)))
    shifted = math.dist((before["x"], before["y"]), (after["x"], after["y"]))
    return shifted > STILL_M or turned > STILL_DEG


def wait_for_trigger(
    client: DroneClient,
    care_url: str | None,
    battery_pct: float,
    arrival: RobotArrival | None = None,
    pad_wait: PadWait | None = None,
) -> str:
    """Block until it is time to go back to the robot; why. RuntimeError if the drone landed."""
    in_call = False
    while True:
        status = client.status()
        telemetry = status["telemetry"]
        if not telemetry["in_flight"]:
            raise RuntimeError("the drone is not flying")
        battery = telemetry.get("battery_pct")
        if battery is not None and battery <= battery_pct:
            return f"battery at {battery} %"
        mission = client.mission()
        step = mission.get("step") or ""
        if mission.get("state") == RETURNING and step.startswith(OWN_RETURNS):
            return f"the mission is flying home ({step}): landing on the robot instead"
        if arrival is not None and mission.get("child") and arrival.stopped(mission):
            return "the robot stands still at the child"
        if pad_wait is not None and pad_wait.due():
            return f"the robot's pad has been in view for {pad_wait.wait_s:.0f} s"
        if care_url is not None:
            state = care_state(care_url)
            if state == CALL_STATE and not in_call:
                log.info("the parent is on the video call: waiting for them to hang up")
            if in_call and state is not None and state != CALL_STATE:
                return "the parent hung up"
            in_call = in_call or state == CALL_STATE
        time.sleep(POLL_S)


def care_state(care_url: str) -> str | None:
    """The robot's care state (GET /status "state"), None when the robot cannot be reached."""
    try:
        with urllib.request.urlopen(f"{care_url}/status", timeout=HTTP_TIMEOUT_S) as reply:
            return json.load(reply).get("state")
    except (OSError, ValueError) as error:
        log.debug("care server: %s", error)
        return None


def robot_position(client: DroneClient) -> tuple[tuple[float, float], str]:
    """Where the robot should be in the takeoff frame, and how that was found."""
    mission = client.mission()
    live = _live_robot(client, mission)
    if live is not None:
        return live, "the robot's live pose"
    child = (mission.get("child") or {}).get("drone")
    if child is not None:
        config = mission.get("config") or {}
        standoff = config.get("standoff_m", DEFAULT_STANDOFF_M)
        x, y, _yaw = standoff_goal(0.0, 0.0, child["x"], child["y"], standoff)
        return (x, y), f"where it drove to: {standoff:.1f} m short of the child"
    return (0.0, 0.0), "the takeoff point"


def _live_robot(client: DroneClient, mission: dict[str, Any]) -> tuple[float, float] | None:
    """The robot's map pose (ROS, not stale) in the takeoff frame, via the mission's transform:
    the takeoff frame in `map` as the ROS frame drone_takeoff = (x, -y) of the takeoff frame."""
    transform = mission.get("map_from_drone")
    try:
        robot = client.map_state().get("robot")
    except DroneApiError:
        return None
    if transform is None or robot is None or robot.get("stale"):
        return None
    dx, dy = robot["x"] - transform["x"], robot["y"] - transform["y"]
    cos, sin = math.cos(transform["yaw"]), math.sin(transform["yaw"])
    ahead, left = cos * dx + sin * dy, -sin * dx + cos * dy
    return ahead, -left


def main() -> None:
    parser = argparse.ArgumentParser(description="land on the robot when it is time")
    parser.add_argument("--url", default=DEFAULT_URL, help="drone/fly (or the simulator)")
    parser.add_argument(
        "--arrived",
        nargs="?",
        const=ROBOT_URL,
        metavar="TARGET",
        help="go back once the robot stands still at the child: its child_goal_server, "
        f"http://<robot>:8001 or ntfy:<topic> (alone: {ROBOT_URL})",
    )
    parser.add_argument("--care", help="the robot's care server, e.g. http://<robot>:8090")
    parser.add_argument("--battery", type=float, default=BATTERY_PCT, help="go back at this %%")
    parser.add_argument("--now", action="store_true", help="go back right away")
    parser.add_argument(
        "--pad-wait", type=float, metavar="S", help="land S seconds after the pad came in view"
    )
    parser.add_argument(
        "--robot", type=float, nargs=2, metavar=("X", "Y"), help="the robot, takeoff frame m"
    )
    parser.add_argument("--pad-height", type=float, default=PAD_HEIGHT_M, help="m above floor")
    parser.add_argument("--takeoff-height", type=float, default=0.0, help="m above the floor")
    add_sim_arguments(parser)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    client = DroneClient(args.url)
    try:
        arrival = RobotArrival(args.arrived) if args.arrived else None
    except ValueError as error:
        parser.error(str(error))
    detect, detect_robot = detectors(client, args)
    pad_wait = (
        None if args.pad_wait is None else PadWait(client, detect, detect_robot, args.pad_wait)
    )
    camera = _record(client, True)
    try:
        if pad_wait is not None and not args.now:
            pad_wait.look_at_robot()
        reason = (
            "now"
            if args.now
            else wait_for_trigger(client, args.care, args.battery, arrival, pad_wait)
        )
    except (RuntimeError, DroneApiError) as error:
        if camera:
            _record(client, False)
        raise SystemExit(f"not docking: {error}") from error
    except KeyboardInterrupt:
        print("stopped waiting: the drone was not touched")
        if camera:
            _record(client, False)
        return
    log.info("going back to the robot: %s", reason)
    stop(client)  # the drone hovers; the mission (or its flight home) ends here
    settings = LandSettings(pad_height_m=args.pad_height, takeoff_height_m=args.takeoff_height)
    if pad_wait is not None and pad_wait.fired:  # the pad is in view: land from this view
        settings = replace(settings, search_tilt_deg=PAD_WAIT_TILT_DEG)
    else:
        robot, source = (tuple(args.robot), "--robot") if args.robot else robot_position(client)
        log.info("the robot should be at (%.1f, %.1f): %s", robot[0], robot[1], source)
        settings = replace(settings, robot_xy=robot)
    try:
        print(land_on_pad(client, detect, settings, detect_robot))
    except KeyboardInterrupt:
        stop(client)
        print("stopped: the drone hovers where it is")
    if camera:
        _record(client, False)


def _record(client: DroneClient, on: bool) -> bool:
    """Start or stop drone/fly's camera recording (the docking flight is on the video too)."""
    try:
        reply = client.record(on)
    except DroneApiError as error:  # e.g. already recording from the page: leave it
        log.warning("camera recording: %s", error)
        return False
    if not on:
        print(f"camera video: {reply.get('path')}")
    return bool(reply.get("ok"))


if __name__ == "__main__":
    main()
