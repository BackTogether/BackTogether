"""Land on the helipad on top of the robot: find the pad, fly over it, centre and go down on it.

    drone/fly 100 sd --fence 15 --max-alt 4   # terminal 1: drone + page, http://127.0.0.1:8077
    drone/land --watch     # no flight, never moves it: is the pad seen? (lines + page overlay)
    drone/land             # flying in MANUAL (after drone/find, or a manual takeoff): land it

Needs the pad model (drone/identify/pad.py: photos in drone/identify/base/, then
`python -m drone.identify.pad train`). Client-side, over the HTTP API, like find_child.py:

0. takes control (resumes /move: it was started on purpose) and climbs to --alt above the
   takeoff point if it is lower (POST /goto where it is). With the robot's expected position
   (--robot, or drone/dock_robot.py: the robot's live pose, where it went, or the takeoff point)
   it first flies to ROBOT_VIEW_M short of it and turns to face it (POST /goto, /turn).
1. searching: camera to --tilt (diagonal); if the pad is not in view, turn slowly on the spot
   (one full turn at most, then "pad not found" and hover: it never lands blind). Two detections
   that agree confirm the pad; one that is not confirmed in time is dropped and the turn goes on.
   Meanwhile the robot detector (drone/identify/robot.py, if trained) looks for the robot itself,
   visible from farther than its pad: two agreeing detections (its floor contact placed on the
   floor) and the drone flies to ROBOT_VIEW_M of it, where the pad comes into view.
2. approaching: each detection is placed on the pad's plane with its own frame's pose and camera
   tilt (X-Drone-Pose: the video latency does not matter) and kept in the takeoff frame; the
   drone steers toward it continuously (turns to face it far away, slides near it). /move cannot
   move the gimbal, so the camera follows the pad down in /tilt steps (the drone hovers ~2 s
   each): a step when the pad nears the bottom of the image, straight down (-90) when close.
3. descending: camera straight down, the same steering centres the drone over the pad while it
   descends whenever it is centred and the detection is fresh (no settle-and-sample). Height
   above the pad from the pad's size in the image, checked against the odometry: a detection
   whose size disagrees with it by more than REJECT_HEIGHT_M is ignored. It lands (POST /land)
   when LAND_FRAMES detections in a row agree on a height below LAND_HEIGHT_M (or the pad left
   the view just after they agreed on one below LOW_LOST_M), and the odometry agrees with them.
   The odometry height is read one of two ways (--alt-source): "floor" = alt + takeoff
   height - pad height (alt measured from the takeoff floor), "pad" = alt itself (a downward
   sensor over the robot reads the height above the pad), or "either" (the default for a first
   test: wider, so less safe against a false square). Status and --watch lines print both
   readings and which one the pad's size matches: hover beside the robot, slide over it, and
   if alt drops by about the pad height it is a downward sensor ("pad").
The pad lost for long, a timeout, Ctrl+C or a STOP from the page (any STOP after this script
took control: it never resumes on its own) end it with the drone hovering. When the server's
safety rules take the drone (flying home, riding out a loss) or the page did, it just ends,
without a STOP of its own (before its own STOP it sends a neutral /move: refused = not ours).
Every frame looked at is posted to the page's video overlay (POST /vision): the pad's box as
the locked target.
"""

import argparse
import logging
import math
import statistics
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from drone.identify.feed import DroneServer
from drone.identify.pad import PAD_SIDE_M, TILT_DEG, Pad, PadDetector
from drone.identify.robot import Robot, RobotDetector
from drone.sdk import camera
from drone.sdk.client import DroneApiError, DroneClient
from drone.sdk.estimate import wrap_deg
from drone.sdk.mission_geometry import body_to_frame, frame_to_body, ground_offset
from drone.sim_vision import SimVision

DEFAULT_URL = "http://127.0.0.1:8077"
LANDED = "landed"
OVERLAY_NAME = "helipad"
SEARCH_ALT_M = 2.0  # above the takeoff point
ALT_TOLERANCE_M = 0.15
PAD_HEIGHT_M = 0.4  # the pad above the floor (Bobac's top disc): measure it
CONFLICT = 409  # the server refused: a STOP, a safety rule or another action owns the drone
BUSY = "busy"  # the server's refusal while an action runs (not a STOP or a safety rule)
OWN_ACTIONS = ("goto", "tilt")  # server states while our own climb or camera move runs
ALT_SOURCES = ("either", "floor", "pad")  # which odometry reading is the height above the pad
# Camera
SEARCH_TILT_DEG = -50.0  # 1.6 m above the pad it sees it from ~0.5 to ~3 m away
DOWN_DEG = -90.0
OVERHEAD_TILT_DEG = -85.0  # this steep: straight down (the estimate clamps at -90)
TILT_STEP_DEG = 25.0  # the camera follows the pad down in steps this big
TILT_TOLERANCE_DEG = 5.0
LOW_V = 0.8  # pad centre below this image row (fraction, y down): next camera step
OVERHEAD_M = 0.3  # pad this close horizontally: camera straight down
# Search
TURN_STICK = 0.6  # ~15 deg/s: the X1 turns 24 deg/s at full stick (estimate.py)
FULL_TURN_DEG = 360.0
AGREE_M = 0.4  # two detections this close are the same pad
CONFIRM_S = 1.5  # ...if this close in time; an unconfirmed one is dropped after it
# The robot (seen from farther than its pad)
ROBOT_VIEW_M = 1.5  # fly to this far from the robot: its pad is then in the diagonal view
ROBOT_AGREE_M = 0.8  # two robot detections this close are the same robot (its box is coarse)
GOTO_MIN_M = 1.0  # expected robot position this much farther than ROBOT_VIEW_M: fly there first
# Steering (stick fractions of the speed cap; /move applies the server's max speed)
GAIN = 0.4  # horizontal stick per metre to the pad
MAX_STICK = 0.25
MIN_STICK = 0.06  # slowest useful horizontal stick
CENTRED_M = 0.05  # no horizontal command closer than this
NEAR_M = 0.8  # closer: slide instead of turning toward the pad
TURN_FIRST_DEG = 25.0  # pad further off the nose: turn on the spot first
YAW_GAIN = 1 / 30  # turn stick per degree of bearing
MAX_TURN = 0.6
# Descent and landing
DESCEND_STICK = 0.25
DESCEND_RADIUS_M = 0.15  # centred this well: descend while centring
FRESH_S = 0.5  # frames older than this are skipped; descend only on a detection this recent
LAND_HEIGHT_M = 0.5  # camera this high above the pad, centred: POST /land
LOW_LOST_M = 0.7  # pad lost this low (it no longer fits in the view), centred: land too...
LOW_LOST_S = 1.5  # ...if it agreed on that height this recently
LAND_FRAMES = 3  # overhead detections in a row that must agree before landing
HEIGHT_SPREAD_M = 0.15  # their size-based heights within this of each other
ODOMETRY_AGREE_M = 0.3  # and within this of the odometry height (--alt-source: see above)
REJECT_HEIGHT_M = 0.5  # overhead detection this far from the odometry height: ignored
MIN_HEIGHT_M = 0.3
# Time
LOST_TIMEOUT_S = 10.0  # pad not seen this long: give up, hover
TIMEOUT_S = 120.0
MOVE_HOLD_S = 0.4  # each /move expires by itself: a stuck script leaves the drone hovering
TICK_S = 0.05
WATCH_LOG_S = 0.5
ACTION_POLL_S = 0.2
MIN_SEARCH_ALT_M = 0.5

log = logging.getLogger("land")

Detect = Callable[[np.ndarray], Pad | None]
DetectRobot = Callable[[np.ndarray], Robot | None]


class Ended(Exception):
    """The landing stops here; the message says why. `owned`: the drone is still this script's
    (it sends STOP so the drone hovers), else a STOP, a safety rule or a pilot has it."""

    def __init__(self, reason: str, owned: bool = True) -> None:
        super().__init__(reason)
        self.owned = owned


@dataclass(frozen=True)
class LandSettings:
    pad_height_m: float = PAD_HEIGHT_M  # the pad above the floor
    takeoff_height_m: float = 0.0  # the takeoff point above the floor
    search_tilt_deg: float = SEARCH_TILT_DEG
    search_alt_m: float = SEARCH_ALT_M  # above the takeoff point; higher already: stays there
    alt_source: str = ALT_SOURCES[0]  # see the module docstring
    robot_xy: tuple[float, float] | None = None  # where the robot should be (takeoff frame)


@dataclass(frozen=True)
class Frame:
    time: float  # X-Frame-Time: unix time on the server's clock (the same Mac)
    pose: dict[str, Any] | None  # X-Drone-Pose
    image: np.ndarray


@dataclass(frozen=True)
class Sighting:
    """One detection placed in the takeoff frame."""

    point: tuple[float, float]
    v: float  # pad centre row, image fraction (y down)
    tilt_deg: float  # camera tilt of that frame
    height_m: float  # camera above the pad: from its size overhead, else the odometry
    floor_m: float  # odometry reading "floor": alt + takeoff height - pad height
    pad_m: float  # odometry reading "pad": alt itself
    seen_at: float  # the frame's time

    def odometry_gap(self, source: str) -> float:
        """Metres between the height and the odometry reading(s) of `source`."""
        readings = {"floor": (self.floor_m,), "pad": (self.pad_m,)}
        chosen = readings.get(source, (self.floor_m, self.pad_m))
        return min(abs(self.height_m - reading) for reading in chosen)


@dataclass(frozen=True)
class Seen:
    """What one frame showed: the pad, else (when asked) the robot."""

    pad: Pad | None
    robot: Robot | None
    ms: float


@dataclass(frozen=True)
class RobotSighting:
    point: tuple[float, float]  # the robot's floor contact, takeoff frame
    seen_at: float  # the frame's time


def land_on_pad(
    client: DroneClient,
    detect: Detect,
    settings: LandSettings | None = None,
    detect_robot: DetectRobot | None = None,
) -> str:
    """Land the hovering drone on the pad: LANDED, or why not (then it hovers, or the server's
    safety rules or the page have it). `detect_robot` finds the robot while the pad is not seen."""
    return Lander(client, detect, settings or LandSettings(), detect_robot).run()


def load_robot_detector() -> DetectRobot | None:
    """The robot detector if it was trained, else None (then only the pad is looked for)."""
    try:
        return RobotDetector().detect
    except FileNotFoundError as error:
        log.info("no robot detector (%s): looking for the pad only", error)
        return None


def floor_height(pose: dict[str, Any], settings: LandSettings) -> float:
    """The odometry reading "floor": alt measured from the takeoff floor."""
    return pose["alt"] + settings.takeoff_height_m - settings.pad_height_m


def heights_text(height: float, floor_m: float, pad_m: float, overhead: bool) -> str:
    """The camera's height above the pad and the two odometry readings; overhead (height from
    the pad's size) also which reading it matches."""
    text = f"{height:.2f} m above the pad (odometry: floor {floor_m:.2f}, pad {pad_m:.2f}"
    if not overhead:
        return text + ")"
    matches = [name for name, m in (("floor", floor_m), ("pad", pad_m))
               if abs(height - m) <= ODOMETRY_AGREE_M]  # fmt: skip
    return text + f"; size matches {' and '.join(matches) or 'neither'})"


def release(client: DroneClient) -> bool:
    """STOP so the drone hovers, unless it is not ours any more: a neutral /move first, and
    no STOP if it is refused (a STOP, or a safety rule flying it home). True if STOP was sent."""
    try:
        client.move(seconds=MOVE_HOLD_S)
    except DroneApiError as error:
        ours = error.message == BUSY and _own_action_runs(client)
        if error.status == CONFLICT and not ours:
            return False
    stop(client)
    return True


def _own_action_runs(client: DroneClient) -> bool:
    """The server also refuses /move while our own climb or camera move runs: still ours."""
    try:
        return client.status()["state"] in OWN_ACTIONS
    except DroneApiError:
        return False


def stop(client: DroneClient) -> None:
    """STOP: the drone hovers. A network error must not hide what ended the landing."""
    try:
        client.stop()
    except DroneApiError as error:
        log.warning("STOP not sent (%s): the last /move expires in %.1f s", error, MOVE_HOLD_S)


def pad_offset(
    pad: Pad, shape: tuple[int, ...], pose: dict[str, Any], settings: LandSettings
) -> tuple[float, float, float]:
    """(forward, right) metres from the drone to the pad in its body frame, and the camera's
    height above the pad, for a frame with this pose (X-Drone-Pose, tilt known)."""
    h, w = shape[:2]
    tilt = pose["tilt_deg"]
    if tilt <= OVERHEAD_TILT_DEG:  # the pad faces the camera: its size gives the height
        focal_px = camera.CAMERA_FOCAL_PX * w / camera.CAMERA_WIDTH_PX
        height = PAD_SIDE_M * focal_px / math.sqrt(max(cv2.contourArea(pad.corners), 1.0))
    else:
        height = floor_height(pose, settings)
    height = max(height, MIN_HEIGHT_M)
    fwd, right = ground_offset(pad.centre[0] / w, pad.centre[1] / h, tilt, height) or (0.0, 0.0)
    return fwd, right, height


def newest_frame(client: DroneClient, after: float) -> Frame | None:
    """The server's latest frame if it is newer than `after` and decodes, else None."""
    jpeg, meta = client.frame_with_meta()
    frame_time = meta["frame_time"]
    if frame_time is None or frame_time <= after:
        return None
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    return None if image is None else Frame(frame_time, meta["pose"], image)


def detect_shown(
    detect: Detect, overlay: DroneServer, frame: Frame, detect_robot: DetectRobot | None = None
) -> Seen:
    """The pad in a frame, else the robot (with `detect_robot`); posted to the page's overlay."""
    started = time.perf_counter()
    pad = detect(frame.image)
    robot = detect_robot(frame.image) if pad is None and detect_robot is not None else None
    seen = Seen(pad, robot, (time.perf_counter() - started) * 1000)
    post_overlay(overlay, frame.time, seen, frame.image.shape)
    return seen


def post_overlay(
    server: DroneServer, frame_time: float, seen: Seen, shape: tuple[int, ...]
) -> None:
    """The pad's (else the robot's) box on the page's video (POST /vision), as a locked target."""
    found = seen.pad or seen.robot
    target = None
    if found is not None:
        source = "pad" if seen.pad is not None else "robot"
        target = {"bbox": found.bbox(shape), "kind": "body",
                  "confidence": round(found.score, 3), "source": source}  # fmt: skip
    state = "searching" if found is None else "locked"
    body = {"frame_time": frame_time, "state": state, "name": OVERLAY_NAME, "ms": round(seen.ms, 1)}
    server.post("/vision", {**body, "faces": [], "persons": [], "target": target})


class Lander:
    def __init__(
        self,
        client: DroneClient,
        detect: Detect,
        settings: LandSettings,
        detect_robot: DetectRobot | None = None,
    ) -> None:
        self.client = client
        self.detect = detect
        self.detect_robot = detect_robot
        self.settings = settings
        self.overlay = DroneServer(client.base_url)
        self.fix: Sighting | None = None
        self.candidate: Sighting | None = None
        self.robot: RobotSighting | None = None  # confirmed: two detections agreed
        self.robot_candidate: RobotSighting | None = None
        self._streak: list[Sighting] = []  # overhead detections in a row
        self._agreed: tuple[float, float] | None = None  # (height, frame time) they agreed on
        self._frame_time = 0.0
        self._deadline = 0.0
        self._phase = ""

    def run(self) -> str:
        if not self.client.status()["telemetry"]["in_flight"]:
            return "not flying"
        self._deadline = time.monotonic() + TIMEOUT_S
        try:
            self._move(resume=True)  # started on purpose: take control after any earlier STOP
            self._climb()
            self._go_near_robot()
            self._search()
            self._approach_and_descend()
        except Ended as end:
            return self._end(str(end), end.owned)
        except DroneApiError as error:
            return self._end(str(error), error.status != CONFLICT)
        except Exception:
            release(self.client)
            raise
        return LANDED

    def _end(self, reason: str, owned: bool) -> str:
        if owned and release(self.client):
            log.info("hovering: %s", reason)
        else:
            log.info("ended, the drone is not ours any more: %s", reason)
        return reason

    # --- stages -------------------------------------------------------------------------------

    def _climb(self) -> None:
        """Up to the search altitude where the drone is (POST /goto), if below it."""
        pose, _tilt = self._hover()
        alt = self.settings.search_alt_m
        if pose["alt"] >= alt - ALT_TOLERANCE_M:
            return
        self._note("climbing", f"climbing from {pose['alt']:.1f} to {alt:.1f} m")
        self.client.goto(pose["x"], pose["y"], alt)
        self._wait("goto")

    def _go_near_robot(self) -> None:
        """Fly to ROBOT_VIEW_M short of where the robot should be, facing it (if it is known and
        farther than that)."""
        robot = self.settings.robot_xy
        if robot is None:
            return
        pose, _tilt = self._hover()
        dx, dy = robot[0] - pose["x"], robot[1] - pose["y"]
        distance = math.hypot(dx, dy)
        bearing = math.degrees(math.atan2(dy, dx))
        if distance > ROBOT_VIEW_M + GOTO_MIN_M:
            keep = (distance - ROBOT_VIEW_M) / distance
            x, y = pose["x"] + dx * keep, pose["y"] + dy * keep
            self._note("to the robot", f"flying to the robot: {distance:.1f} m, stopping "
                       f"{ROBOT_VIEW_M:.1f} m short at ({x:.1f}, {y:.1f})")  # fmt: skip
            self.client.goto(x, y, max(pose["alt"], self.settings.search_alt_m))
            self._wait("goto")
            self._hover()
        self._note("facing", f"turning to face the robot ({bearing:.0f} deg)")
        self.client.turn(heading=bearing)
        self._wait("turn")

    def _search(self) -> None:
        """Camera diagonal, then turn on the spot until two detections agree; a confirmed robot
        without its pad in view: fly to ROBOT_VIEW_M of it."""
        self._tilt(self.settings.search_tilt_deg)
        turned, last_heading = 0.0, None
        while True:
            pose, _tilt = self._state()
            self._look()
            if self.fix is not None:
                return
            candidate = self.candidate
            if candidate is not None and time.time() - candidate.seen_at <= CONFIRM_S:
                self._note("confirming", "pad seen, confirming")
                self._move()
                continue
            self.candidate = None  # not confirmed in time: a false detection, turn on
            robot = self.robot
            if robot is not None and time.time() - robot.seen_at <= LOST_TIMEOUT_S:
                self._toward_robot(pose, robot)
                turned, last_heading = 0.0, None  # the turn starts again if the robot is lost
                continue
            if last_heading is not None:
                turned += abs(wrap_deg(pose["heading_deg"] - last_heading))
            last_heading = pose["heading_deg"]
            if turned >= FULL_TURN_DEG:
                raise Ended("pad not found after a full turn")
            self._note("turning", f"pad not in view: turning ({turned:.0f} deg so far)")
            self._move(turn=TURN_STICK)

    def _toward_robot(self, pose: dict[str, Any], robot: RobotSighting) -> None:
        """Steer to ROBOT_VIEW_M of the robot, facing it; there, hover until the pad is seen."""
        fwd, right = frame_to_body(
            robot.point[0] - pose["x"], robot.point[1] - pose["y"], pose["heading_deg"]
        )
        distance = math.hypot(fwd, right)
        if distance <= ROBOT_VIEW_M:
            self._note("at the robot", f"{distance:.1f} m from the robot, looking for its pad")
            bearing = math.degrees(math.atan2(right, fwd))
            self._move(turn=max(-MAX_TURN, min(MAX_TURN, bearing * YAW_GAIN)))
            return
        self._note("robot seen", f"robot seen {distance:.1f} m away: flying to it")
        left = distance - ROBOT_VIEW_M  # to the viewing point, along the line to the robot
        scale = left / distance
        move_fwd, move_right, turn = _steer(fwd * scale, right * scale, left, False)
        self._move(move_fwd, move_right, 0.0, turn)

    def _approach_and_descend(self) -> None:
        while True:
            pose, tilt = self._state()
            if tilt is None:
                raise Ended("camera tilt unknown")
            self._look()
            fix = self.fix
            age = time.time() - fix.seen_at
            if age > LOST_TIMEOUT_S:
                raise Ended(f"pad not seen for {age:.0f} s")
            fwd, right = frame_to_body(
                fix.point[0] - pose["x"], fix.point[1] - pose["y"], pose["heading_deg"]
            )
            distance = math.hypot(fwd, right)
            overhead = tilt <= OVERHEAD_TILT_DEG
            if not overhead and self._camera_lower(fix, tilt, distance):
                step = max(DOWN_DEG, tilt - TILT_STEP_DEG)
                self._tilt(DOWN_DEG if distance < OVERHEAD_M else step)
                continue
            fresh = age <= FRESH_S
            centred = distance <= DESCEND_RADIUS_M
            if overhead and centred and self._low_enough(fresh):
                self._land(fix, distance)
                return
            descend = overhead and centred and fresh and fix.height_m > LAND_HEIGHT_M
            stage = "descending" if descend else "centring" if overhead else "approaching"
            if not fresh:
                stage = "pad lost"
            where = f"{distance:.2f} m to the pad, {_heights(fix)}, camera {tilt:.0f} deg"
            self._note(stage, where)
            move_fwd, move_right, turn = _steer(fwd, right, distance, overhead)
            self._move(move_fwd, move_right, -DESCEND_STICK if descend else 0.0, turn)

    def _camera_lower(self, fix: Sighting, tilt: float, distance: float) -> bool:
        """The pad close, or near the bottom of a frame taken at the current tilt."""
        same_tilt = abs(fix.tilt_deg - tilt) <= TILT_TOLERANCE_DEG
        return distance < OVERHEAD_M or (same_tilt and fix.v > LOW_V)

    def _low_enough(self, fresh: bool) -> bool:
        """Pad in view: LAND_FRAMES overhead detections agreed on a height below LAND_HEIGHT_M
        in the latest frames. Pad lost: they agreed on one below LOW_LOST_M just before."""
        if self._agreed is None:
            return False
        height, agreed_at = self._agreed
        age = time.time() - agreed_at
        if fresh:
            return age <= FRESH_S and height <= LAND_HEIGHT_M
        return age <= LOW_LOST_S and height <= LOW_LOST_M

    def _land(self, fix: Sighting, distance: float) -> None:
        self._hover()
        self._note("landing", f"landing: {_heights(fix)}, {distance:.2f} m off the pad")
        answer = self.client.land()
        if not answer.get("ok"):
            raise Ended(f"land refused: {answer.get('error')}")

    # --- seeing -------------------------------------------------------------------------------

    def _look(self) -> None:
        """Detect the pad in the newest frame (if new and recent) and keep it if it agrees."""
        frame = newest_frame(self.client, self._frame_time)
        if frame is None:
            return
        self._frame_time = frame.time
        pose = frame.pose
        if pose is None or pose.get("tilt_deg") is None or time.time() - frame.time > FRESH_S:
            return
        looking_for_robot = self.fix is None  # once the pad is found, only the pad matters
        seen = detect_shown(
            self.detect, self.overlay, frame, self.detect_robot if looking_for_robot else None
        )
        pad = seen.pad
        if seen.robot is not None:
            self._keep_robot(self._place_robot(seen.robot, frame))
        sighting = self._place(pad, frame) if pad is not None else None
        kept = sighting is not None and self._keep(sighting)
        if pose["tilt_deg"] <= OVERHEAD_TILT_DEG:
            self._count_overhead(sighting if kept else None)

    def _place(self, pad: Pad, frame: Frame) -> Sighting | None:
        """The detection in the takeoff frame; None overhead when its size disagrees with the
        odometry (a bigger white square is not the pad)."""
        pose = frame.pose
        fwd, right, height = pad_offset(pad, frame.image.shape, pose, self.settings)
        dx, dy = body_to_frame(fwd, right, pose["heading_deg"])
        point = (pose["x"] + dx, pose["y"] + dy)
        v = pad.centre[1] / frame.image.shape[0]
        floor_m = floor_height(pose, self.settings)
        sighting = Sighting(point, v, pose["tilt_deg"], height, floor_m, pose["alt"], frame.time)
        if sighting.odometry_gap(self.settings.alt_source) > REJECT_HEIGHT_M:
            self._note("rejected", f"detection ignored: {_heights(sighting)} disagree")
            return None
        return sighting

    def _place_robot(self, robot: Robot, frame: Frame) -> RobotSighting | None:
        """The robot's floor contact (the bottom middle of its box) on the floor, takeoff frame."""
        pose = frame.pose
        h, w = frame.image.shape[:2]
        x, y, bw, bh = robot.box
        camera_above_floor = max(pose["alt"] + self.settings.takeoff_height_m, MIN_HEIGHT_M)
        ground = ground_offset((x + bw / 2) / w, (y + bh) / h, pose["tilt_deg"], camera_above_floor)
        if ground is None:
            return None
        dx, dy = body_to_frame(*ground, pose["heading_deg"])
        return RobotSighting((pose["x"] + dx, pose["y"] + dy), frame.time)

    def _keep_robot(self, sighting: RobotSighting | None) -> None:
        """Like _keep for the pad: a second agreeing detection confirms the robot."""
        if sighting is None:
            return
        for known in (self.robot, self.robot_candidate):
            agrees = known is not None and math.dist(sighting.point, known.point) <= ROBOT_AGREE_M
            if agrees and (known is self.robot or sighting.seen_at - known.seen_at <= CONFIRM_S):
                if self.robot is None:
                    self._note("robot found", "robot confirmed")
                self.robot, self.robot_candidate = sighting, None
                return
        self.robot_candidate = sighting

    def _keep(self, sighting: Sighting) -> bool:
        """A detection near the pad becomes the fix (True); one elsewhere needs a second that
        agrees (a single wrong detection does not move the target)."""
        if self.fix is not None and _apart(sighting, self.fix) <= AGREE_M:
            self.fix, self.candidate = sighting, None
            return True
        candidate = self.candidate
        if (
            candidate is not None
            and _apart(sighting, candidate) <= AGREE_M
            and sighting.seen_at - candidate.seen_at <= CONFIRM_S
        ):
            if self.fix is None:
                self._note("found", f"pad confirmed {sighting.height_m:.1f} m below the camera")
            self.fix, self.candidate = sighting, None
            return True
        self.candidate = sighting
        return False

    def _count_overhead(self, sighting: Sighting | None) -> None:
        """Track overhead detections in a row; when the last LAND_FRAMES agree with each other
        and the odometry, remember the height they agree on. A kept detection that disagrees
        with it forgets it; a missing one does not (the pad may have left the view, low)."""
        if sighting is None:
            self._streak = []
            return
        self._streak = [*self._streak, sighting][-LAND_FRAMES:]
        heights = [s.height_m for s in self._streak]
        source = self.settings.alt_source
        if (
            len(heights) == LAND_FRAMES
            and max(heights) - min(heights) <= HEIGHT_SPREAD_M
            and all(s.odometry_gap(source) <= ODOMETRY_AGREE_M for s in self._streak)
        ):
            self._agreed = statistics.median(heights), sighting.seen_at
        elif self._agreed and abs(sighting.height_m - self._agreed[0]) > HEIGHT_SPREAD_M:
            self._agreed = None

    # --- the drone ----------------------------------------------------------------------------

    def _state(self) -> tuple[dict[str, Any], float | None]:
        """(pose, camera tilt or None until anchored) now; Ended on a STOP from outside, the
        drone not flying, or a timeout."""
        if time.monotonic() > self._deadline:
            raise Ended(f"not landed within {TIMEOUT_S:.0f} s")
        status = self.client.status()
        if status["remote_locked"]:
            raise Ended("STOP from the page", owned=False)
        telemetry = status["telemetry"]
        if not telemetry["in_flight"]:
            raise Ended("the drone is not flying", owned=False)
        time.sleep(TICK_S)
        return telemetry["pose"], telemetry["tilt"]["deg"]

    def _hover(self) -> tuple[dict[str, Any], float | None]:
        """Right before /goto, /tilt or /land (the server does not refuse those after a STOP):
        status again, and a neutral /move, refused if a STOP or a safety rule has the drone."""
        state = self._state()
        self._move()
        return state

    def _move(
        self,
        fwd: float = 0.0,
        right: float = 0.0,
        up: float = 0.0,
        turn: float = 0.0,
        resume: bool = False,
    ) -> None:
        try:
            self.client.move(fwd, right, up, turn, seconds=MOVE_HOLD_S, resume=resume)
        except DroneApiError as error:
            raise Ended(f"move refused: {error.message}", error.status != CONFLICT) from error

    def _tilt(self, deg: float) -> None:
        """POST /tilt and wait for it (the drone hovers meanwhile)."""
        self._hover()
        self._note(f"camera {deg:.0f}", f"camera to {deg:.0f} deg, hovering")
        self.client.tilt(deg)
        self._wait("tilt")

    def _wait(self, action: str) -> None:
        """Until the server's action ends; Ended unless it is done (a STOP stops it, a safety
        rule may land it: then the drone is not ours any more)."""
        while True:
            status = self.client.status()
            if status["state"] != action:
                break
            time.sleep(ACTION_POLL_S)
        task = status["task"]
        if task["state"] != "done":
            owned = task["state"] not in ("stopped", "landed")
            raise Ended(f"{action} {task['state']}: {task['message']}", owned)

    def _note(self, phase: str, line: str) -> None:
        """One log line per change of phase."""
        if phase != self._phase:
            self._phase = phase
            log.info("%s", line)


def _heights(sighting: Sighting) -> str:
    overhead = sighting.tilt_deg <= OVERHEAD_TILT_DEG
    return heights_text(sighting.height_m, sighting.floor_m, sighting.pad_m, overhead)


def _steer(fwd: float, right: float, distance: float, overhead: bool) -> tuple[float, float, float]:
    """(forward, right, turn) sticks toward the pad: turn to face it far away, slide near it."""
    if distance < CENTRED_M:
        return 0.0, 0.0, 0.0
    speed = min(MAX_STICK, max(MIN_STICK, GAIN * distance))
    if not overhead and distance > NEAR_M:
        bearing = math.degrees(math.atan2(right, fwd))
        turn = max(-MAX_TURN, min(MAX_TURN, bearing * YAW_GAIN))
        return (0.0 if abs(bearing) > TURN_FIRST_DEG else speed), 0.0, turn
    return speed * fwd / distance, speed * right / distance, 0.0


def _apart(a: Sighting, b: Sighting) -> float:
    return math.hypot(a.point[0] - b.point[0], a.point[1] - b.point[1])


def watch(
    client: DroneClient,
    detect: Detect,
    settings: LandSettings,
    detect_robot: DetectRobot | None = None,
) -> None:
    """No flight (never moves the drone): the pad (else the robot) on the live video, a line
    about twice a second, and its box on the page."""
    overlay = DroneServer(client.base_url)
    last_time = last_log = 0.0
    while True:
        try:
            frame = newest_frame(client, last_time)
        except DroneApiError as error:  # no frame yet (503), wifi
            log.info("no frame: %s", error.message)
            time.sleep(WATCH_LOG_S)
            continue
        if frame is None:
            time.sleep(TICK_S)
            continue
        last_time = frame.time
        seen = detect_shown(detect, overlay, frame, detect_robot)
        if time.monotonic() - last_log >= WATCH_LOG_S:
            last_log = time.monotonic()
            log.info("%s (%.0f ms)", _watch_line(seen, frame, settings), seen.ms)


def _watch_line(seen: Seen, frame: Frame, settings: LandSettings) -> str:
    pose = frame.pose
    tilt = None if pose is None else pose.get("tilt_deg")
    camera_text = "camera tilt unknown" if tilt is None else f"camera {tilt:.0f} deg"
    pad = seen.pad
    if pad is None and seen.robot is not None:
        return f"no pad; robot at {seen.robot.box} px, score {seen.robot.score:.2f}, {camera_text}"
    if pad is None:
        return f"no pad, {camera_text}"
    pixels = f"pad at ({pad.centre[0]:.0f}, {pad.centre[1]:.0f}) px, score {pad.score:.2f}"
    if tilt is None:
        return f"{pixels}, {camera_text}"
    fwd, right, height = pad_offset(pad, frame.image.shape, pose, settings)
    overhead = tilt <= OVERHEAD_TILT_DEG
    heights = heights_text(height, floor_height(pose, settings), pose["alt"], overhead)
    return f"{pixels}: {fwd:.2f} m ahead, {-right:.2f} m left, camera {heights}, {camera_text}"


def add_sim_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--sim", action="store_true", help="simulator: fake pad and robot detectors (sim_vision.py)"
    )
    parser.add_argument(
        "--sim-robot",
        type=float,
        nargs=2,
        default=(0.0, 0.0),
        metavar=("X", "Y"),
        help="--sim: where the simulated robot stands, takeoff frame m (default the takeoff point)",
    )


def detectors(client: DroneClient, args: argparse.Namespace) -> tuple[Detect, DetectRobot | None]:
    """The pad and robot detectors: the trained models, or with --sim the simulator's fakes."""
    if args.sim:
        vision = SimVision(client, tuple(args.sim_robot), args.pad_height, args.takeoff_height)
        return vision.pad, vision.robot
    try:
        detect = PadDetector().detect
    except FileNotFoundError as error:
        raise SystemExit(str(error)) from error
    return detect, load_robot_detector()


def main() -> None:
    parser = argparse.ArgumentParser(description="land the hovering drone on the robot's pad")
    parser.add_argument("--url", default=DEFAULT_URL, help="drone/fly (or the simulator)")
    parser.add_argument(
        "--pad-height", type=float, default=PAD_HEIGHT_M, help="pad above the floor, m"
    )
    parser.add_argument(
        "--takeoff-height", type=float, default=0.0, help="takeoff point above the floor, m"
    )
    parser.add_argument(
        "--tilt", type=float, default=SEARCH_TILT_DEG, help="camera tilt while searching"
    )
    parser.add_argument(
        "--alt", type=float, default=SEARCH_ALT_M, help="search altitude above takeoff, m"
    )
    parser.add_argument(
        "--alt-source",
        choices=ALT_SOURCES,
        default=ALT_SOURCES[0],
        help="odometry height above the pad: floor = alt + takeoff - pad height, pad = alt",
    )
    parser.add_argument(
        "--robot",
        type=float,
        nargs=2,
        metavar=("X", "Y"),
        help="where the robot should be, takeoff frame metres: fly near it first",
    )
    parser.add_argument("--watch", action="store_true", help="no flight: detect on the video")
    add_sim_arguments(parser)
    args = parser.parse_args()
    low, high = -TILT_DEG[1], -TILT_DEG[0]  # the detector's training range
    if not low <= args.tilt <= high:
        parser.error(f"--tilt must be within {low:g}..{high:g} (the pad detector's range)")
    if args.alt < MIN_SEARCH_ALT_M:
        parser.error(f"--alt must be at least {MIN_SEARCH_ALT_M:g} m")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    client = DroneClient(args.url)
    try:
        telemetry = client.status()["telemetry"]
    except DroneApiError as error:
        raise SystemExit(f"no drone at {args.url} ({error}): start drone/fly first") from error
    detect, detect_robot = detectors(client, args)
    robot_xy = tuple(args.robot) if args.robot else None
    settings = LandSettings(
        args.pad_height, args.takeoff_height, args.tilt, args.alt, args.alt_source, robot_xy
    )
    if args.watch:
        if not telemetry["in_flight"] and not telemetry["manual_session"]:
            try:  # Manual Control starts the video (no takeoff)
                client.manual()
            except DroneApiError as error:
                log.warning("manual control: %s (press MANUAL CONTROL in the page)", error)
        try:
            watch(client, detect, settings, detect_robot)
        except KeyboardInterrupt:
            print("stopped watching")
        return
    try:
        result = land_on_pad(client, detect, settings, detect_robot)
    except KeyboardInterrupt:
        stop(client)  # Ctrl+C is the human's STOP: always sent
        print("stopped: the drone hovers where it is")
        return
    print(result)


if __name__ == "__main__":
    main()
