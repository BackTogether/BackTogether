"""Land on the helipad on top of the robot: find the pad, fly over it, centre and go down on it.

    drone/fly 100 sd --fence 15 --max-alt 4   # terminal 1: drone + page, http://127.0.0.1:8077
    drone/land --watch     # no flight, never moves it: is the pad seen? (lines + page overlay)
    drone/land             # flying in MANUAL (after drone/find, or a manual takeoff): land it

Needs the pad model (drone/identify/pad.py: photos in drone/identify/base/, then
`python -m drone.identify.pad train`). Client-side, over the HTTP API, like find_child.py:

0. takes control (resumes /move: it was started on purpose) and climbs to --alt above the
   takeoff point if it is lower (POST /goto where it is).
1. searching: camera to --tilt (diagonal); if the pad is not in view, turn slowly on the spot
   (one full turn at most, then "pad not found" and hover: it never lands blind). Two detections
   that agree confirm the pad.
2. approaching: each detection is placed on the pad's plane with its own frame's pose and camera
   tilt (X-Drone-Pose: the video latency does not matter) and kept in the takeoff frame; the
   drone steers toward it continuously (turns to face it far away, slides near it). /move cannot
   move the gimbal, so the camera follows the pad down in /tilt steps (the drone hovers ~2 s
   each): a step when the pad nears the bottom of the image, straight down (-90) when close.
3. descending: camera straight down, the same steering centres the drone over the pad while it
   descends whenever it is centred and the detection is fresh (no settle-and-sample). Height
   above the pad from the pad's size in the image; low enough (or the pad leaves the view while
   low) -> POST /land.
The pad lost for long, a timeout, Ctrl+C or a STOP from the page (any STOP after this script
took control: it never resumes on its own) end it with the drone hovering. Every frame looked
at is posted to the page's video overlay (POST /vision): the pad's box as the locked target.
"""

import argparse
import logging
import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from drone.identify.feed import DroneServer
from drone.identify.pad import PAD_SIDE_M, Pad, PadDetector
from drone.sdk import camera
from drone.sdk.client import DroneApiError, DroneClient
from drone.sdk.estimate import wrap_deg
from drone.sdk.mission_geometry import body_to_frame, frame_to_body, ground_offset

DEFAULT_URL = "http://127.0.0.1:8077"
LANDED = "landed"
OVERLAY_NAME = "helipad"
SEARCH_ALT_M = 2.0  # above the takeoff point
ALT_TOLERANCE_M = 0.15
PAD_HEIGHT_M = 0.4  # the pad above the floor (Bobac's top disc): measure it
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
CONFIRM_S = 1.5  # ...if this close in time
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
FRESH_S = 0.5  # descend only on a detection this recent
LAND_HEIGHT_M = 0.5  # camera this high above the pad, centred: POST /land
LOW_LOST_M = 0.7  # pad lost this low (it no longer fits in the view), centred: land too
MIN_HEIGHT_M = 0.3
# Time
LOST_TIMEOUT_S = 10.0  # pad not seen this long: give up, hover
TIMEOUT_S = 120.0
MOVE_HOLD_S = 0.4  # each /move expires by itself: a stuck script leaves the drone hovering
TICK_S = 0.05
WATCH_LOG_S = 0.5
ACTION_POLL_S = 0.2

log = logging.getLogger("land")

Detect = Callable[[np.ndarray], Pad | None]


class Ended(Exception):
    """The landing stops here with the drone hovering; the message says why."""


@dataclass(frozen=True)
class LandSettings:
    pad_height_m: float = PAD_HEIGHT_M  # the pad above the floor
    takeoff_height_m: float = 0.0  # the takeoff point above the floor
    search_tilt_deg: float = SEARCH_TILT_DEG
    search_alt_m: float = SEARCH_ALT_M  # above the takeoff point; higher already: stays there


@dataclass(frozen=True)
class Sighting:
    """One detection placed in the takeoff frame."""

    point: tuple[float, float]
    v: float  # pad centre row, image fraction (y down)
    tilt_deg: float  # camera tilt of that frame
    height_m: float  # camera above the pad then
    seen_at: float  # time.monotonic()


def land_on_pad(client: DroneClient, detect: Detect, settings: LandSettings | None = None) -> str:
    """Land the hovering drone on the pad: LANDED, or why not (then it hovers)."""
    return Lander(client, detect, settings or LandSettings()).run()


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
        height = pose["alt"] + settings.takeoff_height_m - settings.pad_height_m
    height = max(height, MIN_HEIGHT_M)
    fwd, right = ground_offset(pad.centre[0] / w, pad.centre[1] / h, tilt, height) or (0.0, 0.0)
    return fwd, right, height


def post_overlay(
    server: DroneServer, frame_time: float, pad: Pad | None, shape: tuple[int, ...], ms: float
) -> None:
    """The pad's box on the page's video (POST /vision), as a locked target."""
    target = None
    if pad is not None:
        bbox = pad.bbox(shape)
        target = {"bbox": bbox, "kind": "body", "confidence": round(pad.score, 3), "source": "pad"}
    state = "searching" if pad is None else "locked"
    body = {"frame_time": frame_time, "state": state, "name": OVERLAY_NAME, "ms": round(ms, 1)}
    server.post("/vision", {**body, "faces": [], "persons": [], "target": target})


class Lander:
    def __init__(self, client: DroneClient, detect: Detect, settings: LandSettings) -> None:
        self.client = client
        self.detect = detect
        self.settings = settings
        self.overlay = DroneServer(client.base_url)
        self.fix: Sighting | None = None
        self.candidate: Sighting | None = None
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
            self._search()
            self._approach_and_descend()
        except (Ended, DroneApiError) as reason:
            self.client.stop()
            log.info("hovering: %s", reason)
            return str(reason)
        return LANDED

    # --- stages -------------------------------------------------------------------------------

    def _climb(self) -> None:
        """Up to the search altitude where the drone is (POST /goto), if below it."""
        pose = self.client.status()["telemetry"]["pose"]
        alt = self.settings.search_alt_m
        if pose["alt"] >= alt - ALT_TOLERANCE_M:
            return
        self._note("climbing", f"climbing from {pose['alt']:.1f} to {alt:.1f} m")
        self.client.goto(pose["x"], pose["y"], alt)
        self._wait("goto")

    def _search(self) -> None:
        """Camera diagonal, then turn on the spot until two detections agree."""
        self._tilt(self.settings.search_tilt_deg)
        turned, last_heading = 0.0, None
        while True:
            pose, _tilt = self._state()
            self._look()
            if self.fix is not None:
                return
            if self.candidate is not None:
                self._note("confirming", "pad seen, confirming")
                self._move()
                continue
            if last_heading is not None:
                turned += abs(wrap_deg(pose["heading_deg"] - last_heading))
            last_heading = pose["heading_deg"]
            if turned >= FULL_TURN_DEG:
                raise Ended("pad not found after a full turn")
            self._note("turning", f"pad not in view: turning ({turned:.0f} deg so far)")
            self._move(turn=TURN_STICK)

    def _approach_and_descend(self) -> None:
        while True:
            pose, tilt = self._state()
            self._look()
            fix = self.fix
            age = time.monotonic() - fix.seen_at
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
            if overhead and centred and _low(fix, fresh):
                self._note("landing", f"{fix.height_m:.2f} m above the pad, {distance:.2f} m off")
                answer = self.client.land()
                if not answer.get("ok"):
                    raise Ended(f"land refused: {answer.get('error')}")
                return
            descend = overhead and centred and fresh
            stage = "descending" if descend else "centring" if overhead else "approaching"
            if not fresh:
                stage = "pad lost"
            self._note(stage, f"{distance:.2f} m to the pad, {fix.height_m:.2f} m above it, "
                              f"camera {tilt:.0f} deg")  # fmt: skip
            move_fwd, move_right, turn = _steer(fwd, right, distance, overhead)
            self._move(move_fwd, move_right, -DESCEND_STICK if descend else 0.0, turn)

    def _camera_lower(self, fix: Sighting, tilt: float, distance: float) -> bool:
        """The pad close, or near the bottom of a frame taken at the current tilt."""
        same_tilt = abs(fix.tilt_deg - tilt) <= TILT_TOLERANCE_DEG
        return distance < OVERHEAD_M or (same_tilt and fix.v > LOW_V)

    # --- seeing -------------------------------------------------------------------------------

    def _look(self) -> None:
        """Detect the pad in the newest frame (if new) and keep it if it agrees."""
        jpeg, meta = self.client.frame_with_meta()
        frame_time, pose = meta["frame_time"], meta["pose"]
        if frame_time is None or frame_time <= self._frame_time:
            return
        self._frame_time = frame_time
        if pose is None or pose.get("tilt_deg") is None:
            return
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return
        started = time.perf_counter()
        found = self.detect(image)
        ms = (time.perf_counter() - started) * 1000
        post_overlay(self.overlay, frame_time, found, image.shape, ms)
        if found is not None:
            self._keep(self._place(found, image.shape, pose))

    def _place(self, pad: Pad, shape: tuple[int, ...], pose: dict[str, Any]) -> Sighting:
        fwd, right, height = pad_offset(pad, shape, pose, self.settings)
        dx, dy = body_to_frame(fwd, right, pose["heading_deg"])
        point = (pose["x"] + dx, pose["y"] + dy)
        return Sighting(point, pad.centre[1] / shape[0], pose["tilt_deg"], height, time.monotonic())

    def _keep(self, sighting: Sighting) -> None:
        """A detection near the pad becomes the fix; one elsewhere needs a second that agrees
        (a single wrong detection does not move the target)."""
        if self.fix is not None and _apart(sighting, self.fix) <= AGREE_M:
            self.fix, self.candidate = sighting, None
            return
        candidate = self.candidate
        if (
            candidate is not None
            and _apart(sighting, candidate) <= AGREE_M
            and sighting.seen_at - candidate.seen_at <= CONFIRM_S
        ):
            if self.fix is None:
                self._note("found", f"pad confirmed {sighting.height_m:.1f} m below the camera")
            self.fix, self.candidate = sighting, None
            return
        self.candidate = sighting

    # --- the drone ----------------------------------------------------------------------------

    def _state(self) -> tuple[dict[str, Any], float]:
        """(pose, camera tilt) now; Ended on a STOP from outside, a timeout or no tilt."""
        if time.monotonic() > self._deadline:
            raise Ended(f"not landed within {TIMEOUT_S:.0f} s")
        status = self.client.status()
        if status["remote_locked"]:
            raise Ended("STOP from the page")
        telemetry = status["telemetry"]
        if not telemetry["in_flight"]:
            raise Ended("the drone is not flying")
        tilt = telemetry["tilt"]["deg"]
        if tilt is None:
            raise Ended("camera tilt unknown")
        time.sleep(TICK_S)
        return telemetry["pose"], tilt

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
            raise Ended(f"move refused: {error.message}") from error

    def _tilt(self, deg: float) -> None:
        """POST /tilt and wait for it (the drone hovers meanwhile)."""
        self._note(f"camera {deg:.0f}", f"camera to {deg:.0f} deg, hovering")
        self.client.tilt(deg)
        self._wait("tilt")

    def _wait(self, action: str) -> None:
        """Until the server's action ends; Ended unless it is done (a STOP stops it)."""
        while True:
            status = self.client.status()
            if status["state"] != action:
                break
            time.sleep(ACTION_POLL_S)
        task = status["task"]
        if task["state"] != "done":
            raise Ended(f"{action} {task['state']}: {task['message']}")

    def _note(self, phase: str, line: str) -> None:
        """One log line per change of phase."""
        if phase != self._phase:
            self._phase = phase
            log.info("%s", line)


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


def _low(fix: Sighting, fresh: bool) -> bool:
    """Low enough to land: seen that low now, or lost from the view while low."""
    return fix.height_m <= (LAND_HEIGHT_M if fresh else LOW_LOST_M)


def _apart(a: Sighting, b: Sighting) -> float:
    return math.hypot(a.point[0] - b.point[0], a.point[1] - b.point[1])


def watch(client: DroneClient, detect: Detect, settings: LandSettings) -> None:
    """No flight (never moves the drone): the pad on the live video, a line about twice a
    second, and its box on the page."""
    overlay = DroneServer(client.base_url)
    last_time = last_log = 0.0
    while True:
        try:
            jpeg, meta = client.frame_with_meta()
        except DroneApiError as error:  # no frame yet (503), wifi
            log.info("no frame: %s", error.message)
            time.sleep(WATCH_LOG_S)
            continue
        frame_time, pose = meta["frame_time"], meta["pose"]
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if frame_time is None or frame_time <= last_time or image is None:
            time.sleep(TICK_S)
            continue
        last_time = frame_time
        started = time.perf_counter()
        pad = detect(image)
        ms = (time.perf_counter() - started) * 1000
        post_overlay(overlay, frame_time, pad, image.shape, ms)
        if time.monotonic() - last_log >= WATCH_LOG_S:
            last_log = time.monotonic()
            log.info("%s (%.0f ms)", _watch_line(pad, image.shape, pose, settings), ms)


def _watch_line(
    pad: Pad | None, shape: tuple[int, ...], pose: dict[str, Any] | None, settings: LandSettings
) -> str:
    tilt = None if pose is None else pose.get("tilt_deg")
    camera_text = "camera tilt unknown" if tilt is None else f"camera {tilt:.0f} deg"
    if pad is None:
        return f"no pad, {camera_text}"
    pixels = f"pad at ({pad.centre[0]:.0f}, {pad.centre[1]:.0f}) px, score {pad.score:.2f}"
    if tilt is None:
        return f"{pixels}, {camera_text}"
    fwd, right, height = pad_offset(pad, shape, pose, settings)
    return (
        f"{pixels}: {fwd:.2f} m ahead, {-right:.2f} m left, camera {height:.2f} m above it, "
        f"{camera_text}"
    )


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
    parser.add_argument("--watch", action="store_true", help="no flight: detect on the video")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    client = DroneClient(args.url)
    try:
        telemetry = client.status()["telemetry"]
    except DroneApiError as error:
        raise SystemExit(f"no drone at {args.url} ({error}): start drone/fly first") from error
    try:
        detect = PadDetector().detect
    except FileNotFoundError as error:
        raise SystemExit(str(error)) from error
    settings = LandSettings(args.pad_height, args.takeoff_height, args.tilt, args.alt)
    if args.watch:
        if not telemetry["in_flight"] and not telemetry["manual_session"]:
            try:  # Manual Control starts the video (no takeoff)
                client.manual()
            except DroneApiError as error:
                log.warning("manual control: %s (press MANUAL CONTROL in the page)", error)
        try:
            watch(client, detect, settings)
        except KeyboardInterrupt:
            print("stopped watching")
        return
    try:
        result = land_on_pad(client, detect, settings)
    except KeyboardInterrupt:
        client.stop()
        print("stopped: the drone hovers where it is")
        return
    print(result)


if __name__ == "__main__":
    main()
