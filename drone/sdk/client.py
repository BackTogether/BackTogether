"""Client for the drone's HTTP API (drone/sdk/API.md), for the robot side.

Python 3.8 compatible and stdlib only (urllib, json, threading): copy this one file to the
robot and import it from rospy code (see ros_node.py).

    drone = DroneClient("http://<DRONE_LAPTOP_IP>:8077")
    drone.goto(2.0, 0.0, alt=1.5)
    for status in drone.events():          # Server-Sent Events, 10 Hz
        print(status["telemetry"]["pose"])
    for jpeg in drone.frames():            # MJPEG frames as JPEG bytes
        ...
    for jpeg, meta in drone.frames_with_meta():   # + {"frame_time", "pose"} of each frame
        drone.mission_target(meta["frame_time"], [0.4, 0.3, 0.6, 0.7], confidence=0.9)
"""

from __future__ import annotations

import base64
import json
import math
import urllib.error
import urllib.request
import zlib
from collections.abc import Iterator
from typing import Any

DEFAULT_URL = "http://127.0.0.1:8077"
TIMEOUT_S = 5.0
STREAM_TIMEOUT_S = 10.0  # no byte for this long on /events or /video.mjpg: the stream ends
SSE_DATA = "data:"
MJPEG_LENGTH_HEADER = "content-length"
FRAME_TIME_HEADER = "x-frame-time"  # unix time (the Mac's clock) of the frame
DRONE_POSE_HEADER = "x-drone-pose"  # JSON: the drone's pose and camera tilt at that time


class DroneApiError(Exception):
    """The API answered an error (status 400, 409, 503...) or could not be reached."""

    def __init__(self, status: int, message: str) -> None:
        super().__init__(f"{status}: {message}")
        self.status = status
        self.message = message


class DroneClient:
    def __init__(self, base_url: str = DEFAULT_URL, timeout: float = TIMEOUT_S) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout

    # --- commands -----------------------------------------------------------------------------

    def status(self) -> dict[str, Any]:
        return self._request("GET", "/status")

    def manual(self) -> dict[str, Any]:
        """Enter Manual Control (the app's sequence). Then take off with the drone's button."""
        return self._request("POST", "/manual")

    def move(
        self,
        fwd: float = 0.0,
        right: float = 0.0,
        up: float = 0.0,
        turn: float = 0.0,
        seconds: float | None = None,
        resume: bool = False,
    ) -> dict[str, Any]:
        """Velocity command, -1..1 of the speed cap, held for `seconds` (default 0.5 s: send it
        again faster than that to keep moving; it stops by itself when you stop sending).
        After a STOP the server answers 409 "stopped" until a move has `resume=True`."""
        body: dict[str, Any] = {"fwd": fwd, "right": right, "up": up, "turn": turn}
        if seconds is not None:
            body["seconds"] = seconds
        if resume:
            body["resume"] = True
        return self._request("POST", "/move", body)

    def goto(self, x: float, y: float, alt: float | None = None) -> dict[str, Any]:
        """Fly to (x, y) in the takeoff frame (metres); runs in the background, see status()."""
        body = {"x": x, "y": y}
        if alt is not None:
            body["alt"] = alt
        return self._request("POST", "/goto", body)

    def turn(self, heading: float | None = None, by: float | None = None) -> dict[str, Any]:
        """Turn to an estimated heading, or by some degrees (+ = right)."""
        body = {"heading": heading} if heading is not None else {"by": by}
        return self._request("POST", "/turn", body)

    def patrol(self, waypoints: list[dict[str, Any]], **options: Any) -> dict[str, Any]:
        """waypoints: [{"x", "y", "alt"?, "dwell_s"?, "scan"?}]; options: loops, dwell_s, scan,
        return_home, land."""
        body = dict(options)
        body["waypoints"] = waypoints
        return self._request("POST", "/patrol", body)

    def land(self) -> dict[str, Any]:
        return self._request("POST", "/land")

    def stop(self) -> dict[str, Any]:
        """STOP: cancels any autopilot action, sticks neutral, the drone hovers."""
        return self._request("POST", "/stop")

    def speed(self, max_speed: float | None = None, sport: bool | None = None) -> dict[str, Any]:
        body: dict[str, Any] = {}
        if max_speed is not None:
            body["max"] = max_speed
        if sport is not None:
            body["sport"] = sport
        return self._request("POST", "/speed", body)

    def tilt(self, deg: float) -> dict[str, Any]:
        """Move the camera to an estimated tilt (0 = level, -90 = down, up to +20); runs in the
        background, see status()["task"]."""
        return self._request("POST", "/tilt", {"deg": deg})

    def camera_tilt(self) -> dict[str, Any]:
        """{"deg" (None until anchored), "low_deg", "high_deg", "anchored", "source"}."""
        return self.status()["telemetry"]["tilt"]

    def photo(self) -> dict[str, Any]:
        """Save the current frame on the Mac; the reply has its path."""
        return self._request("POST", "/photo")

    def record(self, on: bool) -> dict[str, Any]:
        """Start or stop an MP4 recording on the Mac; the reply has its path."""
        return self._request("POST", "/record", {"on": on})

    def sim_truth(self) -> dict[str, Any]:
        """Simulator only: the true pose, the true distance to the wall ahead (front_m)."""
        return self._request("GET", "/sim/truth")

    def frame(self) -> bytes:
        """The latest frame as JPEG bytes."""
        return self.frame_with_meta()[0]

    def frame_with_meta(self) -> tuple[bytes, dict[str, Any]]:
        """The latest frame and {"frame_time", "pose"} (see frames_with_meta)."""
        with self._open("GET", "/frame.jpg", None, self.timeout) as response:
            headers = {key.lower(): value for key, value in response.headers.items()}
            return response.read(), _frame_meta(headers)

    # --- lost-child mission (API.md, "Mission") -------------------------------------------------

    def mission(self) -> dict[str, Any]:
        """The mission's state, child position (drone, launch and map frames) and more."""
        return self._request("GET", "/mission")

    def mission_start(self, **config: Any) -> dict[str, Any]:
        """Start the mission: waypoints, frame, strategy, patrol_alt_m, orbit_radius_m... (see
        API.md). Flying or not: on the ground it waits for the button takeoff."""
        return self._request("POST", "/mission/start", config)

    def mission_target(
        self,
        frame_time: float | None,
        bbox: list[float],
        confidence: float = 1.0,
        label: str | None = None,
    ) -> dict[str, Any]:
        """Report a detection: bbox [x0, y0, x1, y1] as fractions of the image (y down),
        frame_time = the frame's X-Frame-Time. The reply says if the mission used it."""
        body: dict[str, Any] = {"frame_time": frame_time, "bbox": bbox, "confidence": confidence}
        if label is not None:
            body["label"] = label
        return self._request("POST", "/mission/target", body)

    def mission_launch_pose(self, x: float, y: float, yaw: float) -> dict[str, Any]:
        """The robot's pose in `map` at takeoff (metres, radians): enables map coordinates."""
        return self._request("POST", "/mission/launch_pose", {"x": x, "y": y, "yaw": yaw})

    def mission_abort(self) -> dict[str, Any]:
        """End the mission: back to the takeoff point and land."""
        return self._request("POST", "/mission/abort")

    # --- map view (API.md "Map view") ---------------------------------------------------------

    def map_state(self) -> dict[str, Any]:
        """What the page's map view shows: drone, robot, child, path, layers, in one frame."""
        return self._request("GET", "/map/state")

    def map_robot_pose(self, x: float, y: float, yaw: float) -> dict[str, Any]:
        """The robot's pose in `map` now (metres, radians). The reply's "has_grid" says
        whether the Mac has an occupancy grid yet."""
        return self._request("POST", "/map/robot_pose", {"x": x, "y": y, "yaw": yaw})

    def map_grid(
        self,
        width: int,
        height: int,
        resolution: float,
        origin: tuple[float, float, float],
        cells: bytes,
        source: str = "",
    ) -> dict[str, Any]:
        """The robot's occupancy grid: `cells` = nav_msgs/OccupancyGrid data as int8 bytes
        (row 0 at the origin, -1 unknown, 0..100), origin = (x, y, yaw) of cell (0, 0)."""
        body = {
            "width": width,
            "height": height,
            "resolution": resolution,
            "origin": {"x": origin[0], "y": origin[1], "yaw": origin[2]},
            "encoding": "zlib+base64",
            "data": base64.b64encode(zlib.compress(cells)).decode("ascii"),
            "source": source,
        }
        return self._request("POST", "/map/grid", body)

    # --- streams ------------------------------------------------------------------------------

    def events(self) -> Iterator[dict[str, Any]]:
        """Status messages from GET /events (10 Hz) until the stream ends or breaks."""
        with self._open("GET", "/events", None, STREAM_TIMEOUT_S) as response:
            data: list[str] = []
            for raw in response:
                line = raw.decode("utf-8").rstrip("\r\n")
                if line.startswith(SSE_DATA):
                    data.append(line[len(SSE_DATA) :].strip())
                elif not line and data:
                    yield json.loads("\n".join(data))
                    data = []

    def frames(self) -> Iterator[bytes]:
        """JPEG frames from GET /video.mjpg until the stream ends or breaks."""
        for jpeg, _meta in self.frames_with_meta():
            yield jpeg

    def frames_with_meta(self) -> Iterator[tuple[bytes, dict[str, Any]]]:
        """(JPEG, {"frame_time", "pose"}) from GET /video.mjpg: frame_time is the unix time
        of the frame on the Mac's clock (send it back with mission_target), pose the drone's
        {"x", "y", "alt", "heading_deg", "tilt_deg"} then. Either is None when unknown."""
        with self._open("GET", "/video.mjpg", None, STREAM_TIMEOUT_S) as response:
            while True:
                headers = _read_part_headers(response)
                if headers is None:
                    return
                yield response.read(int(headers[MJPEG_LENGTH_HEADER])), _frame_meta(headers)

    # --- plumbing -----------------------------------------------------------------------------

    def _request(
        self, method: str, path: str, body: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        with self._open(method, path, body, self.timeout) as response:
            return json.loads(response.read().decode("utf-8") or "{}")

    def _open(self, method: str, path: str, body: dict[str, Any] | None, timeout: float) -> Any:
        data = None if body is None else json.dumps(body).encode("utf-8")
        request = urllib.request.Request(self.base_url + path, data=data, method=method)
        if data is not None:
            request.add_header("Content-Type", "application/json")
        try:
            return urllib.request.urlopen(request, timeout=timeout)
        except urllib.error.HTTPError as error:
            raise DroneApiError(error.code, _error_message(error)) from error
        except (urllib.error.URLError, OSError) as error:
            raise DroneApiError(0, f"cannot reach {self.base_url}: {error}") from error


def _error_message(error: urllib.error.HTTPError) -> str:
    try:
        return json.loads(error.read().decode("utf-8")).get("error") or error.reason
    except ValueError:
        return str(error.reason)


def _read_part_headers(response: Any) -> dict[str, str] | None:
    """Skip to the next MJPEG part and return its headers, lower-case names (None at the
    end). The part always has a Content-Length."""
    headers: dict[str, str] = {}
    while True:
        line = response.readline()
        if not line:
            return None
        stripped = line.decode("latin-1").strip()
        name, colon, value = stripped.partition(":")
        if colon:
            headers[name.strip().lower()] = value.strip()
        elif not stripped and MJPEG_LENGTH_HEADER in headers:
            return headers


def _frame_meta(headers: dict[str, str]) -> dict[str, Any]:
    raw_time, raw_pose = headers.get(FRAME_TIME_HEADER), headers.get(DRONE_POSE_HEADER)
    return {
        "frame_time": float(raw_time) if raw_time else None,
        "pose": json.loads(raw_pose) if raw_pose else None,
    }


def standoff_goal(
    robot_x: float, robot_y: float, child_x: float, child_y: float, standoff: float
) -> tuple[float, float, float]:
    """A goal (x, y, yaw) on the line from the robot to the child, `standoff` metres short of
    the child, facing it (ROS: yaw in radians, counter-clockwise). Closer than that: stay."""
    dx, dy = child_x - robot_x, child_y - robot_y
    distance = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    if distance <= standoff:
        return robot_x, robot_y, yaw
    keep = (distance - standoff) / distance
    return robot_x + dx * keep, robot_y + dy * keep, yaw
