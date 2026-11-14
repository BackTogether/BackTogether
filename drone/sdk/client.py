"""Client for the drone's HTTP API (drone/sdk/API.md), for the robot side.

Python 3.8 compatible and stdlib only (urllib, json, threading): copy this one file to the
robot and import it from rospy code (see ros_node.py).

    drone = DroneClient("http://<DRONE_LAPTOP_IP>:8077")
    drone.goto(2.0, 0.0, alt=1.5)
    for status in drone.events():          # Server-Sent Events, 10 Hz
        print(status["telemetry"]["pose"])
    for jpeg in drone.frames():            # MJPEG frames as JPEG bytes
        ...
"""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from collections.abc import Iterator
from typing import Any

DEFAULT_URL = "http://127.0.0.1:8077"
TIMEOUT_S = 5.0
STREAM_TIMEOUT_S = 10.0  # no byte for this long on /events or /video.mjpg: the stream ends
SSE_DATA = "data:"
MJPEG_LENGTH_HEADER = b"content-length:"


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
        with self._open("GET", "/frame.jpg", None, self.timeout) as response:
            return response.read()

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
        with self._open("GET", "/video.mjpg", None, STREAM_TIMEOUT_S) as response:
            while True:
                length = _read_part_length(response)
                if length is None:
                    return
                yield response.read(length)

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


def _read_part_length(response: Any) -> int | None:
    """Skip to the next MJPEG part and return its Content-Length (None at the end)."""
    length = None
    while True:
        line = response.readline()
        if not line:
            return None
        stripped = line.strip()
        if stripped.lower().startswith(MJPEG_LENGTH_HEADER):
            length = int(stripped[len(MJPEG_LENGTH_HEADER) :])
        elif not stripped and length is not None:
            return length
