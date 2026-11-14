"""Client for the hoverctl web app (drone/hoverctl/web.py), through its public HTTP API only.

The web app owns the drone link (Bluetooth, Wi-Fi RPC, stick sender with its safety cap and
watchdog); this client reads its video and state and posts sticks like a browser tab does.
"""

import json
import time
import urllib.request
from collections.abc import Iterator
from dataclasses import dataclass
from typing import Any

DEFAULT_URL = "http://127.0.0.1:8077"
TIMEOUT_S = 2.0
PATROL_TAB = "patrol"  # the web app picks the pilot tab by name; keep ours distinct
JPEG_START = b"\xff\xd8"
JPEG_END = b"\xff\xd9"
MJPEG_CHUNK = 65536


@dataclass(frozen=True)
class DroneState:
    """The fields the patrol needs from /api/state; None when unknown."""

    flight_state: str | None
    in_flight: bool
    mode: str | None
    battery_pct: int | None
    position: tuple[float, float, float] | None  # odometry, z down, origin at takeoff
    odometry_age_s: float | None
    state_age_s: float | None

    @property
    def altitude_m(self) -> float | None:
        return None if self.position is None else -self.position[2]


def _age(now: float, ts: float | None) -> float | None:
    return None if ts is None else now - ts


def parse_state(raw: dict[str, Any]) -> DroneState:
    drone = raw.get("drone") or {}
    now = raw.get("now", time.time())
    flight = drone.get("flight_state") or {}
    odometry = drone.get("odometry") or {}
    position = odometry.get("position")
    return DroneState(
        flight_state=flight.get("value"),
        in_flight=bool(flight.get("in_flight")),
        mode=(drone.get("mode") or {}).get("name"),
        battery_pct=(drone.get("battery") or {}).get("soc"),
        position=None if position is None else (position["x"], position["y"], position["z"]),
        odometry_age_s=_age(now, odometry.get("ts")),
        state_age_s=_age(now, flight.get("ts")),
    )


class HoverWeb:
    def __init__(self, base_url: str = DEFAULT_URL) -> None:
        self.base_url = base_url.rstrip("/")

    def _request(self, path: str, body: dict | None = None) -> bytes:
        """GET without a body, POST with one."""
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(self.base_url + path, data=data)
        with urllib.request.urlopen(request, timeout=TIMEOUT_S) as reply:
            return reply.read()

    def state(self) -> DroneState:
        return parse_state(json.loads(self._request("/api/state")))

    def send_sticks(
        self, pitch: float = 0.0, roll: float = 0.0, thrust: float = 0.0, yaw: float = 0.0
    ) -> None:
        """Axes in -1..1 with hoverctl's signs: + forward, right, up, turn right."""
        axes = {"pitch": pitch, "roll": roll, "thrust": thrust, "yaw": yaw, "gimbal": 0.0}
        self._request(f"/api/sticks?tab={PATROL_TAB}", axes)

    def stop(self) -> None:
        self._request("/api/stop", {})

    def land(self) -> dict[str, Any]:
        return json.loads(self._request("/api/drone/land", {}))

    def frames(self) -> Iterator[bytes]:
        """JPEG frames from /video.mjpg, as they arrive (blocks until the next one)."""
        with urllib.request.urlopen(self.base_url + "/video.mjpg", timeout=TIMEOUT_S) as stream:
            yield from split_jpegs(iter(lambda: stream.read1(MJPEG_CHUNK), b""))


def split_jpegs(chunks: Iterator[bytes]) -> Iterator[bytes]:
    """Cut whole JPEGs out of a byte stream (the multipart headers between them are skipped)."""
    buffer = b""
    for chunk in chunks:
        buffer += chunk
        while True:
            start = buffer.find(JPEG_START)
            end = buffer.find(JPEG_END, start + 2) if start >= 0 else -1
            if end < 0:
                # Keep a possible marker split across chunks (its first byte may end this one).
                buffer = buffer[start:] if start >= 0 else buffer[-1:]
                break
            yield buffer[start : end + 2]
            buffer = buffer[end + 2 :]
