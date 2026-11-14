"""Local web UI: MJPEG video, virtual joysticks, a STOP button and the drone's Bluetooth state.

Uses only the stdlib HTTP server: no dependency to install on the drone's Wi-Fi (no internet),
and plain HTTP POSTs at 20 Hz from one local browser tab are plenty. The sender's watchdog
turns the sticks neutral if the tab stops posting (closed, crashed, network hiccup).
"""

import json
import math
import threading
import time
from collections.abc import Callable
from dataclasses import asdict
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import TYPE_CHECKING, Any
from urllib.parse import parse_qs, urlsplit

from .pbwire import VIDEO_QUALITIES, drone_network_ip
from .protocol import AXIS_DIRECTIONS, AXIS_NAMES, NEUTRAL, Axes
from .sticks import StickSender
from .teleop import KEY_BINDINGS
from .video import VideoReceiver

if TYPE_CHECKING:
    from .ble import DroneLink

PAGE_TEMPLATE = Path(__file__).with_name("web.html")
BINDINGS_PLACEHOLDER = "/*KEY_BINDINGS*/{}"
DIRECTIONS_PLACEHOLDER = "/*AXIS_DIRECTIONS*/{}"
MJPEG_BOUNDARY = "frame"
FRAME_WAIT_S = 1.0
MAX_BODY_BYTES = 1024
DRONE_PATHS = (
    "/api/drone/manual",
    "/api/drone/land",
    "/api/drone/wifi",
    "/api/drone/video",
    "/api/drone/video-stop",
    "/api/drone/video-quality",
)
MIN_WIFI_PASSWORD = 8
PILOT_HOLD_S = 0.5  # ignore other tabs' neutral posts this long after the pilot's last values
TAB_SEEN_S = 2.0  # a tab that posted this recently counts as open
NO_LINK = {"ok": False, "error": "bluetooth is off or unavailable"}


def _unit(value: object) -> float:
    """Clamp a browser value to -1..1; NaN or infinity count as neutral."""
    number = float(value)
    return max(-1.0, min(1.0, number)) if math.isfinite(number) else 0.0


class _Handler(BaseHTTPRequestHandler):
    server: "WebServer"

    def log_message(self, format: str, *args: object) -> None:
        pass  # 20 Hz stick posts would flood the terminal

    def do_GET(self) -> None:
        routes = {"/": self._page, "/video.mjpg": self._mjpeg, "/api/state": self._state}
        route = routes.get(self.path.split("?")[0])
        if route:
            route()
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        command = self._drone_commands().get(self.path)
        if command:
            self._drone_command(command)
            return
        url = urlsplit(self.path)
        if url.path == "/api/sticks":
            axes = self._read_axes()
            tab = parse_qs(url.query).get("tab", [""])[0]
            if self.server.pilot.accept(tab, axes):
                self.server.sender.set_input(axes)
        elif self.path == "/api/max":
            self.server.sender.set_max(self._read_max())
        elif self.path == "/api/sport":
            self.server.sender.set_sport(self._read_json().get("on") is True)
        elif self.path == "/api/stop":
            self.server.sender.set_input(NEUTRAL)
        else:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self._send(HTTPStatus.NO_CONTENT, "text/plain", b"")

    def _drone_commands(self) -> dict[str, Callable[[], dict[str, Any]]]:
        link = self.server.link
        if link is None:
            return {path: lambda: NO_LINK for path in DRONE_PATHS}
        return {
            "/api/drone/manual": link.enter_manual,
            "/api/drone/land": link.land,
            "/api/drone/wifi": lambda: self._start_wifi(link),
            "/api/drone/video": link.start_video,
            "/api/drone/video-stop": link.stop_video,
            "/api/drone/video-quality": lambda: self._set_video_quality(link),
        }

    def _set_video_quality(self, link: "DroneLink") -> dict[str, Any]:
        quality = str(self._read_json().get("quality", ""))
        if quality not in VIDEO_QUALITIES:
            return {"ok": False, "error": f"quality must be one of {', '.join(VIDEO_QUALITIES)}"}
        result = link.set_video_quality(quality)
        if result["ok"]:
            self.server.receiver.restart_decoder()  # the frame size may change
        return result

    def _read_json(self) -> dict[str, Any]:
        """The request body as a JSON object; anything else reads as {}."""
        try:
            body = json.loads(self._read_body() or b"{}")
        except ValueError:
            return {}
        return body if isinstance(body, dict) else {}

    def _start_wifi(self, link: "DroneLink") -> dict[str, Any]:
        password = str(self._read_json().get("password", ""))
        if len(password) < MIN_WIFI_PASSWORD:
            return {"ok": False, "error": f"password needs {MIN_WIFI_PASSWORD}+ characters"}
        return link.start_wifi(password)

    def _drone_command(self, command: Callable[[], dict[str, Any]]) -> None:
        result = command()
        reply = {"ok": bool(result.get("ok")), "error": result.get("error")}
        self._send(HTTPStatus.OK, "application/json", json.dumps(reply).encode())

    def _read_body(self) -> bytes:
        length = min(int(self.headers.get("Content-Length", 0)), MAX_BODY_BYTES)
        return self.rfile.read(length)

    def _read_max(self) -> float:
        """The new speed cap; a bad body keeps the current one."""
        try:
            value = float(self._read_json()["max"])
        except (ValueError, TypeError, KeyError):
            return self.server.sender.max_magnitude
        return value if math.isfinite(value) else self.server.sender.max_magnitude

    def _read_axes(self) -> Axes:
        try:
            body = json.loads(self._read_body() or b"{}")
            return Axes(**{axis: _unit(body.get(axis, 0.0)) for axis in AXIS_NAMES})
        except (ValueError, TypeError, AttributeError):
            return NEUTRAL

    def _send(self, status: HTTPStatus, content_type: str, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _page(self) -> None:
        self._send(HTTPStatus.OK, "text/html; charset=utf-8", self.server.page)

    def _state(self) -> None:
        sender = self.server.sender
        receiver = self.server.receiver
        state = {
            "sticks": {
                "output": asdict(sender.output),
                "max": sender.max_magnitude,
                "sent": sender.sent,
                "non_neutral_sent": sender.non_neutral_sent,
                "last_packet": sender.last_packet,
                "gate": sender.gate_state,
                "control_source": sender.control_source,
                "sport": sender.sport,
                "errors": sender.errors,
                "last_error": sender.last_error,
                "target": f"{sender.address[0]}:{sender.address[1]}",
                "dry_run": sender.dry_run,
                "locked": sender.locked,
                "silent": sender.silent,
                "interlock": sender.interlocked,
                "open_tabs": self.server.pilot.open_tabs(),
            },
            "now": time.time(),
            "drone": self.server.link.snapshot() if self.server.link else None,
            "ble_note": self.server.ble_note,
            "video": {
                **receiver.stats(),
                "decoding": receiver.decoder is not None,
                "local_ip": drone_network_ip(),
            },
        }
        self._send(HTTPStatus.OK, "application/json", json.dumps(state).encode())

    def _mjpeg(self) -> None:
        decoder = self.server.receiver.decoder
        if not decoder:
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "decoding is off")
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        seen = 0
        try:
            while True:
                number, jpeg = decoder.wait_frame(seen, FRAME_WAIT_S)
                if number == seen:
                    continue
                seen = number
                head = f"--{MJPEG_BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                head += f"Content-Length: {len(jpeg)}\r\n\r\n"
                self.wfile.write(head.encode() + jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass  # browser tab closed


class PilotFilter:
    """Picks which browser tab's stick posts count.

    Every open tab posts its sticks at 20 Hz. Without this, an idle tab's neutral posts
    interleave with the flying tab's values, so the sticks flicker to zero and the ramp keeps
    restarting at its first step. The tab that last sent values is the pilot; while it is
    active, other tabs' neutral posts are ignored. STOP is not filtered.
    """

    def __init__(self) -> None:
        self._pilot = ""
        self._pilot_at = 0.0
        self._seen: dict[str, float] = {}
        self._lock = threading.Lock()

    def accept(self, tab: str, axes: Axes) -> bool:
        now = time.monotonic()
        with self._lock:
            self._seen[tab] = now
            if axes != NEUTRAL:
                self._pilot, self._pilot_at = tab, now
                return True
            return tab == self._pilot or now - self._pilot_at > PILOT_HOLD_S

    def open_tabs(self) -> int:
        now = time.monotonic()
        with self._lock:
            return sum(1 for seen in self._seen.values() if now - seen < TAB_SEEN_S)


class WebServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        bind: str,
        port: int,
        sender: StickSender,
        receiver: VideoReceiver,
        link: "DroneLink | None" = None,
        ble_note: str = "",
    ):
        super().__init__((bind, port), _Handler)
        self.sender = sender
        self.receiver = receiver
        self.pilot = PilotFilter()
        self.link = link  # None: Bluetooth off or unavailable (`ble_note` says which)
        if link:
            sender.set_session_gate(link.in_manual_session)
        self.ble_note = ble_note
        bindings = {key: list(value) for key, value in KEY_BINDINGS.items()}
        page = PAGE_TEMPLATE.read_text().replace(BINDINGS_PLACEHOLDER, json.dumps(bindings))
        page = page.replace(DIRECTIONS_PLACEHOLDER, json.dumps(AXIS_DIRECTIONS))
        self.page = page.encode()
