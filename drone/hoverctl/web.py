"""Local web UI: MJPEG video, virtual joysticks, a STOP button and the drone's Bluetooth state.

Uses only the stdlib HTTP server: no dependency to install on the drone's Wi-Fi (no internet),
and plain HTTP POSTs at 20 Hz from one local browser tab are plenty. The sender's watchdog
turns the sticks neutral if the tab stops posting (closed, crashed, network hiccup).
"""

import json
import math
import time
from collections.abc import Callable
from dataclasses import asdict
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import TYPE_CHECKING, Any

from .pbwire import drone_network_ip
from .protocol import AXIS_NAMES, NEUTRAL, Axes
from .sticks import StickSender
from .teleop import KEY_BINDINGS
from .video import VideoReceiver

if TYPE_CHECKING:
    from .ble import DroneLink

PAGE_TEMPLATE = Path(__file__).with_name("web.html")
BINDINGS_PLACEHOLDER = "/*KEY_BINDINGS*/{}"
MJPEG_BOUNDARY = "frame"
FRAME_WAIT_S = 1.0
MAX_BODY_BYTES = 1024
DRONE_PATHS = (
    "/api/drone/manual",
    "/api/drone/land",
    "/api/drone/wifi",
    "/api/drone/video",
    "/api/drone/video-stop",
)
MIN_WIFI_PASSWORD = 8
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
        if self.path == "/api/sticks":
            self.server.sender.set_input(self._read_axes())
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
            "/api/drone/manual": link.set_manual,
            "/api/drone/land": link.land,
            "/api/drone/wifi": lambda: self._start_wifi(link),
            "/api/drone/video": link.start_video,
            "/api/drone/video-stop": link.stop_video,
        }

    def _start_wifi(self, link: "DroneLink") -> dict[str, Any]:
        try:
            password = str(json.loads(self._read_body() or b"{}").get("password", ""))
        except (ValueError, AttributeError):
            password = ""
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
                "errors": sender.errors,
                "last_error": sender.last_error,
                "target": f"{sender.address[0]}:{sender.address[1]}",
                "dry_run": sender.dry_run,
                "locked": sender.locked,
                "interlock": sender.interlocked,
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
        self.link = link  # None: Bluetooth off or unavailable (`ble_note` says which)
        self.ble_note = ble_note
        bindings = {key: list(value) for key, value in KEY_BINDINGS.items()}
        page = PAGE_TEMPLATE.read_text().replace(BINDINGS_PLACEHOLDER, json.dumps(bindings))
        self.page = page.encode()
