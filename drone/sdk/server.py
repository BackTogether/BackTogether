"""HTTP API for the drone (ROS bridge style: JSON, CORS *, 400 / 409 errors), on top of the
hoverctl web UI: one process owns the Bluetooth link, the sticks and the video, and serves
both. The routes are documented in drone/sdk/API.md.

Autopilot and the web page: a flight key or pad in the page (non-neutral) cancels any running
autopilot action, like STOP; while an action runs, the page's idle neutral posts are ignored.

Outside commands (/move, e.g. ROS cmd_vel or drone.patrol.bounce --sdk) run under a "remote"
task that applies the same safety rules (Navigator.supervise). STOP wins over them too: after
any STOP, /move answers 409 until a client sends it with "resume": true.
"""

import contextlib
import functools
import json
import math
import threading
import time
import traceback
from collections.abc import Callable
from http import HTTPStatus
from typing import Any
from urllib.parse import parse_qs, urlsplit

from drone.hoverctl.protocol import NEUTRAL
from drone.hoverctl.sticks import GATE_LIVE
from drone.hoverctl.web import WebHandler, WebServer

from .drone import Drone
from .navigate import Navigator
from .patrol import Patrol, PatrolOptions, Waypoint
from .safety import Interrupted, NotReady, SafetyLanding, SafetyLimits
from .sim import FAULTS, SimLink
from .telemetry import Pose, Tilt
from .tilt import GIMBAL_BOTTOM_DEG, GIMBAL_DEADZONE_PCT, GIMBAL_TOP_DEG, gimbal_rate

MAX_JSON_BYTES = 64 * 1024  # a long patrol fits
MAX_MOVE_S = 5.0
DEFAULT_MOVE_S = 0.5  # /move without seconds: the watchdog of a velocity stream
SSE_IDLE_S = 5.0  # telemetry is 10 Hz; this long without it ends the stream
MAX_PATH_POINTS = 400  # sent to the page
MOVE_AXES = ("fwd", "right", "up", "turn")
REMOTE_TASK = "remote"  # the safety task that watches /move streams
TAKEOVER_WAIT_S = 1.0  # an explicit action waits this long for the remote watcher to end
STOPPED = 'stopped: send /move with "resume": true to take control again'

Reply = tuple[HTTPStatus, dict[str, Any]]


class BadRequest(ValueError):
    """A 400: the message says what is wrong with the body."""


class TaskRunner:
    """Runs one autopilot action at a time on a thread; STOP cancels it through the Drone."""

    def __init__(self, navigator: Navigator) -> None:
        self.navigator = navigator
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._status: dict[str, Any] = {"name": None, "state": "idle", "message": None}
        self.patrol: Patrol | None = None  # the last patrol, for its waypoints and progress

    def busy(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def start(
        self,
        name: str,
        action: Callable[[], Any],
        patrol: Patrol | None = None,
        detail: dict[str, Any] | None = None,
    ) -> bool:
        """False if another action runs (the API answers 409). `detail` goes in the status."""
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return False
            if patrol:
                self.patrol = patrol
            self._status = {
                "name": name,
                "state": "running",
                "message": None,
                "started_at": time.time(),
                "detail": detail or {},
            }
            self._thread = threading.Thread(
                target=self._run, args=(action,), name=f"task-{name}", daemon=True
            )
            self._thread.start()
            return True

    def join(self, timeout: float) -> None:
        with self._lock:
            thread = self._thread
        if thread is not None:
            thread.join(timeout)

    def status(self) -> dict[str, Any]:
        with self._lock:
            status = dict(self._status)
        if self.patrol:
            status["patrol"] = self.patrol.progress()
        return status

    def _run(self, action: Callable[[], Any]) -> None:
        try:
            result = self.navigator.run_guarded(action)
            state, message = "done", _describe(result)
        except Interrupted as stop:
            state, message = "stopped", str(stop)
        except SafetyLanding as reason:
            state, message = "landed", f"safety landing: {reason}"
        except (NotReady, ValueError, TimeoutError, RuntimeError) as error:
            state, message = "failed", str(error)
        except Exception as error:
            print(f"task failed:\n{traceback.format_exc()}", flush=True)
            state, message = "failed", f"{type(error).__name__}: {error}"
        with self._lock:
            self._status.update(state=state, message=message, ended_at=time.time())


def _describe(result: Any) -> str:
    if isinstance(result, Pose):
        return f"at x {result.x:.2f} y {result.y:.2f} alt {result.alt:.2f}"
    if isinstance(result, Tilt):
        return f"camera tilt {result.deg:.0f} deg (estimated)"
    return str(result)


class ApiServer(WebServer):
    """The hoverctl web UI plus the SDK's HTTP API, for one Drone."""

    def __init__(
        self,
        bind: str,
        port: int,
        drone: Drone,
        limits: SafetyLimits | None = None,
        ble_note: str = "",
    ) -> None:
        super().__init__(
            bind, port, drone.sticks, drone.video, drone.link, ble_note, handler=ApiHandler
        )
        self.drone = drone
        self.navigator = Navigator(drone, limits)
        self.tasks = TaskRunner(self.navigator)
        self.sim = drone.link if isinstance(drone.link, SimLink) else None
        self.extra_state = self.estimate
        self.remote_locked = False  # set by STOP: /move needs "resume" to fly again

    def autopilot_active(self) -> bool:
        return self.tasks.busy() or self.drone.command_active()

    def stop_all(self) -> None:
        """STOP: cancel the running action and lock out /move streams until they resume."""
        self.remote_locked = True
        self.drone.stop()

    def status(self, telemetry: Any = None) -> dict[str, Any]:
        """GET /status and every /events message."""
        telemetry = telemetry or self.drone.telemetry()
        task = self.tasks.status()
        running = task["state"] == "running"
        return {
            "state": task["name"] if running else "idle",
            "message": task["message"],
            "task": task,
            "telemetry": telemetry.as_dict(),
            "speed": {"max": self.drone.speed_cap, "sport": self.drone.sticks.sport},
            "recording": self.drone.recording(),
            "target": self.navigator.target,
            "limits": self.navigator.limits.as_dict(),
            "estimator": {
                "name": type(self.drone.estimator).__name__,
                "yaw_rate_dps": getattr(self.drone.estimator, "yaw_rate_dps", None),
            },
            "remote_locked": self.remote_locked,
            "simulated": self.sim is not None,
            "updated_at": time.time(),
        }

    def estimate(self) -> dict[str, Any]:
        """Extra /api/state fields for the page's compass and map."""
        path = self.drone.path()
        step = max(1, math.ceil(len(path) / MAX_PATH_POINTS))
        patrol = self.tasks.patrol
        limits = self.navigator.limits
        estimator = self.drone.estimator
        return {
            "estimate": {
                "estimator": type(estimator).__name__,
                "yaw_rate_dps": getattr(estimator, "yaw_rate_dps", None),
                "pose": self.drone.pose().as_dict(),
                "path": [[round(x, 2), round(y, 2)] for x, y in path[::step]],
                "waypoints": [[w.x, w.y] for w in patrol.waypoints] if patrol else [],
                "target": self.navigator.target,
                "fence": [limits.x_min, limits.x_max, limits.y_min, limits.y_max],
                "task": self.tasks.status(),
                "tilt": self.drone.camera_tilt().as_dict(),
            },
            "recording": self.drone.recording(),
            "simulated": self.sim is not None,
        }


class ApiHandler(WebHandler):
    server: ApiServer

    def end_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        super().end_headers()

    def do_OPTIONS(self) -> None:  # CORS preflight for JSON POSTs from another origin
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self) -> None:
        routes = {"/status": self._status, "/events": self._events, "/frame.jpg": self._frame}
        if self.server.sim:
            routes["/sim/truth"] = self._sim_truth
        route = routes.get(urlsplit(self.path).path)
        if route:
            route()
        else:
            super().do_GET()

    def do_POST(self) -> None:
        path = urlsplit(self.path).path
        page_routes = {
            "/api/sticks": self._page_sticks,
            "/api/stop": self._page_stop,
            "/api/drone/land": self._page_land,
            "/api/drone/tilt": self._page_tilt,
        }
        if path in page_routes:
            page_routes[path]()
            return
        route = self._api_routes().get(path)
        if route is None:
            super().do_POST()
            return
        try:
            status, payload = route(self._json_body())
        except BadRequest as error:
            status, payload = HTTPStatus.BAD_REQUEST, {"error": str(error)}
        self._reply(status, payload)

    # --- GET ----------------------------------------------------------------------------------

    def _status(self) -> None:
        self._reply(HTTPStatus.OK, self.server.status())

    def _frame(self) -> None:
        frame = self.server.drone.latest_frame()
        if frame is None:
            self._reply(HTTPStatus.SERVICE_UNAVAILABLE, {"error": "no video frame yet"})
            return
        self._send(HTTPStatus.OK, "image/jpeg", frame)

    def _sim_truth(self) -> None:
        self._reply(HTTPStatus.OK, self.server.sim.truth())

    def _events(self) -> None:
        """Server-Sent Events: one `status` event per telemetry reading (10 Hz)."""
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        stream = self.server.drone.telemetry_stream(SSE_IDLE_S)
        with contextlib.closing(stream), contextlib.suppress(BrokenPipeError, ConnectionError):
            for telemetry in stream:
                data = json.dumps(self.server.status(telemetry))
                self.wfile.write(f"event: status\ndata: {data}\n\n".encode())
                self.wfile.flush()

    # --- the web page's own routes, aware of the autopilot ------------------------------------

    def _page_sticks(self) -> None:
        axes = self._read_axes()
        tab = parse_qs(urlsplit(self.path).query).get("tab", [""])[0]
        if self.server.autopilot_active():
            if axes == NEUTRAL:  # an idle tab must not fight the autopilot
                self._send(HTTPStatus.NO_CONTENT, "text/plain", b"")
                return
            self.server.stop_all()  # the pilot takes over
        if self.server.pilot.accept(tab, axes):
            self.server.sender.set_input(axes)
        self._send(HTTPStatus.NO_CONTENT, "text/plain", b"")

    def _page_stop(self) -> None:
        self.server.stop_all()
        self._send(HTTPStatus.NO_CONTENT, "text/plain", b"")

    def _page_tilt(self) -> None:
        """The page's tilt presets: the /tilt action, answered like the other page buttons."""
        try:
            status, payload = self._tilt(self._json_body())
        except BadRequest as error:
            status, payload = HTTPStatus.BAD_REQUEST, {"error": str(error)}
        ok = status in (HTTPStatus.OK, HTTPStatus.ACCEPTED)
        self._reply(HTTPStatus.OK, {"ok": ok, "error": None if ok else payload["error"]})

    def _page_land(self) -> None:
        self.server.stop_all()
        result = self.server.drone.land()
        self._reply(HTTPStatus.OK, {"ok": result["ok"], "error": result["error"]})

    # --- API ----------------------------------------------------------------------------------

    def _api_routes(self) -> dict[str, Callable[[dict[str, Any]], Reply]]:
        routes = {
            "/manual": self._manual,
            "/move": self._move,
            "/goto": self._goto,
            "/turn": self._turn,
            "/patrol": self._patrol,
            "/land": self._land,
            "/stop": self._stop,
            "/speed": self._speed,
            "/tilt": self._tilt,
            "/photo": self._photo,
            "/record": self._record,
            "/drone/photo": self._drone_photo,
            "/drone/record": self._drone_record,
        }
        if self.server.sim:
            routes.update(
                {
                    "/sim/button": self._sim_button,
                    "/sim/battery": self._sim_battery,
                    "/sim/cut": self._sim_cut,
                    "/sim/push": self._sim_push,
                }
            )
        return routes

    def _manual(self, body: dict[str, Any]) -> Reply:
        return _result(self.server.drone.enter_manual())

    def _move(self, body: dict[str, Any]) -> Reply:
        values = {axis: _number(body, axis, 0.0, -1.0, 1.0) for axis in MOVE_AXES}
        seconds = _number(body, "seconds", DEFAULT_MOVE_S, 0.0, MAX_MOVE_S)
        server = self.server
        if _flag(body, "resume", False):
            server.remote_locked = False
        if server.remote_locked:
            return HTTPStatus.CONFLICT, {"error": STOPPED}
        blocking = server.navigator.blocking_reason()
        if blocking:
            return HTTPStatus.CONFLICT, {"error": blocking}
        if server.tasks.busy() and server.tasks.status()["name"] != REMOTE_TASK:
            return _busy()
        scale = min(1.0, server.navigator.limits.max_speed / max(server.drone.speed_cap, 1.0))
        server.drone.command(**{axis: v * scale for axis, v in values.items()}, hold_s=seconds)
        if not server.tasks.busy() and self._not_ready() is None:
            server.tasks.start(REMOTE_TASK, server.navigator.supervise)
        return HTTPStatus.OK, {"ok": True}

    def _goto(self, body: dict[str, Any]) -> Reply:
        x, y = _number(body, "x"), _number(body, "y")
        alt = _number(body, "alt", None)
        navigator = self.server.navigator
        not_ready = self._not_ready()
        if not_ready:
            return not_ready
        try:
            navigator.limits.check_target(x, y, navigator.drone.pose().alt if alt is None else alt)
        except ValueError as error:
            raise BadRequest(str(error)) from error
        return self._start("goto", functools.partial(navigator.goto, x, y, alt))

    def _turn(self, body: dict[str, Any]) -> Reply:
        navigator = self.server.navigator
        if "heading" in body:
            heading = _number(body, "heading", None, -360.0, 360.0)
            action = functools.partial(navigator.turn_to, heading)
        elif "by" in body:
            action = functools.partial(navigator.turn_by, _number(body, "by", None, -179.0, 179.0))
        else:
            raise BadRequest('give "heading" (degrees) or "by" (degrees, + = right)')
        return self._not_ready() or self._start("turn", action)

    def _patrol(self, body: dict[str, Any]) -> Reply:
        raw = body.get("waypoints")
        if not isinstance(raw, list) or not raw:
            raise BadRequest('"waypoints" must be a non-empty list of {"x", "y", ...}')
        waypoints = [_waypoint(item) for item in raw]
        options = PatrolOptions(
            loops=int(_number(body, "loops", 1, 0, 1000)),
            dwell_s=_number(body, "dwell_s", PatrolOptions.dwell_s, 0.0, 600.0),
            scan=_flag(body, "scan", False),
            return_home=_flag(body, "return_home", True),
            land=_flag(body, "land", False),
        )
        not_ready = self._not_ready()
        if not_ready:
            return not_ready
        try:
            patrol = Patrol(self.server.navigator, waypoints, options)
        except ValueError as error:
            raise BadRequest(str(error)) from error
        return self._start("patrol", patrol.run, patrol)

    def _land(self, body: dict[str, Any]) -> Reply:
        self.server.stop_all()
        return _result(self.server.drone.land())

    def _stop(self, body: dict[str, Any]) -> Reply:
        self.server.stop_all()
        return HTTPStatus.OK, {"ok": True}

    def _speed(self, body: dict[str, Any]) -> Reply:
        drone = self.server.drone
        if "max" in body:
            drone.set_speed(_number(body, "max", None, 0.0, 100.0))
        if "sport" in body:
            drone.set_sport(_flag(body, "sport", False))
        return HTTPStatus.OK, {"ok": True, "max": drone.speed_cap, "sport": drone.sticks.sport}

    def _tilt(self, body: dict[str, Any]) -> Reply:
        deg = _number(body, "deg", None, GIMBAL_BOTTOM_DEG, GIMBAL_TOP_DEG)
        busy = self._free()
        if busy:
            return busy
        drone = self.server.drone
        if drone.sticks.gate_state != GATE_LIVE:
            return HTTPStatus.CONFLICT, {"error": "sticks locked: press Manual control first"}
        if gimbal_rate(drone.speed_cap) <= 0:
            return HTTPStatus.CONFLICT, {
                "error": f"speed {drone.speed_cap:.0f} is inside the gimbal dead zone "
                f"({GIMBAL_DEADZONE_PCT} %): raise the speed"
            }
        action = functools.partial(drone.set_tilt, deg)
        return self._start("tilt", action, detail={"target_deg": deg})

    def _photo(self, body: dict[str, Any]) -> Reply:
        try:
            path = self.server.drone.take_photo()
        except RuntimeError as error:
            return HTTPStatus.CONFLICT, {"error": str(error)}
        return HTTPStatus.OK, {"ok": True, "path": str(path)}

    def _record(self, body: dict[str, Any]) -> Reply:
        drone = self.server.drone
        if _flag(body, "on", None):
            try:
                path = drone.start_recording()
            except RuntimeError as error:
                return HTTPStatus.CONFLICT, {"error": str(error)}
            return HTTPStatus.OK, {"ok": True, "on": True, "path": str(path)}
        path = drone.stop_recording()
        error = (drone.recording()["last"] or {}).get("error")
        return HTTPStatus.OK, {
            "ok": path is not None,
            "on": False,
            "path": str(path) if path else None,
            "error": None if path else error or "nothing recorded (no keyframe arrived)",
        }

    def _drone_photo(self, body: dict[str, Any]) -> Reply:
        return _result(self.server.drone.drone_photo())

    def _drone_record(self, body: dict[str, Any]) -> Reply:
        return _result(self.server.drone.drone_record(bool(_flag(body, "on", None))))

    def _sim_button(self, body: dict[str, Any]) -> Reply:
        return _result(self.server.sim.press_button())

    def _sim_battery(self, body: dict[str, Any]) -> Reply:
        self.server.sim.set_battery(_number(body, "pct", None, 0.0, 100.0))
        return HTTPStatus.OK, {"ok": True}

    def _sim_cut(self, body: dict[str, Any]) -> Reply:
        what = body.get("what")
        if what not in FAULTS:
            raise BadRequest(f'"what" must be one of {", ".join(FAULTS)}')
        self.server.sim.cut(what, _number(body, "seconds", None, 0.0, 600.0))
        return HTTPStatus.OK, {"ok": True}

    def _sim_push(self, body: dict[str, Any]) -> Reply:
        self.server.sim.push(_number(body, "dx", 0.0), _number(body, "dy", 0.0))
        return HTTPStatus.OK, {"ok": True}

    # --- helpers ------------------------------------------------------------------------------

    def _free(self) -> Reply | None:
        """A 409 if an action runs; a /move stream's watcher gives way to explicit actions."""
        tasks = self.server.tasks
        if tasks.busy() and tasks.status()["name"] == REMOTE_TASK:
            self.server.drone.stop()
            tasks.join(TAKEOVER_WAIT_S)
        return _busy() if tasks.busy() else None

    def _not_ready(self) -> Reply | None:
        busy = self._free()
        if busy:
            return busy
        try:
            self.server.navigator.require_ready()
        except NotReady as error:
            return HTTPStatus.CONFLICT, {"error": str(error)}
        return None

    def _start(
        self,
        name: str,
        action: Callable[[], Any],
        patrol: Patrol | None = None,
        detail: dict[str, Any] | None = None,
    ) -> Reply:
        if not self.server.tasks.start(name, action, patrol, detail):
            return _busy()
        return HTTPStatus.ACCEPTED, {"accepted": True}

    def _json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", 0))
        if length > MAX_JSON_BYTES:
            raise BadRequest(f"body over {MAX_JSON_BYTES} bytes")
        raw = self.rfile.read(length) if length else b""
        try:
            body = json.loads(raw or b"{}")
        except ValueError as error:
            raise BadRequest(f"body is not JSON: {error}") from error
        if not isinstance(body, dict):
            raise BadRequest("body must be a JSON object")
        return body

    def _reply(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        self._send(status, "application/json", json.dumps(payload).encode())


_REQUIRED = object()


def _number(
    body: dict[str, Any],
    key: str,
    default: Any = _REQUIRED,
    low: float = -math.inf,
    high: float = math.inf,
) -> Any:
    """body[key] as a finite float in low..high; `default` when missing (400 if required)."""
    if key not in body or body[key] is None:
        if default is _REQUIRED:
            raise BadRequest(f'"{key}" is required')
        return default
    value = body[key]
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
        raise BadRequest(f'"{key}" must be a number')
    if not low <= value <= high:
        raise BadRequest(f'"{key}" must be in {low:g}..{high:g}')
    return float(value)


def _flag(body: dict[str, Any], key: str, default: bool | None) -> bool | None:
    value = body.get(key, default)
    if value is None and default is None:
        raise BadRequest(f'"{key}" (true or false) is required')
    if not isinstance(value, bool):
        raise BadRequest(f'"{key}" must be true or false')
    return value


def _waypoint(item: Any) -> Waypoint:
    if not isinstance(item, dict):
        raise BadRequest('each waypoint is {"x", "y", "alt"?, "dwell_s"?, "scan"?}')
    scan = item.get("scan")
    if scan is not None and not isinstance(scan, bool):
        raise BadRequest('waypoint "scan" must be true or false')
    return Waypoint(
        x=_number(item, "x"),
        y=_number(item, "y"),
        alt=_number(item, "alt", None),
        dwell_s=_number(item, "dwell_s", None, 0.0, 600.0),
        scan=scan,
    )


def _busy() -> Reply:
    return HTTPStatus.CONFLICT, {"error": "busy"}


def _result(result: dict[str, Any]) -> Reply:
    return HTTPStatus.OK, {"ok": bool(result.get("ok")), "error": result.get("error")}
