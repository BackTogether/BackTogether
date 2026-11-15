"""HTTP API for the drone (ROS bridge style: JSON, CORS *, 400 / 409 errors), on top of the
hoverctl web UI: one process owns the Bluetooth link, the sticks and the video, and serves
both. The routes are documented in drone/sdk/API.md.

Autopilot and the web page: a flight key or pad in the page (non-neutral) cancels any running
autopilot action, like STOP; while an action runs, the page's idle neutral posts are ignored.

Outside commands (/move, e.g. ROS cmd_vel or drone.patrol.bounce --sdk) run under a "remote"
task that applies the same safety rules (Navigator.supervise). STOP wins over them too: after
any STOP, /move answers 409 until a client sends it with "resume": true.

The lost-child mission (mission.py, /mission/*) runs as one autopilot action; STOP ends it too.
Every video frame (/frame.jpg, each /video.mjpg part) carries X-Frame-Time and X-Drone-Pose, so
a detector can report a bbox for that exact frame (POST /mission/target), and everything it
saw in it for the page's video overlay (POST /vision, vision.py).
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
from drone.hoverctl.web import MJPEG_BOUNDARY, WebHandler, WebServer

from .drone import Drone
from .map_image import decode_cells, grid_from_cells
from .map_view import MapView
from .mission import (
    ANCHOR_CENTRE,
    ANCHOR_FEET,
    ANCHORS,
    APPROACHES,
    FRAME_LAUNCH,
    FRAMES,
    STRATEGIES,
    STRATEGY_WAYPOINTS,
    Detection,
    Mission,
    MissionConfig,
)
from .mission_geometry import LaunchPose
from .navigate import Navigator
from .patrol import Patrol, PatrolOptions, Waypoint
from .safety import Interrupted, NotReady, SafetyLanding, SafetyLimits
from .sim import FAULTS, SimDetector, SimLink
from .splat import Registration
from .telemetry import Pose, Tilt
from .tilt import GIMBAL_BOTTOM_DEG, GIMBAL_TOP_DEG
from .vision import VisionLog, parse_result

MAX_JSON_BYTES = 64 * 1024  # a long patrol fits
MAX_MOVE_S = 5.0
DEFAULT_MOVE_S = 0.5  # /move without seconds: the watchdog of a velocity stream
SSE_IDLE_S = 5.0  # telemetry is 10 Hz; this long without it ends the stream
MAX_PATH_POINTS = 400  # sent to the page
MOVE_AXES = ("fwd", "right", "up", "turn")
REMOTE_TASK = "remote"  # the safety task that watches /move streams
TAKEOVER_WAIT_S = 1.0  # an explicit action waits this long for the remote watcher to end
STOPPED = 'stopped: send /move with "resume": true to take control again'
MISSION_TASK = "mission"
MISSION_OWNS = "stopped: the mission flies the drone now"  # /move clients take it as an end
MJPEG_IDLE_S = 5.0  # no frame this long: re-subscribe (the stream itself stays open)
FRAME_TIME_HEADER = "X-Frame-Time"
DRONE_POSE_HEADER = "X-Drone-Pose"
MAX_LABEL_CHARS = 64
MS_PER_S = 1000.0
MAP_MAX_JSON_BYTES = 32 * 1024 * 1024  # POST /map/grid: a large compressed grid
PNG_TYPE = "image/png"

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
        map_view: MapView | None = None,
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
        self.stopped_at: float | None = None  # last explicit STOP (page, API, ROS), for the page
        self.mission = Mission(self.navigator)
        self.vision = VisionLog()
        self.sim_detector = SimDetector(drone, self._sim_detection) if self.sim else None
        if self.sim_detector:
            self.sim_detector.start()
        self.map_view = map_view or MapView()

    def autopilot_active(self) -> bool:
        return self.tasks.busy() or self.drone.command_active()

    def stop_all(self) -> None:
        """STOP: cancel the running action and lock out /move streams until they resume."""
        self.remote_locked = True
        self.mission.halt()
        self.drone.stop()

    def press_stop(self) -> None:
        """An explicit STOP (not LAND, not a pilot taking over): the page shows it."""
        self.stop_all()
        self.stopped_at = time.time()

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
            "mission": self.mission.status(),
            "simulated": self.sim is not None,
            "updated_at": time.time(),
        }

    def frame_headers(self, decoded_at: float) -> dict[str, str]:
        """X-Frame-Time and X-Drone-Pose for a frame decoded at unix time `decoded_at`. The
        frame time is when its picture reached the Mac (decoded_at - the decoder's lag, when
        the telemetry has it)."""
        lag_ms = getattr(self.drone.telemetry(), "video_lag_ms", None) or 0.0
        frame_time = decoded_at - lag_ms / MS_PER_S
        headers = {FRAME_TIME_HEADER: f"{frame_time:.3f}"}
        sample = self.mission.history.at(frame_time)
        if sample:
            pose, tilt = sample
            headers[DRONE_POSE_HEADER] = json.dumps(
                {
                    "x": round(pose.x, 3),
                    "y": round(pose.y, 3),
                    "alt": round(pose.alt, 3),
                    "heading_deg": round(pose.heading_deg, 1),
                    "tilt_deg": None if tilt is None else round(tilt, 1),
                },
                separators=(",", ":"),
            )
        return headers

    def _sim_detection(
        self,
        frame_time: float,
        body: tuple[float, ...],
        face: tuple[float, ...] | None,
        confidence: float,
        label: str,
    ) -> None:
        """The fake detector: the body to the mission, body and face to the page's overlay."""
        self.mission.report_target(Detection(frame_time, body, confidence, label, ANCHOR_FEET))
        faces = [] if face is None else [{"bbox": list(face), "name": label, "target": True}]
        target = {"bbox": list(body), "kind": "body", "confidence": confidence, "source": "sim"}
        self.vision.add(
            {
                "frame_time": frame_time,
                "state": "locked",
                "name": label,
                "faces": faces,
                "persons": [{"bbox": list(body), "score": confidence}],
                "target": target,
                "ms": None,
            }
        )

    def estimate(self) -> dict[str, Any]:
        """Extra /api/state fields for the page's compass and map."""
        path = self.drone.path()
        step = max(1, math.ceil(len(path) / MAX_PATH_POINTS))
        patrol = self.mission.patrol or self.tasks.patrol
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
                "stopped_at": self.stopped_at,
                "remote_locked": self.remote_locked,
            },
            "recording": self.drone.recording(),
            "simulated": self.sim is not None,
        }

    def map_state(self) -> dict[str, Any]:
        """GET /map/state: the page's map view (map_view.py)."""
        patrol = self.mission.patrol or self.tasks.patrol
        limits = self.navigator.limits
        return self.map_view.state(
            mission=self.mission.status(),
            pose=self.drone.pose(),
            path=self.drone.path(),
            waypoints=[(w.x, w.y) for w in patrol.waypoints] if patrol else [],
            fence=(limits.x_min, limits.x_max, limits.y_min, limits.y_max),
            target=self.navigator.target,
        )


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
        routes = {
            "/status": self._status,
            "/events": self._events,
            "/frame.jpg": self._frame,
            "/video.mjpg": self._video,
            "/mission": self._mission,
            "/vision": self._vision,
        }
        if self.server.sim:
            routes["/sim/truth"] = self._sim_truth
        routes.update(self._map_get_routes())
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
        map_route = self._map_post_routes().get(path)
        if map_route:
            self._map_post(map_route)
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
        drone = self.server.drone
        frame, age = drone.latest_frame(), drone.frame_age_s()
        if frame is None or age is None:
            self._reply(HTTPStatus.SERVICE_UNAVAILABLE, {"error": "no video frame yet"})
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store")
        self.send_header(
            "Access-Control-Expose-Headers", f"{FRAME_TIME_HEADER}, {DRONE_POSE_HEADER}"
        )
        for name, value in self.server.frame_headers(time.time() - age).items():
            self.send_header(name, value)
        self.end_headers()
        self.wfile.write(frame)

    def _video(self) -> None:
        """MJPEG like hoverctl's page stream, each part with X-Frame-Time and X-Drone-Pose."""
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        with contextlib.suppress(BrokenPipeError, ConnectionError):
            while True:
                for jpeg in self.server.drone.frames(MJPEG_IDLE_S):
                    head = f"--{MJPEG_BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                    head += f"Content-Length: {len(jpeg)}\r\n"
                    for name, value in self.server.frame_headers(time.time()).items():
                        head += f"{name}: {value}\r\n"
                    self.wfile.write(head.encode() + b"\r\n" + jpeg + b"\r\n")

    def _mission(self) -> None:
        self._reply(HTTPStatus.OK, self.server.mission.status())

    def _vision(self) -> None:
        """The detector's results newer than ?since= (frame time), and the mission's step."""
        since = parse_qs(urlsplit(self.path).query).get("since", ["0"])[0]
        try:
            after = float(since)
        except ValueError:
            self._reply(HTTPStatus.BAD_REQUEST, {"error": '"since" must be a number'})
            return
        mission = self.server.mission.status()
        self._reply(
            HTTPStatus.OK,
            {
                "results": self.server.vision.since(after),
                "mission": {"state": mission["state"], "step": mission["step"]},
                "now": time.time(),
            },
        )

    def _sim_truth(self) -> None:
        truth = self.server.sim.truth()
        truth["child"] = self.server.sim_detector.truth()
        launch = self.server.mission.status()["launch_pose"]
        if launch:
            pose = LaunchPose(**launch)
            x, y = pose.map_from_launch(truth["child"]["fwd"], truth["child"]["left"])
            truth["child"]["map"] = {"x": round(x, 3), "y": round(y, 3)}
        self._reply(HTTPStatus.OK, truth)

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
        self.server.press_stop()
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
            "/mission/start": self._mission_start,
            "/mission/target": self._mission_target,
            "/mission/glimpse": self._mission_glimpse,
            "/mission/launch_pose": self._mission_launch_pose,
            "/mission/abort": self._mission_abort,
            "/vision": self._vision_post,
        }
        if self.server.sim:
            routes.update(
                {
                    "/sim/button": self._sim_button,
                    "/sim/battery": self._sim_battery,
                    "/sim/cut": self._sim_cut,
                    "/sim/push": self._sim_push,
                    "/sim/child": self._sim_child,
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
        running = server.tasks.status()["name"] if server.tasks.busy() else None
        if running == MISSION_TASK and not server.mission.accepts_moves():
            return HTTPStatus.CONFLICT, {"error": MISSION_OWNS}
        if running not in (None, REMOTE_TASK, MISSION_TASK):
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
        self.server.press_stop()
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
        anchor = bool(_flag(body, "anchor", False))
        action = functools.partial(drone.set_tilt, deg, anchor)
        return self._start("tilt", action, detail={"target_deg": deg, "anchor": anchor})

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

    def _mission_start(self, body: dict[str, Any]) -> Reply:
        config = _mission_config(body)
        busy = self._free()
        if busy:
            return busy
        mission = self.server.mission
        try:
            mission.prepare(config)
        except ValueError as error:
            raise BadRequest(str(error)) from error
        return self._start(MISSION_TASK, mission.run, detail={"config": config.as_dict()})

    def _mission_target(self, body: dict[str, Any]) -> Reply:
        corners = _bbox(body)
        label = body.get("label")
        if label is not None and not isinstance(label, str):
            raise BadRequest('"label" must be a string')
        detection = Detection(
            frame_time=_number(body, "frame_time", time.time(), 0.0),
            bbox=tuple(corners),
            confidence=_number(body, "confidence", 1.0, 0.0, 1.0),
            label=label[:MAX_LABEL_CHARS] if label else None,
            anchor=_choice(body, "anchor", ANCHOR_CENTRE, ANCHORS),
        )
        return _accepted(*self.server.mission.report_target(detection))

    def _mission_glimpse(self, body: dict[str, Any]) -> Reply:
        glimpse = Detection(
            frame_time=_number(body, "frame_time", time.time(), 0.0),
            bbox=tuple(_bbox(body)),
            confidence=0.0,
        )
        return _accepted(*self.server.mission.report_glimpse(glimpse))

    def _vision_post(self, body: dict[str, Any]) -> Reply:
        try:
            result = parse_result(body)
        except ValueError as error:
            raise BadRequest(str(error)) from error
        self.server.vision.add(result)
        return HTTPStatus.OK, {"ok": True}

    def _mission_launch_pose(self, body: dict[str, Any]) -> Reply:
        pose = LaunchPose(
            _number(body, "x"), _number(body, "y"), _number(body, "yaw", None, -10.0, 10.0)
        )
        self.server.mission.set_launch_pose(pose)
        return HTTPStatus.OK, {"ok": True}

    def _mission_abort(self, body: dict[str, Any]) -> Reply:
        if not self.server.mission.abort():
            return HTTPStatus.CONFLICT, {"error": "no mission running"}
        return HTTPStatus.OK, {"ok": True}

    def _sim_child(self, body: dict[str, Any]) -> Reply:
        detector = self.server.sim_detector
        detect = _flag(body, "detect", detector.detect)
        detector.place(
            _number(body, "fwd", detector.fwd),
            _number(body, "left", detector.left),
            detect,
            _number(body, "miss", detector.miss, 0.0, 1.0),
        )
        return HTTPStatus.OK, {"ok": True, "child": detector.truth()}

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

    # --- map view (map_view.py, API.md "Map view") -----------------------------------------------

    def _map_get_routes(self) -> dict[str, Callable[[], None]]:
        return {
            "/map/state": self._map_state,
            "/map/grid.png": lambda: self._map_png(self.server.map_view.grid_png()),
            "/map/splat.png": lambda: self._map_png(self._splat_png()),
        }

    def _map_post_routes(self) -> dict[str, Callable[[dict[str, Any]], Reply]]:
        return {
            "/map/robot_pose": self._map_robot_pose,
            "/map/grid": self._map_grid,
            "/map/splat/registration": self._map_registration,
        }

    def _map_post(self, route: Callable[[dict[str, Any]], Reply]) -> None:
        try:
            status, payload = route(self._json_body(MAP_MAX_JSON_BYTES))
        except BadRequest as error:
            status, payload = HTTPStatus.BAD_REQUEST, {"error": str(error)}
        self._reply(status, payload)

    def _map_state(self) -> None:
        self._reply(HTTPStatus.OK, self.server.map_state())

    def _splat_png(self) -> bytes | None:
        splat = self.server.map_view.splat
        return splat.png if splat else None

    def _map_png(self, png: bytes | None) -> None:
        if png is None:
            self._reply(HTTPStatus.NOT_FOUND, {"error": "no such map layer"})
            return
        self._send(HTTPStatus.OK, PNG_TYPE, png)

    def _map_robot_pose(self, body: dict[str, Any]) -> Reply:
        view = self.server.map_view
        view.set_robot_pose(_number(body, "x"), _number(body, "y"), _number(body, "yaw", 0.0))
        return HTTPStatus.OK, {"ok": True, "has_grid": view.has_grid()}

    def _map_grid(self, body: dict[str, Any]) -> Reply:
        origin = body.get("origin") or {}
        if not isinstance(origin, dict) or not isinstance(body.get("data"), str):
            raise BadRequest('give "origin" {"x", "y", "yaw"} and "data" (base64 string)')
        try:
            cells = decode_cells(body["data"], body.get("encoding", "zlib+base64"))
            grid = grid_from_cells(
                cells,
                int(_number(body, "width", None, 1)),
                int(_number(body, "height", None, 1)),
                _number(body, "resolution", None, 0.0),
                (_number(origin, "x", 0.0), _number(origin, "y", 0.0), _number(origin, "yaw", 0.0)),
                str(body.get("source") or "POST /map/grid")[:MAX_LABEL_CHARS],
            )
        except (ValueError, TypeError) as error:
            raise BadRequest(f"bad grid: {error}") from error
        self.server.map_view.set_grid(grid)
        return HTTPStatus.OK, {"ok": True, "width": grid.width, "height": grid.height}

    def _map_registration(self, body: dict[str, Any]) -> Reply:
        view = self.server.map_view
        if view.splat is None:
            return HTTPStatus.CONFLICT, {"error": "no splat layer (start with --splat)"}
        current = view.registration
        registration = Registration(
            scale=_number(body, "scale", current.scale, 1e-6),
            rotation_deg=_number(body, "rotation_deg", current.rotation_deg, -360.0, 360.0),
            x=_number(body, "x", current.x),
            y=_number(body, "y", current.y),
        )
        try:
            saved = view.set_registration(registration, bool(_flag(body, "save", False)))
        except OSError as error:
            return HTTPStatus.CONFLICT, {"error": f"cannot save: {error}"}
        return HTTPStatus.OK, {"ok": True, "saved": str(saved) if saved else None}

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

    def _json_body(self, limit: int = MAX_JSON_BYTES) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", 0))
        if length > limit:
            raise BadRequest(f"body over {limit} bytes")
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


def _choice(body: dict[str, Any], key: str, default: str, choices: tuple[str, ...]) -> str:
    value = body.get(key, default)
    if value not in choices:
        raise BadRequest(f'"{key}" must be one of {", ".join(choices)}')
    return value


def _mission_config(body: dict[str, Any]) -> MissionConfig:
    """POST /mission/start's body (API.md); missing fields keep MissionConfig's defaults."""
    defaults = MissionConfig()
    raw = body.get("waypoints", [])
    if not isinstance(raw, list):
        raise BadRequest('"waypoints" must be a list of {"x", "y", ...}')

    def number(key: str, low: float = 0.0, high: float = math.inf) -> float:
        return _number(body, key, getattr(defaults, key), low, high)

    tilt = defaults.patrol_tilt_deg
    if "patrol_tilt_deg" in body:  # null: leave the camera where it is
        tilt = _number(body, "patrol_tilt_deg", None, GIMBAL_BOTTOM_DEG, GIMBAL_TOP_DEG)
    return MissionConfig(
        waypoints=tuple(_waypoint(item) for item in raw),
        frame=_choice(body, "frame", FRAME_LAUNCH, FRAMES),
        strategy=_choice(body, "strategy", STRATEGY_WAYPOINTS, STRATEGIES),
        loops=int(number("loops", 0, 1000)),
        dwell_s=number("dwell_s", 0, 600),
        scan=bool(_flag(body, "scan", defaults.scan)),
        patrol_alt_m=number("patrol_alt_m"),
        patrol_tilt_deg=tilt,
        align_m=number("align_m", 0, 5),
        approach=_choice(body, "approach", defaults.approach, APPROACHES),
        approach_speed=number("approach_speed", 0.05, 1),
        approach_tilt_deg=number("approach_tilt_deg", GIMBAL_BOTTOM_DEG, 0),
        orbit_alt_m=number("orbit_alt_m"),  # null or missing: the current altitude
        orbit_radius_m=number("orbit_radius_m", 0.5, 5),
        orbit_turns=number("orbit_turns", 0, 5),
        standoff_m=number("standoff_m", 0, 5),
        launch_height_m=number("launch_height_m", 0, 3),
        target_height_m=number("target_height_m", 0, 2),
        min_confidence=number("min_confidence", 0, 1),
        takeoff_timeout_s=number("takeoff_timeout_s", 1, 600),
        patrol_timeout_s=number("patrol_timeout_s", 1, 3600),
        approach_timeout_s=number("approach_timeout_s", 5, 600),
        lost_timeout_s=number("lost_timeout_s", 1, 120),
        look_max_deg=number("look_max_deg", 0, 360),
    )


def _bbox(body: dict[str, Any]) -> list[float]:
    """The body's "bbox": [x0, y0, x1, y1], fractions of the image."""
    bbox = body.get("bbox")
    if not isinstance(bbox, list) or len(bbox) != 4:
        raise BadRequest('"bbox" must be [x0, y0, x1, y1], fractions of the image')
    corners = [_number({"v": value}, "v", None, 0.0, 1.0) for value in bbox]
    if not (corners[0] < corners[2] and corners[1] < corners[3]):
        raise BadRequest('"bbox" needs x0 < x1 and y0 < y1')
    return corners


def _accepted(accepted: bool, why: str) -> Reply:
    """The answer to a detection or a glimpse."""
    return HTTPStatus.OK, {
        "accepted": accepted,
        "state": why if accepted else None,
        "reason": None if accepted else why,
    }


def _busy() -> Reply:
    return HTTPStatus.CONFLICT, {"error": "busy"}


def _result(result: dict[str, Any]) -> Reply:
    return HTTPStatus.OK, {"ok": bool(result.get("ok")), "error": result.get("error")}
