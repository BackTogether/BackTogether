"""Run the care flow (care.py) on its own, with the screen page, without the rest of the robot.

Serves the part of the bridge API in interfaces/ROBOT_API.md that the care flow needs, so the
screen page works unchanged once the real bridge calls CareSession itself:

    GET  /ui/           screen page (bobac/robot/ui/)
    GET  /status        contract fields + "care" (extra details for the screen)
    POST /care/start    {"zone": str, "found_time": epoch seconds (optional)} -> the robot
                        reached the lost child in that zone
    POST /ui/answer     {"answer": "ok" | "help" | "yes" | "no"}
    POST /stop

On the robot (system Python 3.8) or on a laptop:

    python3 care_server.py            # then open http://localhost:8000/ui/
    curl -X POST localhost:8000/care/start -d '{"zone": "sofa"}'

Messages to the parents need TELEGRAM_BOT_TOKEN and TELEGRAM_CHAT_IDS (see caregiver.py). Spoken
answers need Vosk and its model (see listen.py); without them only the buttons work. The video
call uses CALL_BASE_URL (a Jitsi server, default in care.py).
"""

import argparse
import json
import logging
import mimetypes
import os
import shutil
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import care
import caregiver
import listen
import speech

DEFAULT_PORT = 8000
HERE = Path(__file__).resolve().parent
UI_DIR = HERE / "ui"
PROFILE_PATH = HERE.parent / "config" / "child.json"
HEAD_CAMERA_DEVICE = "/dev/head_camera"
CAMERA_WARMUP_FRAMES = 5  # the first frames of a USB camera are dark
BUZZER_TIMEOUT_S = 5
CALL_PROFILE_DIR = Path.home() / "leuk" / "call-firefox"
# The call window must use the camera and microphone with nobody there to tap "Allow".
CALL_FIREFOX_PREFS = (
    'user_pref("media.navigator.permission.disabled", true);\n'
    'user_pref("permissions.default.camera", 1);\n'
    'user_pref("permissions.default.microphone", 1);\n'
    'user_pref("media.autoplay.default", 0);\n'
    'user_pref("browser.shell.checkDefaultBrowser", false);\n'
    'user_pref("browser.sessionstore.resume_from_crash", false);\n'
)
IDLE_STATUS = {"state": "idle", "answer": None, "updated_at": 0.0, "care": None}

log = logging.getLogger("care_server")


def take_photo(path: str) -> bool:
    """Save one head-camera frame. False if OpenCV or the camera is not available (e.g. the
    camera is already open in a ROS node)."""
    try:
        import cv2  # only the robot needs it; keeps the laptop run dependency-free
    except ImportError:
        log.warning("OpenCV not installed: no photo")
        return False
    camera = cv2.VideoCapture(HEAD_CAMERA_DEVICE)
    try:
        frame = None
        for _ in range(CAMERA_WARMUP_FRAMES):
            grabbed, frame = camera.read()
            if not grabbed:
                log.warning("could not read %s: no photo", HEAD_CAMERA_DEVICE)
                return False
        return bool(cv2.imwrite(path, frame))
    finally:
        camera.release()


def buzz() -> None:
    """Sound the base buzzer (needs the ROS environment and rei_robot_base running)."""
    if shutil.which("rosservice") is None:
        log.warning("rosservice not found: no buzzer")
        return
    command = ["rosservice", "call", "/set_buzzer", "data: true"]
    try:
        subprocess.run(command, timeout=BUZZER_TIMEOUT_S, check=False, capture_output=True)
    except subprocess.TimeoutExpired:
        log.warning("buzzer call timed out")


class CallWindow:
    """The video call, full screen on the robot display, in a Firefox of its own (Jitsi servers
    do not allow being embedded in our page). Opens on top of the screen page, closes after."""

    def __init__(self):
        self._process = None
        self._lock = threading.Lock()

    def show(self, url) -> None:
        with self._lock:
            self._close()
            if url:
                self._open(url)

    def _open(self, url: str) -> None:
        if shutil.which("firefox") is None:
            log.warning("firefox not installed: open the call yourself: %s", url)
            return
        CALL_PROFILE_DIR.mkdir(parents=True, exist_ok=True)
        (CALL_PROFILE_DIR / "user.js").write_text(CALL_FIREFOX_PREFS)
        env = dict(os.environ, DISPLAY=os.environ.get("DISPLAY", ":0"))
        command = ["firefox", "--new-instance", "--profile", str(CALL_PROFILE_DIR), "--kiosk", url]
        self._process = subprocess.Popen(
            command, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        log.info("video call window opened: %s", url)

    def _close(self) -> None:
        if self._process and self._process.poll() is None:
            self._process.terminate()
            log.info("video call window closed")
        self._process = None


class CareServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, profile, telegram, ears, call_base):
        super().__init__(address, Handler)
        self.profile = profile
        self.telegram = telegram
        self.ears = ears
        self.call_base = call_base
        self.call_window = CallWindow()
        self.session = None
        self.session_thread = None
        self.session_lock = threading.Lock()

    def start_session(self, zone: str, found_time: float) -> bool:
        with self.session_lock:
            if self.session_thread and self.session_thread.is_alive():
                return False
            if self.telegram:
                self.telegram.skip_backlog()
            self.session = care.CareSession(
                self.profile,
                zone,
                found_time,
                speech,
                self.telegram,
                take_photo,
                buzz,
                self.ears,
                self.call_base,
                self.call_window.show,
            )
            self.session_thread = threading.Thread(target=self.session.run, daemon=True)
            self.session_thread.start()
            return True


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/status":
            self._send_status()
        elif self.path.split("?")[0].startswith("/ui/"):
            self._send_ui_file(self.path.split("?")[0][len("/ui/") :] or "index.html")
        else:
            self._send_json(404, {"error": "not found"})

    def do_POST(self):
        body = self._read_json()
        if body is None:
            self._send_json(400, {"error": "body must be a JSON object"})
        elif self.path == "/care/start":
            self._start(body)
        elif self.path == "/ui/answer":
            self._answer(body)
        elif self.path == "/stop":
            if self.server.session:
                self.server.session.stop()
            self._send_json(200, {"ok": True})
        else:
            self._send_json(404, {"error": "not found"})

    def do_OPTIONS(self):  # CORS preflight, so a page served from a laptop can post
        self._send_json(204, None)

    def _start(self, body: dict) -> None:
        zone = body.get("zone")
        found_time = body.get("found_time", time.time())
        if not isinstance(zone, str) or not zone or not isinstance(found_time, (int, float)):
            self._send_json(400, {"error": "zone (str) required, found_time must be a number"})
        elif self.server.start_session(zone, float(found_time)):
            self._send_json(202, {"accepted": True})
        else:
            self._send_json(409, {"error": "busy"})

    def _answer(self, body: dict) -> None:
        session = self.server.session
        if session and session.answer(body.get("answer")):
            self._send_json(200, {"ok": True})
        else:
            self._send_json(400, {"error": "no question is waiting for that answer"})

    def _send_status(self) -> None:
        session = self.server.session
        care_status = session.snapshot() if session else IDLE_STATUS
        status = {
            # contract fields; the ones the real bridge fills from the robot stay null here
            "state": care_status["state"],
            "place": care_status["care"]["zone"] if care_status["care"] else None,
            "person_found": session is not None,
            "answer": care_status["answer"],
            "battery_v": None,
            "estop_pressed": None,
            "bumper_hit": None,
            "smoke": None,
            "temperature_c": None,
            "message": "",
            "updated_at": care_status["updated_at"],
            "care": care_status["care"],
        }
        self._send_json(200, status)

    def _send_ui_file(self, relative: str) -> None:
        target = (UI_DIR / relative).resolve()
        if UI_DIR not in target.parents or not target.is_file():
            self._send_json(404, {"error": "not found"})
            return
        content = target.read_bytes()
        content_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        self._send(200, content, content_type)

    def _read_json(self):
        length = int(self.headers.get("Content-Length") or 0)
        if length == 0:
            return {}
        try:
            body = json.loads(self.rfile.read(length))
        except ValueError:
            return None
        return body if isinstance(body, dict) else None

    def _send_json(self, code: int, payload) -> None:
        content = b"" if payload is None else json.dumps(payload).encode()
        self._send(code, content, "application/json")

    def _send(self, code: int, content: bytes, content_type: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, format, *args):  # the page polls /status every second: keep quiet
        pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")
    profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    ears = listen if listen.available() else None
    call_base = os.environ.get("CALL_BASE_URL", "")
    telegram = caregiver.from_environment()
    server = CareServer(("0.0.0.0", args.port), profile, telegram, ears, call_base)
    log.info("care server on port %d, screen page at /ui/", args.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("bye")


if __name__ == "__main__":
    main()
