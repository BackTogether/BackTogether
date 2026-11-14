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
    curl -X POST localhost:8000/care/start -d '{"zone": "park"}'

Messages to the parents go through ntfy if NTFY_TOPIC is set (no account needed, see
ntfy_channel.py), else Telegram with TELEGRAM_BOT_TOKEN and TELEGRAM_CHAT_IDS (caregiver.py). Spoken
answers need Vosk and its model (see listen.py); without them only the buttons work. The video
call uses CALL_BASE_URL (a Jitsi server, default in care.py).
"""

import argparse
import contextlib
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
import marionette
import ntfy_channel
import speech

DEFAULT_PORT = 8000
HERE = Path(__file__).resolve().parent
UI_DIR = HERE / "ui"
PROFILE_PATH = HERE.parent / "config" / "child.json"
BUZZER_TIMEOUT_S = 5
CALL_PROFILE_DIR = Path.home() / "leuk" / "call-firefox"
CALL_LOG_PATH = Path.home() / "leuk" / "call-firefox.log"
CALL_PROFILE_LOCKS = ("lock", ".parentlock")  # left behind when Firefox is killed
CALL_CLOSE_TIMEOUT_S = 3
CALL_MARIONETTE_PORT = 2829  # localhost only: lets this program press Join in the call window
CALL_JOIN_TIMEOUT_S = 300  # the call page can take minutes to load on the robot's connection
CALL_WATCH_POLL_S = 1
# Runs in the call page: once in the call, the number of people in it (robot included);
# before that, press Join on the name screen when it can be pressed and return -1.
CALL_WATCH_SCRIPT = """
const page = window.wrappedJSObject || window;
try { if (page.APP.conference.isJoined()) return page.APP.conference.membersCount; } catch (e) {}
const join = document.querySelector('[data-testid="prejoin.joinMeeting"]');
if (join && !join.disabled && join.getAttribute("aria-disabled") !== "true") join.click();
return -1;
"""
# The call window must use the camera and microphone with nobody there to tap "Allow".
CALL_FIREFOX_PREFS = (
    'user_pref("media.navigator.permission.disabled", true);\n'
    'user_pref("permissions.default.camera", 1);\n'
    'user_pref("permissions.default.microphone", 1);\n'
    'user_pref("media.autoplay.default", 0);\n'
    f'user_pref("marionette.port", {CALL_MARIONETTE_PORT});\n'
    # Firefox's default camera on the robot is the floor-level base camera: use the head one
    'user_pref("media.video_loopback_dev", "A68-1600W: A68-1600W");\n'
    'user_pref("browser.shell.checkDefaultBrowser", false);\n'
    'user_pref("browser.sessionstore.resume_from_crash", false);\n'
)
IDLE_STATUS = {"state": "idle", "answer": None, "updated_at": 0.0, "care": None}

log = logging.getLogger("care_server")


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
        self._members = None  # people in the call, robot included; None until it is in

    def members(self):
        """People in the call counting the robot, or None while it is not (yet) in a call."""
        return self._members

    def show(self, url) -> None:
        with self._lock:
            self._close()
            if url:
                self._open(url)

    def _open(self, url: str) -> None:
        if shutil.which("firefox") is None:
            log.warning("firefox not installed: open the call yourself: %s", url)
            return
        self._free_profile()
        CALL_PROFILE_DIR.mkdir(parents=True, exist_ok=True)
        (CALL_PROFILE_DIR / "user.js").write_text(CALL_FIREFOX_PREFS)
        env = dict(os.environ, DISPLAY=os.environ.get("DISPLAY", ":0"))
        command = ["firefox", "--new-instance", "--marionette", "--profile", str(CALL_PROFILE_DIR)]
        command += ["--kiosk", url]
        # Firefox errors go to a file: when the window does not appear, the reason is there.
        with open(str(CALL_LOG_PATH), "ab") as output:
            self._process = subprocess.Popen(command, env=env, stdout=output, stderr=output)
        log.info("video call window opened: %s (firefox log: %s)", url, CALL_LOG_PATH)
        threading.Thread(target=self._watch, args=(self._process,), daemon=True).start()

    def _watch(self, process) -> None:
        """Press Join for the child, then keep counting the people in the call.

        The call server only turns the camera and microphone on when Join is pressed on its
        name screen (skipping that screen by URL joins muted, without camera), and a lost child
        will not press it. Firefox allows one control connection, so this one thread does both."""
        browser = marionette.Marionette(CALL_MARIONETTE_PORT)
        deadline = time.time() + CALL_JOIN_TIMEOUT_S
        try:
            while process.poll() is None:
                try:
                    members = int(browser.run_script(CALL_WATCH_SCRIPT))
                except (OSError, TypeError, ValueError):
                    members = -1  # Firefox is starting, or the page is changing
                if members >= 0 and self._members is None:
                    log.info("call window: the robot is in the call")
                if members < 0 and self._members is None and time.time() > deadline:
                    log.warning("call window: not in the call after %d s", CALL_JOIN_TIMEOUT_S)
                    deadline = float("inf")  # say it once
                if members >= 0:
                    self._members = members
                time.sleep(CALL_WATCH_POLL_S)
        finally:
            browser.close()
            self._members = None

    def _close(self) -> None:
        if self._process and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(CALL_CLOSE_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
            log.info("video call window closed")
        self._process = None
        self._members = None

    def _free_profile(self) -> None:
        """A call Firefox left by an earlier run (server killed or restarted during a call) still
        holds the camera, the microphone and the profile, and the new call would hang: end it."""
        if shutil.which("pkill") is not None:
            leftover = subprocess.run(
                ["pkill", "-f", str(CALL_PROFILE_DIR)], check=False, stdout=subprocess.DEVNULL
            )
            if leftover.returncode == 0:
                log.warning("closed a call window left open by an earlier run")
                time.sleep(CALL_CLOSE_TIMEOUT_S)
        for lock in CALL_PROFILE_LOCKS:
            with contextlib.suppress(OSError):  # no stale lock: nothing to remove
                (CALL_PROFILE_DIR / lock).unlink()


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
                buzz,
                self.ears,
                self.call_base,
                self.call_window.show,
                self.call_window.members,
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
    first_parent = profile["parents"][0]["name"].split()[0]
    parents = ntfy_channel.from_environment(first_parent) or caregiver.from_environment()
    log.info("parents are told through %s", type(parents).__name__ if parents else "nothing")
    server = CareServer(("0.0.0.0", args.port), profile, parents, ears, call_base)
    log.info("care server on port %d, screen page at /ui/", args.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("bye")
    finally:
        # never leave the call window open: it would keep the camera and block the next call
        if server.session:
            server.session.stop()
        server.call_window.show(None)


if __name__ == "__main__":
    main()
