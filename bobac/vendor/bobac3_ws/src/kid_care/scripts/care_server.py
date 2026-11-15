"""The care flow (care.py) as one program that needs no ROS.

The robot's screen shows nothing of its own: the only window this opens is the video call
with the parents, full screen. Normally started by the launch file (`roslaunch kid_care
kid_care.launch`) through kid_care_node.py, which loads the keys from the package's hidden
.env first. The HTTP API:

    GET  /status        contract fields + "care" (the details of the episode)
    POST /care/start    {"found_time": epoch seconds (optional)} -> the robot reached the
                        lost child
    POST /ui/answer     {"answer": "ok" | "help" | "yes" | "no"}, for the child's answers
    POST /stop

Also runs on its own, on the robot (system Python 3.8) or on a laptop:

    python3 care_server.py --port 8090
    curl -X POST localhost:8090/care/start -d '{}'
    curl localhost:8090/status

Messages to the parents go through ntfy if NTFY_TOPIC is set (no account needed, see
ntfy_channel.py), else Telegram with TELEGRAM_BOT_TOKEN and TELEGRAM_CHAT_IDS (caregiver.py). Spoken
answers need Vosk and its model (see listen.py). The video call uses CALL_BASE_URL (a Jitsi
server, default in care.py). The child profile is found next to this file, or where
KID_CARE_PROFILE points (the launch file sets it, so an installed package works too). Its name
gives way to the one the parent typed on the web page for the person being searched
(KID_CARE_PERSON): scripts/web_inbox.py's request.json in that person's folder of KID_CARE_FACES.
"""

import argparse
import contextlib
import json
import logging
import os
import shutil
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import agent_process
import care
import caregiver
import listen
import marionette
import ntfy_channel
import speech

DEFAULT_PORT = 8000
HERE = Path(__file__).resolve().parent
PROFILE_PATH = Path(os.environ.get("KID_CARE_PROFILE") or HERE.parent / "config" / "child.json")
# The person face_check searches for, and face_rec's folders (the web's requests land there)
PERSON = os.environ.get("KID_CARE_PERSON", "")
FACE_DIRECTORY = Path(os.environ.get("KID_CARE_FACES") or Path.home() / ".reinovo" / "face_data")
WEB_REQUEST_FILE = "request.json"
BUZZER_TIMEOUT_S = 5
CALL_PROFILE_DIR = Path.home() / "leuk" / "call-firefox"
CALL_LOG_PATH = Path.home() / "leuk" / "call-firefox.log"
FIREFOX_PROFILE_LOCKS = ("lock", ".parentlock")  # left behind when Firefox is killed
FIREFOX_CLOSE_TIMEOUT_S = 3
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
# Full screen windows nobody can click away: no first-run pages, no "restore session".
KIOSK_FIREFOX_PREFS = (
    'user_pref("browser.shell.checkDefaultBrowser", false);\n'
    'user_pref("browser.sessionstore.resume_from_crash", false);\n'
    'user_pref("browser.aboutwelcome.enabled", false);\n'
    'user_pref("datareporting.policy.firstRunURL", "");\n'
)
# The call window must use the camera and microphone with nobody there to tap "Allow".
CALL_FIREFOX_PREFS = KIOSK_FIREFOX_PREFS + (
    'user_pref("media.navigator.permission.disabled", true);\n'
    'user_pref("permissions.default.camera", 1);\n'
    'user_pref("permissions.default.microphone", 1);\n'
    'user_pref("media.autoplay.default", 0);\n'
    f'user_pref("marionette.port", {CALL_MARIONETTE_PORT});\n'
    # Firefox's default camera on the robot is the floor-level base camera: use the head one
    'user_pref("media.video_loopback_dev", "A68-1600W: A68-1600W");\n'
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


def open_kiosk(profile_dir: Path, prefs: str, url: str, log_path: Path, extra_args=()):
    """`url` full screen on the robot display, in a Firefox of its own; None without Firefox."""
    if shutil.which("firefox") is None:
        log.warning("firefox not installed: open it yourself: %s", url)
        return None
    free_profile(profile_dir)
    profile_dir.mkdir(parents=True, exist_ok=True)
    (profile_dir / "user.js").write_text(prefs)
    env = dict(os.environ, DISPLAY=os.environ.get("DISPLAY", ":0"))
    command = ["firefox", "--new-instance", *extra_args, "--profile", str(profile_dir)]
    command += ["--kiosk", url]
    # Firefox errors go to a file: when the window does not appear, the reason is there.
    with open(str(log_path), "ab") as output:
        return subprocess.Popen(command, env=env, stdout=output, stderr=output)


def close_window(process) -> None:
    if process and process.poll() is None:
        process.terminate()
        try:
            process.wait(FIREFOX_CLOSE_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def free_profile(profile_dir: Path) -> None:
    """A Firefox left by an earlier run (server killed or restarted) still holds the profile, and
    the call one also the camera and the microphone, so the new window would hang: end it."""
    if shutil.which("pkill") is not None:
        leftover = subprocess.run(
            ["pkill", "-f", str(profile_dir)], check=False, stdout=subprocess.DEVNULL
        )
        if leftover.returncode == 0:
            log.warning("closed a window left open by an earlier run (%s)", profile_dir.name)
            time.sleep(FIREFOX_CLOSE_TIMEOUT_S)
    for lock in FIREFOX_PROFILE_LOCKS:
        with contextlib.suppress(OSError):  # no stale lock: nothing to remove
            (profile_dir / lock).unlink()


class CallWindow:
    """The video call, full screen on the robot display, in a Firefox of its own. It is the
    only thing the robot puts on its screen: it opens when the episode starts and closes with
    it, so the camera is free for the next one."""

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
        self._process = open_kiosk(
            CALL_PROFILE_DIR, CALL_FIREFOX_PREFS, url, CALL_LOG_PATH, ("--marionette",)
        )
        if not self._process:
            return
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
            close_window(self._process)
            log.info("video call window closed")
        self._process = None
        self._members = None


def requested_name(person: str, faces: Path = FACE_DIRECTORY):
    """The name the parent typed on the web page for `person`, None without one. The web's folders
    are in capitals (JORGE) and face_check's person is not: compared without case."""
    if not person:
        return None
    for folder in sorted(faces.glob("*")):
        if folder.name.lower() != person.lower():
            continue
        try:
            name = json.loads((folder / WEB_REQUEST_FILE).read_text(encoding="utf-8")).get("name")
        except (OSError, ValueError, AttributeError):
            continue  # a folder of photos added by hand, without a request
        if isinstance(name, str) and name.strip():
            return name.strip()
    return None


class CareServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, profile, telegram, ears, call_base):
        super().__init__(address, Handler)
        self.profile = profile
        self.telegram = telegram
        self.ears = ears
        self.call_base = call_base
        self.call_window = CallWindow()
        self.agent = agent_process if agent_process.available() else None
        self.session = None
        self.session_thread = None
        self.session_lock = threading.Lock()

    def start_session(self, found_time: float) -> bool:
        with self.session_lock:
            if self.session_thread and self.session_thread.is_alive():
                return False
            if self.telegram:
                self.telegram.skip_backlog()
            profile = self.profile
            name = requested_name(PERSON)
            if name:  # read now, not at start-up: the request may arrive after the launch
                log.info("the child is %s (the web page's request for %s)", name, PERSON)
                profile = {**self.profile, "name": name}
            self.session = care.CareSession(
                profile,
                found_time,
                speech,
                self.telegram,
                buzz,
                self.ears,
                self.call_base,
                self.call_window.show,
                self.call_window.members,
                self.agent,
            )
            self.session_thread = threading.Thread(target=self.session.run, daemon=True)
            self.session_thread.start()
            return True


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/status":
            self._send_status()
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
        found_time = body.get("found_time", time.time())
        if not isinstance(found_time, (int, float)):
            self._send_json(400, {"error": "found_time must be a number"})
        elif self.server.start_session(float(found_time)):
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
            "place": None,  # contract field: there are no named places in this demo
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

    def log_message(self, format, *args):  # /status can be polled once a second: keep quiet
        pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, signal.default_int_handler)  # roslaunch stop = Ctrl+C: clean up
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")
    profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    ears = listen if listen.available() else None
    call_base = os.environ.get("CALL_BASE_URL", "")
    first_parent = profile["parents"][0]["name"].split()[0]
    parents = ntfy_channel.from_environment(first_parent) or caregiver.from_environment()
    log.info("parents are told through %s", type(parents).__name__ if parents else "nothing")
    server = CareServer(("0.0.0.0", args.port), profile, parents, ears, call_base)
    log.info("care server on port %d", args.port)
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
