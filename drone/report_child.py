"""Tell the robot where the child is, and keep telling it until it answers.

The Mac flies the drone on the drone's own Wi-Fi; the Bobac is on another network. The report
leaves over whatever second link the Mac has (the targets below), again and again until the
robot acknowledges it. The robot ignores repeats of a mission_id, so a repeat after a lost
answer, or the same report over two links at once, moves it only once.

Targets (--to, any number; the first acknowledgement ends them all). Without --to: ROBOT_URL,
the robot's child_goal_server through ngrok, which needs only internet on the Mac (an iPhone
on USB while the Wi-Fi is on the drone). The robot runs `ngrok http 8001` on our ngrok
account, whose free domain never changes, as a service that starts at boot and restarts
(bobac/vendor/bobac3_ws/src/bringup/systemd/ngrok.service). Never run ngrok on the Mac.
    http(s)://<robot>:8001  the robot's child_goal_server, when the Mac has a route to the robot
    ntfy:<topic>            through the internet (https://ntfy.sh), when the Mac only has
                            internet: the robot runs child_relay.py on the same topic
                            (ntfy:http://<server>/<topic> for another ntfy server)

What is sent: {"mission_id", "x", "y", "frame": "robot"}: the child x metres ahead and y metres
to the left of the robot where it stood at takeoff (the mission's `launch` frame: the drone takes
off from the robot's top, camera toward the robot's front). The robot places that on its map with
its own pose, so it must stand still from the takeoff until the report. --heading-offset turns
the report when the drone's camera did not point along the robot's front at takeoff.

The drone server keeps showing the last mission's child. Only a child reported after --watch
started (or after the time given to deliver()) is sent: never the one of an earlier rehearsal.
That compares the drone server's clock with ours: run --watch on the machine that runs drone/fly.

    python3 -m drone.report_child --ping                       # through ngrok, before the flight
    python3 -m drone.report_child --x 2 --y 0 --follow         # through ngrok, by hand
    python3 -m drone.report_child --to http://<ROBOT_IP>:8001 --ping        # before the flight
    python3 -m drone.report_child --to http://<ROBOT_IP>:8001 --watch       # start it BEFORE
                # drone/find: reads drone/fly's GET /mission and reports the child when found
    python3 -m drone.report_child --to http://<ROBOT_IP>:8001 --x 3.6 --y 1.4   # by hand
    python3 -m drone.report_child --to http://<ROBOT_IP>:8001 --x 2 --y 0 --follow
                # by hand, then show the robot's state and pose until it arrives or fails
"""

import argparse
import http.client
import json
import logging
import math
import secrets
import threading
import time
import urllib.error
import urllib.request
from typing import Any

DRONE_URL = "http://127.0.0.1:8077"
# The robot's child_goal_server through ngrok (`ngrok http 8001` on the robot): the default target
ROBOT_URL = "https://<ROBOT_NGROK_DOMAIN>"
# ngrok's free plan answers requests without this header with a browser warning page
NGROK_SKIP_WARNING = ("ngrok-skip-browser-warning", "1")
# ngrok's own failures (header ngrok-error-code), said as what to fix on the robot
NGROK_ERRORS = {
    "ERR_NGROK_3200": "the robot's ngrok is offline: robot off or without internet "
    "(on the robot: systemctl --user status ngrok)",
    "ERR_NGROK_8012": "the robot's ngrok is up but child_goal_server is not "
    "(on the robot: roslaunch bringup state_demo.launch)",
}
CHILD_FOUND_PATH = "/child_found"
STATUS_PATH = "/status"
MISSION_PATH = "/mission"
NTFY_PREFIX = "ntfy:"
NTFY_SERVER = "https://ntfy.sh"
ACK_SUFFIX = "-ack"  # the relay answers on "<topic>-ack"
ROBOT_FRAME = "robot"  # ahead / left of the robot where it stood at takeoff
# A "never" from a child_goal_server that repeats the mission_id in it. The robot's server
# today repeats none: its 400 (a malformed report, which we do not send) is retried like the rest
ROBOT_REFUSALS = (400, 409)
HTTP_TIMEOUT_S = 6.0  # the venue Wi-Fi answers the robot's pings in 0.3-4.6 s
HTTP_RETRY_FIRST_S = 0.5
HTTP_RETRY_MAX_S = 2.0
# ntfy.sh allows a burst of 60 requests, then one per 5 s: each try is a publish and a listen
NTFY_RETRY_FIRST_S = 6.0
NTFY_RETRY_MAX_S = 8.0
NTFY_SINCE_MARGIN_S = 30  # answers published this long before our first try still count
GIVE_UP_S = 600.0
WATCH_POLL_S = 0.5
WAIT_SLICE_S = 0.5
WATCH_TIMEOUT_S = 2.0
PING_COUNT = 3
PING_WAIT_S = 10.0
FOLLOW_POLL_S = 1.0  # an ntfy link polls at its own, slower pace (its retry_first_s)
ROBOT_ARRIVED = "arrived"
ROBOT_STOPPED = ("failed", "idle")  # idle: the goal was cancelled

log = logging.getLogger("report_child")
# No proxy, whatever the system settings say: the robot is on a local network
_opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))


class LinkError(Exception):
    """This try got no acknowledgement from the robot: try again."""


class RefusedError(LinkError):
    """The robot said no to this report (a 400 or 409 repeating its mission_id): repeating it
    cannot help."""


class HttpTarget:
    """The robot's child_goal_server, reached directly."""

    retry_first_s, retry_max_s = HTTP_RETRY_FIRST_S, HTTP_RETRY_MAX_S

    def __init__(self, url: str) -> None:
        self.name = url.rstrip("/")

    def send(self, body: dict[str, Any], wait_s: float) -> dict[str, Any]:
        """The robot's acknowledgement of a report, else LinkError (or RefusedError)."""
        data = json.dumps(body).encode()
        answer = _http("POST", self.name + CHILD_FOUND_PATH, data, HTTP_TIMEOUT_S)
        reply = _answer(*answer, key=body["mission_id"])
        if not _acknowledges(reply, body["mission_id"]):
            raise LinkError("not an answer to our report: is that the child_goal_server?")
        return reply

    def ping(self) -> dict[str, Any]:
        """The robot's GET /status, else LinkError."""
        return _answer(*_http("GET", self.name + STATUS_PATH, None, HTTP_TIMEOUT_S))


class NtfyTarget:
    """The robot's child_relay.py, through an ntfy topic: publish, and listen for its answer."""

    retry_first_s, retry_max_s = NTFY_RETRY_FIRST_S, NTFY_RETRY_MAX_S

    def __init__(self, spec: str) -> None:
        server, _, topic = spec.rpartition("/") if "://" in spec else (NTFY_SERVER, "", spec)
        if not topic or "://" not in server:
            raise ValueError(f"ntfy target needs a topic: {NTFY_PREFIX}{spec}")
        self.name = NTFY_PREFIX + spec
        self._topic_url = f"{server}/{topic}"
        self._first_sent: dict[str, float] = {}  # key -> when we first published it

    def send(self, body: dict[str, Any], wait_s: float) -> dict[str, Any]:
        return self._round_trip(body, body["mission_id"], wait_s)

    def ping(self) -> dict[str, Any]:
        key = "ping-" + secrets.token_hex(4)
        return self._round_trip({"ping": key}, key, PING_WAIT_S)

    def _round_trip(self, body: dict[str, Any], key: str, wait_s: float) -> dict[str, Any]:
        """Publish `body` and wait about wait_s for the relay's answer to `key`. The listening
        starts before the publishing, so the answer cannot slip in between; an answer to an
        earlier try of the same key counts too (ntfy replays the last since_s seconds)."""
        first_sent = self._first_sent.setdefault(key, time.monotonic())
        since_s = int(time.monotonic() - first_sent) + NTFY_SINCE_MARGIN_S
        url = f"{self._topic_url}{ACK_SUFFIX}/json?since={since_s}s"
        deadline = time.monotonic() + wait_s
        refusal = None
        try:
            with _opener.open(url, timeout=max(wait_s, HTTP_TIMEOUT_S)) as answers:
                self._publish(body)
                for line in answers:  # one JSON event per line; stays open for new ones
                    answer = _object(_object(line).get("message") or "")
                    status = answer.get("status")
                    if answer.get("ack") == key and isinstance(status, int):
                        reply = _object(json.dumps(answer.get("reply")))
                        if not 200 <= status < 300:
                            if refusal is None or reply.get("mission_id") == key:
                                refusal = (status, json.dumps(reply))
                        elif "mission_id" not in body or _acknowledges(reply, key):
                            return reply
                    if time.monotonic() > deadline:
                        break
        except TimeoutError:
            pass  # nothing more within wait_s
        except (OSError, http.client.HTTPException) as error:
            raise LinkError(f"ntfy: {_reason(error)}") from error
        if refusal is None:
            raise LinkError("sent through ntfy, no answer from the robot's relay yet")
        return _answer(*refusal, key=key)

    def _publish(self, body: dict[str, Any]) -> None:
        status, text = _http("POST", self._topic_url, json.dumps(body).encode(), HTTP_TIMEOUT_S)
        if not 200 <= status < 300:
            raise LinkError(f"ntfy refused the message: HTTP {status} {text[:80]}")


def parse_target(text: str) -> HttpTarget | NtfyTarget:
    if text.startswith(NTFY_PREFIX):
        return NtfyTarget(text[len(NTFY_PREFIX) :])
    if text.startswith(("http://", "https://")):
        return HttpTarget(text)
    raise ValueError(f"target must be http://<robot>:<port> or {NTFY_PREFIX}<topic>: {text!r}")


class Delivery:
    """One report on its way: a thread per target, each retrying until one of them gets the
    robot's acknowledgement, the robot refuses the report, or GIVE_UP_S has passed."""

    def __init__(self, body: dict[str, Any], targets: list[str]) -> None:
        self.body = body  # {"mission_id", "x", "y", "frame"}
        self.ack: dict[str, Any] | None = None  # {"target", "reply", "attempts", "seconds"}
        self.done = threading.Event()  # acknowledged, refused or given up
        self._targets = [parse_target(target) for target in targets]
        self._started = time.monotonic()
        self._lock = threading.Lock()
        self._running = len(self._targets)

    def start(self) -> "Delivery":
        names = ", ".join(target.name for target in self._targets)
        where = "x {x:+.2f} y {y:+.2f} m ahead / left of the robot at takeoff".format(**self.body)
        log.info("robot link: child %s at %s -> %s", self.body["mission_id"], where, names)
        for target in self._targets:
            threading.Thread(target=self._run, args=(target,), daemon=True).start()
        return self

    def wait(self) -> bool:
        """Block until the delivery ended. True if the robot acknowledged the report."""
        # in slices: on Windows a wait without a timeout does not let Ctrl+C through
        while not self.done.wait(WAIT_SLICE_S):
            pass
        return self.ack is not None

    def _run(self, target: HttpTarget | NtfyTarget) -> None:
        try:
            self._retry(target)
        except Exception:  # a bug here must never reach the flight
            log.exception("robot link: %s stopped on an unexpected error", target.name)
        with self._lock:
            self._running -= 1
            given_up = self._running == 0 and not self.done.is_set()
        if given_up:
            seconds = time.monotonic() - self._started
            log.error("robot link: GAVE UP on %s after %.0f s", self.body["mission_id"], seconds)
            self.done.set()

    def _retry(self, target: HttpTarget | NtfyTarget) -> None:
        wait_s = target.retry_first_s
        attempts, last_error = 0, None
        while not self.done.is_set() and time.monotonic() - self._started < GIVE_UP_S:
            attempts += 1
            tried_at = time.monotonic()
            try:
                reply = target.send(self.body, wait_s)
            except RefusedError as error:
                self._end(logging.ERROR, f"robot REFUSED {error}: not sent again", target, None)
                return
            except LinkError as error:
                if str(error) != last_error:  # say each reason once, not at every try
                    last_error = str(error)
                    log.warning("robot link: %s try %d: %s; retrying", target.name, attempts, error)
            else:
                ack = {"target": target.name, "reply": reply, "attempts": attempts}
                self._end(logging.INFO, f"ROBOT ACKNOWLEDGED, {_describe(reply)}", target, ack)
                return
            self.done.wait(max(0.0, wait_s - (time.monotonic() - tried_at)))
            wait_s = min(wait_s * 2, target.retry_max_s)

    def _end(
        self, level: int, outcome: str, target: HttpTarget | NtfyTarget, ack: dict[str, Any] | None
    ) -> None:
        """The first acknowledgement or refusal ends the delivery on every link."""
        seconds = round(time.monotonic() - self._started, 1)
        with self._lock:
            if self.done.is_set():
                return  # another link was faster
            self.ack = ack and {**ack, "seconds": seconds}
            self.done.set()
        name = self.body["mission_id"]
        log.log(level, "robot link: %s: %s (via %s, %.1f s)", name, outcome, target.name, seconds)


_deliveries: dict[str, Delivery | None] = {}  # mission_id -> its delivery (None: nothing sent)
_deliveries_lock = threading.Lock()


def deliver(
    mission: dict[str, Any], targets: list[str], not_before: float, heading_offset_deg: float = 0.0
) -> Delivery | None:
    """Report the mission's child to the robot, once per child report: call it with every
    GET /mission status. Returns at once (the sending is in background threads) and never
    raises. `not_before`: time.time() taken before your own POST /mission/start; a child
    reported earlier is an earlier mission's (the server keeps showing it) and is not sent."""
    try:
        child = mission.get("child")
        if not child or not targets:
            return None
        key = mission_id(mission)
        with _deliveries_lock:
            if key in _deliveries:
                return _deliveries[key]
            _deliveries[key] = None
        body = child_report(mission, heading_offset_deg)
        reported_at = child.get("time") or mission.get("started_at") or 0.0
        if reported_at < not_before:
            log.error(
                "robot link: child %s NOT SENT: it was reported at %s, before this started (an "
                "earlier mission's child). If it is the right one, by hand: %s",
                key,
                time.strftime("%H:%M:%S", time.localtime(reported_at)),
                _by_hand(body, targets),
            )
            return None
        delivery = Delivery(body, targets).start()
        with _deliveries_lock:
            _deliveries[key] = delivery
        return delivery
    except ValueError as error:  # no usable position, or a bad target
        log.error("robot link: the child was NOT reported to the robot: %s", error)
        return None
    except Exception:  # the flight loop calls this: log, never raise
        log.exception("robot link: the child was NOT reported to the robot")
        return None


def finish() -> bool:
    """Wait for the reports still on their way (Ctrl+C gives up). True if the robot
    acknowledged every one. Call it before the script ends: the sending threads die with it."""
    with _deliveries_lock:
        sent = [delivery for delivery in _deliveries.values() if delivery is not None]
    try:
        for delivery in sent:
            if not delivery.done.is_set():
                name = delivery.body["mission_id"]
                log.info("robot link: still sending %s (Ctrl+C gives up)", name)
                delivery.wait()
    except KeyboardInterrupt:
        log.error("robot link: stopped before the robot acknowledged")
    return all(delivery.ack is not None for delivery in sent)


def mission_id(mission: dict[str, Any]) -> str:
    """One id per child report of one mission run: what the robot uses to ignore repeats."""
    return f"{int(mission.get('started_at') or 0)}-{mission['child'].get('id')}"


def child_report(mission: dict[str, Any], heading_offset_deg: float = 0.0) -> dict[str, Any]:
    """What the robot gets for the child of a GET /mission status: its position relative to
    the robot at takeoff. heading_offset_deg: how far to the left of the robot's front the
    drone's camera pointed at takeoff."""
    child = mission["child"]
    launch = child.get("launch")
    if not launch:
        raise ValueError(
            f"the mission has no launch-frame position for the child (no aligning leg?): "
            f"drone frame {child.get('drone')}"
        )
    turn = math.radians(heading_offset_deg)
    x = launch["x"] * math.cos(turn) - launch["y"] * math.sin(turn)
    y = launch["x"] * math.sin(turn) + launch["y"] * math.cos(turn)
    return _body(mission_id(mission), x, y)


def ping(targets: list[str], count: int = PING_COUNT) -> bool:
    """Ask the robot for its status through each target, `count` times. True if every target
    answered at least once."""
    all_ok = True
    for target in map(parse_target, targets):
        answered = 0
        for _ in range(count):
            asked_at = time.monotonic()
            try:
                status = target.ping()
            except LinkError as error:
                print(f"FAIL {target.name}: {error}")
                continue
            answered += 1
            print(f"OK   {target.name}  {time.monotonic() - asked_at:.2f} s  {_robot_line(status)}")
        print(f"     {target.name}: {answered}/{count} answered")
        all_ok = all_ok and answered > 0
    return all_ok


def follow(target_name: str, key: str) -> bool:
    """Log the robot's state and pose through one target until it arrived for report `key`
    (True), or failed, was cancelled or took another report (False)."""
    target = parse_target(target_name)
    poll_s = max(FOLLOW_POLL_S, target.retry_first_s)
    last_line = last_error = None
    while True:
        try:
            status = target.ping()
        except LinkError as error:
            if str(error) != last_error:
                last_error = str(error)
                log.warning("robot status via %s: %s; retrying", target.name, error)
            time.sleep(poll_s)
            continue
        last_error = None
        line = _robot_line(status)
        if line != last_line:
            last_line = line
            log.info("%s", line)
        if status.get("mission_id") not in (key, None):
            log.error("the robot took another report: %s", status.get("mission_id"))
            return False
        if status.get("state") == ROBOT_ARRIVED:
            return True
        if status.get("state") in ROBOT_STOPPED:
            return False
        time.sleep(poll_s)


def watch(drone_url: str, targets: list[str], heading_offset_deg: float) -> None:
    """Poll the drone server's GET /mission and report every child found from now on. Runs
    until Ctrl+C, next to drone/find: a process of its own, so nothing here can disturb the
    flight."""
    started_at = time.time()
    url = drone_url.rstrip("/") + MISSION_PATH
    log.info("watching %s: a child found from now on goes to %s", url, ", ".join(targets))
    last_state = last_error = None
    while True:
        try:
            mission = _answer(*_http("GET", url, None, WATCH_TIMEOUT_S))
        except LinkError as error:
            if str(error) != last_error:
                last_error = str(error)
                log.warning("no mission status from %s: %s (is drone/fly running?)", url, error)
            time.sleep(WATCH_POLL_S)
            continue
        last_error = None
        if mission.get("state") != last_state:
            last_state = mission.get("state")
            log.info("mission %s", last_state)
        deliver(mission, targets, started_at, heading_offset_deg)
        time.sleep(WATCH_POLL_S)


def _body(key: str, x: float, y: float) -> dict[str, Any]:
    return {"mission_id": key, "x": round(x, 3), "y": round(y, 3), "frame": ROBOT_FRAME}


def _by_hand(body: dict[str, Any], targets: list[str]) -> str:
    to = " ".join(f"--to {target}" for target in targets)
    return (
        f"python3 -m drone.report_child {to} --x {body['x']} --y {body['y']} "
        f"--mission-id {body['mission_id']}"
    )


def _http(method: str, url: str, body: bytes | None, timeout: float) -> tuple[int, str]:
    """(status, text) of one request; an HTTP error status is returned, not raised."""
    request = urllib.request.Request(url, data=body, method=method)
    request.add_header(*NGROK_SKIP_WARNING)
    if body is not None:
        request.add_header("Content-Type", "application/json")
    try:
        with _opener.open(request, timeout=timeout) as response:
            return response.status, response.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as error:
        ngrok_error = error.headers.get("ngrok-error-code")
        if ngrok_error:
            raise LinkError(NGROK_ERRORS.get(ngrok_error, f"ngrok: {ngrok_error}")) from error
        return error.code, error.read().decode("utf-8", "replace")
    except (OSError, http.client.HTTPException) as error:  # URLError and timeouts are OSErrors
        raise LinkError(_reason(error)) from error


def _answer(status: int, text: str, key: str | None = None) -> dict[str, Any]:
    """The JSON object of a 2xx answer. The robot's own refusal of report `key` (a 400 or 409
    that repeats its mission_id) is a RefusedError. Anything else is a LinkError, worth another
    try: 503 "no pose yet", no status, or a 4xx that names no mission_id (the robot's server
    today, or whatever else answers at that address)."""
    reply = _object(text)
    if 200 <= status < 300:
        return reply
    reason = f"HTTP {status}: {reply.get('error') or text[:80]}"
    if status in ROBOT_REFUSALS and key is not None and reply.get("mission_id") == key:
        raise RefusedError(reason)
    raise LinkError(reason)


def _acknowledges(reply: dict[str, Any], key: str) -> bool:
    """True when a 2xx answer is the child_goal_server's to report `key`: it repeats the
    mission_id, or names none and says accepted or duplicate (the robot's server today)."""
    if "mission_id" in reply:
        return reply["mission_id"] == key
    return bool(reply.get("accepted") or reply.get("duplicate"))


def _object(text: str | bytes) -> dict[str, Any]:
    """The JSON object in `text`, or {} when it is not one."""
    try:
        value = json.loads(text)
    except ValueError:
        return {}
    return value if isinstance(value, dict) else {}


def _reason(error: BaseException) -> str:
    reason = getattr(error, "reason", None) or error
    return str(reason) or type(reason).__name__


def _describe(reply: dict[str, Any]) -> str:
    goal = reply.get("goal") or {}
    where = f"goal x {goal['x']:.2f} y {goal['y']:.2f} (map)" if goal else "no goal in the answer"
    if reply.get("duplicate"):
        return f"it already had it (robot {reply.get('state')}), {where}"
    return f"driving, {where}"


def _robot_line(status: dict[str, Any]) -> str:
    pose = status.get("pose")
    place = (
        f"at x {pose['x']:.2f} y {pose['y']:.2f} yaw {math.degrees(pose['yaw']):.0f} deg (map)"
        if pose
        else "NO POSE (no fresh tf map -> base_footprint: it cannot place the child)"
    )
    return f"robot {status.get('state')} ({status.get('message')}), {place}"


def main() -> None:
    parser = argparse.ArgumentParser(description="report the child's position to the robot")
    parser.add_argument(
        "--to",
        action="append",
        metavar="TARGET",
        help=f"http://<robot>:8001 or {NTFY_PREFIX}<topic>; repeat for more than one link "
        f"(default {ROBOT_URL})",
    )
    parser.add_argument("--ping", action="store_true", help="check the link(s), send no report")
    parser.add_argument(
        "--watch",
        nargs="?",
        const=DRONE_URL,
        metavar="DRONE_URL",
        help=f"report the child the running mission finds (default {DRONE_URL})",
    )
    parser.add_argument("--x", type=float, help="by hand: metres ahead of the robot")
    parser.add_argument("--y", type=float, help="by hand: metres to the left of the robot")
    parser.add_argument("--mission-id", help="by hand: default a new one (the robot drives again)")
    parser.add_argument(
        "--follow",
        action="store_true",
        help="by hand: after the acknowledgement, follow the robot until it arrives or fails",
    )
    parser.add_argument(
        "--heading-offset",
        type=float,
        default=0.0,
        help="--watch: degrees the drone's camera pointed left of the robot's front at takeoff",
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    args.to = args.to or [ROBOT_URL]
    try:
        for target in args.to:
            parse_target(target)
    except ValueError as error:
        parser.error(str(error))
    if args.ping:
        raise SystemExit(0 if ping(args.to) else 1)
    if args.watch:
        try:
            if not ping(args.to, count=1):
                log.warning("robot link: NOT answering now; a report is retried until it does")
            watch(args.watch, args.to, args.heading_offset)
        except KeyboardInterrupt:
            # a report may still wait for its acknowledgement: Ctrl+C again gives up
            raise SystemExit(0 if finish() else 1) from None
    if args.x is None or args.y is None:
        parser.error("give --ping, --watch, or --x and --y")
    body = _body(args.mission_id or f"manual-{int(time.time())}", args.x, args.y)
    try:
        delivery = Delivery(body, args.to).start()
        if not delivery.wait():
            raise SystemExit(1)
        if args.follow:
            raise SystemExit(0 if follow(delivery.ack["target"], body["mission_id"]) else 1)
    except KeyboardInterrupt:
        raise SystemExit(1) from None


if __name__ == "__main__":
    main()
