"""Lost-child search mission (demo, Mon 16 Nov): the drone takes off from the top of the robot,
patrols, flies over a detection, reports the child's position (drone frame and the robot's
`map`), orbits the child, then flies back to the takeoff point and lands.

Run by the server (`POST /mission/start`, API.md), or in Python with a Navigator:

    mission = Mission(navigator)
    mission.prepare(MissionConfig(waypoints=(Waypoint(3, 0), Waypoint(3, 2))))
    mission.run()          # blocking; detections come in with report_target() from any thread

States (`status()["state"]`), in order: idle, launching (waits for the button takeoff),
climbing, aligning (the calibration leg), patrolling, approaching (back to patrolling when the
child is lost), reporting, orbiting, returning; then one end state: landed, stopped (STOP: the
drone hovers where it was, `stopped_in` says in which state), aborted (abort(): back to the
takeoff point and landed), failed (`message` says why; landed at the takeoff point if it flew).

Frames:
- drone: the odometry frame of this flight. Origin = takeoff point, x/y horizontal, y 90 deg
  clockwise of x seen from above; its axes are not aligned with the heading or the room.
- launch: the robot at takeoff, ROS convention (x forward, y left). ASSUMPTION: the drone sits
  on the robot with its camera facing the robot's forward. The aligning leg flies `align_m`
  straight ahead (camera forward, no yaw): the odometry displacement direction is the robot's
  forward axis in the drone frame. Its error is about atan(odometry noise / leg) ~ 4 deg.
- map: the robot's map, once the robot sent its pose at launch (set_launch_pose; the ROS node
  does it from tf map -> base_footprint). The takeoff point is taken to be base_footprint.

Detections carry the time of the frame they were found in (X-Frame-Time of /frame.jpg or of
each /video.mjpg part); the pose and camera tilt at that time come from a 10 s history, so the
detector's latency does not matter. A detection's `anchor` says which point of the bbox to put
on the floor, through the pinhole model of camera.py:
- "feet" (a whole body, drone/identify/feed.py): its floor contact point
  (mission_geometry.floor_contact), on the floor: exact whatever the child's height;
- "centre" (a head, a face, the default): the bbox centre, target_height_m above the floor.

Approach (`approach`), both slow (`approach_speed`) and at the patrol altitude (no descent):
- "track" (default): approach_track.py, the camera follows the child the whole way down to
  -90, then the drone centres the bbox in the image and takes the fix there.
- "fly_over": goto the floor point of the triggering detection, then camera down and
  stop-and-correct until the bbox is centred (below).
Both end exactly over the child: the report has the child's position and the drone's pose at
the fix (child["fix"]). The orbit then flies at `orbit_alt_m`; None (the default) keeps the
current altitude.

Safety: every flight step runs under the Navigator's rules (geofence, battery, link loss);
STOP (Drone.stop through the server) ends the mission at once; each step has a timeout.
"""

import contextlib
import math
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import asdict, dataclass, field, replace
from typing import Any

from .approach_track import (
    ChildFix,
    OverheadSample,
    TrackApproach,
    TrackSettings,
    centre_error_px,
    child_fix,
)
from .client import standoff_goal
from .estimate import DEFAULT_YAW_RATE_DPS, wrap_deg
from .mission_geometry import (
    Alignment,
    LaunchPose,
    body_to_frame,
    floor_contact,
    frame_to_body,
    ground_offset,
    wrap_rad,
)
from .navigate import (
    ALT_GAIN,
    APPROACH_FRACTION,
    CRUISE_MAX,
    FLYING_PREFIX,
    HOME,
    MAX_PULSE_S,
    MAX_VERTICAL,
    MIN_PULSE_S,
    PULSE_S_PER_M,
    SETTLE_S,
    TICK_S,
    YAW_GAIN,
    Navigator,
)
from .patrol import Patrol, PatrolOptions, Waypoint
from .safety import Interrupted, NotReady, ReturnHome, SafetyLanding
from .telemetry import Pose, Telemetry

# States
IDLE = "idle"
LAUNCHING = "launching"
CLIMBING = "climbing"
ALIGNING = "aligning"
PATROLLING = "patrolling"
APPROACHING = "approaching"
REPORTING = "reporting"
ORBITING = "orbiting"
RETURNING = "returning"
LANDED = "landed"
STOPPED = "stopped"
ABORTED = "aborted"
FAILED = "failed"
END_STATES = (IDLE, LANDED, STOPPED, ABORTED, FAILED)
SEARCH_STATES = (LAUNCHING, CLIMBING, ALIGNING, PATROLLING, APPROACHING)  # detections count
# Frames and patrol strategies
FRAME_DRONE = "drone"
FRAME_LAUNCH = "launch"
FRAME_MAP = "map"
FRAMES = (FRAME_LAUNCH, FRAME_DRONE, FRAME_MAP)
STRATEGY_WAYPOINTS = "waypoints"  # the SDK's Patrol
STRATEGY_EXTERNAL = "external"  # hover under the safety rules while a client flies /move
STRATEGIES = (STRATEGY_WAYPOINTS, STRATEGY_EXTERNAL)
APPROACH_FLY_OVER = "fly_over"  # goto the detection, then centre with the camera down
APPROACH_TRACK = "track"  # approach_track.py: the camera follows the child all the way
APPROACHES = (APPROACH_FLY_OVER, APPROACH_TRACK)
# Which point of a detection's bbox goes on the floor
ANCHOR_CENTRE = "centre"  # the bbox centre, target_height_m above the floor (head, face)
ANCHOR_FEET = "feet"  # a standing body: its floor contact point, on the floor
ANCHORS = (ANCHOR_CENTRE, ANCHOR_FEET)
# Why the mission interrupted its own step (Drone.stop), by priority
REASON_TARGET = "target"
REASON_TIMEOUT = "timeout"
REASON_ABORT = "abort"
REASON_STOP = "stop"
REASON_PRIORITY = {REASON_TARGET: 1, REASON_TIMEOUT: 1, REASON_ABORT: 2, REASON_STOP: 3}
# Timing
WAIT_POLL_S = 0.05
ODOMETRY_WAIT_S = 3.0
LAND_TIMEOUT_S = 30.0
HISTORY_S = 10.0  # pose/tilt history for detections
HISTORY_SAMPLES = 120  # telemetry is 10 Hz
MAX_DETECTION_AGE_S = 3.0  # an older frame does not start an approach
RETRY_COOLDOWN_S = 5.0  # after a failed approach, before a detection starts another one
REPORT_HOLD_S = 3.0  # hover over the child while the robot picks up the position
HISTORY_STATES = 40
# Aligning leg
ALIGN_FRACTION = 0.35  # forward stick, fraction of the speed cap
ALIGN_TIMEOUT_S = 10.0
ALIGN_MIN_M = 0.5  # shorter displacement: no alignment (map frame unavailable)
LAUNCH_POSE_WAIT_S = 15.0  # a map-frame patrol waits this long for the robot's launch pose
# Approach
COARSE_M = 0.8  # farther than this from the drone: fly over the point with goto first
COARSE_TOLERANCE_M = 0.3
CENTRED_M = 0.25  # bbox centre within this of the image centre's floor point
CENTRED_COUNT = 2  # consecutive centred detections to accept the position
MIN_RANGE_M = 0.3  # height above the target used when the altitude is lower than that
TILT_OK_DEG = 2.0
FENCE_INSET_M = 0.3
# Orbit (sticks: % of 100; the X1's horizontal m/s per stick % is unmeasured, the sim's is 0.02)
ORBIT_MPS_PER_STICK_PCT = 0.02  # feed-forward only: the position loop corrects the rest
ORBIT_MAX_SPEED_MPS = 0.35
ORBIT_YAW_FRACTION = 0.8  # of the fastest turn at the current speed cap
ORBIT_POSITION_GAIN = 0.8  # m/s per metre off the circle
DEFAULT_ORBIT_TILT_DEG = -30.0

PatrolStrategy = Callable[[Navigator], object]
"""A patrol: flies until it ends (returns) or Drone.stop() interrupts it (raises Interrupted).
The mission stops it with Drone.stop() when a detection comes. E.g. the SDK's Patrol(...).run
(the default), an in-process DVD patrol (drone/patrol/dvd.py), or the lawnmower sweep:
drone.patrol.sweep.MissionSweep([...flags]) (it resumes where it was after an approach). Over
HTTP, strategy "external" + `drone/patrol/live sweep` does the same from another process."""


@dataclass(frozen=True)
class MissionConfig:
    """Everything POST /mission/start takes (API.md). Metres, seconds, degrees."""

    waypoints: tuple[Waypoint, ...] = ()  # in `frame`; alt None = patrol_alt_m
    frame: str = FRAME_LAUNCH
    strategy: str = STRATEGY_WAYPOINTS
    patrol: PatrolStrategy | None = None  # Python only: replaces waypoints and strategy
    loops: int = 0  # 0 = until found or patrol_timeout_s
    dwell_s: float = 1.0
    scan: bool = False
    patrol_alt_m: float = 2.0  # above the takeoff point
    patrol_tilt_deg: float | None = -40.0  # None: leave the camera where it is
    align_m: float = 1.5  # the calibration leg; 0 = none (then only frame "drone")
    approach: str = APPROACH_TRACK
    approach_speed: float = 0.2  # horizontal stick fraction while approaching
    approach_tilt_deg: float = -90.0
    orbit_alt_m: float | None = None  # None: orbit at the current altitude (no descent)
    orbit_radius_m: float = 1.5
    orbit_turns: float = 2.0  # 0 = no orbit
    standoff_m: float = 0.8  # the robot's goal stops this short of the child
    launch_height_m: float = 0.0  # the takeoff point above the floor (the robot's top)
    target_height_m: float = 0.5  # a "centre" detection's point above the floor
    min_confidence: float = 0.5
    takeoff_timeout_s: float = 120.0
    patrol_timeout_s: float = 600.0
    approach_timeout_s: float = 90.0
    lost_timeout_s: float = 10.0

    def as_dict(self) -> dict[str, Any]:
        data = asdict(replace(self, patrol=None))  # asdict deep-copies: a patrol may hold threads
        data["patrol"] = getattr(self.patrol, "__name__", "custom") if self.patrol else None
        return data


@dataclass(frozen=True)
class Detection:
    """A detector's report. bbox = (x0, y0, x1, y1), fractions of the image (0..1, y down)."""

    frame_time: float  # unix time of the frame (X-Frame-Time)
    bbox: tuple[float, float, float, float]
    confidence: float = 1.0
    label: str | None = None
    anchor: str = ANCHOR_CENTRE
    received_at: float = field(default_factory=time.time)

    @property
    def centre(self) -> tuple[float, float]:
        x0, y0, x1, y1 = self.bbox
        return (x0 + x1) / 2, (y0 + y1) / 2

    def floor_point(self, tilt_deg: float) -> tuple[float, float, bool]:
        """(u, v) of the point to place, and True when that point is on the floor (a body's
        feet), False when it is target_height_m above it (the bbox centre)."""
        if self.anchor == ANCHOR_FEET:
            return (*floor_contact(self.bbox, tilt_deg), True)
        return (*self.centre, False)

    def as_dict(self) -> dict[str, Any]:
        return {**asdict(self), "age_s": round(time.time() - self.frame_time, 2)}


class PoseHistory:
    """The last HISTORY_S of (time, pose, camera tilt), fed by Drone.on_telemetry."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._samples: deque[tuple[float, Pose, float | None]] = deque(maxlen=HISTORY_SAMPLES)

    def add(self, telemetry: Telemetry) -> None:
        if telemetry.pose is None:
            return
        tilt = telemetry.tilt.deg if telemetry.tilt else None
        with self._lock:
            self._samples.append((telemetry.time, telemetry.pose, tilt))

    def at(self, when: float) -> tuple[Pose, float | None] | None:
        """The sample nearest to unix time `when`; None if none within HISTORY_S."""
        with self._lock:
            samples = list(self._samples)
        if not samples:
            return None
        sample_time, pose, tilt = min(samples, key=lambda sample: abs(sample[0] - when))
        return (pose, tilt) if abs(sample_time - when) <= HISTORY_S else None


class Mission:
    """The state machine. prepare() then run() (blocking, on the server's task thread);
    report_target(), set_launch_pose(), abort(), halt() and status() from any thread."""

    def __init__(self, navigator: Navigator) -> None:
        self.navigator = navigator
        self.drone = navigator.drone
        self.history = PoseHistory()
        self.drone.on_telemetry(self.history.add)
        self.patrol: Patrol | None = None  # the running waypoint patrol, for the page's map
        self._lock = threading.Lock()
        self._config: MissionConfig | None = None
        self._state = IDLE
        self._reset()

    def _reset(self) -> None:
        now = time.time()
        self._step = ""
        self._message: str | None = None
        self._stopped_in: str | None = None
        self._started_at = now
        self._since = now
        self._states: list[dict[str, Any]] = []
        self._reason: str | None = None
        self._latest: Detection | None = None
        self._used_frame_time = 0.0
        self._detections = 0
        self._attempts = 0
        self._ignore_until = 0.0
        self._alignment: Alignment | None = None
        self._launch_pose: LaunchPose | None = None
        self._child: dict[str, Any] | None = None
        self._children = 0
        self._patrol_deadline: float | None = None

    # --- control from other threads -----------------------------------------------------------

    def prepare(self, config: MissionConfig) -> None:
        """Set up a new mission (run() flies it). Raises ValueError for a bad config."""
        _check_config(config, self.navigator)
        with self._lock:
            if self._state not in END_STATES:
                raise ValueError(f"a mission is running ({self._state})")
            self._reset()
            self._config = config
            self._state, self._step = IDLE, "starting"

    def running(self) -> bool:
        with self._lock:
            return self._state not in END_STATES or self._step == "starting"

    def report_target(self, detection: Detection) -> tuple[bool, str]:
        """A detection; (accepted, why). During the patrol a fresh one starts the approach."""
        with self._lock:
            config, state = self._config, self._state
            if config is None or state not in SEARCH_STATES:
                return False, f"not searching (mission {state})"
            if detection.confidence < config.min_confidence:
                return False, f"confidence under {config.min_confidence}"
            self._latest = detection
            self._detections += 1
            fresh = detection.frame_time >= time.time() - MAX_DETECTION_AGE_S
            trigger = state == PATROLLING and fresh and time.time() >= self._ignore_until
        if trigger:
            self._interrupt(REASON_TARGET)
        return True, state

    def set_launch_pose(self, pose: LaunchPose) -> None:
        with self._lock:
            self._launch_pose = pose

    def abort(self) -> bool:
        """Stop the current step, fly back to the takeoff point and land. False if idle."""
        if not self.running():
            return False
        self._interrupt(REASON_ABORT)
        return True

    def halt(self) -> None:
        """STOP from the server (before it stops the drone): the mission ends, hovering."""
        if self.running():
            with self._lock:
                self._reason = REASON_STOP

    def accepts_moves(self) -> bool:
        """True while an outside client flies the patrol with /move (strategy external)."""
        with self._lock:
            config = self._config
            return (
                self._state == PATROLLING
                and config is not None
                and config.patrol is None
                and config.strategy == STRATEGY_EXTERNAL
            )

    def status(self) -> dict[str, Any]:
        """GET /mission, and "mission" in /status and /events."""
        with self._lock:
            config, alignment, launch = self._config, self._alignment, self._launch_pose
            latest, child = self._latest, self._child
            status = {
                "state": self._state,
                "step": self._step,
                "message": self._message,
                "stopped_in": self._stopped_in,
                "started_at": self._started_at,
                "since": self._since,
                "states": list(self._states),
                "detections": self._detections,
                "attempts": self._attempts,
            }
        pose = self.drone.pose()
        status.update(
            {
                "config": config.as_dict() if config else None,
                "last_detection": latest.as_dict() if latest else None,
                "alignment": asdict(alignment) if alignment else None,
                "launch_pose": asdict(launch) if launch else None,
                "map_from_drone": _map_transform(alignment, launch, config),
                "pose_map": _pose_in_map(pose, alignment, launch, config),
                "child": _child_report(child, alignment, launch, config),
                "patrol": self.patrol.progress() if self.patrol else None,
                "safety": self.navigator.blocking_reason(),
                "accepts_moves": self.accepts_moves(),
                "updated_at": time.time(),
            }
        )
        return status

    # --- the flight -----------------------------------------------------------------------------

    def run(self) -> str:
        """Fly the prepared mission; returns how it ended. Raises Interrupted after a STOP,
        SafetyLanding after a safety landing, RuntimeError after a failure."""
        try:
            try:
                return self._finish(LANDED, self._fly())
            except Interrupted:
                if self._take_reason() != REASON_ABORT:
                    raise
                return self._finish(ABORTED, self._go_home("mission aborted"))
            except ReturnHome as why:
                self._set(RETURNING, f"safety rule: {why}")
                return self.navigator.return_home_and_land(str(why))  # raises SafetyLanding
            except (NotReady, TimeoutError, ValueError, RuntimeError) as error:
                ending = self._go_home(f"failed: {error}")
                self._finish(FAILED, ending)
                raise RuntimeError(ending) from error
        except Interrupted:
            with self._lock:
                self._stopped_in = self._state
            self._finish(STOPPED, f"STOP during {self._stopped_in}: hovering")
            raise
        except SafetyLanding as why:
            self._finish(LANDED, f"safety landing: {why}")
            raise
        except (NotReady, TimeoutError, ValueError, RuntimeError) as error:
            if self._current_state() not in END_STATES:  # it failed on the way home
                self._finish(FAILED, str(error))
            raise
        finally:
            self.patrol = None

    def _fly(self) -> str:
        config = self._require_config()
        self._launch(config)
        self._set(CLIMBING, f"to {config.patrol_alt_m:.1f} m")
        self.navigator.climb_to(config.patrol_alt_m)
        self._set_tilt(config.patrol_tilt_deg)  # anchors the estimate: detections need it
        self._align(config)
        while True:
            ended = self._search(config)
            if ended:
                return self._go_home(f"child not found: {ended}")
            fix = self._approach(config)
            if fix is not None:
                break
        self._report(fix)
        self._orbit(config, fix.child)
        return self._go_home("child reported")

    def _launch(self, config: MissionConfig) -> None:
        """Wait for the button takeoff (the X1 has no takeoff request)."""
        telemetry = self.drone.telemetry()
        if telemetry.in_flight:
            return
        self._set(LAUNCHING, "press the drone's button")
        if not telemetry.manual_session:
            result = self.drone.enter_manual()
            if not result.get("ok"):
                raise RuntimeError(f"manual control failed: {result.get('error')}")
        epoch = self.drone.epoch
        deadline = time.monotonic() + config.takeoff_timeout_s
        while not (self.drone.telemetry().flight_state or "").startswith(FLYING_PREFIX):
            self.drone.check(epoch)
            if time.monotonic() > deadline:
                raise TimeoutError(f"no takeoff within {config.takeoff_timeout_s:.0f} s")
            time.sleep(WAIT_POLL_S)
        self._odometry_xy(epoch)  # the first sample of the new frame

    def _align(self, config: MissionConfig) -> None:
        """The calibration leg: straight ahead, camera forward, no yaw."""
        if config.align_m <= 0:
            return
        self._set(ALIGNING, f"{config.align_m:.1f} m straight ahead")
        self.navigator.require_ready()
        epoch = self.drone.epoch
        self.drone.neutral()
        self.drone.sleep(SETTLE_S, epoch)
        start = self._odometry_xy(epoch)
        deadline = time.monotonic() + ALIGN_TIMEOUT_S
        try:
            while time.monotonic() < deadline:
                self.navigator.guard(epoch)
                pose = self.drone.pose()
                if math.hypot(pose.x - start[0], pose.y - start[1]) >= config.align_m:
                    break
                self.navigator.send(epoch, fwd=ALIGN_FRACTION)
                self.drone.sleep(TICK_S, epoch)
        finally:
            self.drone.neutral()
        self.drone.sleep(SETTLE_S, epoch)
        end = self._odometry_xy(epoch)
        dx, dy = end[0] - start[0], end[1] - start[1]
        leg = math.hypot(dx, dy)
        if leg < ALIGN_MIN_M:
            self._note(f"aligning leg only {leg:.2f} m: no map frame")
            return
        alignment = Alignment(round(math.degrees(math.atan2(dy, dx)), 2), round(leg, 3))
        with self._lock:
            self._alignment = alignment

    def _search(self, config: MissionConfig) -> str | None:
        """Patrol until a detection (None) or the end of the patrol (why it ended)."""
        self._set(PATROLLING, "starting")
        if self._patrol_deadline is None:
            self._patrol_deadline = time.monotonic() + config.patrol_timeout_s
        remaining = self._patrol_deadline - time.monotonic()
        if remaining <= 0:
            return f"patrol timeout ({config.patrol_timeout_s:.0f} s)"
        timer = threading.Timer(remaining, self._interrupt, (REASON_TIMEOUT,))
        timer.daemon = True
        timer.start()
        try:
            if self._fresh_trigger():
                return None
            self._set_tilt(config.patrol_tilt_deg)
            strategy = self._strategy(config, remaining)
            self._step_note("flying")
            outcome = strategy(self.navigator)
            return f"patrol ended: {outcome}"
        except Interrupted:
            reason = self._pending_reason()
            if reason == REASON_TARGET:
                self._take_reason()
                return None
            if reason == REASON_TIMEOUT:
                self._take_reason()
                return f"patrol timeout ({config.patrol_timeout_s:.0f} s)"
            raise
        finally:
            timer.cancel()
            self.patrol = None

    def _strategy(self, config: MissionConfig, remaining_s: float) -> PatrolStrategy:
        if config.patrol is not None:
            return config.patrol
        if config.strategy == STRATEGY_EXTERNAL:
            self._step_note("waiting for /move from the external patrol")
            return lambda navigator: navigator.hover(remaining_s)
        waypoints = [self._to_drone_frame(config, waypoint) for waypoint in config.waypoints]
        pose = self.drone.pose()
        nearest = min(
            range(len(waypoints)),
            key=lambda i: math.hypot(waypoints[i].x - pose.x, waypoints[i].y - pose.y),
        )
        options = PatrolOptions(
            loops=config.loops,
            dwell_s=config.dwell_s,
            scan=config.scan,
            return_home=False,
            land=False,
        )
        with_alt = [
            w if w.alt is not None else replace(w, alt=config.patrol_alt_m) for w in waypoints
        ]
        self.patrol = Patrol(self.navigator, with_alt[nearest:] + with_alt[:nearest], options)
        return lambda navigator: self.patrol.run()

    def _approach(self, config: MissionConfig) -> ChildFix | None:
        """Fly over the detection, camera down, centre the child. The fix (the child and the
        drone over it), or None when lost or out of time (the patrol resumes)."""
        with self._lock:
            self._attempts += 1
            trigger = self._latest
        self._set(APPROACHING, "locating")
        deadline = time.monotonic() + config.approach_timeout_s
        try:
            if config.approach == APPROACH_TRACK:
                return self._track(config, trigger, deadline)
            if trigger is not None:
                self._fly_over(config, trigger, deadline)
            self._set_tilt(config.approach_tilt_deg)
            return self._centre(config, deadline)
        except (TimeoutError, ValueError) as error:
            self._note(f"approach {self._attempts}: {error}; patrolling again")
            with self._lock:
                self._ignore_until = time.time() + RETRY_COOLDOWN_S
            return None

    def _track(self, config: MissionConfig, trigger: Detection | None, deadline: float) -> ChildFix:
        settings = TrackSettings(
            speed=config.approach_speed,
            launch_height_m=config.launch_height_m,
            target_height_m=config.target_height_m,
            lost_timeout_s=config.lost_timeout_s,
        )
        approach = TrackApproach(
            self.navigator, self.history.at, self._fresh_detection, settings, self._step_note
        )
        return approach.run(deadline, trigger)

    def _fly_over(self, config: MissionConfig, detection: Detection, deadline: float) -> None:
        located = self._locate(config, detection)
        if located is None:
            return
        (x, y), _ = located
        pose = self.drone.pose()
        if math.hypot(x - pose.x, y - pose.y) <= COARSE_M:
            return
        x, y = self._inside_fence(x, y)
        self._step_note(f"flying over ({x:.1f}, {y:.1f})")
        self.navigator.goto(
            x, y, None, COARSE_TOLERANCE_M, _left(deadline), max_fraction=config.approach_speed
        )

    def _centre(self, config: MissionConfig, deadline: float) -> ChildFix:
        """Visual servo, stop-and-correct: use only frames taken after the drone settled."""
        settled = time.time()
        last_seen = time.time()
        centred: list[OverheadSample] = []
        while True:
            if time.monotonic() > deadline:
                raise TimeoutError(f"not centred within {config.approach_timeout_s:.0f} s")
            detection = self._fresh_detection(settled)
            located = self._locate(config, detection) if detection else None
            if located is None:
                if time.time() - last_seen > config.lost_timeout_s:
                    raise TimeoutError(f"child not seen for {config.lost_timeout_s:.0f} s")
                self._step_note(f"holding: no detection for {time.time() - last_seen:.0f} s")
                self.navigator.hover(TICK_S)
                continue
            last_seen = time.time()
            child, (fwd, right) = located
            error = math.hypot(fwd, right)
            self._step_note(f"centring: {error:.2f} m off")
            if error <= CENTRED_M:
                pose, tilt = self.history.at(detection.frame_time)
                error_px = centre_error_px(detection)
                centred.append(OverheadSample(child, pose, tilt, error_px, detection.frame_time))
                if len(centred) >= CENTRED_COUNT:
                    return child_fix(centred)
                continue
            centred.clear()
            self._pulse(fwd, right, min(APPROACH_FRACTION, config.approach_speed))
            self.navigator.hover(SETTLE_S)
            settled = time.time()

    def _report(self, fix: ChildFix) -> None:
        photo = None
        with contextlib.suppress(RuntimeError):  # no frame: the position is what matters
            photo = str(self.drone.take_photo())
        with self._lock:
            self._children += 1
            self._child = {
                "id": self._children,
                "time": time.time(),
                "drone": {"x": round(fix.child[0], 3), "y": round(fix.child[1], 3)},
                "fix": fix.as_dict(),
                "photo": photo,
            }
        self._set(REPORTING, "hovering over the child")
        self.navigator.hover(REPORT_HOLD_S)

    def _orbit(self, config: MissionConfig, child: tuple[float, float]) -> None:
        if config.orbit_turns <= 0:
            return
        alt = self.drone.pose().alt if config.orbit_alt_m is None else config.orbit_alt_m
        self._set(ORBITING, f"at {alt:.1f} m")
        if config.orbit_alt_m is not None:
            self.navigator.climb_to(alt)
        pose = self.drone.pose()
        away = math.atan2(pose.y - child[1], pose.x - child[0])
        if math.hypot(pose.x - child[0], pose.y - child[1]) < MIN_RANGE_M:
            away = math.radians(pose.heading_deg) + math.pi  # straight over it: back off
        radius = config.orbit_radius_m
        start = (child[0] + radius * math.cos(away), child[1] + radius * math.sin(away))
        try:
            self.navigator.limits.check_target(*self._circle_extent(child, radius), alt)
        except ValueError as error:
            self._note(f"no orbit: {error}")
            return
        self._step_note("to the circle")
        self.navigator.goto(*start, alt)
        self.navigator.turn_to(_bearing_deg(start, child))
        height = max(alt + config.launch_height_m - config.target_height_m, 0.0)
        tilt = -math.degrees(math.atan2(height, radius)) if height else DEFAULT_ORBIT_TILT_DEG
        self._set_tilt(tilt)
        self._circle(config, child, away, alt)

    def _circle(
        self, config: MissionConfig, child: tuple[float, float], start_angle: float, alt: float
    ) -> None:
        """Fly the circle clockwise (seen from above), facing the centre: feed-forward
        sideways speed and yaw rate (the calibrated yaw_rate_dps), plus position and heading
        correction on the estimate."""
        radius = config.orbit_radius_m
        cap = min(self.drone.speed_cap, self.navigator.limits.max_speed)
        yaw_rate = getattr(self.drone.estimator, "yaw_rate_dps", DEFAULT_YAW_RATE_DPS)
        fastest_dps = yaw_rate * cap / 100
        rate_dps = min(math.degrees(ORBIT_MAX_SPEED_MPS / radius), ORBIT_YAW_FRACTION * fastest_dps)
        if rate_dps <= 0:
            raise ValueError("orbit: no yaw rate at this speed cap")
        duration = 360.0 * config.orbit_turns / rate_dps
        mps_per_fraction = ORBIT_MPS_PER_STICK_PCT * cap
        self.navigator.require_ready()
        epoch = self.drone.epoch
        began = time.monotonic()
        try:
            while (elapsed := time.monotonic() - began) < duration:
                self.navigator.guard(epoch)
                angle = start_angle + math.radians(rate_dps) * elapsed
                pose = self.drone.pose()
                speed = radius * math.radians(rate_dps)
                vx = -speed * math.sin(angle) + ORBIT_POSITION_GAIN * (
                    child[0] + radius * math.cos(angle) - pose.x
                )
                vy = speed * math.cos(angle) + ORBIT_POSITION_GAIN * (
                    child[1] + radius * math.sin(angle) - pose.y
                )
                fwd, right = frame_to_body(vx, vy, pose.heading_deg)
                facing = wrap_deg(_bearing_deg((pose.x, pose.y), child) - pose.heading_deg)
                self.navigator.send(
                    epoch,
                    fwd=_clip(fwd / mps_per_fraction, CRUISE_MAX),
                    right=_clip(right / mps_per_fraction, CRUISE_MAX),
                    up=_clip(ALT_GAIN * (alt - pose.alt), MAX_VERTICAL),
                    turn=_clip(rate_dps / fastest_dps + YAW_GAIN * facing, 1.0),
                )
                self._step_note(
                    f"turn {elapsed / duration * config.orbit_turns:.1f} of {config.orbit_turns:g}"
                )
                self.drone.sleep(TICK_S, epoch)
        finally:
            self.drone.neutral()

    def _go_home(self, why: str) -> str:
        """Back to the takeoff point and land; returns how it ended."""
        with self._lock:
            self._reason = None
        if not self.drone.telemetry().in_flight:
            return f"{why} (on the ground)"
        config = self._require_config()
        self._set(RETURNING, why)
        alt = max(self.drone.pose().alt, config.orbit_alt_m or 0.0)
        alt = self.navigator.limits.clamp_alt(alt)
        try:
            self.navigator.goto(*HOME, alt)
        except TimeoutError:
            self._note("takeoff point not reached in time: landing here")
        self._step_note("landing")
        result = self.drone.land()
        if not result["ok"]:
            raise RuntimeError(f"land failed: {result['error']}")
        epoch = self.drone.epoch
        deadline = time.monotonic() + LAND_TIMEOUT_S
        while self.drone.telemetry().in_flight and time.monotonic() < deadline:
            self.drone.sleep(WAIT_POLL_S, epoch)
        return f"{why}: landed at the takeoff point"

    # --- helpers --------------------------------------------------------------------------------

    def _locate(
        self, config: MissionConfig, detection: Detection
    ) -> tuple[tuple[float, float], tuple[float, float]] | None:
        """(child in the drone frame, body-frame error between the child and the floor point
        under the image centre), from the pose and tilt at the detection's frame time."""
        sample = self.history.at(detection.frame_time)
        if sample is None or sample[1] is None:
            return None
        pose, tilt = sample
        u, v, on_floor = detection.floor_point(tilt)
        above_floor = 0.0 if on_floor else config.target_height_m
        height = max(pose.alt + config.launch_height_m - above_floor, MIN_RANGE_M)
        target = ground_offset(u, v, tilt, height)
        if target is None:
            return None
        centre = ground_offset(0.5, 0.5, tilt, height) or (0.0, 0.0)
        dx, dy = body_to_frame(*target, pose.heading_deg)
        error = (target[0] - centre[0], target[1] - centre[1])
        return (pose.x + dx, pose.y + dy), error

    def _pulse(self, fwd_m: float, right_m: float, fraction: float) -> None:
        """One short body-frame push (independent of the heading estimate), then neutral."""
        distance = math.hypot(fwd_m, right_m)
        seconds = min(MAX_PULSE_S, max(MIN_PULSE_S, distance * PULSE_S_PER_M))
        self.navigator.require_ready()
        epoch = self.drone.epoch
        deadline = time.monotonic() + seconds
        try:
            while time.monotonic() < deadline:
                self.navigator.guard(epoch)
                self.navigator.send(
                    epoch,
                    fwd=fraction * fwd_m / distance,
                    right=fraction * right_m / distance,
                )
                self.drone.sleep(TICK_S, epoch)
        finally:
            self.drone.neutral()

    def _set_tilt(self, deg: float | None) -> None:
        if deg is None:
            return
        tilt = self.drone.camera_tilt()
        if tilt.deg is not None and abs(tilt.deg - deg) <= TILT_OK_DEG:
            return
        self._step_note(f"camera to {deg:.0f} deg")
        self.drone.set_tilt(deg)

    def _to_drone_frame(self, config: MissionConfig, waypoint: Waypoint) -> Waypoint:
        if config.frame == FRAME_DRONE:
            return waypoint
        with self._lock:
            alignment = self._alignment
        if alignment is None:
            raise ValueError(f'waypoints in frame "{config.frame}" need the aligning leg')
        fwd, left = waypoint.x, waypoint.y
        if config.frame == FRAME_MAP:
            fwd, left = self._wait_launch_pose().launch_from_map(waypoint.x, waypoint.y)
        x, y = alignment.drone_from_launch(fwd, left)
        return replace(waypoint, x=round(x, 3), y=round(y, 3))

    def _wait_launch_pose(self) -> LaunchPose:
        self._step_note("waiting for the robot's launch pose (POST /mission/launch_pose)")
        epoch = self.drone.epoch
        deadline = time.monotonic() + LAUNCH_POSE_WAIT_S
        while time.monotonic() < deadline:
            with self._lock:
                pose = self._launch_pose
            if pose is not None:
                return pose
            self.navigator.guard(epoch)
            self.drone.sleep(TICK_S, epoch)
        raise ValueError("map-frame waypoints: no launch pose from the robot")

    def _inside_fence(self, x: float, y: float) -> tuple[float, float]:
        limits = self.navigator.limits
        return (
            min(max(x, limits.x_min + FENCE_INSET_M), limits.x_max - FENCE_INSET_M),
            min(max(y, limits.y_min + FENCE_INSET_M), limits.y_max - FENCE_INSET_M),
        )

    @staticmethod
    def _circle_extent(child: tuple[float, float], radius: float) -> tuple[float, float]:
        """The circle's corner farthest from the takeoff point (checked against the fence)."""
        return (
            child[0] + math.copysign(radius, child[0]),
            child[1] + math.copysign(radius, child[1]),
        )

    def _odometry_xy(self, epoch: int) -> tuple[float, float]:
        telemetry = self.drone.wait_odometry(time.time(), ODOMETRY_WAIT_S, epoch)
        if telemetry is None or telemetry.position is None:
            raise TimeoutError("no odometry")
        return telemetry.position[0], telemetry.position[1]

    def _fresh_detection(self, after: float) -> Detection | None:
        """The latest detection from a frame taken after `after`, once."""
        with self._lock:
            detection = self._latest
            if detection is None or detection.frame_time <= max(after, self._used_frame_time):
                return None
            self._used_frame_time = detection.frame_time
            return detection

    def _fresh_trigger(self) -> bool:
        """A detection that came while climbing or aligning starts the approach at once."""
        with self._lock:
            detection = self._latest
            return (
                detection is not None
                and detection.frame_time >= time.time() - MAX_DETECTION_AGE_S
                and time.time() >= self._ignore_until
            )

    def _interrupt(self, reason: str) -> None:
        with self._lock:
            current = self._reason
            if current is not None and REASON_PRIORITY[current] >= REASON_PRIORITY[reason]:
                return
            self._reason = reason
        self.drone.stop()

    def _current_state(self) -> str:
        with self._lock:
            return self._state

    def _pending_reason(self) -> str | None:
        with self._lock:
            return self._reason

    def _take_reason(self) -> str | None:
        with self._lock:
            reason, self._reason = self._reason, None
            return reason

    def _require_config(self) -> MissionConfig:
        with self._lock:
            if self._config is None:
                raise RuntimeError("prepare() the mission first")
            return self._config

    def _set(self, state: str, step: str = "") -> None:
        with self._lock:
            now = time.time()
            if state != self._state:
                self._states = [*self._states, {"state": state, "at": now}][-HISTORY_STATES:]
                self._since = now
            self._state, self._step = state, step

    def _step_note(self, step: str) -> None:
        with self._lock:
            self._step = step

    def _note(self, message: str) -> None:
        print(f"mission: {message}", flush=True)
        with self._lock:
            self._message = message

    def _finish(self, state: str, message: str) -> str:
        self._set(state, "")
        self._note(message)
        return message


def _check_config(config: MissionConfig, navigator: Navigator) -> None:
    """What can be checked before takeoff; raises ValueError."""
    limits = navigator.limits
    if config.frame not in FRAMES:
        raise ValueError(f'"frame" must be one of {", ".join(FRAMES)}')
    if config.strategy not in STRATEGIES:
        raise ValueError(f'"strategy" must be one of {", ".join(STRATEGIES)}')
    if config.approach not in APPROACHES:
        raise ValueError(f'"approach" must be one of {", ".join(APPROACHES)}')
    uses_waypoints = config.patrol is None and config.strategy == STRATEGY_WAYPOINTS
    if uses_waypoints and not config.waypoints:
        raise ValueError('strategy "waypoints" needs at least one waypoint')
    if config.frame != FRAME_DRONE and config.align_m <= 0 and uses_waypoints:
        raise ValueError(f'waypoints in frame "{config.frame}" need align_m > 0')
    for alt in (config.patrol_alt_m, config.orbit_alt_m):
        if alt is not None:
            limits.check_target(0.0, 0.0, alt)
    if uses_waypoints and config.frame == FRAME_DRONE:
        for waypoint in config.waypoints:
            alt = config.patrol_alt_m if waypoint.alt is None else waypoint.alt
            limits.check_target(waypoint.x, waypoint.y, alt)


def _map_transform(
    alignment: Alignment | None, launch: LaunchPose | None, config: MissionConfig | None
) -> dict[str, float] | None:
    """The drone frame in `map`, as the ROS frame "drone_takeoff" (x, -y of the drone frame):
    translation and yaw (radians)."""
    if alignment is None or launch is None or config is None:
        return None
    return {
        "x": launch.x,
        "y": launch.y,
        "z": config.launch_height_m,
        "yaw": wrap_rad(launch.yaw + math.radians(alignment.forward_deg)),
    }


def _pose_in_map(
    pose: Pose, alignment: Alignment | None, launch: LaunchPose | None, config: MissionConfig | None
) -> dict[str, float] | None:
    if alignment is None or launch is None or config is None or not pose.in_flight:
        return None
    return _drone_in_map(pose.x, pose.y, pose.alt, pose.heading_deg, alignment, launch, config)


def _drone_in_map(
    x: float,
    y: float,
    alt: float,
    heading_deg: float,
    alignment: Alignment,
    launch: LaunchPose,
    config: MissionConfig,
) -> dict[str, float]:
    """A drone-frame pose in `map`: z above the floor, yaw in radians (ROS)."""
    map_x, map_y = launch.map_from_launch(*alignment.launch_from_drone(x, y))
    yaw = launch.yaw + math.radians(alignment.forward_deg - heading_deg)
    return {
        "x": round(map_x, 3),
        "y": round(map_y, 3),
        "z": round(alt + config.launch_height_m, 3),
        "yaw": round(wrap_rad(yaw), 4),
    }


def _child_report(
    child: dict[str, Any] | None,
    alignment: Alignment | None,
    launch: LaunchPose | None,
    config: MissionConfig | None,
) -> dict[str, Any] | None:
    """The child in every frame we can: drone always, launch once aligned, map (and the
    robot's goal: `standoff_m` short of the child, facing it) once the launch pose is known.
    Same for the drone's pose at the fix (`fix`: drone frame, `fix.launch`, `fix.map`)."""
    if child is None:
        return None
    fix = child.get("fix")
    report = {**child, "launch": None, "map": None, "goal": None}
    if fix:
        report["fix"] = {**fix, "launch": None, "map": None}
    if alignment is None or config is None:
        return report
    fwd, left = alignment.launch_from_drone(child["drone"]["x"], child["drone"]["y"])
    report["launch"] = {"x": round(fwd, 3), "y": round(left, 3)}
    if fix:
        drone = fix["drone"]
        fix_fwd, fix_left = alignment.launch_from_drone(drone["x"], drone["y"])
        report["fix"]["launch"] = {"x": round(fix_fwd, 3), "y": round(fix_left, 3)}
    if launch is None:
        return report
    x, y = launch.map_from_launch(fwd, left)
    report["map"] = {"x": round(x, 3), "y": round(y, 3)}
    if fix:
        drone = fix["drone"]
        report["fix"]["map"] = _drone_in_map(
            drone["x"], drone["y"], drone["alt"], drone["heading_deg"], alignment, launch, config
        )
    goal_x, goal_y, goal_yaw = standoff_goal(launch.x, launch.y, x, y, config.standoff_m)
    report["goal"] = {"x": round(goal_x, 3), "y": round(goal_y, 3), "yaw": round(goal_yaw, 4)}
    return report


def _bearing_deg(origin: tuple[float, float], target: tuple[float, float]) -> float:
    """Heading (drone-frame degrees) that faces `target` from `origin`."""
    return math.degrees(math.atan2(target[1] - origin[1], target[0] - origin[0]))


def _left(deadline: float) -> float:
    left = deadline - time.monotonic()
    if left <= 0:
        raise TimeoutError("out of time")
    return left


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
