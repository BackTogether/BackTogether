"""A simulated HoverAir X1 behind the same interface as the real one.

    drone = connect_sim()          # a Drone, like Drone.connect(), but nothing leaves the laptop
    drone.enter_manual()
    drone.link.press_button()      # the drone's takeoff button
    drone.wait_for_takeoff()

What it models, on purpose close to what we measured on the X1:
- `SimLink`: DroneLink's snapshot() shape and commands. States idle -> pre_takeoff ->
  taking_off (its own climb, drifting backward) -> flying_manual -> landing -> idle.
- Kinematics at 50 Hz from the stick packets actually sent (ramp, gates and cap included):
  velocity lag, a true yaw rate 10 % off the estimator's default.
- Odometry at a fixed 1 Hz with noise and a random-walk drift, in a frame that resets at
  takeoff and is rotated from the drone's heading by `frame_yaw_offset_deg`. On the ground it
  reads an old frame, like the real drone.
- `SimCamera`: an ffmpeg test pattern encoded to H.264 and fed to the real decoder; or recorded
  drone frames (`video_dir`: frame_*.jpg, looped at 30 fps), so drone/identify/feed.py can run
  against the simulator and draw on the page.
- A square room around the start point (`room_half_m`, a rectangle with `room_half_y_m`; by
  default larger than the default geofence, so only patrols that look for walls meet them).
  The drone stops at a wall (and `wall_hits` counts it); truth()["front_m"] is the true
  distance to the wall ahead, for testing camera-based patrols without a depth model
  (drone.patrol.bounce --fake-depth).
- Faults for testing: set_battery(), cut("link" | "telemetry" | "video", seconds), push().
- `SimDetector`: a fake lost child standing in the room and a fake detector that reports the
  child's body box (like drone/identify/feed.py: a standing person, anchor "feet") whenever the
  child is in the simulated camera's view (true pose, the camera tilt estimate, the intrinsics
  of camera.py), with latency, misses and noise, plus a face box while the camera is not too
  steep to see it. The mission (mission.py) gets the body; the page's overlay both (/vision);
  the server places the child (/sim/child).
"""

import math
import os
import random
import shutil
import subprocess
import threading
import time
from collections.abc import Callable
from typing import Any

from drone.hoverctl.decoder import MjpegDecoder
from drone.hoverctl.pbwire import IN_FLIGHT_STATES, VIDEO_QUALITY_SD, FlightMode
from drone.hoverctl.protocol import AXIS_LIMIT, NEUTRAL, encode_rocker_command
from drone.hoverctl.sticks import StickSender

from .drone import Drone
from .estimate import DEFAULT_YAW_RATE_DPS, Estimator, wrap_deg
from .mission_geometry import project_box, project_person

PHYSICS_PERIOD_S = 0.02
MAX_SPEED_MPS = 2.0  # horizontal, at stick 100
MAX_CLIMB_MPS = 1.0
TRUE_YAW_RATE_DPS = DEFAULT_YAW_RATE_DPS * 1.1  # calibrate_yaw() should find this
VELOCITY_TAU_S = 0.4
ODOMETRY_PERIOD_S = 1.0
ODOMETRY_NOISE_M = 0.03
VELOCITY_NOISE_MPS = 0.03
DRIFT_STEP_M = 0.01  # random walk of the odometry bias, per sample
DEFAULT_FRAME_YAW_OFFSET_DEG = 20.0
OLD_FRAME_POSITION = (3.2, -1.5, 0.05)  # what the odometry reads on the ground
PRE_TAKEOFF_S = 1.0
TAKING_OFF_S = 1.5
TAKEOFF_ALT_M = 1.0
TAKEOFF_DRIFT_MPS = 0.15  # backward, like the X1's own climb
LANDING_MPS = 0.6
BATTERY_START_PCT = 90.0
BATTERY_DRAIN_PCT_S = 0.05
CRITICAL_BATTERY_PCT = 8.0  # the drone lands by itself
SIM_RSSI = -55
SIM_LINK_AGE_S = 0.2
MOTOR_RPM_FLYING = 9000
FAULTS = ("link", "telemetry", "video")
DEFAULT_ROOM_HALF_M = 10.0  # walls this far from the start point, in x and y
WALL_CONTACT_M = 0.15  # the cage touches the wall this far from the centre
FRONT_CORRIDOR_HALF_WIDTH_M = 0.5  # like drone.patrol.front_distance's obstacle corridor
VIDEO_SOURCE = "testsrc2=size=640x360:rate=30"
REPLAY_FPS = 30  # recorded frames (video_dir) are played at this rate
REPLAY_PATTERN = "frame_*.jpg"
VIDEO_READ_BYTES = 1 << 16
# The fake child, in the robot's frame at launch (x forward, y left; the drone starts facing +x)
SIM_CHILD_FWD_M = 3.5
SIM_CHILD_LEFT_M = 1.5
SIM_CHILD_TALL_M = 1.2  # a standing child
SIM_CHILD_WIDTH_M = 0.4  # shoulders
SIM_FACE_M = 0.18  # face box width and height
SIM_FACE_BELOW_TOP_M = 0.12  # face centre below the top of the head
FACE_HIDDEN_TILT_DEG = -60.0  # camera steeper than this: the top of the head, no face
DETECT_PERIOD_S = 0.3  # with the latency: ~2 detections a second, like a CPU detector
DETECT_LATENCY_S = 0.25
DETECT_MISS = 0.2  # fraction of frames where the child is not detected
DETECT_NOISE = 0.01  # bbox centre noise, fraction of the image
DETECT_CONFIDENCE = 0.85
DETECT_LABEL = "sim child"
OK = {"ok": True, "error": None}
NOT_CONNECTED = {"ok": False, "error": "not connected"}


class SimLink:
    """The simulated drone: DroneLink's interface plus fault injection."""

    def __init__(
        self,
        frame_yaw_offset_deg: float = DEFAULT_FRAME_YAW_OFFSET_DEG,
        seed: int | None = None,
        room_half_m: float = DEFAULT_ROOM_HALF_M,
        room_half_y_m: float | None = None,
    ) -> None:
        """The room is +-room_half_m in x (the start heading) and +-room_half_y_m in y
        (default: square)."""
        self.frame_yaw_offset_deg = frame_yaw_offset_deg
        half_y = room_half_m if room_half_y_m is None else room_half_y_m
        self.room = (-room_half_m, room_half_m, -half_y, half_y)  # world x, y
        self.sticks: StickSender | None = None  # set by connect_sim
        self.photos = 0  # drone-side photos taken (take_photo)
        self.recording = False  # drone-side recording (record_video)
        self._random = random.Random(seed)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="sim-physics", daemon=True)
        now = time.time()
        self._state = "idle"
        self._state_since = time.monotonic()
        self._mode = FlightMode.HOVER
        self._session = False
        self._video_quality = VIDEO_QUALITY_SD
        self._battery = BATTERY_START_PCT
        self._x = self._y = self._alt = 0.0  # true position, world frame, alt up
        self._heading = 0.0  # true heading, world degrees
        self._vx = self._vy = self._vz = 0.0
        self._origin = (0.0, 0.0)
        self._frame_yaw = 0.0
        self._bias = (0.0, 0.0)
        self._odometry: tuple[tuple[float, ...], tuple[float, ...]] | None = None
        self._odometry_ts: float | None = None
        self._next_odometry = 0.0
        self._seen = now  # last time the link delivered anything
        self._wall_hits = 0
        self._touching_wall = False
        self._cut_until = dict.fromkeys(FAULTS, 0.0)

    # --- DroneLink interface ------------------------------------------------------------------

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join()

    def snapshot(self) -> dict[str, Any]:
        now = time.time()
        with self._lock:
            up = self._link_up(now)
            seen = self._seen
            odometry, odometry_ts = self._odometry, self._odometry_ts
            state, mode, battery, session = self._state, self._mode, self._battery, self._session
            quality = self._video_quality
        position, velocity = odometry if odometry else (None, None)
        flying = state in IN_FLIGHT_STATES
        return {
            "connection": {
                "connected": up,
                "rssi": SIM_RSSI,
                "last_message_age_s": SIM_LINK_AGE_S if up else round(now - seen, 2),
                "error": None if up else "simulated link loss",
            },
            "wifi": {"state": "on", "ssid": "simulator", "password": None},
            "app_announced": OK,
            "manual_entry": {
                "session": session,
                "steps": ["connect", "wifi", "preview", "manual", "video"] if session else [],
                "error": None,
            },
            "video_quality": {
                "wanted": quality,
                "applied": None,
                "drone_default": None,
                "error": None,
            },
            "mode": {"value": int(mode), "name": mode.name, "ts": seen},
            "flight_state": {"value": state, "in_flight": flying, "ts": seen},
            "battery": {
                "soc": round(battery),
                "voltage": None,
                "current": None,
                "temp": None,
                "ts": seen,
                "charging": False,
                "charger_connected": False,
                "fully_charged": False,
                "charger_ts": seen,
            },
            "odometry": {
                "position": _xyz(position),
                "velocity": _xyz(velocity),
                "motors_rpm": [MOTOR_RPM_FLYING if flying else 0] * 4,
                "battery_voltage": None,
                "total_current": None,
                "ts": odometry_ts,
            },
            "storage": {"available_bytes": None, "total_bytes": None, "ts": None},
            "exception": {"errors": None, "event": None, "ts": None},
            "camera_tilt": {"degrees": None, "ts": None},  # the X1 never reports it
        }

    def is_manual(self) -> bool:
        with self._lock:
            return self._link_up(time.time()) and self._mode == FlightMode.MANUAL

    def in_manual_session(self) -> bool:
        with self._lock:
            return self._session

    def enter_manual(self) -> dict[str, Any]:
        with self._lock:
            self._session = True
            self._mode = FlightMode.MANUAL
            if self._state == "flying_traj":
                self._set_state("flying_manual")
        return OK

    def set_manual(self) -> dict[str, Any]:
        return self.enter_manual()

    def land(self) -> dict[str, Any]:
        with self._lock:
            if not self._link_up(time.time()):
                return NOT_CONNECTED
            if self._state in IN_FLIGHT_STATES - {"landing"} or self._state == "pre_takeoff":
                self._set_state("landing")
        return OK

    def start_wifi(self, password: str) -> dict[str, Any]:
        return OK

    def start_video(self) -> dict[str, Any]:
        return OK

    def stop_video(self) -> dict[str, Any]:
        with self._lock:
            self._session = False
        return OK

    def set_video_quality(self, quality: str) -> dict[str, Any]:
        with self._lock:
            self._video_quality = quality
        return OK

    def take_photo(self) -> dict[str, Any]:
        self.photos += 1
        return OK

    def record_video(self, on: bool) -> dict[str, Any]:
        self.recording = on
        return OK

    # --- simulator controls -------------------------------------------------------------------

    def press_button(self) -> dict[str, Any]:
        """The drone's takeoff button."""
        with self._lock:
            if self._state != "idle":
                return {"ok": False, "error": f"button ignored: {self._state}"}
            self._set_state("pre_takeoff")
        return OK

    def set_battery(self, pct: float) -> None:
        with self._lock:
            self._battery = max(0.0, min(100.0, pct))

    def cut(self, what: str, seconds: float) -> None:
        """Lose the Bluetooth link, the odometry or the video for `seconds`."""
        if what not in FAULTS:
            raise ValueError(f"cut one of {', '.join(FAULTS)}")
        with self._lock:
            self._cut_until[what] = time.time() + seconds

    def video_cut(self) -> bool:
        with self._lock:
            return time.time() < self._cut_until["video"]

    def push(self, dx: float, dy: float) -> None:
        """A gust: move the drone by (dx, dy) metres in the takeoff frame."""
        with self._lock:
            yaw = math.radians(self._frame_yaw)
            self._x += dx * math.cos(yaw) - dy * math.sin(yaw)
            self._y += dx * math.sin(yaw) + dy * math.cos(yaw)

    def truth(self) -> dict[str, float]:
        """The true pose in the takeoff frame (what a perfect estimator would say), the true
        distance to the wall ahead (`front_m`, metres) and the number of wall contacts."""
        with self._lock:
            x, y = self._to_frame(self._x - self._origin[0], self._y - self._origin[1])
            heading = wrap_deg(self._heading - self._frame_yaw)
            return {
                "x": x,
                "y": y,
                "alt": self._alt,
                "heading_deg": heading,
                "front_m": self._front_m(),
                "wall_hits": self._wall_hits,
            }

    def frame_point(self, world_x: float, world_y: float) -> tuple[float, float]:
        """A world point (the start point at 0, 0, x = the start heading) in the takeoff frame."""
        with self._lock:
            return self._to_frame(world_x - self._origin[0], world_y - self._origin[1])

    # --- physics ------------------------------------------------------------------------------

    def _run(self) -> None:
        last = time.monotonic()
        while not self._stop.wait(PHYSICS_PERIOD_S):
            now = time.monotonic()
            with self._lock:
                self._step(now - last, now)
            last = now

    def _step(self, dt: float, now: float) -> None:
        sticks = self.sticks.output if self.sticks else NEUTRAL
        elapsed = now - self._state_since
        if self._state == "pre_takeoff" and elapsed >= PRE_TAKEOFF_S:
            self._new_frame()
            self._set_state("taking_off")
        elif self._state == "taking_off":
            self._fly(dt, -TAKEOFF_DRIFT_MPS, 0.0, TAKEOFF_ALT_M / TAKING_OFF_S, 0.0)
            if elapsed >= TAKING_OFF_S:
                manual = self._mode == FlightMode.MANUAL
                self._set_state("flying_manual" if manual else "flying_traj")
        elif self._state == "flying_manual":
            self._fly(
                dt,
                sticks.pitch / AXIS_LIMIT * MAX_SPEED_MPS,
                sticks.roll / AXIS_LIMIT * MAX_SPEED_MPS,
                sticks.thrust / AXIS_LIMIT * MAX_CLIMB_MPS,
                sticks.yaw / AXIS_LIMIT * TRUE_YAW_RATE_DPS,
            )
        elif self._state == "flying_traj":
            self._fly(dt, 0.0, 0.0, 0.0, 0.0)
        elif self._state == "landing":
            self._fly(dt, 0.0, 0.0, -LANDING_MPS, 0.0)
            if self._alt <= 0:
                self._alt = self._vx = self._vy = self._vz = 0.0
                self._set_state("idle")
        if self._state in IN_FLIGHT_STATES:
            self._battery = max(0.0, self._battery - BATTERY_DRAIN_PCT_S * dt)
            if self._battery <= CRITICAL_BATTERY_PCT and self._state != "landing":
                self._set_state("landing")
        self._sample_odometry(now)

    def _fly(self, dt: float, fwd: float, right: float, up: float, yaw_rate: float) -> None:
        heading = math.radians(self._heading)
        target_vx = fwd * math.cos(heading) - right * math.sin(heading)
        target_vy = fwd * math.sin(heading) + right * math.cos(heading)
        blend = min(1.0, dt / VELOCITY_TAU_S)
        self._vx += (target_vx - self._vx) * blend
        self._vy += (target_vy - self._vy) * blend
        self._vz += (up - self._vz) * blend
        self._x += self._vx * dt
        self._y += self._vy * dt
        self._alt = max(0.0, self._alt + self._vz * dt)
        self._heading = wrap_deg(self._heading + yaw_rate * dt)
        self._stop_at_walls()

    def _stop_at_walls(self) -> None:
        x_min, x_max, y_min, y_max = self.room
        x = min(max(self._x, x_min + WALL_CONTACT_M), x_max - WALL_CONTACT_M)
        y = min(max(self._y, y_min + WALL_CONTACT_M), y_max - WALL_CONTACT_M)
        touching = (x, y) != (self._x, self._y)
        if touching and not self._touching_wall:
            self._wall_hits += 1
        self._touching_wall = touching
        if x != self._x:
            self._vx = 0.0
        if y != self._y:
            self._vy = 0.0
        self._x, self._y = x, y

    def _front_m(self) -> float:
        """Distance ahead to the room's wall, within a corridor as wide as the drone's path."""
        heading = math.radians(self._heading)
        dx, dy = math.cos(heading), math.sin(heading)
        offsets = (-FRONT_CORRIDOR_HALF_WIDTH_M, 0.0, FRONT_CORRIDOR_HALF_WIDTH_M)
        return min(self._wall_ahead(self._x - o * dy, self._y + o * dx, dx, dy) for o in offsets)

    def _wall_ahead(self, x: float, y: float, dx: float, dy: float) -> float:
        """Distance from (x, y) along (dx, dy) to the room's wall (0 if outside)."""
        x_min, x_max, y_min, y_max = self.room
        hits = []
        if dx:
            hits.append(((x_max if dx > 0 else x_min) - x) / dx)
        if dy:
            hits.append(((y_max if dy > 0 else y_min) - y) / dy)
        return max(0.0, min(hits))

    def _new_frame(self) -> None:
        """Takeoff: the odometry starts over here, its axes rotated from the heading."""
        self._origin = (self._x, self._y)
        self._frame_yaw = self._heading + self.frame_yaw_offset_deg
        self._bias = (0.0, 0.0)

    def _sample_odometry(self, now: float) -> None:
        if now < self._next_odometry:
            return
        self._next_odometry = now + ODOMETRY_PERIOD_S
        wall = time.time()
        if not self._link_up(wall):
            return
        self._seen = wall
        if wall < self._cut_until["telemetry"]:
            return
        if self._state not in IN_FLIGHT_STATES:
            self._odometry = (OLD_FRAME_POSITION, (0.0, 0.0, 0.0))
            self._odometry_ts = wall
            return
        noise = self._random.gauss
        self._bias = (
            self._bias[0] + noise(0, DRIFT_STEP_M),
            self._bias[1] + noise(0, DRIFT_STEP_M),
        )
        x, y = self._to_frame(self._x - self._origin[0], self._y - self._origin[1])
        vx, vy = self._to_frame(self._vx, self._vy)
        position = (
            x + self._bias[0] + noise(0, ODOMETRY_NOISE_M),
            y + self._bias[1] + noise(0, ODOMETRY_NOISE_M),
            -self._alt + noise(0, ODOMETRY_NOISE_M),
        )
        velocity = tuple(v + noise(0, VELOCITY_NOISE_MPS) for v in (vx, vy, -self._vz))
        self._odometry = (position, velocity)
        self._odometry_ts = wall

    def _to_frame(self, dx: float, dy: float) -> tuple[float, float]:
        yaw = math.radians(self._frame_yaw)
        return dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)

    def _set_state(self, state: str) -> None:
        self._state = state
        self._state_since = time.monotonic()

    def _link_up(self, now: float) -> bool:
        return now >= self._cut_until["link"]


class SimSticks(StickSender):
    """The real stick sender (ramp, cap, watchdog, interlock), without the UDP socket."""

    def __init__(self, link: SimLink, max_magnitude: float) -> None:
        super().__init__(host="sim", max_magnitude=max_magnitude, dry_run=True, gate=link.is_manual)
        self.dry_run = False  # nothing is printed or sent: the packet only feeds the physics
        self.set_session_gate(link.in_manual_session)

    def _send(self, axes: Any) -> None:
        if self.silent:
            return
        seq_num = self._seq_num
        self._seq_num += 1
        now = time.time()
        packet = encode_rocker_command(
            seq_num, int(now * 1000), axes, self.control_source, self.sport
        )
        self._record(seq_num, now, axes, packet, self.sport)


class SimCamera:
    """An ffmpeg test pattern as H.264, fed to the real decoder (VideoReceiver's interface)."""

    def __init__(self, link: SimLink, video_dir: str | None = None) -> None:
        self.video_dir = video_dir
        self.decoder = MjpegDecoder("h264")
        self.port = 0
        self.packets = 0
        self.bytes = 0
        self._link = link
        self._listeners: list[Callable[[bytes], None]] = []
        self._process: subprocess.Popen | None = None
        self._thread = threading.Thread(target=self._run, name="sim-camera", daemon=True)

    def start(self) -> None:
        if not shutil.which("ffmpeg"):
            raise SystemExit("ffmpeg not found: install it (brew / apt / winget install ffmpeg)")
        source = ["-f", "lavfi", "-i", VIDEO_SOURCE]
        if self.video_dir:
            source = [
                "-stream_loop", "-1", "-framerate", str(REPLAY_FPS), "-pattern_type", "glob",
                "-i", os.path.join(self.video_dir, REPLAY_PATTERN),
            ]  # fmt: skip
        command = [
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-re",
            *source,
            "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
            # SPS/PPS before every keyframe, like the drone (recordings start on one)
            "-g", "30", "-x264-params", "repeat-headers=1",
            "-pix_fmt", "yuv420p", "-f", "h264", "pipe:1",
        ]  # fmt: skip
        self._process = subprocess.Popen(command, stdout=subprocess.PIPE)
        self._thread.start()

    def stop(self) -> None:
        if self._process:
            self._process.kill()
            self._process.wait()
        if self._thread.is_alive():
            self._thread.join()
        self.decoder.close()

    def restart_decoder(self) -> None:
        self.decoder.restart()

    def add_stream_listener(self, listener: Callable[[bytes], None]) -> None:
        self._listeners = [*self._listeners, listener]

    def remove_stream_listener(self, listener: Callable[[bytes], None]) -> None:
        self._listeners = [item for item in self._listeners if item != listener]

    def stats(self) -> dict[str, Any]:
        """VideoReceiver.stats() shape, for the web UI."""
        size = self.decoder.size or (None, None)
        return {
            "port": self.port,
            "packets": self.packets,
            "bytes": self.bytes,
            "pps": 0.0,
            "bytes_per_s": 0,
            "senders": ["simulator"],
            "kind": "annexb (simulated)",
            "codec": self.decoder.codec,
            "frames": self.decoder.frames,
            "width": size[0],
            "height": size[1],
            "fps": round(self.decoder.fps, 1),
            "decode_lag_ms": None if self.decoder.lag_ms is None else round(self.decoder.lag_ms),
            "decode_queued": self.decoder.queued,
            "dump": None,
            "previews": [],
        }

    def _run(self) -> None:
        output = self._process.stdout.fileno()
        while chunk := os.read(output, VIDEO_READ_BYTES):
            if self._link.video_cut():
                continue
            self.packets += 1
            self.bytes += len(chunk)
            self.decoder.feed(chunk)
            for listener in self._listeners:
                listener(chunk)


Box = tuple[float, float, float, float]
Report = Callable[[float, Box, Box | None, float, str], Any]


class SimDetector:
    """See the module docstring. `report(frame_time, body, face, confidence, label)` gets each
    detection (boxes = x0, y0, x1, y1 as fractions of the image, y down; face None when the
    camera looks down too steeply to see it)."""

    def __init__(self, drone: Drone, report: Report, seed: int | None = None) -> None:
        self.drone = drone
        self.link: SimLink = drone.link
        self.report = report
        self.fwd = SIM_CHILD_FWD_M
        self.left = SIM_CHILD_LEFT_M
        self.detect = True
        self.miss = DETECT_MISS
        self.reports = 0
        self._random = random.Random(seed)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="sim-detector", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def place(
        self, fwd: float, left: float, detect: bool | None = None, miss: float | None = None
    ) -> None:
        """Put the child `fwd` m ahead of and `left` m left of the launch point."""
        self.fwd, self.left = fwd, left
        if detect is not None:
            self.detect = detect
        if miss is not None:
            self.miss = miss

    def truth(self) -> dict[str, Any]:
        """Where the child is (launch and takeoff frames) and its true bbox in the camera
        (None out of view), e.g. for a detector on the robot side in tests."""
        x, y = self._frame_xy()
        bbox = self.observe()
        return {
            "fwd": self.fwd,
            "left": self.left,
            "x": round(x, 3),
            "y": round(y, 3),
            "detect": self.detect,
            "miss": self.miss,
            "in_view": bbox is not None,
            "bbox": None if bbox is None else [round(v, 4) for v in bbox],
            "reports": self.reports,
        }

    def observe(self) -> Box | None:
        """The child's true body box in the camera now, None when it is out of view."""
        seen = self._body_frame()
        if seen is None:
            return None
        fwd, right, floor = seen
        tilt = self._tilt_deg()
        return project_person(fwd, right, floor, tilt, SIM_CHILD_TALL_M, SIM_CHILD_WIDTH_M)

    def observe_face(self) -> Box | None:
        """The child's face box now; None out of view or seen from too steeply above."""
        seen = self._body_frame()
        tilt = self._tilt_deg()
        if seen is None or tilt < FACE_HIDDEN_TILT_DEG:
            return None
        fwd, right, floor = seen
        down = floor - SIM_CHILD_TALL_M + SIM_FACE_BELOW_TOP_M
        return project_box(fwd, right, down, tilt, SIM_FACE_M)

    def _body_frame(self) -> tuple[float, float, float] | None:
        """(forward, right) of the child's feet from the drone, and the floor below the camera."""
        if not self.drone.telemetry().in_flight or self.link.video_cut():
            return None
        truth = self.link.truth()
        x, y = self._frame_xy()
        heading = math.radians(truth["heading_deg"])
        dx, dy = x - truth["x"], y - truth["y"]
        fwd = dx * math.cos(heading) + dy * math.sin(heading)
        right = -dx * math.sin(heading) + dy * math.cos(heading)
        return fwd, right, truth["alt"]

    def _frame_xy(self) -> tuple[float, float]:
        return self.link.frame_point(self.fwd, -self.left)  # world y is to the right

    def _tilt_deg(self) -> float:
        tilt = self.drone.camera_tilt()  # the sim has no gimbal: the estimate is the truth
        return tilt.deg if tilt.deg is not None else (tilt.low_deg + tilt.high_deg) / 2

    def _run(self) -> None:
        while not self._stop.wait(DETECT_PERIOD_S):
            frame_time = time.time()
            body = self.observe() if self.detect else None
            if body is None or self._random.random() < self.miss:
                continue
            noise = self._random.gauss(0, DETECT_NOISE), self._random.gauss(0, DETECT_NOISE)
            face = self.observe_face()
            time.sleep(DETECT_LATENCY_S)
            self.reports += 1
            self.report(
                frame_time,
                _shifted(body, noise),
                None if face is None else _shifted(face, noise),
                DETECT_CONFIDENCE,
                DETECT_LABEL,
            )


def _shifted(box: Box, noise: tuple[float, float]) -> Box:
    """The box moved by `noise` (image fractions), clipped to the image."""
    x0, y0, x1, y1 = box
    return (
        round(min(max(x0 + noise[0], 0.0), 1.0), 4),
        round(min(max(y0 + noise[1], 0.0), 1.0), 4),
        round(min(max(x1 + noise[0], 0.0), 1.0), 4),
        round(min(max(y1 + noise[1], 0.0), 1.0), 4),
    )


def connect_sim(
    max_speed: float = 60.0,
    estimator: Estimator | None = None,
    frame_yaw_offset_deg: float = DEFAULT_FRAME_YAW_OFFSET_DEG,
    seed: int | None = None,
    room_half_m: float = DEFAULT_ROOM_HALF_M,
    room_half_y_m: float | None = None,
    video_dir: str | None = None,
) -> Drone:
    """A started Drone on the simulator; the SimLink is `drone.link`. `video_dir`: recorded
    frames (frame_*.jpg) as the camera instead of the test pattern."""
    link = SimLink(frame_yaw_offset_deg, seed, room_half_m, room_half_y_m)
    sticks = SimSticks(link, max_speed)
    link.sticks = sticks
    drone = Drone(link, sticks, SimCamera(link, video_dir), estimator)
    drone.start()
    return drone


def _xyz(values: tuple[float, ...] | None) -> dict[str, float] | None:
    return None if values is None else dict(zip("xyz", values, strict=True))
