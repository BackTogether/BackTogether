"""`Drone`: one object for the HoverAir X1 (or the simulator) from Python on the Mac.

It wraps the Bluetooth link (`DroneLink`), the 30 Hz stick sender (`StickSender`) and the video
receiver + decoder, and adds: velocity commands with a watchdog, a cancel-everything STOP,
telemetry and video streams (callbacks and iterators), and the pose estimator.

    with Drone.connect(max_speed=50) as drone:
        drone.wait_connected()
        drone.enter_manual()
        drone.wait_for_takeoff()          # press the drone's button
        drone.move(fwd=0.5, seconds=1.0)
        drone.land()

Motion values are -1..1, a fraction of the speed cap (`set_speed`), with hoverctl's signs:
fwd/right/up/turn right are positive, relative to the camera.
"""

import math
import threading
import time
import traceback
from collections import deque
from collections.abc import Callable, Iterator
from dataclasses import replace
from pathlib import Path
from typing import Any, Protocol

from drone.hoverctl.decoder import MjpegDecoder
from drone.hoverctl.pbwire import VIDEO_QUALITY_SD
from drone.hoverctl.protocol import NEUTRAL, Axes
from drone.hoverctl.sticks import DEFAULT_MAX_MAGNITUDE, StickSender
from drone.hoverctl.video import VideoReceiver

from .estimate import DeadReckoner, Estimator
from .recording import Mp4Recorder, save_jpeg
from .safety import Interrupted
from .telemetry import Broadcaster, Pose, Telemetry, Tilt, telemetry_from_snapshot
from .tilt import GIMBAL_BOTTOM_DEG, GIMBAL_TOP_DEG, TiltEstimator, hold_time

CONTROL_PERIOD_S = 1 / 30  # the stick sender's rate
TELEMETRY_PERIOD_S = 0.1
DEFAULT_HOLD_S = 0.5  # a velocity command lapses to neutral after this (watchdog)
MAX_HOLD_S = 10.0
WAIT_POLL_S = 0.05
FRAME_WAIT_S = 1.0
CONNECT_TIMEOUT_S = 30.0
PATH_MIN_STEP_M = 0.1
PATH_MAX_POINTS = 2000
FLYING_PREFIX = "flying"
NO_LINK = {"ok": False, "error": "no bluetooth link"}
# set_tilt
TILT_TOLERANCE_DEG = 1.0
TILT_ANCHOR_MARGIN_S = 2.0  # hold camera-up this much longer than a full sweep to anchor
TILT_TICK_S = 0.05
TILT_HOLD_S = 0.3


class Link(Protocol):
    """What Drone needs from DroneLink (the simulator's SimLink has the same)."""

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def snapshot(self) -> dict[str, Any]: ...
    def enter_manual(self) -> dict[str, Any]: ...
    def land(self) -> dict[str, Any]: ...
    def set_video_quality(self, quality: str) -> dict[str, Any]: ...
    def take_photo(self) -> dict[str, Any]: ...
    def record_video(self, on: bool) -> dict[str, Any]: ...


def _unit(value: float) -> float:
    """Clamp to -1..1; NaN or infinity count as neutral."""
    return max(-1.0, min(1.0, value)) if math.isfinite(value) else 0.0


class Drone:
    """See the module docstring. Thread-safe; start() before use, close() at the end."""

    def __init__(
        self,
        link: Link | None,
        sticks: StickSender,
        video: VideoReceiver | Any | None,
        estimator: Estimator | None = None,
        tilt_estimator: TiltEstimator | None = None,
    ) -> None:
        """`link` None: no Bluetooth (no telemetry, commands fail). `video` is a VideoReceiver
        or anything with `.decoder`, `.start()`, `.stop()`, `.restart_decoder()`."""
        self.link = link
        self.sticks = sticks
        self.video = video
        self.estimator: Estimator = estimator or DeadReckoner()
        self.tilt_estimator = tilt_estimator or TiltEstimator()
        self._lock = threading.Lock()
        self._epoch = 0  # bumped by stop(): every autopilot action started before is cancelled
        self._command: Axes | None = None
        self._command_until = 0.0
        self._telemetry: Telemetry | None = None
        self._was_in_flight = False
        self._path: deque[tuple[float, float]] = deque(maxlen=PATH_MAX_POINTS)
        self._frame: bytes | None = None
        self._frame_at: float | None = None  # monotonic
        self._recorder: Mp4Recorder | None = None
        self._last_recording: dict[str, Any] | None = None
        self._telemetry_bus: Broadcaster[Telemetry] = Broadcaster("telemetry")
        self._frame_bus: Broadcaster[bytes] = Broadcaster("frame")
        self._closing = threading.Event()
        self._threads = [
            threading.Thread(target=self._control_loop, name="drone-control", daemon=True),
            threading.Thread(target=self._frame_loop, name="drone-frames", daemon=True),
        ]
        self._started = False

    @classmethod
    def connect(
        cls,
        max_speed: float = DEFAULT_MAX_MAGNITUDE,
        video_quality: str = VIDEO_QUALITY_SD,
        estimator: Estimator | None = None,
        **link_options: Any,
    ) -> "Drone":
        """The real drone: Bluetooth (needs `uv run --with bleak`), sticks and video, started.
        `link_options` go to DroneLink (e.g. wifi_password)."""
        from drone.hoverctl.ble import DroneLink  # bleak is only needed for the real drone

        link = DroneLink(video_quality=video_quality, **link_options)
        sticks = StickSender(max_magnitude=max_speed, gate=link.is_manual)
        sticks.set_session_gate(link.in_manual_session)
        video = VideoReceiver(decoder=MjpegDecoder())
        drone = cls(link, sticks, video, estimator)
        drone.start()
        return drone

    # --- lifecycle ----------------------------------------------------------------------------

    def start(self) -> None:
        """Start the parts and the control thread. Safe to call twice."""
        if self._started:
            return
        self._started = True
        if self.link:
            self.link.start()
        self.sticks.start()
        if self.video:
            self.video.start()
        self._publish_telemetry()
        for thread in self._threads:
            thread.start()

    def close(self) -> None:
        """Neutral, finish any recording, then stop everything (the stick sender sends 0.5 s
        of neutral)."""
        self.stop()
        self.stop_recording()
        self._closing.set()
        for thread in self._threads:
            if thread.is_alive():
                thread.join()
        self.sticks.stop()
        if self.video:
            self.video.stop()
        if self.link:
            self.link.stop()

    def __enter__(self) -> "Drone":
        self.start()
        return self

    def __exit__(self, exc_type: type[BaseException] | None, *_: object) -> None:
        """Always neutral; on an exception (Ctrl-C included) land if flying."""
        self.stop()
        if exc_type is not None and self.telemetry().in_flight:
            print(f"drone: {exc_type.__name__} in flight, landing", flush=True)
            self.land()
        self.close()

    # --- connection and drone commands ----------------------------------------------------

    def wait_connected(self, timeout: float = CONNECT_TIMEOUT_S) -> bool:
        """True once the Bluetooth link is up."""
        deadline = time.monotonic() + timeout
        while not self.telemetry().connected:
            if time.monotonic() >= deadline:
                return False
            time.sleep(WAIT_POLL_S)
        return True

    def enter_manual(self) -> dict[str, Any]:
        """The app's Manual Control sequence (joins the drone's Wi-Fi if needed)."""
        return self.link.enter_manual() if self.link else NO_LINK

    def land(self) -> dict[str, Any]:
        """Cancel the autopilot, neutral sticks, and ask the drone to land."""
        self.stop()
        return self.link.land() if self.link else NO_LINK

    def emergency_stop(self) -> dict[str, Any]:
        """Same as land(): neutral at once, then land. The X1 has no motor kill command."""
        return self.land()

    def stop(self) -> None:
        """STOP: cancel any autopilot action and command, sticks neutral (the drone hovers)."""
        with self._lock:
            self._epoch += 1
            self._command = None
            self.sticks.set_input(NEUTRAL)

    def set_speed(self, max_speed: float) -> None:
        """Speed cap in stick %, 0..100: motion values of 1.0 send this much."""
        self.sticks.set_max(max_speed)

    @property
    def speed_cap(self) -> float:
        return self.sticks.max_magnitude

    def set_sport(self, on: bool) -> None:
        """is_sport_mode in every stick packet (untested on the X1, see drone/README.md)."""
        self.sticks.set_sport(on)

    def set_video_quality(self, quality: str) -> dict[str, Any]:
        """Video quality "sd" or "hd" (untested); restarts the decoder when the drone accepts."""
        if not self.link:
            return NO_LINK
        result = self.link.set_video_quality(quality)
        if result["ok"] and self.video:
            self.video.restart_decoder()
        return result

    def wait_for_takeoff(
        self,
        timeout: float | None = None,
        on_state: Callable[[str], None] | None = None,
    ) -> bool:
        """Wait for the drone's button takeoff (the X1 has no takeoff request): PRE_TAKEOFF,
        TAKING_OFF (its own climb), then flying. `on_state(state)` gets every change (default:
        print). True once flying, False on timeout."""
        report = on_state or (lambda state: print(f"drone: {state}", flush=True))
        deadline = None if timeout is None else time.monotonic() + timeout
        last = None
        while deadline is None or time.monotonic() < deadline:
            state = self.telemetry().flight_state
            if state != last:
                report(str(state))
                last = state
            if state and state.startswith(FLYING_PREFIX):
                return True
            time.sleep(WAIT_POLL_S)
        return False

    # --- motion -------------------------------------------------------------------------------

    @property
    def epoch(self) -> int:
        """Changes at every stop(); an action started at epoch E is cancelled once it differs."""
        return self._epoch

    def check(self, epoch: int) -> None:
        """Raise Interrupted if stop() was called since `epoch`."""
        if epoch != self._epoch:
            raise Interrupted("stopped")

    def command(
        self,
        fwd: float = 0.0,
        right: float = 0.0,
        up: float = 0.0,
        turn: float = 0.0,
        hold_s: float = DEFAULT_HOLD_S,
        epoch: int | None = None,
        tilt: float = 0.0,
    ) -> None:
        """Velocity-style command, -1..1 of the speed cap (`tilt`: camera up +). It lapses to
        neutral after `hold_s` unless renewed (the watchdog). With `epoch`, raise Interrupted
        if STOP came since."""
        axes = Axes(
            roll=_unit(right),
            pitch=_unit(fwd),
            thrust=_unit(up),
            yaw=_unit(turn),
            gimbal=_unit(tilt),
        )
        with self._lock:
            if epoch is not None and epoch != self._epoch:
                raise Interrupted("stopped")
            self._command = axes
            self._command_until = time.monotonic() + min(max(hold_s, 0.0), MAX_HOLD_S)
            self.sticks.set_input(axes)

    def neutral(self) -> None:
        """Sticks neutral without cancelling the running action (the autopilot uses it)."""
        with self._lock:
            self._command = None
            self.sticks.set_input(NEUTRAL)

    def command_active(self) -> bool:
        with self._lock:
            return self._command is not None

    def move(
        self,
        fwd: float = 0.0,
        right: float = 0.0,
        up: float = 0.0,
        turn: float = 0.0,
        seconds: float = 1.0,
    ) -> None:
        """Hold a command for `seconds`, then neutral. Blocks; STOP raises Interrupted."""
        epoch = self._epoch
        self.command(fwd, right, up, turn, hold_s=seconds, epoch=epoch)
        try:
            self.sleep(seconds, epoch)
        finally:
            self.neutral()

    def hover(self, seconds: float) -> None:
        """Neutral for `seconds`. Blocks; STOP raises Interrupted."""
        epoch = self._epoch
        self.neutral()
        self.sleep(seconds, epoch)

    def sleep(self, seconds: float, epoch: int) -> None:
        """Sleep, raising Interrupted as soon as STOP comes."""
        deadline = time.monotonic() + seconds
        while (left := deadline - time.monotonic()) > 0:
            self.check(epoch)
            time.sleep(min(left, WAIT_POLL_S))
        self.check(epoch)

    def camera_tilt(self) -> Tilt:
        """The camera tilt estimate (the X1 never reports it): see tilt.py."""
        return self.tilt_estimator.tilt()

    def set_tilt(self, deg: float) -> Tilt:
        """Move the camera to a tilt (deg, +15 top .. 0 level .. -90 down) with the calibrated
        model (tilt.py), open loop. If the estimate is not anchored, first hold camera-up for a
        full sweep at the current speed cap (+2 s) to reach the top stop; then hold up or down
        for hold_time(change). Blocks (up to ~10 s + ~10 s at cap 100); STOP raises Interrupted.
        Raises ValueError if the speed cap is inside the gimbal dead zone."""
        target = min(max(deg, GIMBAL_BOTTOM_DEG), GIMBAL_TOP_DEG)
        cap = self.speed_cap
        full_sweep = hold_time(GIMBAL_TOP_DEG - GIMBAL_BOTTOM_DEG, cap)
        epoch = self._epoch
        try:
            if not self.camera_tilt().anchored:
                self._hold_tilt(1.0, full_sweep + TILT_ANCHOR_MARGIN_S, epoch)
            now = self.camera_tilt().deg
            if now is None:
                raise RuntimeError("the tilt did not anchor: are the sticks live (MANUAL)?")
            change = target - now
            if abs(change) > TILT_TOLERANCE_DEG:
                self._hold_tilt(math.copysign(1.0, change), hold_time(abs(change), cap), epoch)
        finally:
            self.neutral()
        return self.camera_tilt()

    def _hold_tilt(self, direction: float, seconds: float, epoch: int) -> None:
        """Full gimbal input (+1 up, -1 down) for `seconds`, then neutral."""
        deadline = time.monotonic() + seconds
        while (left := deadline - time.monotonic()) > 0:
            self.command(hold_s=TILT_HOLD_S, epoch=epoch, tilt=direction)
            self.sleep(min(TILT_TICK_S, left), epoch)
        self.neutral()

    # --- telemetry and estimate ---------------------------------------------------------------

    def telemetry(self) -> Telemetry:
        """The latest reading (10 Hz), with the estimator's pose."""
        with self._lock:
            assert self._telemetry is not None, "call start() first"
            return self._telemetry

    def on_telemetry(self, callback: Callable[[Telemetry], None]) -> Callable[[], None]:
        """Call `callback(telemetry)` at 10 Hz on the control thread (keep it quick).
        Returns a function that unsubscribes."""
        return self._telemetry_bus.on(callback)

    def telemetry_stream(self, timeout: float | None = None) -> Iterator[Telemetry]:
        """Iterate over telemetry readings; ends after `timeout` s without one."""
        return self._telemetry_bus.stream(timeout)

    def wait_odometry(self, after: float | None, timeout: float, epoch: int) -> Telemetry | None:
        """Wait for an odometry sample newer than unix time `after`; None on timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.check(epoch)
            telemetry = self.telemetry()
            sample = telemetry.odometry_time
            if sample is not None and (after is None or sample > after):
                return telemetry
            time.sleep(WAIT_POLL_S)
        return None

    def pose(self) -> Pose:
        return self.estimator.pose()

    def path(self) -> list[tuple[float, float]]:
        """Estimated (x, y) track since takeoff."""
        with self._lock:
            return list(self._path)

    # --- video --------------------------------------------------------------------------------

    def latest_frame(self) -> bytes | None:
        """The last decoded frame as JPEG bytes, None before the first."""
        with self._lock:
            return self._frame

    def frame_age_s(self) -> float | None:
        with self._lock:
            return None if self._frame_at is None else time.monotonic() - self._frame_at

    def on_frame(self, callback: Callable[[bytes], None]) -> Callable[[], None]:
        """Call `callback(jpeg)` for every decoded frame; returns an unsubscribe function."""
        return self._frame_bus.on(callback)

    def frames(self, timeout: float | None = None) -> Iterator[bytes]:
        """Iterate over new JPEG frames; ends after `timeout` s without one."""
        return self._frame_bus.stream(timeout)

    def take_photo(self, path: Path | None = None) -> Path:
        """Save the latest decoded frame (default recordings/photo_<time>.jpg)."""
        frame = self.latest_frame()
        if frame is None:
            raise RuntimeError("no video frame yet")
        return save_jpeg(frame, path)

    def start_recording(self, path: Path | None = None) -> Path:
        """Record the live stream to an MP4 (default recordings/video_<time>.mp4), remuxed
        without re-encoding; the file starts at the next keyframe. Already recording: that
        recording's path."""
        if self.video is None:
            raise RuntimeError("no video receiver")
        with self._lock:
            if self._recorder is not None:
                return self._recorder.path
            self._recorder = Mp4Recorder(path)
            self.video.add_stream_listener(self._recorder.feed)
            return self._recorder.path

    def stop_recording(self) -> Path | None:
        """Finish the MP4; its path, or None if nothing was recorded (no keyframe came)."""
        with self._lock:
            recorder, self._recorder = self._recorder, None
        if recorder is None:
            return None
        self.video.remove_stream_listener(recorder.feed)
        path = recorder.stop()
        with self._lock:
            self._last_recording = {"path": str(path) if path else None, "error": recorder.error}
        return path

    def recording(self) -> dict[str, Any]:
        """{"on", "path", "elapsed_s", "bytes", "writing", "error", "last"}: JSON-ready."""
        with self._lock:
            recorder, last = self._recorder, self._last_recording
        if recorder is None:
            return {"on": False, "last": last}
        return {
            "on": True,
            "path": str(recorder.path),
            "elapsed_s": round(time.time() - recorder.started_at, 1),
            "bytes": recorder.bytes,
            "writing": recorder.writing,
            "error": recorder.error,
            "last": last,
        }

    def drone_photo(self) -> dict[str, Any]:
        """A full-quality photo on the drone's SD card (the app's shutter, over Wi-Fi)."""
        return self.link.take_photo() if self.link else NO_LINK

    def drone_record(self, on: bool) -> dict[str, Any]:
        """Start or stop a recording on the drone's SD card (the app's record button)."""
        return self.link.record_video(on) if self.link else NO_LINK

    # --- threads ------------------------------------------------------------------------------

    def _control_loop(self) -> None:
        last = time.monotonic()
        next_telemetry = last
        while not self._closing.wait(CONTROL_PERIOD_S):
            now = time.monotonic()
            try:
                self._expire_command(now)
                self.estimator.on_command(self.sticks.output, now - last)
                self.tilt_estimator.on_command(self.sticks.output, now - last)
                if now >= next_telemetry:
                    next_telemetry = now + TELEMETRY_PERIOD_S
                    self._publish_telemetry()
            except Exception:  # the loop must survive a bad reading or a buggy estimator
                print(f"drone control loop:\n{traceback.format_exc()}", flush=True)
            last = now

    def _expire_command(self, now: float) -> None:
        """Renew the live command (the sender drops input after 0.3 s), or end it."""
        with self._lock:
            if self._command is None:
                return
            if now < self._command_until:
                self.sticks.set_input(self._command)
            else:
                self._command = None
                self.sticks.set_input(NEUTRAL)

    def _publish_telemetry(self) -> None:
        snapshot = self.link.snapshot() if self.link else None
        telemetry = telemetry_from_snapshot(snapshot, self.sticks, self.frame_age_s())
        self.estimator.update(telemetry)
        self.tilt_estimator.update(telemetry)
        pose = self.estimator.pose()
        telemetry = replace(telemetry, pose=pose, tilt=self.tilt_estimator.tilt())
        with self._lock:
            self._telemetry = telemetry
            self._record_path(telemetry.in_flight, pose)
        self._telemetry_bus.publish(telemetry)

    def _record_path(self, in_flight: bool, pose: Pose) -> None:
        if in_flight and not self._was_in_flight:
            self._path.clear()  # new takeoff, new frame
        self._was_in_flight = in_flight
        if not in_flight:
            return
        last = self._path[-1] if self._path else None
        if last is None or math.hypot(pose.x - last[0], pose.y - last[1]) >= PATH_MIN_STEP_M:
            self._path.append((pose.x, pose.y))

    def _frame_loop(self) -> None:
        decoder = getattr(self.video, "decoder", None)
        if decoder is None:
            return
        seen = 0
        while not self._closing.is_set():
            number, jpeg = decoder.wait_frame(seen, FRAME_WAIT_S)
            if number == seen or not jpeg:
                continue
            seen = number
            with self._lock:
                self._frame = jpeg
                self._frame_at = time.monotonic()
            self._frame_bus.publish(jpeg)
