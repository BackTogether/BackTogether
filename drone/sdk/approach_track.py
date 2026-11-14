"""Tracking approach: the drone closes in on the child at a fixed altitude, steered by the camera.

The other approach (mission.py, "fly_over") turns one detection into a floor point, flies there
blind with goto and only then looks down. This one keeps the child in view the whole way:

- the gimbal follows the child vertically (bbox centre kept near the image's middle row), so as
  the drone gets closer the camera tilts down by itself, and reaches -90 when it is overhead;
- far away the drone turns to face the child and creeps forward; near it (NEAR_M) it stops
  turning and slides forward/sideways, so the target below never ends up behind the yaw axis;
- the speed shrinks with the distance, and the drone waits when the child is about to leave the
  bottom of the image (the gimbal is slow: ~6 deg/s at speed cap 60);
- the altitude is held where the approach started: it never descends on its own.

Each detection is placed on the floor with the pose and camera tilt of its frame (latency does
not matter) and kept in the drone frame; the steering uses it against the current pose, so the
drone keeps going between detections. Overhead (camera at OVERHEAD_TILT_DEG or lower, within
CENTRED_M), it hovers and the child's position is the median of SAMPLES fresh fixes.

Needs the camera tilted down (patrol tilt <= about -25 deg) so that every detection meets the
floor, and the tilt estimate anchored (the mission does both before patrolling).
"""

import math
import statistics
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from . import camera
from .mission_geometry import body_to_frame, frame_to_body, ground_offset
from .navigate import ALT_GAIN, COMMAND_HOLD_S, MAX_VERTICAL, TICK_S, Navigator
from .telemetry import Pose

NEAR_M = 1.0  # closer than this: slide instead of turning
SLOWDOWN_M = 1.5  # the speed falls linearly inside this distance
MIN_FRACTION = 0.08  # slowest useful forward/sideways stick while not centred
TURN_FIRST_DEG = 25.0  # child further off the nose than this: turn on the spot first
YAW_GAIN = 1 / 30  # turn stick per degree of bearing
MAX_TURN = 0.5
V_DEADBAND = 0.08  # bbox centre this close to the middle row: the gimbal stays
WAIT_V = 0.8  # bbox centre lower than this: stop and let the gimbal catch up
GIMBAL_STICK = 1.0  # full gimbal input: the dead zone is 13.5 % of the stick
OVERHEAD_TILT_DEG = -85.0
CENTRED_M = 0.25
KEEP_CENTRED_M = 0.4  # once collecting fixes overhead, noise up to this does not restart them
SAMPLES = 4  # fixes overhead whose median is the child's position
HOLD_S = 0.8  # last fix older than this: hover until a new one comes
MIN_RANGE_M = 0.3

PoseAt = Callable[[float], tuple[Pose, float | None] | None]


class Sighting(Protocol):
    """A detection: mission.Detection fits."""

    frame_time: float

    @property
    def centre(self) -> tuple[float, float]: ...


@dataclass(frozen=True)
class TrackSettings:
    speed: float  # forward stick fraction far from the child
    height_offset_m: float  # launch_height_m - target_height_m: altitude -> height above it
    lost_timeout_s: float


@dataclass(frozen=True)
class Fix:
    """One detection placed in the drone frame."""

    child: tuple[float, float]
    v: float  # bbox centre row, image fraction (y down)
    tilt_deg: float  # camera tilt of that frame


class TrackApproach:
    """run() flies the approach and returns the child's drone-frame position; TimeoutError
    when out of time or the child is lost, ValueError when the tilt estimate is missing."""

    def __init__(
        self,
        navigator: Navigator,
        pose_at: PoseAt,
        next_sighting: Callable[[float], Sighting | None],
        settings: TrackSettings,
        note: Callable[[str], None],
    ) -> None:
        self.navigator = navigator
        self.drone = navigator.drone
        self.pose_at = pose_at
        self.next_sighting = next_sighting  # the newest unused one from a frame after `after`
        self.settings = settings
        self.note = note

    def run(self, deadline: float, trigger: Sighting | None = None) -> tuple[float, float]:
        """`trigger`: the detection that started the approach; the drone turns toward it
        until the camera sees the child again."""
        self.navigator.require_ready()
        epoch = self.drone.epoch
        hold_alt = self.drone.pose().alt
        started = last_seen = time.time()
        fix = self._place(trigger) if trigger else None
        samples: list[tuple[float, float]] = []
        try:
            while True:
                if time.monotonic() > deadline:
                    raise TimeoutError("not overhead in time")
                self.navigator.guard(epoch)
                sighting = self.next_sighting(started)
                new = self._place(sighting) if sighting else None
                if new is not None:
                    fix, last_seen = new, time.time()
                pose = self.drone.pose()
                up = _clip(ALT_GAIN * (hold_alt - pose.alt), MAX_VERTICAL)
                unseen = time.time() - last_seen
                if fix is None:
                    if unseen > self.settings.lost_timeout_s:
                        raise TimeoutError(f"child not seen for {unseen:.0f} s")
                    self.note(f"track: holding, child not seen for {unseen:.1f} s")
                    self._send(epoch, up=up)
                    self.drone.sleep(TICK_S, epoch)
                    continue
                fwd, right = frame_to_body(
                    fix.child[0] - pose.x, fix.child[1] - pose.y, pose.heading_deg
                )
                distance = math.hypot(fwd, right)
                if unseen > HOLD_S:
                    if unseen > self.settings.lost_timeout_s:
                        raise TimeoutError(f"child not seen for {unseen:.0f} s")
                    self.note(f"track: child not seen for {unseen:.1f} s, turning to where it was")
                    samples.clear()
                    self._send(epoch, up=up, turn=_search_turn(fwd, right, distance))
                    self.drone.sleep(TICK_S, epoch)
                    continue
                tilt_now = self._tilt_now()
                v_now = fix.v + (tilt_now - fix.tilt_deg) / camera.CAMERA_VFOV_DEG
                if distance <= (KEEP_CENTRED_M if samples else CENTRED_M):
                    overhead = tilt_now <= OVERHEAD_TILT_DEG
                    if overhead and new is not None:
                        samples.append(new.child)
                        if len(samples) >= SAMPLES:
                            return _median(samples)
                    self.note(
                        f"track: overhead, {len(samples)}/{SAMPLES} fixes"
                        if overhead
                        else f"track: over the child, camera {tilt_now:.0f} deg going down"
                    )
                    tilt = 0.0 if overhead else -GIMBAL_STICK
                    self._send(epoch, up=up, tilt=tilt)
                    self.drone.sleep(TICK_S, epoch)
                    continue
                samples.clear()
                move_fwd, move_right, turn = self._steer(fwd, right, distance, v_now)
                self.note(f"track: {distance:.2f} m to go, camera {tilt_now:.0f} deg")
                self._send(epoch, move_fwd, move_right, up, turn, _gimbal(v_now))
                self.drone.sleep(TICK_S, epoch)
        finally:
            self.drone.neutral()

    def _place(self, sighting: Sighting) -> Fix | None:
        """The detection on the floor, in the drone frame; None without pose or tilt."""
        sample = self.pose_at(sighting.frame_time)
        if sample is None or sample[1] is None:
            return None
        pose, tilt = sample
        u, v = sighting.centre
        height = max(pose.alt + self.settings.height_offset_m, MIN_RANGE_M)
        ground = ground_offset(u, v, tilt, height)
        if ground is None:
            return None
        dx, dy = body_to_frame(*ground, pose.heading_deg)
        return Fix((pose.x + dx, pose.y + dy), v, tilt)

    def _steer(
        self, fwd: float, right: float, distance: float, v_now: float
    ) -> tuple[float, float, float]:
        """(forward, right, turn) sticks toward the child."""
        fraction = max(MIN_FRACTION, self.settings.speed * min(1.0, distance / SLOWDOWN_M))
        if v_now > WAIT_V:
            fraction = 0.0  # about to leave the image: the gimbal first
        if distance > NEAR_M:
            bearing = math.degrees(math.atan2(right, fwd))
            turn = _clip(bearing * YAW_GAIN, MAX_TURN)
            if abs(bearing) > TURN_FIRST_DEG:
                return 0.0, 0.0, turn
            return fraction, 0.0, turn
        return fraction * fwd / distance, fraction * right / distance, 0.0

    def _tilt_now(self) -> float:
        tilt = self.drone.camera_tilt().deg
        if tilt is None:
            raise ValueError("track: the camera tilt is not anchored")
        return tilt

    def _send(
        self,
        epoch: int,
        fwd: float = 0.0,
        right: float = 0.0,
        up: float = 0.0,
        turn: float = 0.0,
        tilt: float = 0.0,
    ) -> None:
        """Like Navigator.send (flight axes under limits.max_speed) plus the gimbal, which is
        not scaled: a scaled-down gimbal stick falls into its dead zone."""
        scale = min(1.0, self.navigator.limits.max_speed / max(self.drone.speed_cap, 1.0))
        self.drone.command(
            fwd * scale, right * scale, up * scale, turn * scale, COMMAND_HOLD_S, epoch, tilt
        )


def _search_turn(fwd: float, right: float, distance: float) -> float:
    """Turn stick toward the last known position while the child is out of view; none when
    it is close (it is below the drone, turning would not bring it into view)."""
    if distance <= NEAR_M:
        return 0.0
    return _clip(math.degrees(math.atan2(right, fwd)) * YAW_GAIN, MAX_TURN)


def _gimbal(v_now: float) -> float:
    """Gimbal stick that brings the bbox centre back to the middle row (+ = camera up)."""
    if v_now > 0.5 + V_DEADBAND:
        return -GIMBAL_STICK
    if v_now < 0.5 - V_DEADBAND:
        return GIMBAL_STICK
    return 0.0


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _median(points: list[tuple[float, float]]) -> tuple[float, float]:
    return statistics.median(p[0] for p in points), statistics.median(p[1] for p in points)
