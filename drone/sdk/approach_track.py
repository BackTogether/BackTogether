"""Tracking approach: the drone closes in on the child at a fixed altitude, steered by the camera,
until it hovers exactly over them with the camera straight down.

The other approach (mission.py, "fly_over") turns one detection into a floor point, flies there
blind with goto and only then looks down. This one keeps the child in view the whole way:

- closing in: the gimbal follows the child vertically (bbox centre kept near the image's middle
  row), so as the drone gets closer the camera tilts down by itself. Far away the drone turns to
  face the child and creeps forward; near it (NEAR_M) it stops turning and slides
  forward/sideways, so the target below never ends up behind the yaw axis. The speed shrinks
  with the distance, and the drone waits when the child is about to leave the bottom of the
  image (the gimbal is slow: ~6 deg/s at speed cap 60). The altitude is held where the approach
  started: it never descends on its own.
- overhead (within CENTRED_M of the child): the camera goes to its bottom stop (-90) and stays
  there; then stop-and-correct on the image itself: short slides until the bbox centre is within
  CENTRE_TOL_PX of the image centre, using only frames taken after the drone settled. SAMPLES
  centred frames in a row give the fix: the child's position (median) and the drone's pose at
  those frames. If the child walks away (OVERHEAD_LEAVE_M), it goes back to closing in.

Each detection is placed on the floor with the pose and camera tilt of its frame (latency does
not matter) and kept in the drone frame; the steering uses it against the current pose, so the
drone keeps going between detections. A body box is placed by its floor contact point
(mission_geometry.floor_contact), anything else by its centre at target_height_m.

Needs the camera tilted down (patrol tilt <= about -25 deg) so that every detection meets the
floor, and the tilt estimate anchored (the mission does both before patrolling).
"""

import math
import statistics
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Protocol

from . import camera
from .mission_geometry import body_to_frame, frame_to_body, ground_offset
from .navigate import (
    ALT_GAIN,
    APPROACH_FRACTION,
    COMMAND_HOLD_S,
    MAX_PULSE_S,
    MAX_VERTICAL,
    MIN_PULSE_S,
    PULSE_S_PER_M,
    SETTLE_S,
    TICK_S,
    Navigator,
)
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
PIN_DOWN = -GIMBAL_STICK  # overhead: keeps the camera at its bottom stop
CENTRED_M = 0.5  # this close to the child: the overhead stage starts
OVERHEAD_TILT_DEG = -89.0  # camera at its bottom stop (the estimate clamps at -90)
CENTRE_TOL_PX = 250.0  # bbox centre to image centre: the fix is placed geometrically anyway
OVERHEAD_LEAVE_M = 1.0  # the child this far from below: close in again
SAMPLES = 3  # centred frames overhead (not necessarily in a row) whose median is the fix
HOLD_S = 0.8  # last fix older than this: hover until a new one comes
MIN_RANGE_M = 0.3

PoseAt = Callable[[float], tuple[Pose, float | None] | None]


class Sighting(Protocol):
    """A detection: mission.Detection fits."""

    frame_time: float

    @property
    def centre(self) -> tuple[float, float]: ...

    def floor_point(self, tilt_deg: float) -> tuple[float, float, bool]: ...


@dataclass(frozen=True)
class TrackSettings:
    speed: float  # forward stick fraction far from the child
    launch_height_m: float  # the takeoff point above the floor
    target_height_m: float  # a bbox centre (not a body) above the floor
    lost_timeout_s: float


@dataclass(frozen=True)
class Fix:
    """One detection placed in the drone frame."""

    child: tuple[float, float]
    v: float  # bbox centre row, image fraction (y down)
    tilt_deg: float  # camera tilt of that frame


@dataclass(frozen=True)
class OverheadSample:
    """One centred frame overhead: the child placed on the floor and the drone at that frame."""

    child: tuple[float, float]
    pose: Pose
    tilt_deg: float
    error_px: float
    frame_time: float


@dataclass(frozen=True)
class ChildFix:
    """The approach's result, in the drone frame: the child and the drone over it."""

    child: tuple[float, float]
    drone: tuple[float, float, float]  # x, y, alt (above the takeoff point)
    heading_deg: float
    tilt_deg: float
    centre_error_px: float  # bbox centre to image centre (median)
    samples: int
    frame_time: float  # the last sample's frame

    def as_dict(self) -> dict[str, Any]:
        """The mission's child["fix"] (drone frame; mission.py adds launch and map)."""
        return {
            "time": self.frame_time,
            "samples": self.samples,
            "centre_error_px": round(self.centre_error_px, 1),
            "tilt_deg": round(self.tilt_deg, 1),
            "drone": {
                "x": round(self.drone[0], 3),
                "y": round(self.drone[1], 3),
                "alt": round(self.drone[2], 3),
                "heading_deg": round(self.heading_deg, 1),
            },
        }


def child_fix(samples: list[OverheadSample]) -> ChildFix:
    """Medians over the samples: robust to one bad frame."""
    return ChildFix(
        child=(
            statistics.median(s.child[0] for s in samples),
            statistics.median(s.child[1] for s in samples),
        ),
        drone=(
            statistics.median(s.pose.x for s in samples),
            statistics.median(s.pose.y for s in samples),
            statistics.median(s.pose.alt for s in samples),
        ),
        heading_deg=samples[-1].pose.heading_deg,
        tilt_deg=statistics.median(s.tilt_deg for s in samples),
        centre_error_px=statistics.median(s.error_px for s in samples),
        samples=len(samples),
        frame_time=samples[-1].frame_time,
    )


def centre_error_px(sighting: Sighting) -> float:
    """Pixels from the bbox centre to the image centre (camera.py resolution)."""
    u, v = sighting.centre
    return math.hypot((u - 0.5) * camera.CAMERA_WIDTH_PX, (v - 0.5) * camera.CAMERA_HEIGHT_PX)


class TrackApproach:
    """run() flies the approach and returns the ChildFix; TimeoutError when out of time or the
    child is lost, ValueError when the tilt estimate is missing."""

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
        self._fix: Fix | None = None
        self._last_seen = 0.0
        self._after = 0.0  # only sightings from frames after this
        self._hold_alt = 0.0

    def run(self, deadline: float, trigger: Sighting | None = None) -> ChildFix:
        """`trigger`: the detection that started the approach; the drone turns toward it
        until the camera sees the child again."""
        self.navigator.require_ready()
        epoch = self.drone.epoch
        self._hold_alt = self.drone.pose().alt
        self._after = self._last_seen = time.time()
        self._fix = self._place(trigger) if trigger else None
        try:
            while True:
                self._close_in(epoch, deadline)
                result = self._overhead(epoch, deadline)
                if result is not None:
                    return result
        finally:
            self.drone.neutral()

    # --- closing in -------------------------------------------------------------------------

    def _close_in(self, epoch: int, deadline: float) -> None:
        """Steer toward the child until within CENTRED_M of it."""
        while True:
            self._check(epoch, deadline)
            self._take_sighting()
            pose = self.drone.pose()
            unseen = time.time() - self._last_seen
            if self._fix is None:
                self._check_lost(unseen)
                self.note(f"track: holding, child not seen for {unseen:.1f} s")
                self._send(epoch)
                self.drone.sleep(TICK_S, epoch)
                continue
            fwd, right = frame_to_body(
                self._fix.child[0] - pose.x, self._fix.child[1] - pose.y, pose.heading_deg
            )
            distance = math.hypot(fwd, right)
            if unseen > HOLD_S:
                self._check_lost(unseen)
                self.note(f"track: child not seen for {unseen:.1f} s, turning to where it was")
                self._send(epoch, turn=_search_turn(fwd, right, distance))
                self.drone.sleep(TICK_S, epoch)
                continue
            if distance <= CENTRED_M:
                return
            tilt_now = self._tilt_now()
            v_now = self._fix.v + (tilt_now - self._fix.tilt_deg) / camera.CAMERA_VFOV_DEG
            move_fwd, move_right, turn = self._steer(fwd, right, distance, v_now)
            self.note(f"track: {distance:.2f} m to go, camera {tilt_now:.0f} deg")
            self._send(epoch, move_fwd, move_right, turn, _gimbal(v_now))
            self.drone.sleep(TICK_S, epoch)

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

    # --- overhead ---------------------------------------------------------------------------

    def _overhead(self, epoch: int, deadline: float) -> ChildFix | None:
        """Camera to -90, then centre the bbox in the image; the fix, or None when the child
        moved away (close in again)."""
        while self._tilt_now() > OVERHEAD_TILT_DEG:
            self._check(epoch, deadline)
            self._check_lost(time.time() - self._last_seen)
            self.note(f"track: over the child, camera {self._tilt_now():.0f} deg going down")
            self._send(epoch, tilt=PIN_DOWN)
            self.drone.sleep(TICK_S, epoch)
        self._after = time.time()  # from now on only frames with the camera straight down
        samples: list[OverheadSample] = []
        while True:
            self._check(epoch, deadline)
            sighting = self.next_sighting(self._after)
            sample = self._overhead_sample(sighting) if sighting else None
            if sample is None:
                self._check_lost(time.time() - self._last_seen)
                self._send(epoch, tilt=PIN_DOWN)
                self.drone.sleep(TICK_S, epoch)
                continue
            self._last_seen = time.time()
            self._fix = Fix(sample.child, sighting.centre[1], sample.tilt_deg)
            pose = self.drone.pose()
            away = math.hypot(sample.child[0] - pose.x, sample.child[1] - pose.y)
            if away > OVERHEAD_LEAVE_M:
                self.note(f"track: child {away:.1f} m away from below, closing in again")
                return None
            if sample.error_px <= CENTRE_TOL_PX:
                samples.append(sample)
                self.note(f"track: overhead and centred, {len(samples)}/{SAMPLES} fixes")
                if len(samples) >= SAMPLES:
                    return child_fix(samples)
                continue
            self.note(f"track: overhead, centring: {sample.error_px:.0f} px off")
            self._nudge(epoch, *self._centre_offset(sighting, sample))

    def _overhead_sample(self, sighting: Sighting) -> OverheadSample | None:
        """A sighting from a frame with the camera straight down, placed; None otherwise."""
        sample = self.pose_at(sighting.frame_time)
        if sample is None or sample[1] is None or sample[1] > OVERHEAD_TILT_DEG:
            return None
        pose, tilt = sample
        placed = self._place(sighting)
        if placed is None:
            return None
        return OverheadSample(
            placed.child, pose, tilt, centre_error_px(sighting), sighting.frame_time
        )

    def _centre_offset(self, sighting: Sighting, sample: OverheadSample) -> tuple[float, float]:
        """Body-frame (forward, right) metres from the drone to the bbox centre on the floor."""
        height = max(sample.pose.alt + self.settings.launch_height_m, MIN_RANGE_M)
        return ground_offset(*sighting.centre, sample.tilt_deg, height) or (0.0, 0.0)

    def _nudge(self, epoch: int, fwd_m: float, right_m: float) -> None:
        """One short body-frame slide toward the bbox centre, then hover to settle (the next
        sightings must come from frames taken after it)."""
        distance = math.hypot(fwd_m, right_m)
        if distance > 0:
            fraction = min(APPROACH_FRACTION, self.settings.speed)
            seconds = min(MAX_PULSE_S, max(MIN_PULSE_S, distance * PULSE_S_PER_M))
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                self.navigator.guard(epoch)
                fwd, right = fraction * fwd_m / distance, fraction * right_m / distance
                self._send(epoch, fwd, right, tilt=PIN_DOWN)
                self.drone.sleep(TICK_S, epoch)
        end = time.monotonic() + SETTLE_S
        while time.monotonic() < end:
            self.navigator.guard(epoch)
            self._send(epoch, tilt=PIN_DOWN)
            self.drone.sleep(TICK_S, epoch)
        self._after = time.time()

    # --- shared -----------------------------------------------------------------------------

    def _take_sighting(self) -> None:
        sighting = self.next_sighting(self._after)
        new = self._place(sighting) if sighting else None
        if new is not None:
            self._fix, self._last_seen = new, time.time()

    def _place(self, sighting: Sighting) -> Fix | None:
        """The detection on the floor, in the drone frame; None without pose or tilt."""
        sample = self.pose_at(sighting.frame_time)
        if sample is None or sample[1] is None:
            return None
        pose, tilt = sample
        u, v, on_floor = sighting.floor_point(tilt)
        above_floor = 0.0 if on_floor else self.settings.target_height_m
        height = max(pose.alt + self.settings.launch_height_m - above_floor, MIN_RANGE_M)
        ground = ground_offset(u, v, tilt, height)
        if ground is None:
            return None
        dx, dy = body_to_frame(*ground, pose.heading_deg)
        return Fix((pose.x + dx, pose.y + dy), sighting.centre[1], tilt)

    def _check(self, epoch: int, deadline: float) -> None:
        if time.monotonic() > deadline:
            raise TimeoutError("not overhead in time")
        self.navigator.guard(epoch)

    def _check_lost(self, unseen: float) -> None:
        if unseen > self.settings.lost_timeout_s:
            raise TimeoutError(f"child not seen for {unseen:.0f} s")

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
        turn: float = 0.0,
        tilt: float = 0.0,
    ) -> None:
        """Like Navigator.send (flight axes under limits.max_speed, altitude held) plus the
        gimbal, which is not scaled: a scaled-down gimbal stick falls into its dead zone."""
        up = _clip(ALT_GAIN * (self._hold_alt - self.drone.pose().alt), MAX_VERTICAL)
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
