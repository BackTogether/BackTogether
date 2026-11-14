"""Telemetry events, the pose estimate, and a small publish/subscribe helper.

Frame: the drone's odometry frame, which resets at every takeoff. x and y are horizontal, z is
down (so altitude = -z). It is not aligned with the drone's heading: `Pose.heading_deg` is the
angle of the camera's forward direction from the frame's +x axis, positive toward +y (turning
right). Units: metres, seconds, degrees.
"""

import contextlib
import queue
import threading
import time
import traceback
from collections.abc import Callable, Iterator
from dataclasses import asdict, dataclass
from typing import Any, Generic, TypeVar

STREAM_QUEUE_SIZE = 64  # per subscriber; the oldest item is dropped when it is full

Vector = tuple[float, float, float]


@dataclass(frozen=True)
class Pose:
    """What the estimator believes. `x`, `y`, `z` in the takeoff frame (z down)."""

    x: float
    y: float
    z: float
    heading_deg: float
    heading_sigma_deg: float  # 1-sigma heading uncertainty
    position_sigma_m: float  # 1-sigma horizontal position uncertainty
    velocity: Vector  # frame velocity used for extrapolation, m/s
    in_flight: bool

    @property
    def alt(self) -> float:
        return -self.z

    def as_dict(self) -> dict[str, Any]:
        return {**asdict(self), "alt": round(self.alt, 3)}


@dataclass(frozen=True)
class Tilt:
    """Camera tilt in degrees (0 = level, + = up, -90 = straight down). `deg` is None until
    the estimate is anchored; [low_deg, high_deg] is the range still possible."""

    deg: float | None
    low_deg: float
    high_deg: float
    anchored: bool
    source: str  # "estimated", "reported" (the drone sent it) or "unknown"

    def as_dict(self) -> dict[str, Any]:
        data = asdict(self)
        return {
            key: _round(value, 1) if isinstance(value, float) else value
            for key, value in data.items()
        }


@dataclass(frozen=True)
class Telemetry:
    """One reading of the drone's state, as the SDK publishes it (10 Hz)."""

    time: float  # unix seconds
    connected: bool  # Bluetooth link up
    link_age_s: float | None  # since the last Bluetooth message
    mode: str | None  # e.g. "MANUAL"
    flight_state: str | None  # e.g. "pre_takeoff", "taking_off", "flying_manual", "idle"
    in_flight: bool
    battery_pct: int | None
    position: Vector | None  # raw odometry, takeoff frame, z down (1 Hz)
    velocity: Vector | None
    odometry_time: float | None  # unix seconds of the last odometry sample
    camera_tilt_deg: float | None  # the X1 has never reported it
    manual_session: bool
    sticks_gate: str  # "no session", "session-neutral" or "live"
    stick_output: dict[str, float]  # what the stick sender sends now, -100..100, our signs
    video_age_s: float | None  # since the last decoded frame; None before the first
    pose: Pose | None = None  # the estimator's belief, filled in by the Drone
    tilt: Tilt | None = None  # the camera tilt estimate, filled in by the Drone

    @property
    def odometry_age_s(self) -> float | None:
        return None if self.odometry_time is None else max(0.0, self.time - self.odometry_time)

    @property
    def altitude_m(self) -> float | None:
        if not self.in_flight or self.position is None:
            return None
        return -self.position[2]

    def as_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["pose"] = self.pose.as_dict() if self.pose else None
        data["tilt"] = self.tilt.as_dict() if self.tilt else None
        data["odometry_age_s"] = _round(self.odometry_age_s)
        data["altitude_m"] = _round(self.altitude_m)
        return data


def _round(value: float | None, digits: int = 3) -> float | None:
    return None if value is None else round(value, digits)


def telemetry_from_snapshot(
    snapshot: dict[str, Any] | None,
    sticks: Any,
    video_age_s: float | None,
) -> Telemetry:
    """Build a Telemetry from `DroneLink.snapshot()` (or the simulator's, same shape).

    `snapshot` None means no Bluetooth link at all. `sticks` is the StickSender.
    """
    now = time.time()
    output = asdict(sticks.output)
    if snapshot is None:
        return Telemetry(
            now, False, None, None, None, False, None, None, None, None, None, False,
            sticks.gate_state, output, video_age_s,
        )  # fmt: skip
    odometry = snapshot["odometry"]
    position = odometry["position"]
    velocity = odometry["velocity"]
    return Telemetry(
        time=now,
        connected=snapshot["connection"]["connected"],
        link_age_s=snapshot["connection"]["last_message_age_s"],
        mode=snapshot["mode"]["name"],
        flight_state=snapshot["flight_state"]["value"],
        in_flight=snapshot["flight_state"]["in_flight"],
        battery_pct=snapshot["battery"]["soc"],
        position=_vector(position),
        velocity=_vector(velocity),
        odometry_time=odometry["ts"] if position else None,
        camera_tilt_deg=snapshot["camera_tilt"]["degrees"],
        manual_session=snapshot["manual_entry"]["session"],
        sticks_gate=sticks.gate_state,
        stick_output=output,
        video_age_s=video_age_s,
    )


def _vector(xyz: dict[str, float] | None) -> Vector | None:
    return None if xyz is None else (xyz["x"], xyz["y"], xyz["z"])


T = TypeVar("T")


class Subscription(Generic[T]):
    """A bounded queue of published items; close() it when done."""

    def __init__(self, owner: "Broadcaster[T]") -> None:
        self._owner = owner
        self._queue: queue.Queue[T] = queue.Queue(STREAM_QUEUE_SIZE)

    def get(self, timeout: float | None = None) -> T | None:
        """The next item, or None after `timeout` seconds without one."""
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def put(self, item: T) -> None:
        while True:
            try:
                self._queue.put_nowait(item)
                return
            except queue.Full:  # a slow reader loses the oldest items, never blocks the source
                with contextlib.suppress(queue.Empty):
                    self._queue.get_nowait()

    def close(self) -> None:
        self._owner.unsubscribe(self)


class Broadcaster(Generic[T]):
    """Fan one stream out to callbacks (run on the publishing thread: keep them quick) and to
    any number of iterators."""

    def __init__(self, name: str) -> None:
        self._name = name
        self._lock = threading.Lock()
        self._callbacks: list[Callable[[T], None]] = []
        self._subscriptions: list[Subscription[T]] = []

    def on(self, callback: Callable[[T], None]) -> Callable[[], None]:
        """Call `callback(item)` for every item; returns a function that unsubscribes."""
        with self._lock:
            self._callbacks.append(callback)

        def remove() -> None:
            with self._lock:
                if callback in self._callbacks:
                    self._callbacks.remove(callback)

        return remove

    def subscribe(self) -> Subscription[T]:
        subscription = Subscription(self)
        with self._lock:
            self._subscriptions.append(subscription)
        return subscription

    def unsubscribe(self, subscription: Subscription[T]) -> None:
        with self._lock:
            if subscription in self._subscriptions:
                self._subscriptions.remove(subscription)

    def stream(self, timeout: float | None = None) -> Iterator[T]:
        """Iterate over new items; ends after `timeout` seconds without one (None: never)."""
        subscription = self.subscribe()
        try:
            while (item := subscription.get(timeout)) is not None:
                yield item
        finally:
            subscription.close()

    def publish(self, item: T) -> None:
        with self._lock:
            callbacks = list(self._callbacks)
            subscriptions = list(self._subscriptions)
        for subscription in subscriptions:
            subscription.put(item)
        for callback in callbacks:
            try:
                callback(item)
            except Exception:  # a user callback must not stop the stream
                print(f"{self._name} callback failed:\n{traceback.format_exc()}", flush=True)
