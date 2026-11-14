"""The patrol's view of the drone, through drone.sdk (it replaces the old hoverweb.py client).

Two ways to reach it, same methods:
- `RemoteDrone`: the SDK server over HTTP (drone/fly, `python -m drone.sdk serve` or `sim`),
  on this machine or another one. Stdlib only.
- `LocalDrone`: a `drone.sdk.Drone` in this process (the real drone, or the simulator).

Either way the SDK's safety applies to every stick command: STOP pre-empts the patrol, a
link/telemetry/video loss hovers then lands, leaving the geofence or a low battery returns to
the takeoff point and lands, and `max_speed` caps the sticks. Remote: the server's limits
(`serve --fence --max-alt`); local: the ones given to LocalDrone.
"""

import time
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from typing import Any

from drone.sdk import Drone, Navigator, SafetyLimits
from drone.sdk.client import DroneApiError, DroneClient

DEFAULT_URL = "http://127.0.0.1:8077"  # drone/fly
STICK_HOLD_S = 0.3  # each command lapses after this (the patrol sends every 100 ms)
TILT_POLL_S = 0.5
STOPPED_PREFIX = "stopped"  # the server's 409 after a STOP


class Refused(Exception):
    """The SDK refused the sticks for now (returning home, riding out a loss, busy)."""


class Stopped(Refused):
    """STOP (button, LAND, a pilot key, the API): the patrol must end."""


@dataclass(frozen=True)
class DroneState:
    """The fields the patrol needs; None when unknown."""

    flight_state: str | None
    in_flight: bool
    mode: str | None
    battery_pct: int | None
    position: tuple[float, float, float] | None  # odometry, z down, origin at takeoff
    altitude_m: float | None  # above the takeoff point (the SDK's estimate)
    tilt_deg: float | None  # camera tilt estimate, 0 = level, -90 = down; None = unknown
    pose_xy: tuple[float, float] | None = None  # the estimator's position, extrapolated
    heading_deg: float | None = None  # the estimator's heading (video yaw where trusted)
    yaw_deg: float | None = None  # measured heading (telemetry "attitude"), when the SDK has it


def _state(telemetry: dict[str, Any]) -> DroneState:
    """From the SDK's telemetry JSON (Telemetry.as_dict(), or /status["telemetry"])."""
    pose, tilt = telemetry.get("pose") or {}, telemetry.get("tilt") or {}
    position = telemetry.get("position")
    in_flight = bool(telemetry.get("in_flight"))
    attitude = telemetry.get("attitude") or {}
    return DroneState(
        flight_state=telemetry.get("flight_state"),
        in_flight=in_flight,
        mode=telemetry.get("mode"),
        battery_pct=telemetry.get("battery_pct"),
        position=None if position is None else tuple(position),
        altitude_m=pose.get("alt") if in_flight else None,
        tilt_deg=tilt.get("deg"),
        pose_xy=(pose["x"], pose["y"]) if "x" in pose else None,
        heading_deg=pose.get("heading_deg"),
        yaw_deg=attitude.get("yaw_deg"),
    )


def _stick_cap(speed_cap: float, max_speed: float) -> float:
    """Stick % that an input of 1.0 really sends: the speed cap, within the SDK's max_speed."""
    return min(speed_cap, max_speed)


class RemoteDrone:
    def __init__(self, base_url: str = DEFAULT_URL) -> None:
        self.client = DroneClient(base_url)
        self._resume = True  # the patrol's first command takes control after an earlier STOP

    def state(self) -> DroneState:
        return _state(self.client.status()["telemetry"])

    def frames(self) -> Iterator[bytes]:
        return self.client.frames()

    def send_sticks(
        self, pitch: float = 0.0, roll: float = 0.0, thrust: float = 0.0, yaw: float = 0.0
    ) -> None:
        """Axes in -1..1 with hoverctl's signs: + forward, right, up, turn right. Raises
        Stopped / Refused when the SDK says no, OSError when it is unreachable."""
        try:
            self.client.move(pitch, roll, thrust, yaw, STICK_HOLD_S, resume=self._resume)
        except DroneApiError as error:
            if error.status == 0:
                raise OSError(error.message) from error
            if error.message.startswith(STOPPED_PREFIX):
                raise Stopped(error.message) from error
            raise Refused(error.message) from error
        self._resume = False

    def run(self, loop: Callable[[], str]) -> str:
        """Run the patrol loop; the server applies the safety rules to its sticks."""
        return loop()

    def stop(self) -> None:
        self.client.stop()

    def land(self) -> dict[str, Any]:
        return self.client.land()

    def set_tilt(self, deg: float) -> float | None:
        """Move the camera to `deg` (re-anchoring the estimate if unknown); the tilt reached."""
        self.client.tilt(deg)
        while (task := self.client.status()["task"])["state"] == "running":
            time.sleep(TILT_POLL_S)
        # STOP during the tilt must end the patrol: its first /move resumes after a STOP.
        if task["state"] == "stopped":
            raise Stopped(f"stopped while tilting the camera: {task['message']}")
        return self.client.camera_tilt()["deg"]

    def yaw_rate_dps(self, yaw_input: float) -> float:
        """Turn rate at `yaw_input` (0..1), from the SDK estimator's calibrated rate."""
        status = self.client.status()
        full_stick = status["estimator"]["yaw_rate_dps"]
        cap = _stick_cap(status["speed"]["max"], status["limits"]["max_speed"])
        return full_stick * yaw_input * cap / 100

    def fake_front_m(self) -> float | None:
        """Simulator only: the true distance to the wall ahead."""
        return self.client.sim_truth()["front_m"]

    def sim_truth(self) -> dict[str, float] | None:
        """Simulator only: the true pose (takeoff frame), front distance and wall hits."""
        return self.client.sim_truth()


class LocalDrone:
    """A drone.sdk Drone in this process; sticks go through its Navigator's safety rules."""

    def __init__(self, drone: Drone, limits: SafetyLimits) -> None:
        self.drone = drone
        self.navigator = Navigator(drone, limits)
        self._epoch = drone.epoch

    def state(self) -> DroneState:
        return _state(self.drone.telemetry().as_dict())

    def frames(self) -> Iterator[bytes]:
        return self.drone.frames()

    def send_sticks(
        self, pitch: float = 0.0, roll: float = 0.0, thrust: float = 0.0, yaw: float = 0.0
    ) -> None:
        """Safety rules first (they may hover, return home or raise Interrupted on STOP)."""
        self.navigator.guard(self._epoch)
        self.navigator.send(self._epoch, fwd=pitch, right=roll, up=thrust, turn=yaw)

    def run(self, loop: Callable[[], str]) -> str:
        """Run the patrol loop under the safety rules: a ReturnHome rule ends it with the drone
        flown home and landed (the outcome is returned); STOP raises Interrupted."""
        self.navigator.require_ready()
        self._epoch = self.drone.epoch
        return str(self.navigator.run_guarded(loop))

    def stop(self) -> None:
        self.drone.stop()

    def land(self) -> dict[str, Any]:
        return self.drone.land()

    def set_tilt(self, deg: float) -> float | None:
        return self.drone.set_tilt(deg).deg

    def yaw_rate_dps(self, yaw_input: float) -> float:
        full_stick = self.drone.estimator.yaw_rate_dps
        cap = _stick_cap(self.drone.speed_cap, self.navigator.limits.max_speed)
        return full_stick * yaw_input * cap / 100

    def fake_front_m(self) -> float | None:
        truth = self.sim_truth()
        return truth["front_m"] if truth else None

    def sim_truth(self) -> dict[str, float] | None:
        truth = getattr(self.drone.link, "truth", None)
        return truth() if truth else None


PatrolDrone = RemoteDrone | LocalDrone
