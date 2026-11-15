"""Safety limits for the autopilot (Navigator, Patrol, HTTP API).

Rules, checked every control tick (10 Hz):
- STOP always wins: Drone.stop() cancels any autopilot action at once (see drone.py).
- Targets outside the geofence box or the altitude limits are refused before flying.
- Low battery, or the estimated position outside the fence or above the ceiling:
  return to the takeoff point and land (`ReturnHome`).
- Bluetooth link, telemetry or video lost: hover (neutral sticks), resume if it comes back,
  land after `loss_land_s`.
- The autopilot never sends more than `max_speed` (stick %, whatever the user's speed cap).
"""

from dataclasses import asdict, dataclass
from typing import Any

from .telemetry import Pose, Telemetry

FENCE_MARGIN_M = 0.5  # odometry noise: only a breach by more than this counts
ALT_MARGIN_M = 0.3

HAZARD_LOSS = "loss"  # hover, then land if it lasts
HAZARD_RETURN = "return"  # go home and land


class Interrupted(Exception):
    """STOP (or a newer command) cancelled the autopilot action."""


class ReturnHome(Exception):
    """A safety rule wants the drone back at the takeoff point, landed."""


class SafetyLanding(Exception):
    """The drone was landed by a safety rule; the reason is the message."""


class NotReady(Exception):
    """The drone cannot fly the autopilot now (not flying, not MANUAL...)."""


@dataclass(frozen=True)
class Hazard:
    kind: str  # HAZARD_LOSS or HAZARD_RETURN
    reason: str


@dataclass(frozen=True)
class SafetyLimits:
    """Defaults fit a small open area. Takeoff-frame metres; altitude is up."""

    x_min: float = -8.0
    x_max: float = 8.0
    y_min: float = -8.0
    y_max: float = 8.0
    min_alt_m: float = 0.5
    max_alt_m: float = 3.0
    low_battery_pct: int = 25
    link_timeout_s: float = 3.0  # no Bluetooth message for this long
    telemetry_timeout_s: float = 3.5  # odometry is 1 Hz
    video_timeout_s: float = 3.0
    loss_land_s: float = 8.0  # hover this long through a loss, then land
    max_speed: float = 60.0  # stick %, 0..100
    require_video: bool = True

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)

    def check_target(self, x: float, y: float, alt: float) -> None:
        """Raise ValueError if a target is outside the fence or the altitude limits."""
        if not (self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max):
            raise ValueError(
                f"target ({x:.1f}, {y:.1f}) is outside the geofence "
                f"x {self.x_min}..{self.x_max}, y {self.y_min}..{self.y_max}"
            )
        if not self.min_alt_m <= alt <= self.max_alt_m:
            raise ValueError(
                f"altitude {alt:.1f} m is outside {self.min_alt_m}..{self.max_alt_m} m"
            )

    def clamp_alt(self, alt: float) -> float:
        return min(max(alt, self.min_alt_m), self.max_alt_m)

    def hazard(self, telemetry: Telemetry, pose: Pose) -> Hazard | None:
        """The most urgent rule broken now, or None."""
        loss = self._loss(telemetry)
        if loss:
            return Hazard(HAZARD_LOSS, loss)
        battery = telemetry.battery_pct
        if battery is not None and battery <= self.low_battery_pct:
            return Hazard(HAZARD_RETURN, f"low battery {battery}%")
        if self._outside_fence(pose):
            return Hazard(HAZARD_RETURN, f"outside the geofence at ({pose.x:.1f}, {pose.y:.1f})")
        if pose.alt > self.max_alt_m + ALT_MARGIN_M:
            return Hazard(HAZARD_RETURN, f"above the ceiling at {pose.alt:.1f} m")
        return None

    def _loss(self, telemetry: Telemetry) -> str | None:
        link_age = telemetry.link_age_s
        if not telemetry.connected or (link_age is not None and link_age > self.link_timeout_s):
            return "bluetooth link lost"
        odometry_age = telemetry.odometry_age_s
        if odometry_age is None or odometry_age > self.telemetry_timeout_s:
            return "telemetry lost"
        video_age = telemetry.video_age_s
        if self.require_video and (video_age is None or video_age > self.video_timeout_s):
            return "video lost"
        return None

    def _outside_fence(self, pose: Pose) -> bool:
        return not (
            self.x_min - FENCE_MARGIN_M <= pose.x <= self.x_max + FENCE_MARGIN_M
            and self.y_min - FENCE_MARGIN_M <= pose.y <= self.y_max + FENCE_MARGIN_M
        )
