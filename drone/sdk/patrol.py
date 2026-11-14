"""`Patrol`: fly waypoints in the takeoff frame, in a loop, with a dwell and an optional 360 scan
at each one, and a callback per waypoint with the latest frame. Safety rules come from the
Navigator (safety.py): STOP ends the patrol at once, low battery or a geofence breach brings
the drone home to land, a link/telemetry/video loss hovers, then lands.
"""

import threading
import time
import traceback
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass
from typing import Any

from .navigate import HOME, Navigator
from .telemetry import Pose

DEFAULT_DWELL_S = 2.0


@dataclass(frozen=True)
class Waypoint:
    """A point in the takeoff frame (metres; x, y as in the odometry, alt up). None fields use
    the patrol's options (alt None: the altitude when the patrol starts)."""

    x: float
    y: float
    alt: float | None = None
    dwell_s: float | None = None
    scan: bool | None = None


@dataclass(frozen=True)
class PatrolOptions:
    loops: int = 1  # 0 = until STOP or a safety rule
    dwell_s: float = DEFAULT_DWELL_S
    scan: bool = False  # turn 360 degrees at every waypoint
    return_home: bool = True  # fly back to the takeoff point at the end
    land: bool = False  # and land there


WaypointCallback = Callable[[int, Waypoint, Pose, bytes | None], None]


class Patrol:
    """Build it, then run() (blocking). `progress()` is safe from any thread."""

    def __init__(
        self,
        navigator: Navigator,
        waypoints: Sequence[Waypoint],
        options: PatrolOptions | None = None,
        on_waypoint: WaypointCallback | None = None,
    ) -> None:
        """Raises ValueError if there are no waypoints or one is outside the safety limits.
        `on_waypoint(index, waypoint, pose, jpeg)` runs on arrival, before the dwell."""
        if not waypoints:
            raise ValueError("a patrol needs at least one waypoint")
        self.navigator = navigator
        self.waypoints = list(waypoints)
        self.options = options or PatrolOptions()
        self.on_waypoint = on_waypoint
        self._lock = threading.Lock()
        self._progress: dict[str, Any] = {"state": "ready", "loop": 0, "index": None, "visits": 0}
        default_alt = self._default_alt()
        for waypoint in self.waypoints:
            alt = default_alt if waypoint.alt is None else waypoint.alt
            navigator.limits.check_target(waypoint.x, waypoint.y, alt)

    def progress(self) -> dict[str, Any]:
        with self._lock:
            return {
                **self._progress,
                "waypoints": [asdict(waypoint) for waypoint in self.waypoints],
                "options": asdict(self.options),
            }

    def run(self) -> str:
        """Fly the patrol; returns how it ended. STOP raises Interrupted, a safety landing
        raises SafetyLanding; a ReturnHome rule flies home, lands and returns its reason."""
        try:
            outcome = self.navigator.run_guarded(self._fly)
        except BaseException as error:
            self._set(state=f"ended: {type(error).__name__}: {error}")
            raise
        self._set(state=outcome)
        return outcome

    def _default_alt(self) -> float:
        """The altitude now, inside the limits: used where a waypoint gives none."""
        return self.navigator.limits.clamp_alt(self.navigator.drone.pose().alt)

    def _fly(self) -> str:
        alt = self._default_alt()
        loop = 0
        while self.options.loops == 0 or loop < self.options.loops:
            loop += 1
            for index, waypoint in enumerate(self.waypoints):
                self._visit(loop, index, waypoint, alt)
        if self.options.return_home:
            self._set(state="returning home", index=None)
            self.navigator.goto(*HOME, alt=alt)
        if self.options.land:
            self._set(state="landing")
            result = self.navigator.drone.land()
            if not result["ok"]:
                return f"done, land failed: {result['error']}"
        return f"done: {loop} loop(s)"

    def _visit(self, loop: int, index: int, waypoint: Waypoint, default_alt: float) -> None:
        self._set(state="flying", loop=loop, index=index)
        alt = default_alt if waypoint.alt is None else waypoint.alt
        pose = self.navigator.goto(waypoint.x, waypoint.y, alt)
        with self._lock:
            self._progress["visits"] += 1
        self._set(state="arrived")
        self._report(index, waypoint, pose)
        dwell = self.options.dwell_s if waypoint.dwell_s is None else waypoint.dwell_s
        if dwell > 0:
            self._set(state="dwelling")
            self.navigator.hover(dwell)
        scan = self.options.scan if waypoint.scan is None else waypoint.scan
        if scan:
            self._set(state="scanning")
            self.navigator.scan()

    def _report(self, index: int, waypoint: Waypoint, pose: Pose) -> None:
        if not self.on_waypoint:
            return
        try:
            self.on_waypoint(index, waypoint, pose, self.navigator.drone.latest_frame())
        except Exception:  # a user callback must not stop the patrol
            print(f"patrol: on_waypoint failed:\n{traceback.format_exc()}", flush=True)

    def _set(self, **changes: Any) -> None:
        with self._lock:
            self._progress.update(changes, updated_at=time.time())
