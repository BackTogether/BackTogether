"""What the page's map view shows (GET /map/state, API.md "Map view"), in one frame.

The frame is the robot's `map` (x, y metres, yaw radians counter-clockwise, ROS) once the
mission has aligned the drone with it (`map_from_drone`: launch pose + aligning leg); before
that the drone is drawn in its takeoff frame with ROS axes (x, -y of the drone frame, as
ros_node's "drone_takeoff"), and the map-only layers (grid, splat, robot) wait. With the drone
on the ground and something to show in `map`, the view stays in `map` without the drone.

Layers: the drone (pose, path, patrol waypoints, fence, target), the child and the robot's goal
(mission), the robot (its live tf pose from ros_node at ~2 Hz, POST /map/robot_pose, and its
pose at launch), the occupancy grid (map_image.py) and the splat (splat.py).
"""

import math
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from .map_image import GridLayer
from .mission_geometry import LaunchPose
from .splat import Registration, SplatLayer
from .telemetry import Pose

FRAME_MAP = "map"
FRAME_TAKEOFF = "takeoff"
ROBOT_STALE_S = 3.0  # ros_node posts at 2 Hz: older than this is shown as stale
MAX_PATH_POINTS = 400
COORD_DIGITS = 3


@dataclass(frozen=True)
class RobotPose:
    x: float
    y: float
    yaw: float
    received_at: float


class ViewFrame:
    """Drone-frame points and headings in the view frame (x right, y up, yaw counter-clockwise):
    `map` through the mission's map_from_drone, else the ROS takeoff frame."""

    def __init__(self, map_from_drone: dict[str, float] | None) -> None:
        self.name = FRAME_MAP if map_from_drone else FRAME_TAKEOFF
        transform = map_from_drone or {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.x, self.y, self.yaw_offset = transform["x"], transform["y"], transform["yaw"]

    def point(self, x: float, y: float) -> list[float]:
        fwd, left = x, -y  # drone frame (y clockwise) -> ROS takeoff frame
        cos, sin = math.cos(self.yaw_offset), math.sin(self.yaw_offset)
        return [
            round(self.x + fwd * cos - left * sin, COORD_DIGITS),
            round(self.y + fwd * sin + left * cos, COORD_DIGITS),
        ]

    def yaw(self, heading_deg: float) -> float:
        return round(self.yaw_offset - math.radians(heading_deg), 4)


class MapView:
    """The layers set from outside (grid, splat, robot pose) plus state() for the page."""

    def __init__(
        self,
        grid: GridLayer | None = None,
        splat: SplatLayer | None = None,
        splat_note: str | None = None,
        splat_path: Path | None = None,
        registration: Registration | None = None,
    ) -> None:
        self._lock = threading.Lock()
        self._grid = grid
        self._grid_version = 1 if grid else 0
        self._robot: RobotPose | None = None
        self.splat = splat
        self.splat_note = splat_note  # why there is no splat layer, if asked for one
        self.splat_path = splat_path
        self.registration = registration or Registration()

    # --- inputs ---------------------------------------------------------------------------------

    def set_grid(self, grid: GridLayer) -> None:
        with self._lock:
            self._grid = grid
            self._grid_version += 1

    def grid_png(self) -> bytes | None:
        with self._lock:
            return self._grid.png if self._grid else None

    def has_grid(self) -> bool:
        with self._lock:
            return self._grid is not None

    def set_robot_pose(self, x: float, y: float, yaw: float) -> None:
        with self._lock:
            self._robot = RobotPose(x, y, yaw, time.time())

    def set_registration(self, registration: Registration, save: bool) -> Path | None:
        """Use `registration` for the splat; with `save`, write it next to the splat file."""
        with self._lock:
            self.registration = registration
        if not save or self.splat_path is None:
            return None
        path = Registration.path_for(self.splat_path)
        registration.save(path)
        return path

    # --- GET /map/state ---------------------------------------------------------------------------

    def state(
        self,
        mission: dict[str, Any],
        pose: Pose,
        path: list[tuple[float, float]],
        waypoints: list[tuple[float, float]],
        fence: tuple[float, float, float, float],
        target: list[float] | None,
    ) -> dict[str, Any]:
        """Everything in one frame. `mission` is Mission.status(); `path`, `waypoints`, `fence`
        (x_min, x_max, y_min, y_max) and `target` are in the drone frame."""
        with self._lock:
            grid, version, robot = self._grid, self._grid_version, self._robot
            registration = self.registration
        view = ViewFrame(mission.get("map_from_drone"))
        launch = mission.get("launch_pose")
        map_content = bool(grid or self.splat or robot or launch)
        frame = view.name
        if frame == FRAME_TAKEOFF and not pose.in_flight and map_content:
            frame = FRAME_MAP  # nothing flies yet: show the map, without the drone
        drone_drawn = frame == view.name
        state = {
            "frame": frame,
            "aligned": view.name == FRAME_MAP,
            "note": _note(frame, view.name, map_content),
            "drone": _drone(pose, view) if drone_drawn else None,
            "path": [view.point(x, y) for x, y in _thin(path)] if drone_drawn else [],
            "waypoints": [view.point(x, y) for x, y in waypoints] if drone_drawn else [],
            "fence": _fence(fence, view) if drone_drawn else None,
            "target": view.point(*target[:2]) if target and drone_drawn else None,
            "takeoff": view.point(0.0, 0.0) if drone_drawn else _launch_xy(launch),
            "planned": _planned(mission, frame),
            **_child(mission.get("child"), frame),
            "robot": _robot(robot) if frame == FRAME_MAP else None,
            "robot_launch": launch if frame == FRAME_MAP else None,
            "grid": {**grid.meta(), "version": version} if grid else None,
            "splat": self._splat_meta(registration),
            "updated_at": time.time(),
        }
        return state

    def _splat_meta(self, registration: Registration) -> dict[str, Any] | None:
        if self.splat is None and self.splat_note is None:
            return None
        meta: dict[str, Any] = {"available": self.splat is not None, "note": self.splat_note}
        if self.splat:
            meta.update(self.splat.meta())
        meta["registration"] = asdict(registration)
        if self.splat_path:
            meta["registration_file"] = str(Registration.path_for(self.splat_path))
        return meta


def _note(frame: str, aligned_frame: str, map_content: bool) -> str | None:
    if frame == FRAME_TAKEOFF and map_content:
        return "takeoff frame: the map layers appear after the launch pose and the aligning leg"
    if frame == FRAME_MAP and aligned_frame == FRAME_TAKEOFF:
        return "drone not aligned with the map yet"
    if frame == FRAME_TAKEOFF:
        return "takeoff frame (no map yet)"
    return None


def _drone(pose: Pose, view: ViewFrame) -> dict[str, Any]:
    x, y = view.point(pose.x, pose.y)
    return {
        "x": x,
        "y": y,
        "yaw": view.yaw(pose.heading_deg),
        "alt": round(pose.alt, 2),
        "sigma_m": round(pose.position_sigma_m, 2),
        "in_flight": pose.in_flight,
    }


def _fence(fence: tuple[float, float, float, float], view: ViewFrame) -> list[list[float]]:
    x_min, x_max, y_min, y_max = fence
    corners = ((x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max))
    return [view.point(x, y) for x, y in corners]


def _thin(path: list[tuple[float, float]]) -> list[tuple[float, float]]:
    step = max(1, math.ceil(len(path) / MAX_PATH_POINTS))
    return path[::step]


def _launch_xy(launch: dict[str, float] | None) -> list[float] | None:
    return [launch["x"], launch["y"]] if launch else None


def _planned(mission: dict[str, Any], frame: str) -> list[list[float]]:
    """The mission's configured waypoints in `map`, before a patrol flies them: given in
    `map`, or in `launch` with the robot's launch pose."""
    config, launch = mission.get("config"), mission.get("launch_pose")
    if frame != FRAME_MAP or not config:
        return []
    points = [(w["x"], w["y"]) for w in config.get("waypoints") or []]
    if config.get("frame") == FRAME_MAP:
        return [[x, y] for x, y in points]
    if config.get("frame") == "launch" and launch:
        pose = LaunchPose(**launch)
        return [[round(v, COORD_DIGITS) for v in pose.map_from_launch(x, y)] for x, y in points]
    return []


def _child(child: dict[str, Any] | None, frame: str) -> dict[str, Any]:
    """The child (and the robot's goal in `map`) in the view frame, or None."""
    if not child:
        return {"child": None, "goal": None}
    if frame == FRAME_MAP:
        where = child.get("map")
    else:
        where = {"x": child["drone"]["x"], "y": -child["drone"]["y"]}  # ROS takeoff axes
    if not where:
        return {"child": None, "goal": None}
    found = {"x": where["x"], "y": where["y"], "id": child.get("id"), "time": child.get("time")}
    return {"child": found, "goal": child.get("goal") if frame == FRAME_MAP else None}


def _robot(robot: RobotPose | None) -> dict[str, Any] | None:
    if robot is None:
        return None
    age = time.time() - robot.received_at
    return {
        "x": robot.x,
        "y": robot.y,
        "yaw": robot.yaw,
        "age_s": round(age, 2),
        "stale": age > ROBOT_STALE_S,
    }
