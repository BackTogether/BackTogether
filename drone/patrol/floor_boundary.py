"""Front distance and wall direction from where the floor ends in the image.

With the camera tilted down, every column of a depth map runs over the floor (depth grows
going up the image) until it meets whatever stands on the floor: a wall, a person, a table leg
(depth there grows going *down* the image, since lower rows hit the obstacle further from the
camera). So the floor-obstacle contact is the farthest point of the column: its argmax. This
only uses the model's depth *ordering*, never its metric scale, which on our recordings was off
by 1.3-2x and unstable on plain walls (see README "Front distance").

The contact row then gives a floor point by geometry alone: a ray `beta` degrees below level
meets the floor at height_m / tan(beta). Fitting a line through the contact points of all
columns gives the wall's direction (scale free: the height only scales the line, not its
angle), so the patrol can turn by the reflection angle.

Column cases: FLOOR (contact found, the rows under it look like a floor plane), NEAR (no floor
under the obstacle: it is closer than the floor seen by the bottom row), CLEAR (floor up to the
top of the image), UNKNOWN (the profile looks like neither).
"""

import math
from dataclasses import dataclass
from enum import Enum

import numpy as np

from drone.patrol.front_distance import camera_rays

# The geometry's field of view: f = 850 px at 1280 px. On the 2026-11-14 wall approach the
# camera tilt from vertical edges (vanishing point), the tilt that makes the floor boundary
# match the odometry travel, and the commanded -30 deg agree only for f = 850-880 px; the
# 627 px of drone/sdk/camera.py (yaw spins) would need the camera at 0.93 m, odometry said 1.72.
GEOMETRY_HFOV_DEG = 74.0
CORRIDOR_HALF_WIDTH_M = 0.5  # the drone's cage plus margin, each side of its path
MIN_FLOOR_ROWS = 0.04  # of the image height: fewer floor rows under the contact = NEAR
EDGE_ROWS = 0.03  # contact this close to the top of the image = floor all the way (CLEAR)
FLOOR_PLANE_TOLERANCE = 0.25  # relative spread of the floor's height along a column
MIN_RAY_DOWN = 0.02  # rays this close to level never meet the floor in practice
FRONT_PERCENTILE = 10  # of the contact distances inside the corridor (robust to stray columns)
MIN_KNOWN_FRACTION = 0.5  # corridor columns that must be FLOOR, NEAR or CLEAR
MIN_NEAR_FRACTION = 0.3  # corridor columns NEAR -> the obstacle is closer than near_m
# Wall line fit (RANSAC on the floor contact points)
RANSAC_ITERATIONS = 60
INLIER_BASE_M = 0.08
INLIER_PER_M = 0.04  # contact points get noisier with distance
MIN_LINE_POINTS = 12
MIN_LINE_FRACTION = 0.25  # of the contact points
CORNER_MIN_ANGLE_DEG = 30.0  # two lines this far apart = a corner


class Column(Enum):
    FLOOR = "floor"
    NEAR = "near"
    CLEAR = "clear"
    UNKNOWN = "unknown"


class Status(Enum):
    CLEAR = "clear"  # floor up to the top of the image in the corridor
    WALL = "wall"  # obstacle contact seen at front_m
    NEAR = "near"  # obstacle closer than near_m (its floor contact is under the image)
    UNKNOWN = "unknown"  # too few readable columns: do not trust this frame


@dataclass(frozen=True)
class Wall:
    """A straight floor-contact line. `incidence_deg`: angle between the drone's heading and
    the wall's normal, + when the wall recedes to the right (0 = head-on)."""

    incidence_deg: float
    distance_m: float  # along the heading, where the line crosses it
    points: int


@dataclass(frozen=True)
class WallReading:
    status: Status
    front_m: float | None  # WALL: contact distance in the corridor; NEAR: near_m; else None
    near_m: float  # floor distance of the bottom row: nearer contacts are not visible
    far_m: float | None  # floor distance of the top row (None: it sees above the horizon)
    walls: tuple[Wall, ...]  # 0, 1 or 2 (a corner) lines, the nearest first
    known_fraction: float  # corridor columns that could be read

    def summary(self) -> str:
        front = "-" if self.front_m is None else f"{self.front_m:4.2f} m"
        walls = " ".join(f"{w.incidence_deg:+.0f}deg@{w.distance_m:.1f}m" for w in self.walls)
        return (
            f"{self.status.value:7s} front={front} (sees {self.near_m:.1f}-"
            f"{'inf' if self.far_m is None else f'{self.far_m:.0f}'} m, "
            f"known {self.known_fraction:.0%}) walls[{walls}]"
        )


def floor_distance(height_m: float, pitch_deg: float, y: float) -> float | None:
    """Horizontal distance where the ray of normalised image row `y` meets the floor."""
    pitch = math.radians(pitch_deg)
    down = math.sin(pitch) + math.cos(pitch) * y
    if down <= MIN_RAY_DOWN:
        return None
    return height_m * (math.cos(pitch) - math.sin(pitch) * y) / down


def classify_columns(
    depth: np.ndarray, pitch_deg: float, y: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """(contact row of each column, Column value of each column); `y`: normalised image row
    of every pixel (camera_rays)."""
    rows = depth.shape[0]
    contact = np.argmax(depth, axis=0)
    kinds = np.full(depth.shape[1], Column.UNKNOWN.value, dtype=object)
    min_floor = max(2, round(MIN_FLOOR_ROWS * rows))
    kinds[contact >= rows - min_floor] = Column.NEAR.value
    kinds[contact <= EDGE_ROWS * rows] = Column.CLEAR.value
    floor_height = _relative_floor_height(depth, pitch_deg, y)
    below = np.arange(rows)[:, None] > contact[None, :]
    floor_height = np.where(below, floor_height, np.nan)
    enough = np.isfinite(floor_height).sum(axis=0) >= min_floor
    if not enough.any():
        return contact, kinds
    with np.errstate(all="ignore"):
        low, mid, high = np.nanpercentile(floor_height[:, enough], [10, 50, 90], axis=0)
    flat = np.zeros(len(contact), dtype=bool)
    flat[enough] = (high - low) <= FLOOR_PLANE_TOLERANCE * mid
    kinds[flat & (kinds == Column.UNKNOWN.value)] = Column.FLOOR.value
    return contact, kinds


def _relative_floor_height(depth: np.ndarray, pitch_deg: float, y: np.ndarray) -> np.ndarray:
    """Height of each pixel's 3D point below the camera, in the model's (unknown) scale: on the
    floor it is the same for every pixel."""
    pitch = math.radians(pitch_deg)
    down = np.sin(pitch) + np.cos(pitch) * y
    return np.where(down > MIN_RAY_DOWN, depth * down, np.nan)


def measure_wall(
    depth: np.ndarray,
    height_m: float,
    pitch_deg: float,
    hfov_deg: float,
    corridor_half_m: float = CORRIDOR_HALF_WIDTH_M,
) -> WallReading:
    """Front obstacle and wall lines from a depth map (any scale); camera `pitch_deg` below
    level at `height_m` above the floor."""
    rows, cols = depth.shape
    x, y = camera_rays(cols, rows, hfov_deg)
    xs = x[0]
    contact, kinds = classify_columns(depth, pitch_deg, y)
    near_m = floor_distance(height_m, pitch_deg, float(y[-1, 0])) or 0.0
    far_m = floor_distance(height_m, pitch_deg, float(y[0, 0]))

    forward = np.full(cols, np.nan)
    right = np.full(cols, np.nan)
    floor = kinds == Column.FLOOR.value
    for column in np.flatnonzero(floor):
        row_y = float(y[contact[column], 0])
        distance = floor_distance(height_m, pitch_deg, row_y)
        if distance is None:
            kinds[column] = Column.CLEAR.value
            continue
        forward[column] = distance
        right[column] = xs[column] * _ray_length(pitch_deg, row_y, distance)
    floor = kinds == Column.FLOOR.value

    corridor = _corridor_columns(xs, kinds, right, near_m, corridor_half_m, pitch_deg, y)
    known = np.isin(kinds[corridor], [k.value for k in (Column.FLOOR, Column.NEAR, Column.CLEAR)])
    known_fraction = float(known.mean()) if corridor.any() else 0.0
    walls = fit_walls(forward[floor], right[floor])
    status, front = _front(kinds[corridor], forward[corridor], near_m, known_fraction)
    return WallReading(status, front, near_m, far_m, walls, known_fraction)


def _ray_length(pitch_deg: float, y: float, forward_m: float) -> float:
    """Camera-z depth of the floor point at `forward_m` on row `y` (x * this = lateral m)."""
    pitch = math.radians(pitch_deg)
    return forward_m / (math.cos(pitch) - math.sin(pitch) * y)


def _corridor_columns(
    xs: np.ndarray,
    kinds: np.ndarray,
    right: np.ndarray,
    near_m: float,
    half_m: float,
    pitch_deg: float,
    y: np.ndarray,
) -> np.ndarray:
    """Columns whose obstacle (or, without one, the floor at the bottom row) is within the
    corridor laterally."""
    bottom_depth = _ray_length(pitch_deg, float(y[-1, 0]), near_m)
    lateral = np.where(np.isfinite(right), right, xs * bottom_depth)
    return np.abs(lateral) <= half_m


def _front(
    kinds: np.ndarray, forward: np.ndarray, near_m: float, known_fraction: float
) -> tuple[Status, float | None]:
    if known_fraction < MIN_KNOWN_FRACTION:
        return Status.UNKNOWN, None
    if np.mean(kinds == Column.NEAR.value) >= MIN_NEAR_FRACTION:
        return Status.NEAR, near_m
    contacts = forward[kinds == Column.FLOOR.value]
    if len(contacts) == 0:
        return Status.CLEAR, None
    return Status.WALL, float(np.percentile(contacts, FRONT_PERCENTILE))


def fit_walls(forward: np.ndarray, right: np.ndarray) -> tuple[Wall, ...]:
    """One line through the contact points, or two (a corner); the nearest first."""
    points = np.column_stack([right, forward])
    first, inliers = _ransac_line(points)
    if first is None:
        return ()
    walls = [first]
    rest = points[~inliers]
    second, _ = _ransac_line(rest)
    if second and abs(second.incidence_deg - first.incidence_deg) >= CORNER_MIN_ANGLE_DEG:
        walls.append(second)
    return tuple(sorted(walls, key=lambda wall: wall.distance_m))


def _ransac_line(points: np.ndarray) -> tuple[Wall | None, np.ndarray]:
    """forward = a + b * right through `points` (columns: right, forward)."""
    count = len(points)
    none = np.zeros(count, dtype=bool)
    if count < MIN_LINE_POINTS:
        return None, none
    rng = np.random.default_rng(0)  # deterministic: same frame, same answer
    tolerance = INLIER_BASE_M + INLIER_PER_M * points[:, 1]
    best = none
    for _ in range(RANSAC_ITERATIONS):
        i, j = rng.choice(count, 2, replace=False)
        (r1, f1), (r2, f2) = points[i], points[j]
        if abs(r2 - r1) < 1e-6:
            continue
        slope = (f2 - f1) / (r2 - r1)
        residual = np.abs(points[:, 1] - (f1 + slope * (points[:, 0] - r1)))
        inliers = residual <= tolerance * math.sqrt(1 + slope * slope)
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < max(MIN_LINE_POINTS, MIN_LINE_FRACTION * count):
        return None, best
    slope, intercept = np.polyfit(points[best, 0], points[best, 1], 1)
    wall = Wall(math.degrees(math.atan(slope)), float(intercept), int(best.sum()))
    return wall, best


def reflection_turn_deg(walls: tuple[Wall, ...]) -> float:
    """Turn (deg, + = right) that sends the heading off the wall(s) like a billiard ball:
    angle of incidence = angle of reflection; off a corner, off both walls in turn."""
    heading = np.array([1.0, 0.0])  # (forward, right)
    for wall in walls:
        slope = math.tan(math.radians(wall.incidence_deg))
        normal = np.array([1.0, -slope]) / math.hypot(1.0, slope)
        if heading @ normal > 0:  # only bounce off a wall we are flying into
            heading = heading - 2 * (heading @ normal) * normal
    return math.degrees(math.atan2(heading[1], heading[0]))
