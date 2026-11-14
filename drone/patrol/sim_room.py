"""A depth camera for the simulator's room, to test the wall perception end to end.

Renders the depth map the drone's camera would see from the simulator's true pose: the floor,
the square room's four walls (drone.sdk.sim: +-room_half_m around the start point) and optional
round pillars standing on the floor (people, table legs; the simulator itself does not stop at
them, `pillar_hits` counts close passes). Like the real model, the depth gets a random scale
per frame (our recordings: 1.3-2x off and unstable) and some per-pixel noise, so only what the
real pipeline relies on (the depth ordering) is right.

The simulator must be started with its takeoff frame aligned with the room
(`frame_yaw_offset_deg=0`, `python -m drone.sdk sim --frame-yaw-offset 0`): then its truth()
pose is in room coordinates.
"""

import math
from dataclasses import dataclass

import numpy as np

from drone.patrol.front_distance import camera_rays

RENDER_WIDTH = 196  # like the model's output at half resolution: plenty for the boundary
RENDER_HEIGHT = 112
SCALE_RANGE = (0.6, 2.0)  # random metric scale of each frame
PIXEL_NOISE = 0.02  # relative
FAR_M = 100.0
PILLAR_TOUCH_M = 0.35  # cage radius + pillar surface: closer counts as a hit


@dataclass(frozen=True)
class Pillar:
    x: float
    y: float
    radius_m: float


@dataclass(frozen=True)
class Room:
    half_m: float
    pillars: tuple[Pillar, ...] = ()


def render_depth(
    room: Room,
    x: float,
    y: float,
    heading_deg: float,
    height_m: float,
    pitch_deg: float,
    hfov_deg: float,
    rng: np.random.Generator,
) -> np.ndarray:
    """Camera-z depth of every pixel, with a random scale and noise like the model's."""
    rx, ry = camera_rays(RENDER_WIDTH, RENDER_HEIGHT, hfov_deg)
    pitch, heading = math.radians(pitch_deg), math.radians(heading_deg)
    forward = math.cos(pitch) - math.sin(pitch) * ry  # per unit of camera z
    down = math.sin(pitch) + math.cos(pitch) * ry
    # world direction of each ray (x, y: the room's axes; heading 0 = +x, + = toward +y)
    dx = forward * math.cos(heading) - rx * math.sin(heading)
    dy = forward * math.sin(heading) + rx * math.cos(heading)
    depth = np.where(down > 0, height_m / np.maximum(down, 1e-9), FAR_M)
    for wall, direction in ((room.half_m - x, dx), (-room.half_m - x, dx)):
        depth = _nearer(depth, wall, direction)
    for wall, direction in ((room.half_m - y, dy), (-room.half_m - y, dy)):
        depth = _nearer(depth, wall, direction)
    for pillar in room.pillars:
        depth = np.minimum(depth, _pillar_depth(pillar, x, y, dx, dy))
    depth = np.minimum(depth, FAR_M)
    scale = rng.uniform(*SCALE_RANGE)
    noise = 1 + PIXEL_NOISE * rng.standard_normal(depth.shape)
    return (depth * scale * noise).astype(np.float32)


def _nearer(depth: np.ndarray, offset: float, direction: np.ndarray) -> np.ndarray:
    """Depth of a vertical wall `offset` metres away along one room axis."""
    with np.errstate(divide="ignore", invalid="ignore"):
        hit = offset / direction
    return np.where((hit > 0) & np.isfinite(hit), np.minimum(depth, hit), depth)


def _pillar_depth(pillar: Pillar, x: float, y: float, dx: np.ndarray, dy: np.ndarray) -> np.ndarray:
    """Depth where each ray meets a vertical cylinder (FAR_M if it misses)."""
    ox, oy = x - pillar.x, y - pillar.y
    a = dx * dx + dy * dy
    b = 2 * (ox * dx + oy * dy)
    c = ox * ox + oy * oy - pillar.radius_m**2
    disc = b * b - 4 * a * c
    with np.errstate(invalid="ignore"):
        hit = (-b - np.sqrt(disc)) / (2 * a)
    return np.where((disc >= 0) & (hit > 0), hit, FAR_M)


def pillar_hit(room: Room, x: float, y: float) -> bool:
    return any(math.hypot(x - p.x, y - p.y) < p.radius_m + PILLAR_TOUCH_M for p in room.pillars)


def demo_pillars(half_m: float) -> tuple[Pillar, ...]:
    """A few people-sized pillars spread over the room (deterministic)."""
    spots = ((0.45, 0.3), (-0.5, 0.55), (0.2, -0.6), (-0.35, -0.25))
    return tuple(Pillar(fx * half_m, fy * half_m, 0.25) for fx, fy in spots)
