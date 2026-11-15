"""Frames and camera geometry of the lost-child mission (mission.py) and its simulator.

- drone frame: the odometry of this flight (origin = takeoff point, y 90 deg clockwise of x
  seen from above). Body frame: forward, right (down) of the drone, from the heading estimate.
- launch frame: the robot at takeoff, ROS convention (x forward, y left).
- map: the robot's map. LaunchPose = the robot's pose there at takeoff.
- image: (u, v) = fractions of the image width and height, y down; a pinhole camera with the
  intrinsics of camera.py (read at every call: the focal length is being re-calibrated) and the
  camera tilt (0 = level, -90 = straight down).

A standing person's floor point: vertical lines converge in the image on the nadir (the point
straight below the camera), so a person's body box runs along the line through the nadir and
their feet are the end of the box nearest to it (floor_contact). With the camera tilted the
nadir is below the image and that is the bottom centre; straight down it is the image centre.
"""

import math
from dataclasses import dataclass

from . import camera
from .estimate import wrap_deg

MIN_RAY_DOWN = 0.05  # rays flatter than this never meet the floor
MIN_DEPTH_M = 0.2  # points closer to the camera plane are not projected


@dataclass(frozen=True)
class LaunchPose:
    """The robot's pose in `map` at takeoff (ROS: metres, yaw in radians, counter-clockwise)."""

    x: float
    y: float
    yaw: float

    def map_from_launch(self, fwd: float, left: float) -> tuple[float, float]:
        cos, sin = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + fwd * cos - left * sin, self.y + fwd * sin + left * cos

    def launch_from_map(self, x: float, y: float) -> tuple[float, float]:
        dx, dy = x - self.x, y - self.y
        cos, sin = math.cos(self.yaw), math.sin(self.yaw)
        return dx * cos + dy * sin, -dx * sin + dy * cos


@dataclass(frozen=True)
class Alignment:
    """The robot's forward axis in the drone frame, from the aligning leg."""

    forward_deg: float  # from the drone frame's +x toward +y (clockwise seen from above)
    leg_m: float

    def launch_from_drone(self, x: float, y: float) -> tuple[float, float]:
        """(forward, left) of the robot at launch. The matrix is a reflection (the drone frame
        is clockwise, ROS is counter-clockwise), so it is its own inverse."""
        cos, sin = (
            math.cos(math.radians(self.forward_deg)),
            math.sin(math.radians(self.forward_deg)),
        )
        return x * cos + y * sin, x * sin - y * cos

    def drone_from_launch(self, fwd: float, left: float) -> tuple[float, float]:
        return self.launch_from_drone(fwd, left)


def ground_offset(
    u: float, v: float, tilt_deg: float, height_m: float
) -> tuple[float, float] | None:
    """Where the ray through image point (u, v) (fractions, y down) meets a plane `height_m`
    below the camera: (forward, right) metres from the drone in its body frame. None if the
    ray does not go down. Pinhole with the intrinsics of camera.py, read at every call."""
    xn = (u - 0.5) * camera.CAMERA_WIDTH_PX / camera.CAMERA_FOCAL_PX
    yn = (v - 0.5) * camera.CAMERA_HEIGHT_PX / camera.CAMERA_FOCAL_PX
    tilt = math.radians(tilt_deg)
    ahead, down = math.cos(tilt), -math.sin(tilt)  # the optical axis: forward, down parts
    ray_down = down + yn * ahead
    if ray_down < MIN_RAY_DOWN:
        return None
    scale = height_m / ray_down
    return scale * (ahead - yn * down), scale * xn


def nadir_v(tilt_deg: float) -> float | None:
    """Image row (fraction, y down; can be > 1) of the point straight below the camera, where
    vertical lines converge. None when the camera does not look down."""
    tilt = math.radians(tilt_deg)
    ahead, down = math.cos(tilt), -math.sin(tilt)
    if down < MIN_RAY_DOWN:
        return None
    return 0.5 + ahead / down * camera.CAMERA_FOCAL_PX / camera.CAMERA_HEIGHT_PX


def floor_contact(bbox: tuple[float, float, float, float], tilt_deg: float) -> tuple[float, float]:
    """(u, v) where a standing person in `bbox` (x0, y0, x1, y1, fractions) touches the floor:
    where the line from the box centre to the nadir leaves the box (the nadir itself when it
    is inside: the person is right below). Camera level or up: the bottom centre."""
    x0, y0, x1, y1 = bbox
    cu, cv = (x0 + x1) / 2, (y0 + y1) / 2
    nv = nadir_v(tilt_deg)
    if nv is None:
        return cu, y1
    if x0 <= 0.5 <= x1 and y0 <= nv <= y1:
        return 0.5, nv
    du, dv = 0.5 - cu, nv - cv
    scales = [half / abs(d) for half, d in (((x1 - x0) / 2, du), ((y1 - y0) / 2, dv)) if d]
    scale = min(scales, default=0.0)
    return cu + scale * du, cv + scale * dv


def body_to_frame(fwd: float, right: float, heading_deg: float) -> tuple[float, float]:
    heading = math.radians(heading_deg)
    return (
        fwd * math.cos(heading) - right * math.sin(heading),
        fwd * math.sin(heading) + right * math.cos(heading),
    )


def frame_to_body(dx: float, dy: float, heading_deg: float) -> tuple[float, float]:
    heading = math.radians(heading_deg)
    return (
        dx * math.cos(heading) + dy * math.sin(heading),
        -dx * math.sin(heading) + dy * math.cos(heading),
    )


def project_box(
    fwd: float, right: float, down: float, tilt_deg: float, size_m: float
) -> tuple[float, float, float, float] | None:
    """The image box (u0, v0, u1, v1) of a `size_m` square facing the camera, centred on a
    body-frame point; None outside the image or behind the camera. The inverse of
    ground_offset (the simulator's fake face uses it)."""
    point = project_point(fwd, right, down, tilt_deg)
    if point is None:
        return None
    u, v, depth = point
    if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
        return None
    half_u = size_m / 2 / depth * camera.CAMERA_FOCAL_PX / camera.CAMERA_WIDTH_PX
    half_v = size_m / 2 / depth * camera.CAMERA_FOCAL_PX / camera.CAMERA_HEIGHT_PX
    return max(0.0, u - half_u), max(0.0, v - half_v), min(1.0, u + half_u), min(1.0, v + half_v)


def project_point(
    fwd: float, right: float, down: float, tilt_deg: float
) -> tuple[float, float, float] | None:
    """(u, v, depth) of a body-frame point in the image (may fall outside 0..1); None behind
    the camera."""
    tilt = math.radians(tilt_deg)
    ahead, below = math.cos(tilt), -math.sin(tilt)
    depth = fwd * ahead + down * below
    if depth < MIN_DEPTH_M:
        return None
    u = 0.5 + right / depth * camera.CAMERA_FOCAL_PX / camera.CAMERA_WIDTH_PX
    v = (
        0.5
        + (down * ahead - fwd * below) / depth * camera.CAMERA_FOCAL_PX / camera.CAMERA_HEIGHT_PX
    )
    return u, v, depth


def project_person(
    fwd: float, right: float, floor_m: float, tilt_deg: float, height_m: float, width_m: float
) -> tuple[float, float, float, float] | None:
    """The image box of a standing person (a vertical segment `height_m` tall, `width_m` wide)
    whose feet are at body-frame (fwd, right), `floor_m` below the camera; clipped to the
    image, None when outside it or behind the camera. What a person detector would give."""
    corners = []
    for down in (floor_m, floor_m - height_m):  # feet, head
        point = project_point(fwd, right, down, tilt_deg)
        if point is None:
            return None
        u, v, depth = point
        half_u = width_m / 2 / depth * camera.CAMERA_FOCAL_PX / camera.CAMERA_WIDTH_PX
        corners += [(u - half_u, v), (u + half_u, v)]
    x0, x1 = min(c[0] for c in corners), max(c[0] for c in corners)
    y0, y1 = min(c[1] for c in corners), max(c[1] for c in corners)
    if x1 <= 0.0 or x0 >= 1.0 or y1 <= 0.0 or y0 >= 1.0:
        return None
    return max(0.0, x0), max(0.0, y0), min(1.0, x1), min(1.0, y1)


def wrap_rad(angle: float) -> float:
    return math.radians(wrap_deg(math.degrees(angle)))
