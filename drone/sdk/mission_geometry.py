"""Frames and camera geometry of the lost-child mission (mission.py) and its simulator.

- drone frame: the odometry of this flight (origin = takeoff point, y 90 deg clockwise of x
  seen from above). Body frame: forward, right (down) of the drone, from the heading estimate.
- launch frame: the robot at takeoff, ROS convention (x forward, y left).
- map: the robot's map. LaunchPose = the robot's pose there at takeoff.
- image: (u, v) = fractions of the image width and height, y down; a pinhole camera with the
  intrinsics of camera.py (read at every call: the focal length is being re-calibrated) and the
  camera tilt (0 = level, -90 = straight down).
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
    ground_offset (the simulator's fake detector uses it)."""
    tilt = math.radians(tilt_deg)
    ahead, below = math.cos(tilt), -math.sin(tilt)  # the optical axis: forward, down parts
    depth = fwd * ahead + down * below
    if depth < MIN_DEPTH_M:
        return None
    fx = camera.CAMERA_FOCAL_PX / camera.CAMERA_WIDTH_PX  # focal length in image fractions
    fy = camera.CAMERA_FOCAL_PX / camera.CAMERA_HEIGHT_PX
    u = 0.5 + right / depth * fx
    v = 0.5 + (down * ahead - fwd * below) / depth * fy
    if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
        return None
    half_u, half_v = size_m / 2 / depth * fx, size_m / 2 / depth * fy
    return max(0.0, u - half_u), max(0.0, v - half_v), min(1.0, u + half_u), min(1.0, v + half_v)


def wrap_rad(angle: float) -> float:
    return math.radians(wrap_deg(math.degrees(angle)))
