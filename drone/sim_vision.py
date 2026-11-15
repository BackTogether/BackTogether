"""Fake pad and robot detectors for the simulator, so drone/land_on_pad.py and drone/dock_robot.py
can be run end to end without a real camera (the simulator's video is a test pattern).

    drone/fly --sim ...  (or python -m drone.sdk sim)
    drone/land --sim                 # land on the simulated robot's pad
    drone/dock --now --sim --robot-sim 0 0

The robot stands at `robot_xy` (takeoff frame, default the takeoff point: the drone took off from
its top), its pad a PAD_SIDE_M square on its top disc at `pad_height_m` above the floor. Each
call reads the simulator's true pose (GET /sim/truth) and the camera tilt, projects the pad's
corners and points around the robot's body through the pinhole model of drone/sdk/camera.py
(mission_geometry.project_point) and returns what the real detectors would: a Pad (all four
corners in view, within PAD_RANGE_M) and a Robot box (within ROBOT_RANGE_M), with MISS_RATE of
the frames missed and a little pixel noise, like a real detector.
"""

import math
import random

import numpy as np

from drone.identify.pad import PAD_SIDE_M, Pad
from drone.identify.robot import Robot
from drone.sdk.client import DroneClient
from drone.sdk.mission_geometry import frame_to_body, project_point

ROBOT_RADIUS_M = 0.25  # Bobac's round base
ROBOT_HEIGHT_M = 0.9  # base, box and screen
ROBOT_RING_POINTS = 12  # points around the body that make its box
PAD_RANGE_M = 4.0  # the pad detector's training range (pad.py RANGE_M)
ROBOT_RANGE_M = 6.0
MISS_RATE = 0.15
NOISE_PX = 2.0
SCORE = 1.0


class SimVision:
    """`pad(image)` and `robot(image)`, drop-in for PadDetector.detect / RobotDetector.detect."""

    def __init__(
        self,
        client: DroneClient,
        robot_xy: tuple[float, float] = (0.0, 0.0),
        pad_height_m: float = 0.4,
        takeoff_height_m: float = 0.0,
        seed: int | None = None,
    ) -> None:
        self.client = client
        self.robot_xy = robot_xy
        self.pad_height_m = pad_height_m
        self.takeoff_height_m = takeoff_height_m
        self.random = random.Random(seed)

    def pad(self, image: np.ndarray) -> Pad | None:
        view = self._view()
        if view is None or self._missed() or self._range(view) > PAD_RANGE_M:
            return None
        half = PAD_SIDE_M / 2
        x, y = self.robot_xy
        corners = [(x - half, y - half), (x + half, y - half), (x + half, y + half),
                   (x - half, y + half)]  # fmt: skip
        pixels = [self._pixel(view, cx, cy, self.pad_height_m, image.shape) for cx, cy in corners]
        if any(p is None or not _inside(p, image.shape) for p in pixels):
            return None
        quad = np.array(pixels, np.float32)
        return Pad(SCORE, quad, (float(quad[:, 0].mean()), float(quad[:, 1].mean())))

    def robot(self, image: np.ndarray) -> Robot | None:
        view = self._view()
        if view is None or self._missed() or self._range(view) > ROBOT_RANGE_M:
            return None
        x, y = self.robot_xy
        pixels = []
        for k in range(ROBOT_RING_POINTS):
            angle = 2 * math.pi * k / ROBOT_RING_POINTS
            px, py = x + ROBOT_RADIUS_M * math.cos(angle), y + ROBOT_RADIUS_M * math.sin(angle)
            for height in (0.0, ROBOT_HEIGHT_M):
                pixel = self._pixel(view, px, py, height, image.shape)
                if pixel is not None:
                    pixels.append(pixel)
        if not pixels:
            return None
        points = np.array(pixels)
        centre = points.mean(0)
        if not _inside(tuple(centre), image.shape):
            return None
        h, w = image.shape[:2]
        x0, y0 = np.clip(points.min(0), 0, (w - 1, h - 1))
        x1, y1 = np.clip(points.max(0), 0, (w - 1, h - 1))
        return Robot(SCORE, (int(x0), int(y0), int(x1 - x0), int(y1 - y0)))

    def _view(self) -> dict[str, float] | None:
        """The true pose and the camera tilt now; None while the tilt is unknown."""
        truth = self.client.sim_truth()
        tilt = self.client.status()["telemetry"]["tilt"]["deg"]
        if tilt is None:
            return None
        return {**truth, "tilt_deg": tilt}

    def _range(self, view: dict[str, float]) -> float:
        return math.hypot(self.robot_xy[0] - view["x"], self.robot_xy[1] - view["y"])

    def _missed(self) -> bool:
        return self.random.random() < MISS_RATE

    def _pixel(
        self, view: dict[str, float], x: float, y: float, height_m: float, shape: tuple[int, ...]
    ) -> tuple[float, float] | None:
        """Pixel (x, y) of a floor-frame point `height_m` above the floor; None behind."""
        fwd, right = frame_to_body(x - view["x"], y - view["y"], view["heading_deg"])
        down = view["alt"] + self.takeoff_height_m - height_m
        point = project_point(fwd, right, down, view["tilt_deg"])
        if point is None:
            return None
        u, v, _depth = point
        h, w = shape[:2]
        noise = self.random.gauss(0, NOISE_PX), self.random.gauss(0, NOISE_PX)
        return u * w + noise[0], v * h + noise[1]


def _inside(pixel: tuple[float, float], shape: tuple[int, ...]) -> bool:
    h, w = shape[:2]
    return 0 <= pixel[0] < w and 0 <= pixel[1] < h
