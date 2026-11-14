"""Approximate intrinsics of the X1's live video (1280x720), for navigation work (optical
flow, visual localisation). From one measurement on 2026-11-14 (a frame at the gimbal's top
stop, camera on the floor: vanishing point of vertical column edges vs the horizon), so take
them as +-10 %. No distortion model yet.

With f, a sideways image shift of dx pixels between two frames is a yaw of about
atan(dx / f): at 30 fps, yaw_rate ~= degrees(atan(dx_px / CAMERA_FOCAL_PX)) * 30.
"""

import math

CAMERA_WIDTH_PX = 1280
CAMERA_HEIGHT_PX = 720
CAMERA_FOCAL_PX = 850.0  # fx = fy, at 1280x720
CAMERA_CX_PX = CAMERA_WIDTH_PX / 2  # principal point: the image centre
CAMERA_CY_PX = CAMERA_HEIGHT_PX / 2
CAMERA_HFOV_DEG = math.degrees(2 * math.atan(CAMERA_WIDTH_PX / 2 / CAMERA_FOCAL_PX))  # ~74
CAMERA_VFOV_DEG = math.degrees(2 * math.atan(CAMERA_HEIGHT_PX / 2 / CAMERA_FOCAL_PX))  # ~46
