"""Approximate intrinsics of the X1's live video (1280x720). Principal point at the centre.

The lens is wide with barrel distortion, so one pinhole focal length doesn't fit everything:

- CAMERA_FOCAL_PX = 850 px (~74 deg HFOV): the image centre, for geometry (floor distances in
  drone/patrol/floor_boundary.py, the child's floor position in mission.py, the simulator).
  Evidence: a vertical column's vanishing point vs the horizon at the top gimbal stop gave
  f = sqrt(222 x 3260) ~ 850 px (2026-11-14), and on the recorded wall approach camera tilt and
  odometry height only agree at 850-880 px (drone/patrol, depth agent report).
- FLOW_FOCAL_PX = 627 px: the effective value across the full width, where barrel distortion
  compresses the edges, for drone/sdk/flow.py. Evidence: three exact 360-degree spins
  (recordings/hover_video_20261114-215432.udp, sticks 100 / 80 / 60, view repeats after
  14.43 / 18.25 / 24.8 s); with f = 627 the video yaw integrates to 360 deg per revolution
  (within 0.6 %), and steady stick-60 turns on two other flights within +1..+8 %.

A proper distortion model (one calibration with a checkerboard) would replace both.
"""

import math

CAMERA_WIDTH_PX = 1280
CAMERA_HEIGHT_PX = 720
CAMERA_FOCAL_PX = 850.0  # centre (geometry), fx = fy, at 1280x720
FLOW_FOCAL_PX = 627.0  # effective across the full width (video yaw), see above
CAMERA_CX_PX = CAMERA_WIDTH_PX / 2  # principal point: the image centre
CAMERA_CY_PX = CAMERA_HEIGHT_PX / 2
CAMERA_HFOV_DEG = math.degrees(2 * math.atan(CAMERA_WIDTH_PX / 2 / CAMERA_FOCAL_PX))  # ~74
CAMERA_VFOV_DEG = math.degrees(2 * math.atan(CAMERA_HEIGHT_PX / 2 / CAMERA_FOCAL_PX))  # ~46
