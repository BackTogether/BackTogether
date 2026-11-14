"""Approximate intrinsics of the X1's live video (1280x720), for navigation work (optical
flow, visual localisation). Pinhole, principal point at the centre, no distortion model.

Focal length, 2026-11-14: 627 px, from three exact 360-degree spins (recordings/
hover_video_20261114-215432.udp at unix 1794691828 / 1794691860 / 1794691900, sticks 100 / 80 /
60, camera level): the view repeats after 14.43 / 18.25 / 24.8 s (phase correlation peak 0.4-0.55
at a 0-2 px shift), and the video yaw of flow.py integrates to 360 deg over each revolution with
f = 627 (to within 0.6 % for all three). The same f gives the steady stick-60 turns of two other
flights (camera at -32 and -50 deg) within +1..+8 % of the spin rate. The earlier 850 px (one
vanishing-point measurement) made every flow yaw ~1.3x too small. The lens is wide and likely
has barrel distortion, so this is the best pinhole fit across the full width; the centre alone
may be a little longer.

With f, a sideways image shift of dx pixels near the centre of a level camera is a yaw of about
atan(dx / f); drone/sdk/flow.py handles a tilted camera and off-centre columns.
"""

import math

CAMERA_WIDTH_PX = 1280
CAMERA_HEIGHT_PX = 720
CAMERA_FOCAL_PX = 627.0  # fx = fy, at 1280x720
CAMERA_CX_PX = CAMERA_WIDTH_PX / 2  # principal point: the image centre
CAMERA_CY_PX = CAMERA_HEIGHT_PX / 2
CAMERA_HFOV_DEG = math.degrees(2 * math.atan(CAMERA_WIDTH_PX / 2 / CAMERA_FOCAL_PX))  # ~91
CAMERA_VFOV_DEG = math.degrees(2 * math.atan(CAMERA_HEIGHT_PX / 2 / CAMERA_FOCAL_PX))  # ~60
