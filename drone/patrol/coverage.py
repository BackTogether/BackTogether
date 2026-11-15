"""Which floor the camera has seen: a grid of floor cells, for the sweep patrol (sweep.py).

A floor point `fwd` m ahead and `right` m to the right of a camera `height_m` above the floor,
pitched `pitch_deg` below level, is in the picture when its ray falls inside the field of view
(pinhole model, HFOV of floor_boundary.py, the video's 16:9 aspect). Only floor up to
SEEN_MAX_M ahead counts as searched: at 4 m a 0.5 m child is still ~90 px wide in the 1280 px
video (f = 850 px), and nearer floor is less often hidden behind tables and people.

The lane spacing comes from the same model: the width of the floor seen at the bottom of the
image (the nearest, narrowest part of the footprint: every point within it passes through the
picture at close range), minus an overlap. At 2 m and -30 deg: 3.48 m wide, 2.78 m apart.

Cells count the legs (lanes, crossings) that saw them, so the overlap between neighbouring
lanes can be measured; frames are the caller's (sweep.py: the lane frame, x along the lanes,
y to the right; heading 0 = +x, + = toward +y).
"""

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from drone.sdk.camera import CAMERA_HEIGHT_PX, CAMERA_WIDTH_PX

CELL_M = 0.5
SEEN_MAX_M = 4.0  # floor farther ahead than this is not counted as searched (see above)
DEFAULT_OVERLAP = 0.2  # of the swath, between neighbouring lanes
GRID_HALF_M = 40.0  # the grid covers +-this around the frame's origin
IMAGE_ASPECT = CAMERA_HEIGHT_PX / CAMERA_WIDTH_PX
MAX_DEPRESSION_DEG = 89.0  # the bottom row of a camera pointing (almost) straight down
STRIP_CHARS = " .:-=+*#%@"  # coverage of a band, 0 .. 100 %
STRIP_WIDTH = 20  # characters
MIN_PATCH_M2 = 1.0  # smaller unseen holes are not reported


@dataclass(frozen=True)
class CameraView:
    height_m: float  # above the floor
    pitch_deg: float  # below level (+)
    hfov_deg: float

    @property
    def vfov_deg(self) -> float:
        return math.degrees(2 * math.atan(math.tan(math.radians(self.hfov_deg) / 2) * IMAGE_ASPECT))

    def sees(self, fwd: np.ndarray, right: np.ndarray) -> np.ndarray:
        """Which floor points (body frame: metres ahead, to the right) are in the picture."""
        pitch = math.radians(self.pitch_deg)
        z = fwd * math.cos(pitch) + self.height_m * math.sin(pitch)  # along the optical axis
        down = -fwd * math.sin(pitch) + self.height_m * math.cos(pitch)  # image y
        half_w = math.tan(math.radians(self.hfov_deg) / 2)
        half_h = math.tan(math.radians(self.vfov_deg) / 2)
        return (z > 0) & (np.abs(right) <= z * half_w) & (np.abs(down) <= z * half_h)

    def near_m(self) -> float:
        """Floor distance of the bottom image row: nearer floor is under the picture."""
        depression = min(self.pitch_deg + self.vfov_deg / 2, MAX_DEPRESSION_DEG)
        return self.height_m / math.tan(math.radians(depression))

    def swath_m(self) -> float:
        """Width of the floor seen at the bottom image row."""
        pitch = math.radians(self.pitch_deg)
        z = self.near_m() * math.cos(pitch) + self.height_m * math.sin(pitch)
        return 2 * z * math.tan(math.radians(self.hfov_deg) / 2)


def lane_spacing(view: CameraView, overlap: float = DEFAULT_OVERLAP) -> float:
    return view.swath_m() * (1 - overlap)


Box = tuple[float, float, float, float]  # x_min, x_max, y_min, y_max


@dataclass(frozen=True)
class Patch:
    """A connected unseen area."""

    x: float  # centre
    y: float
    area_m2: float


class CoverageMap:
    """Floor cells seen by the camera, each with the number of legs that saw it."""

    def __init__(self, cell_m: float = CELL_M, half_m: float = GRID_HALF_M) -> None:
        self.cell_m = cell_m
        self.half_m = half_m
        cells = round(2 * half_m / cell_m)
        self.centres = (np.arange(cells) + 0.5) * cell_m - half_m  # same along x and y
        self.legs = np.zeros((cells, cells), dtype=np.int16)  # [ix, iy]
        self._last_leg = np.full((cells, cells), -1, dtype=np.int32)

    def mark(
        self, x: float, y: float, heading_deg: float, view: CameraView, range_m: float, leg: int
    ) -> None:
        """The camera at (x, y) facing `heading_deg` sees the floor up to `range_m` ahead."""
        if range_m <= 0:
            return
        ix, iy = self._window(x, range_m), self._window(y, range_m)
        cx, cy = np.meshgrid(self.centres[ix], self.centres[iy], indexing="ij")
        heading = math.radians(heading_deg)
        dx, dy = cx - x, cy - y
        fwd = dx * math.cos(heading) + dy * math.sin(heading)
        right = -dx * math.sin(heading) + dy * math.cos(heading)
        seen = view.sees(fwd, right) & (fwd <= range_m)
        last = self._last_leg[ix, iy]
        new = seen & (last != leg)
        self.legs[ix, iy] += new
        self._last_leg[ix, iy] = np.where(new, leg, last)

    def _window(self, centre: float, range_m: float) -> slice:
        first = max(0, math.floor((centre - range_m + self.half_m) / self.cell_m))
        last = min(len(self.centres), math.ceil((centre + range_m + self.half_m) / self.cell_m))
        return slice(first, max(first, last))

    def _inside(self, box: Box) -> tuple[slice, slice]:
        x_min, x_max, y_min, y_max = box
        xs = np.flatnonzero((self.centres >= x_min) & (self.centres <= x_max))
        ys = np.flatnonzero((self.centres >= y_min) & (self.centres <= y_max))
        if not len(xs) or not len(ys):
            return slice(0, 0), slice(0, 0)
        return slice(xs[0], xs[-1] + 1), slice(ys[0], ys[-1] + 1)

    def fraction(self, box: Box) -> float:
        """Seen fraction of the cells inside `box`."""
        region = self.legs[self._inside(box)]
        return float((region > 0).mean()) if region.size else 0.0

    def overlap(self, box: Box) -> float:
        """Fraction of the seen cells inside `box` that two or more legs saw."""
        region = self.legs[self._inside(box)]
        seen = region > 0
        return float((region[seen] >= 2).mean()) if seen.any() else 0.0

    def seen_box(self) -> Box | None:
        """The bounding box of the seen cells."""
        xs, ys = np.nonzero(self.legs)
        if not len(xs):
            return None
        half = self.cell_m / 2
        c = self.centres
        return c[xs.min()] - half, c[xs.max()] + half, c[ys.min()] - half, c[ys.max()] + half

    def strip(self, box: Box, width: int = STRIP_WIDTH) -> str:
        """One character per band across y (left to right): how much of it was seen."""
        region = self.legs[self._inside(box)] > 0
        if not region.size:
            return ""
        bands = np.array_split(region, min(width, region.shape[1]), axis=1)
        top = len(STRIP_CHARS) - 1
        return "".join(STRIP_CHARS[round(band.mean() * top)] for band in bands)

    def patches(self, box: Box, min_area_m2: float = MIN_PATCH_M2) -> list[Patch]:
        """Connected unseen areas inside `box`, the largest first."""
        sx, sy = self._inside(box)
        unseen = self.legs[sx, sy] == 0
        cell_area = self.cell_m**2
        found = []
        for cells in _components(unseen):
            area = len(cells) * cell_area
            if area < min_area_m2:
                continue
            ix, iy = np.array(cells).T
            x = float(self.centres[sx][ix].mean())
            y = float(self.centres[sy][iy].mean())
            found.append(Patch(x, y, area))
        return sorted(found, key=lambda patch: -patch.area_m2)


def _components(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    """4-connected groups of True cells (the grids are small: a plain flood fill)."""
    todo = mask.copy()
    groups = []
    for start in zip(*np.nonzero(todo), strict=True):
        if not todo[start]:
            continue
        todo[start] = False
        group, queue = [], deque([start])
        while queue:
            i, j = queue.popleft()
            group.append((int(i), int(j)))
            for ni, nj in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                inside = 0 <= ni < mask.shape[0] and 0 <= nj < mask.shape[1]
                if inside and todo[ni, nj]:
                    todo[ni, nj] = False
                    queue.append((ni, nj))
        groups.append(group)
    return groups
