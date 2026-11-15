"""Draw a simulated patrol's path in its room as a PNG (pillow only), and its coverage."""

import math
from collections.abc import Callable
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from drone.patrol.coverage import CoverageMap
from drone.patrol.sim_room import Room

PX_PER_M = 30
MARGIN_PX = 40
COVER_CELL_M = 1.0
COVER_RADIUS_M = 0.75  # a cell counts as seen when the path passes this close to its centre
BACKGROUND = (250, 250, 247)
WALL = (40, 40, 40)
PILLAR = (200, 80, 60)
PATH_START = (60, 120, 220)
PATH_END = (230, 140, 40)
TEXT = (30, 30, 30)
REFLECT = (40, 160, 90)
FALLBACK = (210, 60, 60)
EXTRA = (140, 140, 140)
# clear.py: "clear" = turned until clear, "best" = 360 without clear, back to the best heading
TURN_COLOURS = {
    "reflect": REFLECT,
    "corner": REFLECT,
    "random": FALLBACK,
    "extra": EXTRA,
    "clear": REFLECT,
    "best": EXTRA,
    "boxed": FALLBACK,
    # sweep.py: where a leg ended
    "wall": REFLECT,
    "obstacle": FALLBACK,
    "side": EXTRA,
}
UNSEEN = (245, 200, 195)  # coverage cells: never seen, seen by one leg, by two or more
SEEN_ONCE = (205, 235, 205)
SEEN_TWICE = (160, 210, 165)
DVD_LEGEND = "green: reflection, red: random fallback, grey: still blocked, path blue->orange"


def coverage(room: Room, path: list[tuple[float, float]]) -> float:
    """Fraction of the room's 1 m cells the path passed near."""
    if not path:
        return 0.0
    points = np.array(path)
    centres = np.arange(-room.half_m + COVER_CELL_M / 2, room.half_m, COVER_CELL_M)
    seen = 0
    for cx in centres:
        for cy in centres:
            distance = np.hypot(points[:, 0] - cx, points[:, 1] - cy)
            seen += bool((distance <= COVER_RADIUS_M).any())
    return seen / len(centres) ** 2


def draw(
    room: Room,
    path: list[tuple[float, float]],
    turns: list[tuple[float, float, float, str]],
    out: Path,
    title: str,
    legend: str = DVD_LEGEND,
    seen: CoverageMap | None = None,
) -> None:
    """`turns`: (x, y, degrees, how) where the patrol turned (how: dvd.TurnChoice.how);
    `seen`: the floor cells the camera saw, in room coordinates, drawn under the path."""
    half_x, half_y = room.half_m, room.half_y
    width = round(2 * half_x * PX_PER_M) + 2 * MARGIN_PX
    size = round(2 * half_y * PX_PER_M) + 2 * MARGIN_PX
    image = Image.new("RGB", (width, size), BACKGROUND)
    pen = ImageDraw.Draw(image)

    def px(x: float, y: float) -> tuple[float, float]:
        # room x to the right, room y up
        return MARGIN_PX + (x + half_x) * PX_PER_M, MARGIN_PX + (half_y - y) * PX_PER_M

    if seen is not None:
        _draw_cells(pen, px, seen, room)
    pen.rectangle([px(-half_x, half_y), px(half_x, -half_y)], outline=WALL, width=3)
    for pillar in room.pillars:
        cx, cy = px(pillar.x, pillar.y)
        r = pillar.radius_m * PX_PER_M
        pen.ellipse([cx - r, cy - r, cx + r, cy + r], fill=PILLAR)
    for i in range(1, len(path)):
        t = i / len(path)
        colour = tuple(round(a + (b - a) * t) for a, b in zip(PATH_START, PATH_END, strict=True))
        pen.line([px(*path[i - 1]), px(*path[i])], fill=colour, width=2)
    if path:
        sx, sy = px(*path[0])
        pen.rectangle([sx - 5, sy - 5, sx + 5, sy + 5], outline=PATH_START, width=2)
    for x, y, degrees, how in turns:
        cx, cy = px(x, y)
        colour = TURN_COLOURS[how]
        pen.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], fill=colour)
        pen.text((cx + 6, cy - 6), f"{degrees:+.0f}", fill=colour)
    pen.text((MARGIN_PX, 10), title, fill=TEXT)
    pen.text((MARGIN_PX, size - MARGIN_PX + 14), legend, fill=TEXT)
    out.parent.mkdir(parents=True, exist_ok=True)
    image.save(out)


def _draw_cells(
    pen: ImageDraw.ImageDraw,
    px: Callable[[float, float], tuple[float, float]],
    seen: CoverageMap,
    room: Room,
) -> None:
    half = seen.cell_m / 2
    inside_x = np.flatnonzero(np.abs(seen.centres) <= room.half_m)
    inside_y = np.flatnonzero(np.abs(seen.centres) <= room.half_y)
    for ix in inside_x:
        for iy in inside_y:
            legs = seen.legs[ix, iy]
            colour = UNSEEN if legs == 0 else SEEN_ONCE if legs == 1 else SEEN_TWICE
            x, y = seen.centres[ix], seen.centres[iy]
            pen.rectangle([px(x - half, y + half), px(x + half, y - half)], fill=colour)


def path_length(path: list[tuple[float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(path, path[1:], strict=False))
