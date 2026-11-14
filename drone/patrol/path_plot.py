"""Draw a simulated patrol's path in its room as a PNG (pillow only), and its coverage."""

import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

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
TURN_COLOURS = {"reflect": REFLECT, "corner": REFLECT, "random": FALLBACK, "extra": EXTRA}


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
) -> None:
    """`turns`: (x, y, degrees, how) where the patrol turned (how: dvd.TurnChoice.how)."""
    size = round(2 * room.half_m * PX_PER_M) + 2 * MARGIN_PX
    image = Image.new("RGB", (size, size), BACKGROUND)
    pen = ImageDraw.Draw(image)

    def px(x: float, y: float) -> tuple[float, float]:
        # room x to the right, room y up
        return MARGIN_PX + (x + room.half_m) * PX_PER_M, MARGIN_PX + (room.half_m - y) * PX_PER_M

    pen.rectangle(
        [px(-room.half_m, room.half_m), px(room.half_m, -room.half_m)], outline=WALL, width=3
    )
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
    legend = "green: reflection, red: random fallback, grey: still blocked, path blue->orange"
    pen.text((MARGIN_PX, size - MARGIN_PX + 14), legend, fill=TEXT)
    out.parent.mkdir(parents=True, exist_ok=True)
    image.save(out)


def path_length(path: list[tuple[float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(path, path[1:], strict=False))
