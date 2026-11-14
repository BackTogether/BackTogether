"""The robot's occupancy grid as a PNG for the page's map view (stdlib only: zlib + struct).

Two sources, one layer:
- a ROS map_server map on disk (`--map-yaml hall.yaml`: the .yaml with `image`, `resolution`,
  `origin`, `negate`, `occupied_thresh`, `free_thresh`, and a binary .pgm image), and
- a nav_msgs/OccupancyGrid pushed by ros_node.py (`POST /map/grid`: int8 cells, -1 unknown,
  0..100 occupied %, row 0 at the origin, i.e. the bottom of the picture).
Cells become three palette colours: unknown transparent, free a faint tone, occupied light, so
the PNG stays small and draws on the page's dark background.
"""

import base64
import math
import struct
import time
import zlib
from dataclasses import dataclass
from pathlib import Path

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
PNG_PALETTE = 3  # colour type: indexed
PNG_RGBA = 6
PNG_BIT_DEPTH = 8
PNG_FILTER_NONE = b"\x00"
UNKNOWN, FREE, OCCUPIED = 0, 1, 2  # palette indexes
PALETTE_RGB = ((0, 0, 0), (26, 30, 38), (205, 210, 218))
PALETTE_ALPHA = (0, 255, 255)
OCCUPIED_THRESH = 0.65  # map_server's defaults
FREE_THRESH = 0.196
MAX_GRID_CELLS = 25_000_000  # 250 m x 250 m at 5 cm
UNKNOWN_CELL = 255  # an int8 -1 read as a byte
OCCUPANCY_MAX = 100


@dataclass(frozen=True)
class GridLayer:
    """A grid map ready for the page: its PNG (north-up: row 0 = the top, y max) and where it
    sits in `map` (origin = the bottom-left corner of cell (0, 0), yaw in radians)."""

    png: bytes
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    source: str
    updated_at: float

    def meta(self) -> dict:
        return {
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": {"x": self.origin_x, "y": self.origin_y, "yaw": self.origin_yaw},
            "size_m": [
                round(self.width * self.resolution, 3),
                round(self.height * self.resolution, 3),
            ],
            "source": self.source,
            "updated_at": self.updated_at,
        }


def png_bytes(
    rows: list[bytes],
    width: int,
    colour_type: int,
    palette: bytes = b"",
    alpha: bytes = b"",
) -> bytes:
    """A PNG from raw 8-bit rows (top first; filter type 0)."""
    header = struct.pack(">IIBBBBB", width, len(rows), PNG_BIT_DEPTH, colour_type, 0, 0, 0)
    pixels = zlib.compress(b"".join(PNG_FILTER_NONE + row for row in rows))
    chunks = [(b"IHDR", header)]
    if palette:
        chunks.append((b"PLTE", palette))
    if alpha:
        chunks.append((b"tRNS", alpha))
    chunks += [(b"IDAT", pixels), (b"IEND", b"")]
    return PNG_SIGNATURE + b"".join(_chunk(kind, data) for kind, data in chunks)


def _chunk(kind: bytes, data: bytes) -> bytes:
    crc = zlib.crc32(kind + data) & 0xFFFFFFFF
    return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", crc)


def _palette_png(indexes: bytes, width: int, height: int) -> bytes:
    rows = [indexes[i * width : (i + 1) * width] for i in range(height)]
    palette = b"".join(bytes(rgb) for rgb in PALETTE_RGB)
    return png_bytes(rows, width, PNG_PALETTE, palette, bytes(PALETTE_ALPHA))


def _occupancy_table() -> bytes:
    """Byte -> palette index for OccupancyGrid cells (int8 read as unsigned)."""
    table = bytearray(256)
    for value in range(OCCUPANCY_MAX + 1):
        occupancy = value / OCCUPANCY_MAX
        if occupancy > OCCUPIED_THRESH:
            table[value] = OCCUPIED
        elif occupancy < FREE_THRESH:
            table[value] = FREE
    return bytes(table)  # everything else (-1 and out of range): unknown


def grid_from_cells(
    cells: bytes,
    width: int,
    height: int,
    resolution: float,
    origin: tuple[float, float, float],
    source: str,
) -> GridLayer:
    """An OccupancyGrid's int8 cells (row 0 = y min) as a layer."""
    _check_size(width, height, resolution)
    if len(cells) != width * height:
        raise ValueError(f"{len(cells)} cells for a {width} x {height} grid")
    indexes = cells.translate(_occupancy_table())
    top_first = b"".join(
        indexes[row * width : (row + 1) * width] for row in range(height - 1, -1, -1)
    )
    png = _palette_png(top_first, width, height)
    return GridLayer(png, width, height, resolution, *origin, source, time.time())


def decode_cells(data: str, encoding: str, max_cells: int = MAX_GRID_CELLS) -> bytes:
    """POST /map/grid's "data": base64 of the int8 cells, zlib-compressed or not."""
    raw = base64.b64decode(data, validate=True)
    if encoding == "base64":
        return raw
    if encoding != "zlib+base64":
        raise ValueError('"encoding" must be "zlib+base64" or "base64"')
    inflate = zlib.decompressobj()
    try:
        cells = inflate.decompress(raw, max_cells + 1)
    except zlib.error as error:
        raise ValueError(f"data is not zlib: {error}") from error
    if len(cells) > max_cells or inflate.unconsumed_tail:
        raise ValueError(f"grid over {max_cells} cells")
    return cells


def load_map_yaml(path: Path) -> GridLayer:
    """A map_server map (.yaml + binary .pgm) as a layer. Raises ValueError / OSError."""
    meta = _read_flat_yaml(path.read_text())
    image = path.parent / str(meta["image"])
    width, height, maxval, pixels = _read_pgm(image.read_bytes())
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    if not isinstance(origin, list) or len(origin) != 3:
        raise ValueError(f"{path}: origin must be [x, y, yaw]")
    resolution = float(meta["resolution"])
    _check_size(width, height, resolution)
    table = _pgm_table(
        maxval,
        bool(int(meta.get("negate", 0))),
        float(meta.get("occupied_thresh", OCCUPIED_THRESH)),
        float(meta.get("free_thresh", FREE_THRESH)),
    )
    return GridLayer(
        _palette_png(pixels.translate(table), width, height),
        width,
        height,
        resolution,
        float(origin[0]),
        float(origin[1]),
        float(origin[2]),
        f"file {path.name}",
        time.time(),
    )


def _pgm_table(maxval: int, negate: bool, occupied: float, free: float) -> bytes:
    """Pixel value -> palette index, map_server's trinary rule (dark = occupied)."""
    table = bytearray(256)
    for value in range(min(maxval, 255) + 1):
        shade = value / maxval
        occupancy = shade if negate else 1.0 - shade
        if occupancy > occupied:
            table[value] = OCCUPIED
        elif occupancy < free:
            table[value] = FREE
    return bytes(table)


def _read_pgm(data: bytes) -> tuple[int, int, int, bytes]:
    """(width, height, maxval, pixels) of a binary 8-bit PGM (P5), top row first."""
    fields: list[bytes] = []
    position = 0
    while len(fields) < 4:
        while position < len(data) and data[position : position + 1].isspace():
            position += 1
        if data[position : position + 1] == b"#":
            position = data.index(b"\n", position) + 1
            continue
        start = position
        while position < len(data) and not data[position : position + 1].isspace():
            position += 1
        fields.append(data[start:position])
    if fields[0] != b"P5":
        raise ValueError("the map image must be a binary PGM (P5), as map_saver writes it")
    width, height, maxval = (int(field) for field in fields[1:])
    if not 0 < maxval <= 255:
        raise ValueError("only 8-bit PGM maps are supported")
    pixels = data[position + 1 : position + 1 + width * height]
    if len(pixels) != width * height:
        raise ValueError("the PGM is truncated")
    return width, height, maxval, pixels


def _read_flat_yaml(text: str) -> dict[str, object]:
    """map_server's yaml: flat `key: value` lines, numbers, strings and one [x, y, yaw] list."""
    values: dict[str, object] = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, raw = (part.strip() for part in line.split(":", 1))
        if raw.startswith("[") and raw.endswith("]"):
            values[key] = [float(item) for item in raw[1:-1].split(",") if item.strip()]
        else:
            values[key] = raw.strip("'\"")
    for key in ("image", "resolution"):
        if key not in values:
            raise ValueError(f"map yaml without {key!r}")
    return values


def _check_size(width: int, height: int, resolution: float) -> None:
    if width <= 0 or height <= 0 or width * height > MAX_GRID_CELLS:
        raise ValueError(f"grid size {width} x {height} out of range")
    if not (math.isfinite(resolution) and resolution > 0):
        raise ValueError("resolution must be > 0")
