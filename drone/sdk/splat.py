"""A Gaussian splat of the hall (3DGS .ply) as a top-down image for the page's map view.

Reads the Gaussian centres (x, y, z), their colour (the SH DC term f_dc_0..2, or plain
red/green/blue) and opacity (a logit in 3DGS files) and projects them straight down onto the
floor plane: each pixel is the opacity-weighted mean colour of the centres above it, its alpha
1 - exp(-sum of opacities). Centres above `z_max` (default: halfway between the floor and the
ceiling, from the 2nd/98th height percentiles) are dropped, so the ceiling does not hide the
floor. The splat's own units and axes are kept; a 2D similarity (Registration: scale, rotation,
offset) places it in the robot's `map`, saved as JSON next to the splat.

numpy is optional for the SDK: without it (or with a bad file) the layer is off and the page
says why (`uv run --with numpy ...`, as drone/fly does).
"""

import importlib.util
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

from .map_image import PNG_RGBA, png_bytes

SH_C0 = 0.28209479177387814  # degree-0 spherical harmonic: colour = 0.5 + SH_C0 * f_dc
MAX_SIDE_PX = 1600
GAUSSIANS_PER_PIXEL = 4.0  # on average over the covered area: coarser pixels, fewer holes
MIN_OPACITY = 0.05  # fainter Gaussians are mostly floaters
EXTENT_PERCENTILES = (1.0, 99.0)  # the image covers this range of the centres, per axis
FLOOR_PERCENTILE, CEILING_PERCENTILE = 2.0, 98.0
DEFAULT_CUT_FRACTION = 0.5  # of the floor-to-ceiling height
CHANNEL_MAX = 255
NO_NUMPY_NOTE = "numpy not installed: no splat layer (uv run --with numpy ...)"
# Which splat axes make the floor plane (u, v) and "up", keeping (u, v, up) right-handed so
# the top-down picture is not mirrored. COLMAP-based 3DGS scenes are often -y up.
UP_AXES = {
    "z": (("x", 1), ("y", 1), ("z", 1)),
    "-z": (("x", 1), ("y", -1), ("z", -1)),
    "y": (("z", 1), ("x", 1), ("y", 1)),
    "-y": (("x", 1), ("z", 1), ("y", -1)),
    "x": (("y", 1), ("z", 1), ("x", 1)),
    "-x": (("z", 1), ("y", 1), ("x", -1)),
}
PLY_TYPES = {
    "float": "f4",
    "float32": "f4",
    "double": "f8",
    "float64": "f8",
    "uchar": "u1",
    "uint8": "u1",
    "char": "i1",
    "int8": "i1",
    "ushort": "u2",
    "uint16": "u2",
    "short": "i2",
    "int16": "i2",
    "uint": "u4",
    "uint32": "u4",
    "int": "i4",
    "int32": "i4",
}


@dataclass
class Registration:
    """map = offset + scale * R(rotation) * (u, v): the splat's floor plane in `map`."""

    scale: float = 1.0
    rotation_deg: float = 0.0
    x: float = 0.0
    y: float = 0.0

    @staticmethod
    def path_for(splat: Path) -> Path:
        return splat.with_suffix(".registration.json")

    @classmethod
    def load(cls, path: Path) -> "Registration":
        if not path.exists():
            return cls()
        values = json.loads(path.read_text())
        return cls(**{key: float(values[key]) for key in asdict(cls()) if key in values})

    def save(self, path: Path) -> None:
        path.write_text(json.dumps(asdict(self), indent=2) + "\n")


@dataclass(frozen=True)
class SplatLayer:
    """The top-down PNG (row 0 = v max) and what it covers in splat units."""

    png: bytes
    bounds: tuple[float, float, float, float]  # u0, v0, u1, v1
    gaussians: int  # drawn (after the cuts)
    floor: float
    ceiling: float
    z_max: float
    up: str
    source: str

    def meta(self) -> dict:
        return {
            "bounds": [round(value, 4) for value in self.bounds],
            "gaussians": self.gaussians,
            "floor": round(self.floor, 3),
            "ceiling": round(self.ceiling, 3),
            "z_max": round(self.z_max, 3),
            "up": self.up,
            "source": self.source,
        }


def numpy_missing() -> bool:
    return importlib.util.find_spec("numpy") is None


def render_splat(path: Path, up: str = "z", z_max: float | None = None) -> SplatLayer:
    """Raises ValueError / OSError for a file it cannot read, ImportError without numpy."""
    import numpy as np  # optional dependency: see the module docstring

    vertices = _read_ply(path, np)
    names = vertices.dtype.names
    (u_axis, u_sign), (v_axis, v_sign), (h_axis, h_sign) = UP_AXES[up]
    u = vertices[u_axis].astype(np.float64) * u_sign
    v = vertices[v_axis].astype(np.float64) * v_sign
    height = vertices[h_axis].astype(np.float64) * h_sign
    colour = _colours(vertices, names, np)
    opacity = _opacities(vertices, names, np)
    floor, ceiling = np.percentile(height, [FLOOR_PERCENTILE, CEILING_PERCENTILE])
    cut = floor + DEFAULT_CUT_FRACTION * (ceiling - floor) if z_max is None else z_max
    u_low, u_high = np.percentile(u, EXTENT_PERCENTILES)
    v_low, v_high = np.percentile(v, EXTENT_PERCENTILES)
    keep = (
        (height <= cut)
        & (opacity >= MIN_OPACITY)
        & (u >= u_low)
        & (u <= u_high)
        & (v >= v_low)
        & (v <= v_high)
    )
    if not keep.any():
        raise ValueError("no Gaussian left under the height cut")
    area = max((u_high - u_low) * (v_high - v_low), 1e-12)
    pixel = max(
        max(u_high - u_low, v_high - v_low) / MAX_SIDE_PX,
        math.sqrt(area * GAUSSIANS_PER_PIXEL / keep.sum()),
    )
    width = max(1, math.ceil((u_high - u_low) / pixel))
    rows = max(1, math.ceil((v_high - v_low) / pixel))
    column = np.clip(((u[keep] - u_low) / pixel).astype(np.int64), 0, width - 1)
    row = np.clip(((v_high - v[keep]) / pixel).astype(np.int64), 0, rows - 1)
    rgba = _accumulate(row * width + column, colour[keep], opacity[keep], width * rows, np)
    png = png_bytes([line.tobytes() for line in rgba.reshape(rows, width * 4)], width, PNG_RGBA)
    bounds = (u_low, v_high - rows * pixel, u_low + width * pixel, v_high)
    return SplatLayer(
        png,
        tuple(float(value) for value in bounds),
        int(keep.sum()),
        float(floor),
        float(ceiling),
        float(cut),
        up,
        path.name,
    )


def _accumulate(index, colour, opacity, cells: int, np):
    """Opacity-weighted mean colour per pixel, alpha = 1 - exp(-total opacity): RGBA bytes."""
    weight = np.bincount(index, weights=opacity, minlength=cells)
    rgba = np.zeros((cells, 4), dtype=np.uint8)
    seen = weight > 0
    for channel in range(3):
        total = np.bincount(index, weights=opacity * colour[:, channel], minlength=cells)
        rgba[seen, channel] = np.clip(total[seen] / weight[seen], 0, 1) * CHANNEL_MAX
    rgba[:, 3] = (1.0 - np.exp(-weight)) * CHANNEL_MAX
    return rgba


def _colours(vertices, names, np):
    if all(f"f_dc_{i}" in names for i in range(3)):
        dc = np.stack([vertices[f"f_dc_{i}"] for i in range(3)], axis=1).astype(np.float64)
        return np.clip(0.5 + SH_C0 * dc, 0.0, 1.0)
    if all(name in names for name in ("red", "green", "blue")):
        rgb = np.stack([vertices[name] for name in ("red", "green", "blue")], axis=1)
        return rgb.astype(np.float64) / CHANNEL_MAX
    return np.full((len(vertices), 3), 0.8)


def _opacities(vertices, names, np):
    if "opacity" not in names:
        return np.ones(len(vertices))
    logit = vertices["opacity"].astype(np.float64)
    return 1.0 / (1.0 + np.exp(-logit))


def _read_ply(path: Path, np):
    """The vertex element of a binary little-endian PLY, as a numpy record array (memmap)."""
    with path.open("rb") as file:
        if file.readline().strip() != b"ply":
            raise ValueError(f"{path.name} is not a PLY file")
        properties: list[tuple[str, str]] = []
        count, element = 0, None  # the vertex data starts right after the header
        while True:
            line = file.readline()
            if not line:
                raise ValueError("PLY header without end_header")
            words = line.decode("ascii", "replace").split()
            if not words:
                continue
            if words[0] == "format" and words[1] != "binary_little_endian":
                raise ValueError(f"PLY format {words[1]}: only binary_little_endian is read")
            if words[0] == "element":
                if element is None and words[1] != "vertex":
                    raise ValueError("the PLY's first element must be the vertices")
                element = words[1]
                if element == "vertex":
                    count = int(words[2])
            if words[0] == "property" and element == "vertex":
                if words[1] == "list":
                    raise ValueError("list properties in the vertex element are not supported")
                if words[1] not in PLY_TYPES:
                    raise ValueError(f"PLY property type {words[1]} is not supported")
                properties.append((words[2], "<" + PLY_TYPES[words[1]]))
            if words[0] == "end_header":
                break
        offset = file.tell()
    names = {name for name, _ in properties}
    if not count or not {"x", "y", "z"} <= names:
        raise ValueError("the PLY has no vertices with x, y, z")
    return np.memmap(path, dtype=np.dtype(properties), mode="r", offset=offset, shape=(count,))
