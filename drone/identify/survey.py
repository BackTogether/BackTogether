"""Can faces be recognised in real drone footage? A report per flight session, no target needed.

Reads the frames that `drone.hoverctl.export` writes, one folder per session:

    recordings/frames/hover_video_<date>-<time>/frame_00001.jpg ...

    uv run --with opencv-python-headless python -m drone.identify.survey recordings/frames

For each session: how many frames show a face, how big the faces are (SFace needs about
MIN_FACE_PX or more) and how long one frame takes. Faces are searched at full size here, so
the report shows every face the drone could ever give us.
"""

import argparse
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from .faces import MAX_DETECTED, MIN_FACE_PX, FaceEmbedder
from .people import is_photo

FULL_SIZE = 0
SIZE_BUCKETS_PX = (24, 40, 64, 100)  # report how many faces are at least this wide


@dataclass
class SessionReport:
    name: str
    frames: int = 0
    frames_with_face: int = 0
    face_widths: list = field(default_factory=list)
    seconds: float = 0.0

    def lines(self) -> list[str]:
        widths = np.array(self.face_widths or [0])
        buckets = "  ".join(
            f">={size}px: {int((widths >= size).sum())}" for size in SIZE_BUCKETS_PX
        )
        return [
            f"{self.name}: {self.frames} frames, {self.frames_with_face} with a face "
            f"({100 * self.frames_with_face / max(self.frames, 1):.0f}%), "
            f"{len(self.face_widths)} faces",
            f"    face width px: median {np.median(widths):.0f}, max {widths.max():.0f}  |  "
            f"{buckets}",
            f"    {1000 * self.seconds / max(self.frames, 1):.0f} ms per frame",
        ]


def sessions(root: Path) -> list[Path]:
    """Session folders under root, or root itself when it holds the frames directly."""
    folders = sorted(path for path in root.iterdir() if path.is_dir())
    return folders or [root]


def frames_of(folder: Path, every: int) -> list[Path]:
    return sorted(path for path in folder.iterdir() if is_photo(path))[::every]


def survey_session(folder: Path, embedder: FaceEmbedder, every: int) -> SessionReport:
    report = SessionReport(folder.name)
    started = time.monotonic()
    for frame in frames_of(folder, every):
        image = cv2.imread(str(frame))
        if image is None:
            continue
        report.frames += 1
        faces = embedder.faces(image)
        report.frames_with_face += bool(faces)
        report.face_widths += [face.box[2] for face in faces]
    report.seconds = time.monotonic() - started
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="face sizes in drone footage, per session")
    parser.add_argument("root", type=Path, help="recordings/frames or one session folder")
    parser.add_argument("--every", type=int, default=1, help="use one frame out of N")
    args = parser.parse_args()
    # Every face, however small, so the report also counts the ones too small to recognise.
    embedder = FaceEmbedder(detect_width=FULL_SIZE, max_faces=MAX_DETECTED, min_face_px=1)
    total = SessionReport("ALL")
    for folder in sessions(args.root):
        report = survey_session(folder, embedder, args.every)
        print("\n".join(report.lines()), flush=True)
        total.frames += report.frames
        total.frames_with_face += report.frames_with_face
        total.face_widths += report.face_widths
        total.seconds += report.seconds
    print("\n".join(total.lines()))
    usable = sum(width >= MIN_FACE_PX for width in total.face_widths)
    print(f"faces big enough to recognise (>= {MIN_FACE_PX} px): {usable}")


if __name__ == "__main__":
    main()
