"""Live search: from the moment it starts, classify every new frame the drone saves.

Start it before (or while) the drone flies. It watches the newest session folder under
recordings/frames/ (or the folder given) and looks at each new frame as soon as it is written,
always the newest one: when the model is slower than the camera, old frames are skipped so the
answer is never behind the drone.

A single good frame could be a look-alike, so a person counts as found after CONFIRM_FRAMES
matching frames. With no match for `timeout_s` the search fails.
"""

import logging
import time
from pathlib import Path

import cv2

from .faces import FaceEmbedder
from .people import People, is_photo
from .search import Sighting

POLL_S = 0.05
CONFIRM_FRAMES = 2  # matching frames needed, so one look-alike frame is not a positive
SEARCH_TIMEOUT_S = 120.0
WRITE_SETTLE_S = 0.05  # a frame younger than this may still be half written

log = logging.getLogger("identify")


def newest_session(root: Path) -> Path:
    """The session folder the drone is writing to now (or root when it holds frames itself)."""
    folders = [path for path in root.iterdir() if path.is_dir()]
    return max(folders, key=lambda path: path.stat().st_mtime) if folders else root


def newest_frame(folder: Path, after: Path | None) -> Path | None:
    """The newest complete frame written after `after`, or None if there is none yet."""
    frames = sorted(path for path in folder.iterdir() if is_photo(path))
    if not frames or (after is not None and frames[-1] <= after):
        return None
    if time.time() - frames[-1].stat().st_mtime < WRITE_SETTLE_S:
        return frames[-2] if len(frames) > 1 and (after is None or frames[-2] > after) else None
    return frames[-1]


def watch(
    target: str,
    root: Path,
    embedder: FaceEmbedder,
    people: People,
    timeout_s: float = SEARCH_TIMEOUT_S,
) -> Sighting | None:
    """Block until `target` is confirmed in the live frames (a Sighting) or timeout (None)."""
    deadline = time.monotonic() + timeout_s
    last = None
    matches = 0
    looked = 0
    best = None
    while time.monotonic() < deadline:
        folder = newest_session(root) if root.is_dir() else root
        frame = newest_frame(folder, last) if folder.is_dir() else None
        if frame is None:
            time.sleep(POLL_S)
            continue
        last = frame
        image = cv2.imread(str(frame))
        if image is None:
            continue
        looked += 1
        found = None
        for face in embedder.faces(image):
            name, similarity = people.classify(face.embedding)
            log.info(
                "%s: face %dpx -> %s %.2f",
                frame.name,
                face.box[2],
                name or "unknown",
                similarity,
            )
            if name == target and (found is None or similarity > found.similarity):
                found = Sighting(frame, face.box, similarity, attempt=matches + 1)
        matches = matches + 1 if found else 0
        if found:
            log.info(
                "%s: %s %.2f (%d/%d)", frame.name, target, found.similarity, matches, CONFIRM_FRAMES
            )
            best = found if best is None or found.similarity > best.similarity else best
        if matches >= CONFIRM_FRAMES:
            log.info("confirmed after looking at %d frames", looked)
            return best
    log.info("timeout after looking at %d frames", looked)
    return None
