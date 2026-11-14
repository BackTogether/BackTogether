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
from collections.abc import Callable, Iterator
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


def live_frames(root: Path, timeout_s: float) -> Iterator[tuple]:
    """Yield (path, image) for each new frame the drone saves, newest first, for timeout_s."""
    deadline = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < deadline:
        folder = newest_session(root) if root.is_dir() else root
        frame = newest_frame(folder, last) if folder.is_dir() else None
        if frame is None:
            time.sleep(POLL_S)
            continue
        last = frame
        image = cv2.imread(str(frame))
        if image is not None:
            yield frame, image


def classified_faces(frame: Path, image, embedder: FaceEmbedder, people: People) -> list:
    """[(name or None, Sighting)] for every face in the frame, logged as it is seen."""
    result = []
    for face in embedder.faces(image):
        name, similarity = people.classify(face.embedding)
        log.info("%s: face %dpx -> %s %.2f", frame.name, face.box[2], name or "unknown", similarity)
        result.append((name, Sighting(frame, face.box, similarity, attempt=0)))
    return result


def watch(
    target: str,
    root: Path,
    embedder: FaceEmbedder,
    people: People,
    timeout_s: float = SEARCH_TIMEOUT_S,
) -> Sighting | None:
    """Block until `target` is confirmed in the live frames (a Sighting) or timeout (None)."""
    matches = 0
    looked = 0
    best = None
    for frame, image in live_frames(root, timeout_s):
        looked += 1
        sightings = [
            s for name, s in classified_faces(frame, image, embedder, people) if name == target
        ]
        found = max(sightings, key=lambda s: s.similarity, default=None)
        matches = matches + 1 if found else 0
        if found:
            found.attempt = matches
            log.info(
                "%s: %s %.2f (%d/%d)", frame.name, target, found.similarity, matches, CONFIRM_FRAMES
            )
            best = found if best is None or found.similarity > best.similarity else best
        if matches >= CONFIRM_FRAMES:
            log.info("confirmed after looking at %d frames", looked)
            return best
    log.info("timeout after looking at %d frames", looked)
    return None


def recognise_all(
    root: Path,
    embedder: FaceEmbedder,
    people: People,
    on_recognised: Callable[[str, Sighting], None],
    timeout_s: float = SEARCH_TIMEOUT_S,
) -> dict:
    """Check every face of the live frames against the whole database. Calls
    on_recognised(name, sighting) the first time each person is confirmed; returns
    {name: best sighting} when the timeout passes."""
    in_a_row = {}  # name -> consecutive frames where that person was seen
    best = {}
    for frame, image in live_frames(root, timeout_s):
        seen = {}
        for name, sighting in classified_faces(frame, image, embedder, people):
            if name and (name not in seen or sighting.similarity > seen[name].similarity):
                seen[name] = sighting
        in_a_row = {name: in_a_row.get(name, 0) + 1 for name in seen}
        for name, sighting in seen.items():
            if in_a_row[name] < CONFIRM_FRAMES:
                continue
            if name not in best:
                log.info("recognised %s (%.2f) in %s", name, sighting.similarity, frame.name)
                on_recognised(name, sighting)
            if name not in best or sighting.similarity > best[name].similarity:
                best[name] = sighting
    return best
