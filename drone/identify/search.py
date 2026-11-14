"""Look for one person in the frames the drone camera saved to a folder.

The first attempt is the fast one (real time). If the person is not there it may be a false
negative, so the next attempts look again, at full size and at every face, including any new
frame the drone saved meanwhile. Only then the search gives up.
"""

import logging
import time
from dataclasses import dataclass
from pathlib import Path

import cv2

from .faces import MAX_DETECTED, FaceEmbedder
from .people import PEOPLE_DIR, People, is_photo

ATTEMPTS = 3
RETRY_WAIT_S = 1.0  # lets the drone save new frames between attempts
FULL_SIZE = 0  # FaceEmbedder detect_width for the thorough look

log = logging.getLogger("identify")


@dataclass
class Sighting:
    frame: Path  # the frame where the person looks most like the reference photos
    box: tuple  # (left, top, width, height) of the face in that frame
    similarity: float  # cosine similarity with the closest reference photo
    attempt: int  # 1 = found by the fast look


def best_sighting(target: str, frames: list, embedder: FaceEmbedder, people: People) -> tuple:
    """(best face of `target` in the frames or None, number of faces looked at)."""
    best = None
    faces_seen = 0
    for frame in frames:
        image = cv2.imread(str(frame))
        if image is None:
            log.warning("cannot read %s", frame)
            continue
        for face in embedder.faces(image):
            faces_seen += 1
            name, similarity = people.classify(face.embedding)
            if name == target and (best is None or similarity > best.similarity):
                best = Sighting(frame, face.box, similarity, attempt=0)
    return best, faces_seen


def search(
    target: str, frames_dir: Path, people_dir: Path = PEOPLE_DIR, attempts: int = ATTEMPTS
) -> Sighting | None:
    """The best sighting of `target` in the folder, or None after `attempts` failed looks."""
    thorough = FaceEmbedder(detect_width=FULL_SIZE, max_faces=MAX_DETECTED)
    people = People(thorough, people_dir)
    if target not in people.names:
        raise ValueError(f"{target} has no usable photo in {people_dir} (known: {people.names})")
    fast = FaceEmbedder()

    for attempt in range(1, attempts + 1):
        frames = sorted(path for path in frames_dir.iterdir() if is_photo(path))
        started = time.monotonic()
        sighting, faces_seen = best_sighting(
            target, frames, fast if attempt == 1 else thorough, people
        )
        elapsed_ms = (time.monotonic() - started) * 1000
        log.info(
            "attempt %d/%d: %d frames, %d faces, %.0f ms per frame, %s",
            attempt,
            attempts,
            len(frames),
            faces_seen,
            elapsed_ms / max(len(frames), 1),
            "found" if sighting else "not found",
        )
        if sighting:
            sighting.attempt = attempt
            return sighting
        if attempt < attempts:
            time.sleep(RETRY_WAIT_S)
    return None
