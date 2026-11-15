"""The people database and the classification head: whose face is this?

One folder of reference photos per person (see the package docstring). Each person is scored by
the mean similarity of their TOP_K closest photos (one lucky photo is not enough: on drone
photos of 25-50 px faces, strangers beat the threshold against a single blurry reference). The
face is that person's when the score passes MATCH_THRESHOLD and beats the next person by
MATCH_MARGIN, else nobody. Keeping other people in the database makes the answer safer: a face
that looks more like someone else is never taken for the person we are looking for.
"""

import json
import logging
from pathlib import Path

import cv2
import numpy as np

from .faces import FaceEmbedder

PEOPLE_DIR = Path(__file__).resolve().parent / "people"
PHOTO_SUFFIXES = {".jpg", ".jpeg", ".png"}
TOP_K = 3  # a person's score: the mean similarity of their TOP_K closest reference faces
THRESHOLD_FILE = "threshold.json"  # optional, calibrated per reference gallery (face_model.py)
# Score above which the face is that person's. Lower than OpenCV's 0.363 for one pair of faces
# because it is a mean of TOP_K: on the 2026-11-15 04:35 flight it kept 145 of 172 faces of the
# target (one pair at 0.363: 148) and called nobody else him (one pair: 5 strangers).
MATCH_THRESHOLD = 0.32
MATCH_MARGIN = 0.08  # ...and by this much above the next person's score (else: nobody)
NO_RUNNER_UP = -1.0  # the score of the next person when there is only one (lowest cosine)

log = logging.getLogger("identify")


def is_photo(path: Path) -> bool:
    return path.suffix.lower() in PHOTO_SUFFIXES


class People:
    def __init__(self, embedder: FaceEmbedder, folder: Path = PEOPLE_DIR) -> None:
        references: dict[str, list[np.ndarray]] = {}
        for photo in sorted(path for path in folder.glob("*/*") if is_photo(path)):
            image = cv2.imread(str(photo))
            faces = embedder.faces(image) if image is not None else []
            if not faces:
                log.warning("no face found in %s: photo ignored", photo)
                continue
            face = faces[0]  # the largest face is the person
            references.setdefault(photo.parent.name, []).append(face.embedding)
        # Keep drone-sized references: discarding them favours phone views at demo altitude.
        self._vectors = {name: np.array(vectors, dtype=np.float32)
                         for name, vectors in references.items()}  # fmt: skip
        self._thresholds = {}
        for name in self._vectors:
            path = folder / name / THRESHOLD_FILE
            self._thresholds[name] = (
                float(json.loads(path.read_text())["threshold"])
                if path.is_file()
                else MATCH_THRESHOLD
            )
        for name, vectors in self._vectors.items():
            if len(vectors) < TOP_K:
                log.warning(
                    "%s has %d usable photos (%d wanted): their score is easier to reach",
                    name,
                    len(vectors),
                    TOP_K,
                )

    @property
    def names(self) -> list[str]:
        return sorted(self._vectors)

    def classify(self, embedding: np.ndarray) -> tuple:
        """(name, score) of the closest person; name is None if nobody is close enough, or if
        the two closest people are too alike to tell apart."""
        scores = sorted(
            ((self.score(name, embedding), name) for name in self._vectors),
            reverse=True,
        )
        if not scores:
            return None, 0.0
        best, name = scores[0]
        runner_up = scores[1][0] if len(scores) > 1 else NO_RUNNER_UP
        confident = best >= self._thresholds[name] and best - runner_up >= MATCH_MARGIN
        return (name if confident else None), best

    def score(self, name: str, embedding: np.ndarray) -> float:
        """Similarity to one person's reference gallery, before thresholding."""
        return _score(self._vectors[name] @ embedding)


def _score(similarities: np.ndarray) -> float:
    """Mean of the TOP_K highest cosine similarities (every vector has length 1)."""
    return float(np.sort(similarities)[-TOP_K:].mean())
