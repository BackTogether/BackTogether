"""The people database and the classification head: whose face is this?

One folder of reference photos per person (see the package docstring). A face is classified
as the person whose photos are most similar to it, or as nobody when even the best match is
too weak. Keeping other people in the database makes the answer safer: a face that looks more
like someone else is never taken for the person we are looking for.
"""

import logging
from pathlib import Path

import cv2
import numpy as np

from .faces import FaceEmbedder

PEOPLE_DIR = Path(__file__).resolve().parent / "people"
PHOTO_SUFFIXES = {".jpg", ".jpeg", ".png"}
# Cosine similarity above which SFace calls two faces the same person (OpenCV's own value).
MATCH_THRESHOLD = 0.363

log = logging.getLogger("identify")


def is_photo(path: Path) -> bool:
    return path.suffix.lower() in PHOTO_SUFFIXES


class People:
    def __init__(self, embedder: FaceEmbedder, folder: Path = PEOPLE_DIR) -> None:
        self._names = []  # one entry per reference photo
        vectors = []
        for photo in sorted(path for path in folder.glob("*/*") if is_photo(path)):
            image = cv2.imread(str(photo))
            faces = embedder.faces(image) if image is not None else []
            if not faces:
                log.warning("no face found in %s: photo ignored", photo)
                continue
            self._names.append(photo.parent.name)
            vectors.append(faces[0].embedding)  # the largest face is the person
        self._vectors = np.array(vectors, dtype=np.float32)

    @property
    def names(self) -> list[str]:
        return sorted(set(self._names))

    def classify(self, embedding: np.ndarray) -> tuple:
        """(name, similarity) of the closest person; name is None if nobody is close enough."""
        if not self._names:
            return None, 0.0
        similarities = self._vectors @ embedding  # cosine: every vector has length 1
        best = int(np.argmax(similarities))
        similarity = float(similarities[best])
        return (self._names[best] if similarity >= MATCH_THRESHOLD else None), similarity
