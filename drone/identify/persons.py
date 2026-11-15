"""People (whole bodies) in a frame: visible from much higher up than faces.

YOLOX-S from the OpenCV Zoo (Apache-2.0, COCO, offline, CPU through OpenCV's dnn), only the
"person" class. Pre- and post-processing follow the zoo's own yolox.py / demo.py: RGB,
letterboxed to 640 x 640 with grey (114) padding, raw 0..255 values, grid decoding per stride.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from . import models

INPUT_SIZE = 640
PAD_VALUE = 114.0
STRIDES = (8, 16, 32)
PERSON_CLASS = 0  # COCO
SCORE_THRESHOLD = 0.35  # objectness x class score
NMS_THRESHOLD = 0.5
MIN_PERSON_PX = 12  # shorter boxes are too small to compare clothes


@dataclass
class Person:
    box: tuple  # (left, top, width, height) in pixels of the original frame
    score: float  # detector confidence, 0..1


def _grids() -> tuple[np.ndarray, np.ndarray]:
    """Cell coordinates and stride of every YOLOX output row."""
    grids, strides = [], []
    for stride in STRIDES:
        cells = INPUT_SIZE // stride
        xs, ys = np.meshgrid(np.arange(cells), np.arange(cells))
        grids.append(np.stack((xs, ys), 2).reshape(-1, 2))
        strides.append(np.full((cells * cells, 1), stride))
    return np.concatenate(grids).astype(np.float32), np.concatenate(strides).astype(np.float32)


class PersonDetector:
    def __init__(self) -> None:
        self._net = cv2.dnn.readNet(str(models.download(models.PERSON_DETECTOR)))
        self._grids, self._strides = _grids()

    def persons(self, image: np.ndarray) -> list[Person]:
        """Every person in a BGR frame, most confident first."""
        blob, scale = _letterbox(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        self._net.setInput(blob)
        rows = self._net.forward(self._net.getUnconnectedOutLayersNames())[0][0]
        centres = (rows[:, :2] + self._grids) * self._strides
        sizes = np.exp(rows[:, 2:4]) * self._strides
        scores = rows[:, 4] * rows[:, 5 + PERSON_CLASS]
        candidates = scores >= SCORE_THRESHOLD
        if not candidates.any():
            return []
        boxes = np.hstack([centres - sizes / 2, sizes])[candidates] / scale  # back to the frame
        scores = scores[candidates]
        keep = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), SCORE_THRESHOLD, NMS_THRESHOLD)
        found = [
            Person(tuple(int(v) for v in boxes[i]), float(scores[i]))
            for i in np.array(keep).flatten()
            if boxes[i][3] >= MIN_PERSON_PX
        ]
        return sorted(found, key=lambda person: person.score, reverse=True)


def _letterbox(rgb: np.ndarray) -> tuple[np.ndarray, float]:
    """The zoo's letterbox: fit inside 640 x 640, top-left aligned, grey padding."""
    scale = min(INPUT_SIZE / rgb.shape[0], INPUT_SIZE / rgb.shape[1])
    height, width = int(rgb.shape[0] * scale), int(rgb.shape[1] * scale)
    padded = np.full((INPUT_SIZE, INPUT_SIZE, 3), PAD_VALUE, dtype=np.float32)
    padded[:height, :width] = cv2.resize(rgb, (width, height)).astype(np.float32)
    return np.transpose(padded, (2, 0, 1))[np.newaxis], scale
