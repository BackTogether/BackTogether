"""Faces in an image and the vector (embedding) that describes each one."""

from dataclasses import dataclass

import cv2
import numpy as np

from . import models

DETECTION_SCORE = 0.7  # YuNet confidence to accept a face
NMS_THRESHOLD = 0.3
MAX_DETECTED = 50
DETECTION_VALUES = 15  # box (4) + five landmarks (10) + score
DETECT_WIDTH = 640  # 11 ms instead of 47 ms at 1280 px on a laptop CPU
MAX_EMBEDDED = 3  # each embedding costs about 25 ms
MIN_FACE_PX = 24  # smaller faces give unreliable vectors


@dataclass
class Face:
    box: tuple  # (left, top, width, height) in pixels
    score: float  # detector confidence, 0..1
    embedding: np.ndarray  # unit-length vector from SFace


class FaceEmbedder:
    def __init__(self, detect_width: int = DETECT_WIDTH, max_faces: int = MAX_EMBEDDED) -> None:
        """`detect_width`: faces are searched on a copy this wide (0 = full size, slower, finds
        smaller faces). `max_faces`: only the largest ones are embedded, to bound the time."""
        detector_path = str(models.download(models.DETECTOR))
        recognizer_path = str(models.download(models.RECOGNIZER))
        self._detector = cv2.FaceDetectorYN.create(
            detector_path, "", (0, 0), DETECTION_SCORE, NMS_THRESHOLD, MAX_DETECTED
        )
        self._recognizer = cv2.FaceRecognizerSF.create(recognizer_path, "")
        self._detect_width = detect_width
        self._max_faces = max_faces

    def faces(self, image: np.ndarray) -> list[Face]:
        """The largest faces in a BGR image that are big enough to recognise, largest first."""
        detections = sorted(self._detect(image), key=lambda d: d[2] * d[3], reverse=True)
        found = []
        for detection in detections[: self._max_faces]:
            left, top, width, height = (int(value) for value in detection[:4])
            if min(width, height) < MIN_FACE_PX:
                continue
            # Align and embed from the full-size image: the vector needs every pixel of the face.
            aligned = self._recognizer.alignCrop(image, detection)
            embedding = self._recognizer.feature(aligned).flatten()
            embedding = embedding / np.linalg.norm(embedding)
            found.append(Face((left, top, width, height), float(detection[-1]), embedding))
        return found

    def _detect(self, image: np.ndarray) -> np.ndarray:
        """YuNet rows (box, 5 landmarks, score) in full-size pixel coordinates."""
        height, width = image.shape[:2]
        scale = 1.0
        if 0 < self._detect_width < width:
            scale = self._detect_width / width
            image = cv2.resize(image, (self._detect_width, round(height * scale)))
        self._detector.setInputSize((image.shape[1], image.shape[0]))
        _, detections = self._detector.detect(image)
        if detections is None:
            return np.empty((0, DETECTION_VALUES), dtype=np.float32)
        detections[:, :-1] /= scale  # everything but the score is a coordinate
        return detections
