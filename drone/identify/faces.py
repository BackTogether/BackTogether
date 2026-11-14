"""Faces in an image and the vector (embedding) that describes each one."""

from dataclasses import dataclass

import cv2
import numpy as np

from . import models

DETECTION_SCORE = 0.7  # YuNet confidence to accept a face
NMS_THRESHOLD = 0.3
MAX_FACES = 50
MIN_FACE_PX = 24  # smaller faces give unreliable vectors


@dataclass
class Face:
    box: tuple  # (left, top, width, height) in pixels
    score: float  # detector confidence, 0..1
    embedding: np.ndarray  # unit-length vector from SFace


class FaceEmbedder:
    def __init__(self) -> None:
        detector_path = str(models.download(models.DETECTOR))
        recognizer_path = str(models.download(models.RECOGNIZER))
        self._detector = cv2.FaceDetectorYN.create(
            detector_path, "", (0, 0), DETECTION_SCORE, NMS_THRESHOLD, MAX_FACES
        )
        self._recognizer = cv2.FaceRecognizerSF.create(recognizer_path, "")

    def faces(self, image: np.ndarray) -> list[Face]:
        """Every face in a BGR image that is big enough to recognise, largest first."""
        height, width = image.shape[:2]
        self._detector.setInputSize((width, height))
        _, detections = self._detector.detect(image)
        found = []
        for detection in detections if detections is not None else []:
            left, top, face_width, face_height = (int(value) for value in detection[:4])
            if min(face_width, face_height) < MIN_FACE_PX:
                continue
            aligned = self._recognizer.alignCrop(image, detection)
            embedding = self._recognizer.feature(aligned).flatten()
            embedding = embedding / np.linalg.norm(embedding)
            box = (left, top, face_width, face_height)
            found.append(Face(box, float(detection[-1]), embedding))
        return sorted(found, key=lambda face: face.box[2] * face.box[3], reverse=True)
