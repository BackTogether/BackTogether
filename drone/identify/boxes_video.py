"""An MP4 of the tracker's view: every frame it processed, with the page's boxes drawn on it
(annotate.py), played back in real time (a frame is repeated until the next one is due, since
the tracker processes fewer frames per second than the drone sends)."""

import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from . import annotate
from .body_track import FrameResult

FPS = 15.0
CODEC = "mp4v"  # in every OpenCV build; QuickTime and VLC play it
MAX_REPEAT_S = 2.0  # a longer gap (no video for a while) is cut short, not frozen for minutes


class BoxesVideo:
    def __init__(self, recordings: Path, target: str) -> None:
        recordings.mkdir(exist_ok=True)
        self.path = recordings / f"boxes-{target}-{time.strftime('%Y%m%d-%H%M%S')}.mp4"
        self.frames = 0
        self._target = target
        self._writer: cv2.VideoWriter | None = None
        self._next = 0.0  # frame time the next video frame stands for

    def add(self, image: np.ndarray, result: FrameResult, frame_time: float) -> None:
        clock = datetime.fromtimestamp(frame_time).strftime("%H:%M:%S")
        view = annotate.draw(image, result, self._target, clock)
        if self._writer is None:
            height, width = view.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*CODEC)
            self._writer = cv2.VideoWriter(str(self.path), fourcc, FPS, (width, height))
            self._next = frame_time
        self._next = max(self._next, frame_time - MAX_REPEAT_S)
        while self._next <= frame_time:
            self._writer.write(view)
            self.frames += 1
            self._next += 1 / FPS

    def close(self) -> Path | None:
        """The video's path, or None if no frame arrived."""
        if self._writer is None:
            return None
        self._writer.release()
        return self.path
