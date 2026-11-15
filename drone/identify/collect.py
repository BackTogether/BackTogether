"""Frames for the body model (body_model.py), saved while the drone looks for the target.

    recordings/collect-<time>/seen/  the target placed by the face, the person detector or the
                                     body model (one frame per SEEN_EVERY_S)
    recordings/collect-<time>/lost/  after a sighting, the target out of sight or only the
                                     tracker's guess, for LOST_AFTER_S (one per LOST_EVERY_S):
                                     the views the model misses, the ones worth teaching it

Raw frames (no overlay) with a .json each: the frame time, the tracker state, the target's box
and source, every person box. Sort them by eye, then retrain: frames with the target go to
drone/identify/body/drone/ (the trainer picks the target there by outfit), frames with only
other people go to a negatives folder (body_model train --negatives <folder>).
"""

import json
import time
from pathlib import Path

import cv2
import numpy as np

from .body_track import FrameResult

SEEN = "seen"
LOST = "lost"
EVERY_S = {SEEN: 1.0, LOST: 0.5}  # the lost views are rarer and more useful
LOST_AFTER_S = 15.0
MAX_FRAMES = 600  # ~45 MB: a guard for a long flight
GUESS = "tracker"  # body_track.py: a source that is only the visual tracker's guess


class Collector:
    def __init__(self, recordings: Path) -> None:
        self.folder = recordings / f"collect-{time.strftime('%Y%m%d-%H%M%S')}"
        self.saved = 0
        self._seen_at: float | None = None
        self._last = {SEEN: 0.0, LOST: 0.0}

    def add(self, image: np.ndarray, result: FrameResult, frame_time: float) -> None:
        kind = self._kind(result, frame_time)
        if kind is None or self.saved >= MAX_FRAMES:
            return
        if frame_time - self._last[kind] < EVERY_S[kind]:
            return
        self._last[kind] = frame_time
        folder = self.folder / kind
        folder.mkdir(parents=True, exist_ok=True)
        stem = f"{frame_time:.2f}"
        cv2.imwrite(str(folder / f"{stem}.jpg"), image)
        target = result.target
        info = {
            "frame_time": frame_time,
            "state": result.state,
            "target": None if target is None else {"box": target.box, "source": target.source},
            "persons": [person.box for person in result.persons],
        }
        (folder / f"{stem}.json").write_text(json.dumps(info) + "\n")
        self.saved += 1

    def _kind(self, result: FrameResult, frame_time: float) -> str | None:
        target = result.target
        if target is not None and target.source != GUESS:
            self._seen_at = frame_time
            return SEEN
        if self._seen_at is not None and frame_time - self._seen_at <= LOST_AFTER_S:
            return LOST
        return None
