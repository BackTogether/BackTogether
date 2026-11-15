"""What the detector saw in each frame, kept a few seconds for the web page's video overlay.

The detector (drone/identify/feed.py, or the simulator's fake one) posts one result per frame it
looked at (POST /vision, API.md "Vision"): every face, every person, the tracked target and the
lock state, with the frame's X-Frame-Time. The page asks for the results newer than the last one
it has (GET /vision?since=) and draws the one whose frame time matches the frame it shows.
"""

import math
import threading
import time
from collections import deque
from typing import Any

KEEP_S = 5.0  # results older than this (frame time) are dropped
MAX_RESULTS = 200  # ~30 fps for KEEP_S, with room
MAX_BOXES = 50  # faces or persons per result
MAX_TEXT = 64
STATES = ("searching", "locked")
TARGET_KINDS = ("body", "head")


def parse_result(body: dict[str, Any]) -> dict[str, Any]:
    """A posted result, checked and trimmed; ValueError says what is wrong."""
    frame_time = _finite(body.get("frame_time"), "frame_time")
    state = body.get("state", STATES[0])
    if state not in STATES:
        raise ValueError(f'"state" must be one of {", ".join(STATES)}')
    faces = [_face(item) for item in _items(body, "faces")]
    persons = [_person(item) for item in _items(body, "persons")]
    target = body.get("target")
    return {
        "frame_time": frame_time,
        "state": state,
        "name": _text(body.get("name")),
        "faces": faces,
        "persons": persons,
        "target": None if target is None else _target(target),
        "ms": _optional(body.get("ms"), "ms"),
    }


class VisionLog:
    """The last KEEP_S of results, by frame time. Thread-safe."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._results: deque[dict[str, Any]] = deque(maxlen=MAX_RESULTS)

    def add(self, result: dict[str, Any]) -> None:
        result = {**result, "received_at": round(time.time(), 3)}
        with self._lock:
            self._results.append(result)
            while self._results and self._results[0]["frame_time"] < time.time() - KEEP_S:
                self._results.popleft()

    def since(self, frame_time: float) -> list[dict[str, Any]]:
        """Results of frames after `frame_time`, oldest first."""
        with self._lock:
            return [r for r in self._results if r["frame_time"] > frame_time]


def _items(body: dict[str, Any], key: str) -> list[Any]:
    items = body.get(key) or []
    if not isinstance(items, list):
        raise ValueError(f'"{key}" must be a list')
    return items[:MAX_BOXES]


def _face(item: Any) -> dict[str, Any]:
    face = _object(item, "face")
    return {
        "bbox": _bbox(face.get("bbox")),
        "name": _text(face.get("name")),
        "similarity": _optional(face.get("similarity"), "similarity"),
        "target": bool(face.get("target", False)),
    }


def _person(item: Any) -> dict[str, Any]:
    person = _object(item, "person")
    return {"bbox": _bbox(person.get("bbox")), "score": _optional(person.get("score"), "score")}


def _target(item: Any) -> dict[str, Any]:
    target = _object(item, "target")
    kind = target.get("kind", TARGET_KINDS[0])
    if kind not in TARGET_KINDS:
        raise ValueError(f'target "kind" must be one of {", ".join(TARGET_KINDS)}')
    return {
        "bbox": _bbox(target.get("bbox")),
        "kind": kind,
        "confidence": _optional(target.get("confidence"), "confidence"),
        "source": _text(target.get("source")),
        "similarity": _optional(target.get("similarity"), "similarity"),
        "face_age_s": _optional(target.get("face_age_s"), "face_age_s"),
    }


def _object(item: Any, what: str) -> dict[str, Any]:
    if not isinstance(item, dict):
        raise ValueError(f'each {what} is {{"bbox": [x0, y0, x1, y1], ...}}')
    return item


def _bbox(value: Any) -> list[float]:
    if not isinstance(value, list) or len(value) != 4:
        raise ValueError('"bbox" must be [x0, y0, x1, y1], fractions of the image')
    x0, y0, x1, y1 = (_finite(v, "bbox") for v in value)
    if not (0.0 <= x0 < x1 <= 1.0 and 0.0 <= y0 < y1 <= 1.0):
        raise ValueError('"bbox" needs 0 <= x0 < x1 <= 1 and 0 <= y0 < y1 <= 1')
    return [round(v, 4) for v in (x0, y0, x1, y1)]


def _finite(value: Any, key: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
        raise ValueError(f'"{key}" must be a number')
    return float(value)


def _optional(value: Any, key: str) -> float | None:
    return None if value is None else round(_finite(value, key), 3)


def _text(value: Any) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise ValueError("names and labels must be strings")
    return value[:MAX_TEXT]
