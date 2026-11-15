"""Feed the lost-child mission: recognise the child by face, then track their body.

    uv run --offline --with opencv-contrib-python-headless python -m drone.identify.feed leo
    uv run --offline --with opencv-contrib-python-headless python -m drone.identify.feed leo \
        --url http://<mac>:8077

Reads the drone SDK server's latest frame (GET /frame.jpg, with its X-Frame-Time) and, for each:
- POST /vision: every face and person and the tracked target, for the web page's video overlay;
- POST /mission/target: the child's box with that frame time (drone/sdk/API.md, "Mission"). A
  body box is sent with anchor "feet" (the mission puts its floor contact point on the floor),
  a head box (no body around the face yet) with anchor "centre" (target_height_m applies).
The tracking itself is body_track.py.

On recorded frames, without a server (annotated copies in --out, a log line per frame):

    uv run --offline --with opencv-contrib-python-headless python -m drone.identify.feed leo \
        --frames recordings/frames/<session> --first 40 --count 90 --out /tmp/annotated
"""

import argparse
import http.client
import json
import logging
import time
import urllib.request
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from . import annotate
from .__main__ import load_people
from .body_model import BodyModel
from .body_track import BODY, BodyTracker, FrameResult, make_tracker
from .collect import Collector
from .faces import FaceEmbedder
from .people import PEOPLE_DIR, People, is_photo
from .persons import PersonDetector

DEFAULT_URL = "http://localhost:8077"
FRAME_TIME_HEADER = "X-Frame-Time"
HTTP_TIMEOUT_S = 2.0
IDLE_S = 0.05  # no new frame yet
FULL_SIZE = 0  # faces searched at full resolution: they are small from the patrol altitude
FACES_EMBEDDED = 6  # largest faces compared with the database per frame (~5 ms each)
ANCHOR_FEET = "feet"  # drone/sdk/mission.py
ANCHOR_CENTRE = "centre"
RECORDED_FPS = 30.0  # the X1's stream: frame times of a recording

log = logging.getLogger("identify")

Box = tuple[int, int, int, int]


class DroneServer:
    """The SDK server: its latest frame, each new one once, and the two reports. Network errors
    (wifi) only skip a frame or a report: the loop keeps going."""

    def __init__(self, url: str) -> None:
        self.url = url.rstrip("/")
        self._last_time = 0.0

    def next(self) -> tuple[float, np.ndarray] | None:
        """(frame time, BGR image) of a frame not returned before, or None."""
        try:
            with urllib.request.urlopen(f"{self.url}/frame.jpg", timeout=HTTP_TIMEOUT_S) as reply:
                frame_time = float(reply.headers.get(FRAME_TIME_HEADER) or time.time())
                data = reply.read()
        except (OSError, http.client.HTTPException, ValueError) as error:
            log.debug("no frame: %s", error)
            return None
        if frame_time <= self._last_time:
            return None
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return None
        self._last_time = frame_time
        return frame_time, image

    def post(self, path: str, body: dict[str, Any]) -> dict[str, Any] | str:
        """The JSON answer, or why it was not sent."""
        request = urllib.request.Request(
            f"{self.url}{path}",
            data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=HTTP_TIMEOUT_S) as reply:
                return json.load(reply)
        except (OSError, http.client.HTTPException, ValueError) as error:
            return f"not sent: {error}"


def fractions(box: Box, shape: tuple[int, ...]) -> list[float]:
    """[x0, y0, x1, y1] as fractions of the image (the API's bbox), inside the image."""
    left, top, width, height = box
    h, w = shape[:2]
    x0, y0 = min(max(left / w, 0.0), 1.0), min(max(top / h, 0.0), 1.0)
    x1, y1 = min(max((left + width) / w, 0.0), 1.0), min(max((top + height) / h, 0.0), 1.0)
    return [round(x0, 4), round(y0, 4), round(x1, 4), round(y1, 4)]


def vision_body(
    result: FrameResult, frame_time: float, shape: tuple[int, ...], target: str
) -> dict[str, Any]:
    """POST /vision's body (drone/sdk/API.md, "Vision")."""
    body: dict[str, Any] = {
        "frame_time": frame_time,
        "state": result.state,
        "name": target,
        "ms": round(result.ms, 1),
        "faces": [
            {
                "bbox": fractions(f.box, shape),
                "name": f.name,
                "similarity": round(f.similarity, 3),
                "target": f.target,
            }
            for f in result.faces
        ],
        "persons": [
            {"bbox": fractions(p.box, shape), "score": round(p.score, 3)} for p in result.persons
        ],
        "target": None,
    }
    if result.target is not None:
        t = result.target
        body["target"] = {
            "bbox": fractions(t.box, shape),
            "kind": t.kind,
            "confidence": t.confidence,
            "source": t.source,
            "similarity": None if t.similarity is None else round(t.similarity, 3),
            "face_age_s": t.face_age_s,
        }
    return body


def mission_body(
    result: FrameResult, frame_time: float, shape: tuple[int, ...], target: str
) -> dict[str, Any] | None:
    """POST /mission/target's body, or None without a target."""
    if result.target is None:
        return None
    t = result.target
    return {
        "frame_time": frame_time,
        "bbox": fractions(t.box, shape),
        "confidence": t.confidence,
        "label": f"{target} {t.kind} ({t.source})",
        "anchor": ANCHOR_FEET if t.kind == BODY else ANCHOR_CENTRE,
    }


def make_tracker_for(target: str, people: People) -> BodyTracker:
    embedder = FaceEmbedder(detect_width=FULL_SIZE, max_faces=FACES_EMBEDDED)
    return BodyTracker(target, embedder, people, PersonDetector(), load_body_model())


def load_body_model() -> BodyModel | None:
    """The body model if it was trained (body_model.py), else None: faces only."""
    try:
        return BodyModel()
    except FileNotFoundError as error:
        log.info("no body model (%s): the lock needs the face", error)
        return None


def run_live(target: str, url: str, people: People, collector: Collector | None = None) -> None:
    server = DroneServer(url)
    tracker = make_tracker_for(target, people)
    last = None
    while True:
        frame = server.next()
        if frame is None:
            time.sleep(IDLE_S)
            continue
        frame_time, image = frame
        result = tracker.update(image, frame_time)
        if collector is not None:
            collector.add(image, result, frame_time)
        server.post("/vision", vision_body(result, frame_time, image.shape, target))
        report = mission_body(result, frame_time, image.shape, target)
        answer = server.post("/mission/target", report) if report else None
        summary = _summary(result, answer)
        if summary != last:
            log.info("%s", summary)
            last = summary


def run_recorded(target: str, people: People, args: argparse.Namespace) -> None:
    """The tracker over recorded frames in order; annotated copies in --out."""
    frames = sorted(path for path in args.frames.iterdir() if is_photo(path))
    frames = frames[args.first : args.first + args.count if args.count else None]
    tracker = make_tracker_for(target, people)
    args.out.mkdir(parents=True, exist_ok=True)
    total_ms = 0.0
    for index, frame in enumerate(frames):
        image = cv2.imread(str(frame))
        if image is None:
            continue
        result = tracker.update(image, index / RECORDED_FPS)
        total_ms += result.ms
        log.info("%s %s", frame.name, _summary(result, None))
        view = annotate.draw(image, result, target, frame.name)
        cv2.imwrite(str(args.out / frame.name), view)
    log.info("%d frames, %.0f ms per frame", len(frames), total_ms / max(len(frames), 1))


def _summary(result: FrameResult, answer: dict[str, Any] | str | None) -> str:
    target = result.target
    seen = "nothing" if target is None else f"{target.kind} {target.confidence} ({target.source})"
    faces = ", ".join(f"{f.name or '?'} {f.similarity:.2f}" for f in result.faces) or "no face"
    text = f"{result.state}: {seen}; {len(result.persons)} persons; faces: {faces}"
    if isinstance(answer, dict):
        text += f"; mission {'accepted' if answer.get('accepted') else answer.get('reason')}"
    elif answer:
        text += f"; mission {answer}"
    return text


def main() -> None:
    parser = argparse.ArgumentParser(description="recognise the child, track the body, feed it")
    parser.add_argument("target", help="the person to look for (folder name in the database)")
    parser.add_argument("--url", default=DEFAULT_URL, help="drone SDK server")
    parser.add_argument("--people", type=Path, default=PEOPLE_DIR, help="people database")
    parser.add_argument("--frames", type=Path, help="recorded frames instead of the server")
    parser.add_argument("--first", type=int, default=0, help="--frames: first frame index")
    parser.add_argument("--count", type=int, default=0, help="--frames: how many (0 = all)")
    parser.add_argument("--out", type=Path, default=Path("annotated"), help="--frames: output")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")
    people = load_people(args.people)
    if args.target not in people.names:
        parser.error(f"{args.target} has no usable photo in {args.people} ({people.names})")
    log.info("tracker %s, faces up to %d per frame", make_tracker(), FACES_EMBEDDED)
    try:
        if args.frames:
            run_recorded(args.target, people, args)
        else:
            log.info("feeding the mission at %s with %s", args.url, args.target)
            run_live(args.target, args.url, people)
    except KeyboardInterrupt:
        print("stopped")


if __name__ == "__main__":
    main()
