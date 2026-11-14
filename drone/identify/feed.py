"""Feed the lost-child mission: recognise the child by face, then keep tracking them without it.

    uv run --with opencv-contrib-python-headless python -m drone.identify.feed leo
    uv run --with opencv-contrib-python-headless python -m drone.identify.feed leo \
        --url http://<mac>:8077

Reads the drone SDK server's latest frame (GET /frame.jpg, with its X-Frame-Time) and posts the
child's box to POST /mission/target (drone/sdk/API.md, "Mission"), with that frame time.

Why a tracker: the approach looks at the child more and more from above (camera down to -90
deg), where no face is visible. So:
- searching: every face is checked against the database; the target seen in CONFIRM_FRAMES
  frames in a row is locked;
- locked: a tracker follows the head box (the face box grown to take in the hair, which stays
  visible from above). Whenever the target's face is seen again the tracker restarts on it
  (fixes drift); a face of someone else inside the box drops the lock (the tracker jumped to
  another person); a tracker failure for LOST_S goes back to searching.

The posted box is the head, so start the mission with target_height_m = the child's head height
(about 1.0 standing, 0.6 sitting). CSRT (opencv-contrib) tracks much better than MIL, the only
tracker in plain opencv without extra model files.
"""

import argparse
import http.client
import json
import logging
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

from .__main__ import load_people
from .faces import FaceEmbedder
from .live import CONFIRM_FRAMES
from .people import PEOPLE_DIR, People

DEFAULT_URL = "http://localhost:8077"
FRAME_TIME_HEADER = "X-Frame-Time"
HTTP_TIMEOUT_S = 2.0
IDLE_S = 0.05  # no new frame yet
HEAD_GROW = 1.6  # head box = face box scaled by this, around the face centre...
HEAD_LIFT = 0.15  # ...and moved up by this fraction of the face height (hair)
TRACK_WIDTH = 640  # the tracker runs on a copy this wide (speed)
LOST_S = 2.0  # tracker failing this long: back to searching
FACE_CONFIDENCE = 0.9  # posted confidence: the target's face is in the frame
TRACK_CONFIDENCE = 0.7  # only the tracker (MissionConfig.min_confidence is 0.5)
LABEL_FACE = "face"
LABEL_TRACK = "track"

log = logging.getLogger("identify")

Box = tuple[int, int, int, int]  # left, top, width, height in pixels


def make_tracker() -> cv2.Tracker:
    """CSRT when opencv-contrib is installed, else MIL."""
    if hasattr(cv2, "TrackerCSRT_create"):
        return cv2.TrackerCSRT_create()
    return cv2.TrackerMIL_create()


class DroneFrames:
    """The SDK server's latest frame, each new one once. Network errors (wifi) only skip a
    frame or a report: the loop keeps going."""

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

    def report(self, frame_time: float, bbox: list[float], confidence: float, label: str) -> str:
        """POST /mission/target; the mission's answer (state, or why it was refused)."""
        body = {"frame_time": frame_time, "bbox": bbox, "confidence": confidence, "label": label}
        request = urllib.request.Request(
            f"{self.url}/mission/target",
            data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=HTTP_TIMEOUT_S) as reply:
                answer = json.load(reply)
        except (OSError, http.client.HTTPException, ValueError) as error:
            return f"not sent: {error}"
        accepted = "accepted" if answer.get("accepted") else "refused"
        return f"{accepted} ({answer.get('reason') or answer.get('state')})"


class ChildTracker:
    """Face recognition until the target is confirmed, then a tracker on the head."""

    def __init__(self, target: str, embedder: FaceEmbedder, people: People) -> None:
        self.target = target
        self.embedder = embedder
        self.people = people
        self.in_a_row = 0
        self.tracker: cv2.Tracker | None = None
        self.scale = 1.0
        self.failing_since: float | None = None

    @property
    def locked(self) -> bool:
        return self.tracker is not None

    def update(self, image: np.ndarray) -> tuple[Box, float, str] | None:
        """(head box, confidence, label) of the child in this frame, or None."""
        target_face, other_faces = self._faces(image)
        if target_face is not None:
            self.in_a_row += 1
            if self.locked or self.in_a_row >= CONFIRM_FRAMES:
                head = head_box(target_face, image.shape)
                self._start_tracker(image, head)
                return head, FACE_CONFIDENCE, LABEL_FACE
            return None
        self.in_a_row = 0
        if not self.locked:
            return None
        box = self._track(image)
        if box is None:
            return None
        if any(_centre_inside(face, box) for face in other_faces):
            log.info("someone else's face in the tracked box: lock dropped")
            self.tracker = None
            return None
        return box, TRACK_CONFIDENCE, LABEL_TRACK

    def _faces(self, image: np.ndarray) -> tuple[Box | None, list[Box]]:
        """The target's face (the most similar one) and the recognised faces of others.
        Unrecognised faces are neither: from far or the side they may be the child."""
        best: tuple[float, Box] | None = None
        others = []
        for face in self.embedder.faces(image):
            name, similarity = self.people.classify(face.embedding)
            if name == self.target:
                if best is None or similarity > best[0]:
                    best = (similarity, face.box)
            elif name is not None:
                others.append(face.box)
        return (best[1] if best else None), others

    def _start_tracker(self, image: np.ndarray, box: Box) -> None:
        if not self.locked:
            log.info("%s locked", self.target)
        small = self._small(image)
        self.tracker = make_tracker()
        self.tracker.init(small, tuple(round(v * self.scale) for v in box))
        self.failing_since = None

    def _track(self, image: np.ndarray) -> Box | None:
        ok, small_box = self.tracker.update(self._small(image))
        if ok:
            self.failing_since = None
            return tuple(round(v / self.scale) for v in small_box)
        now = time.monotonic()
        self.failing_since = self.failing_since or now
        if now - self.failing_since > LOST_S:
            log.info("tracker lost %s for %.0f s: searching again", self.target, LOST_S)
            self.tracker = None
        return None

    def _small(self, image: np.ndarray) -> np.ndarray:
        width = image.shape[1]
        self.scale = min(1.0, TRACK_WIDTH / width)
        if self.scale == 1.0:
            return image
        return cv2.resize(image, None, fx=self.scale, fy=self.scale)


def head_box(face: Box, shape: tuple[int, ...]) -> Box:
    """The face box grown around its centre and lifted to take in the hair, inside the image."""
    left, top, width, height = face
    cx, cy = left + width / 2, top + height / 2 - HEAD_LIFT * height
    half_w, half_h = width * HEAD_GROW / 2, height * HEAD_GROW / 2
    x0, y0 = max(0, round(cx - half_w)), max(0, round(cy - half_h))
    x1, y1 = min(shape[1], round(cx + half_w)), min(shape[0], round(cy + half_h))
    return x0, y0, x1 - x0, y1 - y0


def fractions(box: Box, shape: tuple[int, ...]) -> list[float]:
    """[x0, y0, x1, y1] as fractions of the image (the mission's bbox)."""
    left, top, width, height = box
    h, w = shape[:2]
    return [
        round(left / w, 4),
        round(top / h, 4),
        round((left + width) / w, 4),
        round((top + height) / h, 4),
    ]


def _centre_inside(face: Box, box: Box) -> bool:
    cx, cy = face[0] + face[2] / 2, face[1] + face[3] / 2
    return box[0] <= cx <= box[0] + box[2] and box[1] <= cy <= box[1] + box[3]


def run(target: str, url: str, people: People) -> None:
    frames = DroneFrames(url)
    child = ChildTracker(target, FaceEmbedder(), people)
    last_answer = None
    while True:
        frame = frames.next()
        if frame is None:
            time.sleep(IDLE_S)
            continue
        frame_time, image = frame
        seen = child.update(image)
        if seen is None:
            continue
        box, confidence, label = seen
        answer = frames.report(frame_time, fractions(box, image.shape), confidence, label)
        if answer != last_answer:
            log.info("%s %s at %s: %s", target, label, box, answer)
            last_answer = answer


def main() -> None:
    parser = argparse.ArgumentParser(description="recognise and track the child for the mission")
    parser.add_argument("target", help="the person to look for (folder name in the database)")
    parser.add_argument("--url", default=DEFAULT_URL, help="drone SDK server")
    parser.add_argument("--people", type=Path, default=PEOPLE_DIR, help="people database")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")
    people = load_people(args.people)
    if args.target not in people.names:
        parser.error(f"{args.target} has no usable photo in {args.people} ({people.names})")
    log.info("tracking %s for the mission at %s (%s)", args.target, args.url, make_tracker())
    try:
        run(args.target, args.url, people)
    except KeyboardInterrupt:
        print("stopped")


if __name__ == "__main__":
    main()
