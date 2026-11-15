"""Lock on the lost child by face, then track their whole body (feed.py posts it to the mission).

Why the body and not the face: during the approach the camera turns down to -90, where the face
disappears; a body box is bigger and steadier than a head box, and its floor contact point (the
feet) is what the mission places on the floor (drone/sdk/mission_geometry.floor_contact), exact
whatever the child's height.

Every frame: faces (YuNet + SFace, faces.py and people.py) and persons (YOLOX, persons.py).
- searching: the target's face in CONFIRM_FRAMES frames in a row locks it. The body is the
  smallest person box that contains the face centre (in a crowd a big box may take in several
  people). No such box (too close, or the body cut by the image edge): the head box is tracked
  until a person box takes it in.
- locked: the body is the person box that overlaps the tracker's box best (IoU >= MIN_IOU),
  checked with the clothes' colour (an HSV histogram learned at the lock and updated slowly,
  because the view turns from the side to the top during the approach). CSRT (MIL without
  opencv-contrib) bridges the frames the detector misses, for BRIDGE_S at most: alone it drifts
  onto the background. After that the body is taken back only by its face, or by a person box
  overlapping the last confirmed one (colours are not enough: on our recordings a stranger in a
  similar hoodie passed the check once the camera had turned). The target's face seen again
  re-confirms, in whichever person box contains it (this fixes a swap; outside the current box
  only after CONFIRM_FRAMES in a row, so one look-alike frame cannot move the lock). A
  recognised face of someone else inside the box, without the target's: the lock is dropped.
  No confirmation (face or detector) for LOST_S: searching again.
- with a body model (body_model.py, trained on photos of the target's clothes): while searching,
  the same person recognised by their body in BODY_CONFIRM_FRAMES frames in a row also locks
  (from behind or above, without the face); while locked, an overlapping box passes if it is
  the target's body or has the target's colours.

Times are frame times (X-Frame-Time, or the recording's frame rate), so a recording replays
like the live feed.
"""

import logging
import time
from dataclasses import dataclass

import cv2
import numpy as np

from .body_model import BodyModel
from .faces import FaceEmbedder
from .live import CONFIRM_FRAMES
from .people import People
from .persons import Person, PersonDetector

HEAD_GROW = 1.6  # head box = face box scaled by this, around the face centre...
HEAD_LIFT = 0.15  # ...and moved up by this fraction of the face height (hair)
TRACK_WIDTH = 640  # the tracker runs on a copy this wide (speed)
BRIDGE_S = 0.7  # the tracker alone is trusted this long after the last confirmation
LOST_S = 2.0  # no face or detector confirmation this long: back to searching
MIN_IOU = 0.3  # a person box must overlap the tracked box this much to be the target
REINIT_IOU = 0.7  # the tracker restarts on the detector's box when they overlap less
HIST_BINS = (18, 8)  # hue, saturation
HIST_RANGES = (0, 180, 0, 256)
HIST_INSET = 0.2  # fraction of the box cut on each side: less floor and background
MIN_COLOUR = 0.2  # correlation with the target's colours to accept an overlapping box
MAX_SIZE_CHANGE = 1.5  # ...and its size (sqrt of the area) within this factor of the last one:
# the approach changes it slowly, two people merged into one box or a swap make it jump
COLOUR_LEARN = 0.1  # weight of a new frame in the target's colour histogram
FACE_CONFIDENCE = 0.9  # posted confidence: the target's face is in this frame
DETECTOR_CONFIDENCE = 0.8  # the person detector found the tracked body
BODY_CONFIDENCE = 0.8  # locked by the body model, without the face
BODY_CONFIRM_FRAMES = 3  # the body model must agree this many frames in a row to lock
TRACKER_CONFIDENCE = 0.4  # only the tracker: shown, below the mission's min_confidence 0.5
SEARCHING = "searching"
LOCKED = "locked"
BODY = "body"
HEAD = "head"

log = logging.getLogger("identify")

Box = tuple[int, int, int, int]  # left, top, width, height in pixels


@dataclass(frozen=True)
class FaceSeen:
    box: Box
    name: str | None  # None: nobody in the database is close enough
    similarity: float  # to that person (or the closest one)
    target: bool


@dataclass(frozen=True)
class Target:
    box: Box
    kind: str  # BODY or HEAD
    confidence: float
    source: str  # "face", "detector" or "tracker": what placed the box in this frame
    similarity: float | None  # last face match of the target
    face_age_s: float | None  # since that match


@dataclass(frozen=True)
class FrameResult:
    state: str
    faces: list[FaceSeen]
    persons: list[Person]
    target: Target | None
    ms: float  # time to process the frame


def make_tracker() -> cv2.Tracker:
    """CSRT when opencv-contrib is installed, else MIL."""
    if hasattr(cv2, "TrackerCSRT_create"):
        return cv2.TrackerCSRT_create()
    return cv2.TrackerMIL_create()


class BodyTracker:
    def __init__(
        self,
        target: str,
        embedder: FaceEmbedder,
        people: People,
        detector: PersonDetector,
        body_model: BodyModel | None = None,
    ) -> None:
        self.target = target
        self.embedder = embedder
        self.people = people
        self.detector = detector
        self.body_model = body_model
        self._now = 0.0  # the frame time of the frame being processed
        self._unlock()

    @property
    def locked(self) -> bool:
        return self._box is not None

    def update(self, image: np.ndarray, frame_time: float) -> FrameResult:
        """The result for one frame; `frame_time` in seconds (unix time of the frame)."""
        started = time.monotonic()
        self._now = frame_time
        faces = self._faces(image)
        persons = self.detector.persons(image)
        target_face = max((f for f in faces if f.target), key=lambda f: f.similarity, default=None)
        self._in_a_row = self._in_a_row + 1 if target_face else 0
        if self.locked:
            target = self._follow(image, faces, persons, target_face)
        elif target_face and self._in_a_row >= CONFIRM_FRAMES:
            target = self._lock(image, target_face, persons)
        else:
            target = self._lock_by_body(image, faces, persons)
        ms = (time.monotonic() - started) * 1000
        return FrameResult(LOCKED if self.locked else SEARCHING, faces, persons, target, ms)

    # --- states -----------------------------------------------------------------------------

    def _lock(self, image: np.ndarray, face: FaceSeen, persons: list[Person]) -> Target:
        log.info("%s locked (face %.2f)", self.target, face.similarity)
        self._seen_face(face)
        body = body_for_face(face.box, persons)
        if body is None:
            self._set(image, head_box(face.box, image.shape), HEAD)
        else:
            self._set(image, body, BODY)
            self._colour = colour_histogram(image, body)
        return self._result(self._box, FACE_CONFIDENCE, "face")

    def _lock_by_body(
        self, image: np.ndarray, faces: list[FaceSeen], persons: list[Person]
    ) -> Target | None:
        """Lock on the person the body model recognises in BODY_CONFIRM_FRAMES frames in a row,
        at the same place (one look-alike frame is not enough)."""
        if self.body_model is None:
            return None
        scored = [(self.body_model.score(image, p.box), p.box) for p in persons]
        score, box = max(scored, default=(-np.inf, None))
        if box is None or score < self.body_model.threshold:
            self._body_candidate, self._body_in_a_row = None, 0
            return None
        same = self._body_candidate is not None and iou(box, self._body_candidate) >= MIN_IOU
        self._body_in_a_row = self._body_in_a_row + 1 if same else 1
        self._body_candidate = box
        if self._body_in_a_row < BODY_CONFIRM_FRAMES:
            return None
        log.info("%s locked by the body (%.2f)", self.target, score)
        self._set(image, box, BODY)
        self._colour = colour_histogram(image, box)
        return self._checked(faces, self._result(box, BODY_CONFIDENCE, "body"))

    def _follow(
        self,
        image: np.ndarray,
        faces: list[FaceSeen],
        persons: list[Person],
        target_face: FaceSeen | None,
    ) -> Target | None:
        unconfirmed = self._now - self._confirmed_at
        if unconfirmed > BRIDGE_S:
            self._tracker = None  # alone for too long: it follows the background by now
        tracked = self._track(image)
        if target_face is not None and self._face_counts(target_face, tracked):
            self._seen_face(target_face)
            self._place_on_face(image, target_face, persons, tracked)
            return self._result(self._box, FACE_CONFIDENCE, "face")
        if self._kind == HEAD:
            body = _containing(_centre(tracked or self._box), persons)
            if body is not None:  # the head's body came into view: track the body from now
                self._set(image, body, BODY)
                self._colour = colour_histogram(image, body)
                return self._checked(faces, self._result(body, DETECTOR_CONFIDENCE, "detector"))
        match = self._match(image, persons, tracked)
        if match is not None:
            self._set(image, match, BODY, tracked)
            self._learn_colour(image, match)
            return self._checked(faces, self._result(match, DETECTOR_CONFIDENCE, "detector"))
        if tracked is not None:
            return self._checked(faces, self._result(tracked, TRACKER_CONFIDENCE, "tracker"))
        if unconfirmed > LOST_S:
            log.info("%s not confirmed for %.0f s: searching again", self.target, LOST_S)
            self._unlock()
        return None

    def _face_counts(self, face: FaceSeen, tracked: Box | None) -> bool:
        """A target face inside the tracked box always counts; elsewhere only CONFIRM_FRAMES in
        a row (one look-alike frame must not move the lock to someone else)."""
        inside = _inside(_centre(face.box), tracked or self._box)
        return inside or self._in_a_row >= CONFIRM_FRAMES

    def _place_on_face(
        self, image: np.ndarray, face: FaceSeen, persons: list[Person], tracked: Box | None
    ) -> None:
        """The target's face is in view: the body is the person box around it."""
        body = body_for_face(face.box, persons)
        if body is not None:
            self._set(image, body, BODY, tracked)
            self._learn_colour(image, body)
            return
        if self._kind == BODY and tracked is not None and _inside(_centre(face.box), tracked):
            self._box, self._confirmed_at = tracked, self._now  # the detector missed the body
            return
        self._set(image, head_box(face.box, image.shape), HEAD, tracked)

    def _checked(self, faces: list[FaceSeen], target: Target) -> Target | None:
        """None (and the lock dropped) if someone else's face is inside the target's box."""
        for face in faces:
            if face.name not in (None, self.target) and _inside(_centre(face.box), target.box):
                log.info("%s's face in the tracked box: lock dropped", face.name)
                self._unlock()
                return None
        return target

    # --- association ------------------------------------------------------------------------

    def _match(self, image: np.ndarray, persons: list[Person], tracked: Box | None) -> Box | None:
        """The person box that is the target: best overlap with the tracked box (or, without
        a tracker, the last confirmed box), size and colours alike."""
        reference = tracked or self._box
        best, best_iou = None, MIN_IOU
        for person in persons:
            overlap = iou(person.box, reference)
            if overlap < best_iou or not _similar_size(person.box, self._box):
                continue
            if self._colour_ok(image, person.box):
                best, best_iou = person.box, overlap
        return best

    def _colour_ok(self, image: np.ndarray, box: Box) -> bool:
        if self.body_model is not None and self.body_model.is_target(image, box):
            return True
        if self._colour is None:
            return True
        similarity = cv2.compareHist(self._colour, colour_histogram(image, box), cv2.HISTCMP_CORREL)
        return similarity >= MIN_COLOUR

    def _learn_colour(self, image: np.ndarray, box: Box) -> None:
        histogram = colour_histogram(image, box)
        if self._colour is None:
            self._colour = histogram
            return
        self._colour = (1 - COLOUR_LEARN) * self._colour + COLOUR_LEARN * histogram

    # --- helpers ----------------------------------------------------------------------------

    def _faces(self, image: np.ndarray) -> list[FaceSeen]:
        seen = []
        for face in self.embedder.faces(image):
            name, similarity = self.people.classify(face.embedding)
            seen.append(FaceSeen(face.box, name, similarity, name == self.target))
        return seen

    def _seen_face(self, face: FaceSeen) -> None:
        self._similarity, self._face_at = face.similarity, self._now

    def _set(self, image: np.ndarray, box: Box, kind: str, tracked: Box | None = None) -> None:
        """Track `box` from now (confirmed in this frame); the tracker restarts unless it
        already follows it."""
        self._box, self._kind, self._confirmed_at = box, kind, self._now
        if tracked is None or iou(tracked, box) < REINIT_IOU:
            self._tracker = make_tracker()
            self._tracker.init(self._small(image), tuple(round(v * self._scale) for v in box))

    def _track(self, image: np.ndarray) -> Box | None:
        if self._tracker is None:
            return None
        ok, small_box = self._tracker.update(self._small(image))
        if not ok:
            return None
        box = tuple(round(v / self._scale) for v in small_box)
        return box if box[2] > 0 and box[3] > 0 else None

    def _small(self, image: np.ndarray) -> np.ndarray:
        self._scale = min(1.0, TRACK_WIDTH / image.shape[1])
        if self._scale == 1.0:
            return image
        return cv2.resize(image, None, fx=self._scale, fy=self._scale)

    def _result(self, box: Box, confidence: float, source: str) -> Target:
        age = None if self._face_at is None else round(self._now - self._face_at, 2)
        return Target(box, self._kind, confidence, source, self._similarity, age)

    def _unlock(self) -> None:
        self._box: Box | None = None  # the last confirmed box (face or detector)
        self._kind = BODY
        self._tracker: cv2.Tracker | None = None
        self._colour: np.ndarray | None = None
        self._confirmed_at = 0.0
        self._similarity: float | None = None
        self._face_at: float | None = None
        self._in_a_row = 0
        self._scale = 1.0
        self._body_candidate: Box | None = None  # the body model's pick while searching
        self._body_in_a_row = 0


def body_for_face(face: Box, persons: list[Person]) -> Box | None:
    """The smallest person box that contains the face centre and is wider than the face."""
    centre = _centre(face)
    boxes = [p.box for p in persons if _inside(centre, p.box) and p.box[2] > face[2]]
    return min(boxes, key=lambda box: box[2] * box[3], default=None)


def head_box(face: Box, shape: tuple[int, ...]) -> Box:
    """The face box grown around its centre and lifted to take in the hair, inside the image."""
    left, top, width, height = face
    cx, cy = left + width / 2, top + height / 2 - HEAD_LIFT * height
    half_w, half_h = width * HEAD_GROW / 2, height * HEAD_GROW / 2
    x0, y0 = max(0, round(cx - half_w)), max(0, round(cy - half_h))
    x1, y1 = min(shape[1], round(cx + half_w)), min(shape[0], round(cy + half_h))
    return x0, y0, x1 - x0, y1 - y0


def colour_histogram(image: np.ndarray, box: Box) -> np.ndarray:
    """Normalised hue-saturation histogram of the middle of the box (the clothes)."""
    left, top, width, height = box
    dx, dy = round(width * HIST_INSET), round(height * HIST_INSET)
    crop = image[max(0, top + dy) : top + height - dy, max(0, left + dx) : left + width - dx]
    if crop.size == 0:
        crop = image[max(0, top) : top + height, max(0, left) : left + width]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    histogram = cv2.calcHist([hsv], [0, 1], None, list(HIST_BINS), list(HIST_RANGES))
    return cv2.normalize(histogram, histogram).astype(np.float32)


def iou(a: Box, b: Box) -> float:
    x0, y0 = max(a[0], b[0]), max(a[1], b[1])
    x1, y1 = min(a[0] + a[2], b[0] + b[2]), min(a[1] + a[3], b[1] + b[3])
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    union = a[2] * a[3] + b[2] * b[3] - inter
    return inter / union if union > 0 else 0.0


def _similar_size(box: Box, reference: Box) -> bool:
    ratio = np.sqrt((box[2] * box[3]) / max(reference[2] * reference[3], 1))
    return 1 / MAX_SIZE_CHANGE <= ratio <= MAX_SIZE_CHANGE


def _containing(point: tuple[float, float], persons: list[Person]) -> Box | None:
    boxes = [p.box for p in persons if _inside(point, p.box)]
    return min(boxes, key=lambda box: box[2] * box[3], default=None)


def _centre(box: Box) -> tuple[float, float]:
    return box[0] + box[2] / 2, box[1] + box[3] / 2


def _inside(point: tuple[float, float], box: Box) -> bool:
    return box[0] <= point[0] <= box[0] + box[2] and box[1] <= point[1] <= box[1] + box[3]
