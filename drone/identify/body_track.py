"""Lock on the lost child by face, then track their whole body (feed.py posts it to the mission).

Why the body and not the face: during the approach the camera turns down to -90, where the face
disappears; a body box is bigger and steadier than a head box, and its floor contact point (the
feet) is what the mission places on the floor (drone/sdk/mission_geometry.floor_contact), exact
whatever the child's height.

Every frame: faces (YuNet + SFace, faces.py and people.py) and persons (YOLOX, persons.py).
- searching: the target's face in LOCK_FACE_FRAMES frames within the last FACE_WINDOW_S locks
  it (gaps allowed: a face seen on and off while the drone turns or the child moves still
  adds up; `face_frames` in each result says how far it is, 0 once locked). The body is the
  smallest person box that contains the face centre in its top HEAD_ZONE (in a crowd a big box
  may take in several people, and a person sitting behind reaches down to the face). No such
  box (too close, or the body cut by the image edge): the head box is tracked until a person
  box takes it in.
- locked: the body is the person box that overlaps the tracker's box (IoU >= MIN_IOU), of a
  similar size, with the clothes' colour of the target (MIN_COLOUR against a gallery of views:
  the one at the lock, never forgotten, plus up to GALLERY_SIZE others as the approach turns the
  view from the side to the top). Views are learned only with the target's face in the box: a
  view learned by continuity alone could be a stranger's, and from then on the colour check
  would let that stranger through. A box is compared with its closest view: no running average
  that a wrong match could drag toward a stranger. Continuity never hops: when another person
  box also touches the tracker's box (CROSSING_IOU: people crossing, sitting together, merged
  leg boxes from the floor), no box confirms the target in that frame (better no confirmation
  than the neighbour). CSRT (MIL without opencv-contrib) bridges the frames the detector
  misses, for BRIDGE_S at most: alone it drifts onto the background or onto whoever is next to
  the target. A frame where nothing places the box (no face, no detector box, the tracker
  failed or past BRIDGE_S) shows the last box again ("held"), so the lock never blinks off
  between confirmations. The target's face inside the box re-confirms it. A target face
  elsewhere moves the lock to that person only with the evidence of a new lock,
  LOCK_FACE_FRAMES in a row: moving is locking on someone else, and from the drone a stranger's
  face often scores just
  above the threshold for a frame or two. Someone else's face recognised with
  OTHER_FACE_SIMILARITY inside the box drops the lock (a weak match is not enough, for the same
  reason). No confirmation (face or detector) for LOST_S: searching again, and only the face
  locks again: clothes alone never start or take back a lock (on the 15 Nov 20:53 flight, legs,
  black trousers and black hoodies all passed the colours).
- the body model (body_model.py) is not used: trained on one outfit, it passed nearly every
  person on that flight (up to 9 in a frame) and scored the target, in other clothes, lower than
  the strangers around him, so it can neither pick him nor veto someone else.

Times are frame times (X-Frame-Time, or the recording's frame rate), so a recording replays
like the live feed.
"""

import logging
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

from .faces import FaceEmbedder
from .people import People
from .persons import Person, PersonDetector

HEAD_GROW = 1.6  # head box = face box scaled by this, around the face centre...
HEAD_LIFT = 0.15  # ...and moved up by this fraction of the face height (hair)
# A body's face is in the top HEAD_ZONE of its box; lower down, the box is someone else's
# reaching down to the face (a person sitting behind the target)
HEAD_ZONE = 0.5
TRACK_WIDTH = 640  # the tracker runs on a copy this wide (speed)
FACE_WINDOW_S = 4.0  # the target's face frames count this long for the lock...
LOCK_FACE_FRAMES = 5  # ...and this many of them lock on the target
BRIDGE_S = 0.7  # the tracker alone is trusted this long after the last confirmation
LOST_S = 2.0  # no face or detector confirmation this long: searching again, by the face
MIN_IOU = 0.3  # a person box must overlap the tracked box this much to be the target...
CROSSING_IOU = 0.1  # ...and no other person box this much (someone crossing or next to them)
REINIT_IOU = 0.7  # the tracker restarts on the detector's box when they overlap less
HIST_BINS = (18, 8)  # hue, saturation
HIST_RANGES = (0, 180, 0, 256)
HIST_INSET = 0.2  # fraction of the box cut on each side: less floor and background
MIN_COLOUR = 0.2  # correlation with the target's colours to accept an overlapping box
OTHER_FACE_SIMILARITY = 0.4  # someone else's face this sure inside the box drops the lock
# A matched box's size (sqrt of the area) is within this factor of the last one: the approach
# changes it slowly, two people merged into one box or a swap make it jump
MAX_SIZE_CHANGE = 1.5
GALLERY_SIZE = 6  # the target's views kept besides the one at the lock
NEW_VIEW = 0.8  # a confirmed view less alike than this to all kept ones is a new view
FACE_CONFIDENCE = 0.9  # posted confidence: the target's face is in this frame
DETECTOR_CONFIDENCE = 0.8  # the person detector found the tracked body
TRACKER_CONFIDENCE = 0.4  # only the tracker: shown, below the mission's min_confidence 0.5
# Nothing placed the box in this frame: the last one is shown again, also below the mission's
# min_confidence: an old image box with the drone's pose now would put the child somewhere else
# once the drone has turned, and the approach already flies on its last fix (HOLD_S)
HELD_CONFIDENCE = TRACKER_CONFIDENCE
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
    source: str  # "face", "detector", "tracker" or "held" (the last box, nothing placed it)
    similarity: float | None  # last face match of the target
    face_age_s: float | None  # since that match


@dataclass(frozen=True)
class FrameResult:
    state: str
    faces: list[FaceSeen]
    persons: list[Person]
    target: Target | None
    ms: float  # time to process the frame
    face_frames: int = 0  # the target's face frames within FACE_WINDOW_S (LOCK_FACE_FRAMES lock)


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
    ) -> None:
        self.target = target
        self.embedder = embedder
        self.people = people
        self.detector = detector
        self._now = 0.0  # the frame time of the frame being processed
        self._face_times: deque[float] = deque()  # frame times of the target's face, searching
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
        # counted only while searching: the lock needs them, a locked target does not
        face_frames = 0 if self.locked else self._count_face(target_face is not None)
        if self.locked:
            target = self._follow(image, faces, persons, target_face)
        elif target_face and face_frames >= LOCK_FACE_FRAMES:
            target = self._lock(image, target_face, persons)
        else:
            target = None
        ms = (time.monotonic() - started) * 1000
        state = LOCKED if self.locked else SEARCHING
        return FrameResult(state, faces, persons, target, ms, face_frames)

    def _count_face(self, seen: bool) -> int:
        """The target's face frames within the last FACE_WINDOW_S, this one included."""
        if seen:
            self._face_times.append(self._now)
        while self._face_times and self._face_times[0] < self._now - FACE_WINDOW_S:
            self._face_times.popleft()
        return len(self._face_times)

    # --- states -----------------------------------------------------------------------------

    def _lock(self, image: np.ndarray, face: FaceSeen, persons: list[Person]) -> Target:
        log.info("%s locked (face %.2f)", self.target, face.similarity)
        self._seen_face(face)
        body = body_for_face(face.box, persons)
        if body is None:
            self._set(image, head_box(face.box, image.shape), HEAD)
        else:
            self._set(image, body, BODY)
            self._gallery = [colour_histogram(image, body)]
        return self._result(self._box, FACE_CONFIDENCE, "face")

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
        if unconfirmed > LOST_S:
            log.info("%s not confirmed for %.0f s: searching again", self.target, LOST_S)
            self._unlock()
            return None
        if self._kind == HEAD:
            body = body_for_face(tracked or self._box, persons)
            if body is not None:  # the head's body came into view: track the body from now
                self._set(image, body, BODY)
                self._gallery = [colour_histogram(image, body)]
                return self._checked(faces, self._result(body, DETECTOR_CONFIDENCE, "detector"))
        box = self._match(image, persons, tracked)
        if box is not None:
            self._set(image, box, BODY, tracked)
            return self._checked(faces, self._result(box, DETECTOR_CONFIDENCE, "detector"))
        if tracked is not None:
            return self._checked(faces, self._result(tracked, TRACKER_CONFIDENCE, "tracker"))
        return self._checked(faces, self._result(self._shown, HELD_CONFIDENCE, "held"))

    def _face_counts(self, face: FaceSeen, tracked: Box | None) -> bool:
        """A target face inside the tracked box always counts; elsewhere only LOCK_FACE_FRAMES
        in a row (a look-alike frame or two must not move the lock to someone else)."""
        inside = _inside(_centre(face.box), tracked or self._box)
        return inside or self._in_a_row >= LOCK_FACE_FRAMES

    def _place_on_face(
        self, image: np.ndarray, face: FaceSeen, persons: list[Person], tracked: Box | None
    ) -> None:
        """The target's face is in view: the body is the person box around it."""
        body = body_for_face(face.box, persons)
        if body is not None:
            self._set(image, body, BODY, tracked)
            self._remember_view(image, body)  # the target's face on it: a sure view
            return
        if self._kind == BODY and tracked is not None and _inside(_centre(face.box), tracked):
            self._box, self._confirmed_at = tracked, self._now  # the detector missed the body
            return
        self._set(image, head_box(face.box, image.shape), HEAD, tracked)

    def _checked(self, faces: list[FaceSeen], target: Target) -> Target | None:
        """None (and the lock dropped) if someone else's face, surely theirs, is inside the
        target's box."""
        for face in faces:
            other = face.name not in (None, self.target)
            sure = other and face.similarity >= OTHER_FACE_SIMILARITY
            if sure and _inside(_centre(face.box), target.box):
                log.info("%s's face in the tracked box: lock dropped", face.name)
                self._unlock()
                return None
        return target

    # --- association ------------------------------------------------------------------------

    def _match(self, image: np.ndarray, persons: list[Person], tracked: Box | None) -> Box | None:
        """The person box that is the target: the only one overlapping the tracked box (or,
        without a tracker, the last confirmed box), size alike, the target's colours. None when
        another person box touches it too (CROSSING_IOU), whatever its colours: that is where a
        lock hops to the neighbour."""
        reference = tracked or self._box
        touching = [p.box for p in persons if iou(p.box, reference) >= CROSSING_IOU]
        if len(touching) != 1:
            return None
        box = touching[0]
        if iou(box, reference) < MIN_IOU or not _similar_size(box, self._box):
            return None
        return box if self._colour(image, box) >= MIN_COLOUR else None

    def _colour(self, image: np.ndarray, box: Box) -> float:
        """Correlation of the box's colours with the closest of the target's views (1 before
        any view)."""
        if not self._gallery:
            return 1.0
        histogram = colour_histogram(image, box)
        return float(max(cv2.compareHist(v, histogram, cv2.HISTCMP_CORREL) for v in self._gallery))

    def _remember_view(self, image: np.ndarray, box: Box) -> None:
        """Keep this confirmed view if it is unlike all kept ones; the view at the lock stays,
        the oldest other one goes beyond GALLERY_SIZE."""
        histogram = colour_histogram(image, box)
        if not self._gallery:
            self._gallery = [histogram]
            return
        closest = max(cv2.compareHist(v, histogram, cv2.HISTCMP_CORREL) for v in self._gallery)
        if closest >= NEW_VIEW:
            return
        self._gallery = [self._gallery[0], *self._gallery[1:][-(GALLERY_SIZE - 1) :], histogram]

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
        self._shown = box  # held again when nothing places the box in a later frame
        age = None if self._face_at is None else round(self._now - self._face_at, 2)
        return Target(box, self._kind, confidence, source, self._similarity, age)

    def _unlock(self) -> None:
        self._box: Box | None = None  # the last confirmed box (face or detector)
        self._shown: Box | None = None  # the last box returned, confirmed or not
        self._kind = BODY
        self._tracker: cv2.Tracker | None = None
        self._gallery: list[np.ndarray] = []  # the target's views: [0] at the lock
        self._confirmed_at = 0.0
        self._similarity: float | None = None
        self._face_at: float | None = None
        self._in_a_row = 0
        self._face_times.clear()
        self._scale = 1.0


def body_for_face(face: Box, persons: list[Person]) -> Box | None:
    """The smallest person box that contains the face centre in its top HEAD_ZONE and is
    wider than the face."""
    centre = _centre(face)
    boxes = [p.box for p in persons if _in_head_zone(centre, p.box) and p.box[2] > face[2]]
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
    ratio = _size(box) / max(_size(reference), 1.0)
    return 1 / MAX_SIZE_CHANGE <= ratio <= MAX_SIZE_CHANGE


def _size(box: Box) -> float:
    return float(np.sqrt(box[2] * box[3]))


def _centre(box: Box) -> tuple[float, float]:
    return box[0] + box[2] / 2, box[1] + box[3] / 2


def _in_head_zone(point: tuple[float, float], box: Box) -> bool:
    """Inside the box, in its top HEAD_ZONE (where the box's own face is)."""
    return _inside(point, box) and point[1] <= box[1] + HEAD_ZONE * box[3]


def _inside(point: tuple[float, float], box: Box) -> bool:
    return box[0] <= point[0] <= box[0] + box[2] and box[1] <= point[1] <= box[1] + box[3]
