"""Draw what the body tracker saw on a frame: the same boxes as the web page's video overlay
(drone/hoverctl/web.html, "vision overlay"): every person thin grey, every face grey, the
target's face white, the tracked body (or head) red with the lock state and confidence."""

import cv2
import numpy as np

from .body_track import FrameResult

PERSON_COLOUR = (130, 130, 130)  # BGR
FACE_COLOUR = (190, 190, 190)
TARGET_FACE_COLOUR = (255, 255, 255)
TARGET_COLOUR = (40, 40, 235)
STATE_COLOUR = (0, 220, 255)
THIN, THICK = 1, 3
FONT = cv2.FONT_HERSHEY_SIMPLEX
SMALL, LARGE = 0.5, 0.8
LABEL_GAP_PX = 6


def draw(image: np.ndarray, result: FrameResult, target: str, caption: str = "") -> np.ndarray:
    """A copy of `image` with the result drawn on it."""
    view = image.copy()
    for person in result.persons:
        _box(view, person.box, PERSON_COLOUR, THIN)
    for face in result.faces:
        colour = TARGET_FACE_COLOUR if face.target else FACE_COLOUR
        _box(view, face.box, colour, THICK if face.target else THIN)
        _label(view, face.box, f"{face.name or '?'} {face.similarity:.2f}", colour, SMALL)
    if result.target is not None:
        t = result.target
        _box(view, t.box, TARGET_COLOUR, THICK)
        face = "" if t.similarity is None else f" face {t.similarity:.2f} {t.face_age_s:.1f}s ago"
        text = f"{target} {t.kind} {t.confidence:.1f} ({t.source}){face}"
        _label(view, t.box, text, TARGET_COLOUR, SMALL, below=True)
    header = f"{result.state.upper()} {target} | {result.ms:.0f} ms {caption}"
    cv2.putText(view, header, (10, 30), FONT, LARGE, STATE_COLOUR, 2)
    return view


def _box(view: np.ndarray, box: tuple, colour: tuple, thickness: int) -> None:
    left, top, width, height = box
    cv2.rectangle(view, (left, top), (left + width, top + height), colour, thickness)


def _label(
    view: np.ndarray, box: tuple, text: str, colour: tuple, scale: float, below: bool = False
) -> None:
    left, top, _, height = box
    y = top + height + 4 * LABEL_GAP_PX if below else max(top - LABEL_GAP_PX, 14)
    cv2.putText(view, text, (left, min(y, view.shape[0] - LABEL_GAP_PX)), FONT, scale, colour, 2)
