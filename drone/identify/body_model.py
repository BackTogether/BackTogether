"""Recognise the target by their body (clothes), without the face: from behind, from the side and
from above, where face recognition cannot work.

    uv run --with opencv-contrib-python-headless python -m drone.identify.body_model train \
        --negatives <video or folder with other people>
    uv run --with opencv-contrib-python-headless python -m drone.identify.body_model check <img>

Training photos: drone/identify/body/ (not in git), the target alone or the biggest person in
each, any angle. Each person box (YOLOX, persons.py) is described by the colours of its torso and
of its legs (HSV histograms, Hellinger-normalised) and an RBF SVM, overfitted to this one person,
scores it. The photos are taken at eye level and the drone looks down diagonally, so the target
crops are augmented like a view from above (squashed, narrower at the feet), small (30-160 px),
with imprecise boxes, light and white balance changes and blur. Negatives: everybody else in the
photos and in the negative video/folder (a drone video of the hall is ideal).

The threshold is calibrated on held-out people: MAX_FALSE_POSITIVES of other people's boxes pass.
Colour only: HOG (body shape) depends on the pose and the viewpoint and generalised worse.
"""

import argparse
import hashlib
import json
import logging
from pathlib import Path

import cv2
import numpy as np

from .models import MODELS_DIR
from .pad import load_negatives
from .people import is_photo
from .persons import PersonDetector

BODY_DIR = Path(__file__).resolve().parent / "body"
MODEL_PATH = MODELS_DIR / "body_svm.xml"
THRESHOLD_PATH = MODELS_DIR / "body_threshold.json"
PHOTO_SIZE = (540, 720)  # the photos are resized to this before detection (portrait phone)
CROP_SIZE = (32, 64)  # width, height of the normalised person crop
TORSO = (0.15, 0.5)  # fractions of the box height
LEGS = (0.5, 0.9)
INSET = 0.2  # fraction of the box width cut on each side (background)
HIST_BINS = (8, 4, 4)  # hue, saturation, value
HIST_RANGES = (0, 180, 0, 256, 0, 256)
# Training
AUGMENTED_PER_PHOTO = 40
NEGATIVE_JITTERS = 3  # crops per other person (the box as detected, then jittered)
BOX_JITTER = 0.06  # std of the box position/size noise, fraction of the box
NEGATIVE_JITTER = 0.08
SQUASH = (0.5, 1.0)  # height kept when seen from above
NARROW = (0.6, 1.0)  # width kept at the feet
DRONE_HEIGHT_PX = (30, 160)
HELD_OUT_EVERY = 4  # every 4th photo is held out to report how well it generalises
SVM_C = 10.0
SVM_GAMMA = 1.0
MAX_FALSE_POSITIVES = 0.01

log = logging.getLogger("identify")

Box = tuple[int, int, int, int]  # left, top, width, height in pixels


class BodyModel:
    def __init__(self) -> None:
        if not MODEL_PATH.is_file() or not THRESHOLD_PATH.is_file():
            raise FileNotFoundError(f"{MODEL_PATH}: train it (python -m drone.identify.body_model)")
        self.svm = cv2.ml.SVM_load(str(MODEL_PATH))
        self.threshold = json.loads(THRESHOLD_PATH.read_text())["threshold"]

    def score(self, image: np.ndarray, box: Box) -> float:
        """How much the person in `box` looks like the target (over `threshold`: it is)."""
        crop = crop_box(image, box)
        if crop is None:
            return -np.inf
        return _scores(self.svm, [crop])[0]

    def is_target(self, image: np.ndarray, box: Box) -> bool:
        return self.score(image, box) >= self.threshold


def features(crop: np.ndarray) -> np.ndarray:
    small = cv2.resize(crop, CROP_SIZE, interpolation=cv2.INTER_AREA)
    width, height = CROP_SIZE
    x0, x1 = round(INSET * width), round((1 - INSET) * width)
    parts = [small[round(a * height) : round(b * height), x0:x1] for a, b in (TORSO, LEGS)]
    return np.concatenate([_histogram(part) for part in parts])


def _histogram(region: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
    counts = cv2.calcHist([hsv], [0, 1, 2], None, list(HIST_BINS), list(HIST_RANGES)).ravel()
    return np.sqrt(counts / (counts.sum() + 1e-6))  # Hellinger: comparable colour fractions


def crop_box(
    image: np.ndarray, box: Box, jitter: float = 0.0, rng: np.random.Generator | None = None
) -> np.ndarray | None:
    """The box's pixels (clipped to the image), optionally with noise on its position and size."""
    x, y, w, h = (float(v) for v in box)
    if jitter and rng is not None:
        x, y = x + rng.normal(0, jitter) * w, y + rng.normal(0, jitter) * h
        w, h = w * np.exp(rng.normal(0, jitter)), h * np.exp(rng.normal(0, jitter))
    x0, y0 = int(max(0, x)), int(max(0, y))
    x1, y1 = int(min(image.shape[1], x + w)), int(min(image.shape[0], y + h))
    if x1 - x0 < CROP_SIZE[0] // 4 or y1 - y0 < CROP_SIZE[1] // 4:
        return None
    return image[y0:y1, x0:x1]


def seen_from_above(crop: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """An eye-level crop made to look like the drone's diagonal view, small and noisy."""
    h, w = crop.shape[:2]
    height = max(8, int(h * rng.uniform(*SQUASH)))
    margin = w * (1 - rng.uniform(*NARROW)) / 2
    corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    warped = np.float32([[0, 0], [w, 0], [w - margin, height], [margin, height]])
    transform = cv2.getPerspectiveTransform(corners, warped)
    view = cv2.warpPerspective(crop, transform, (w, height), borderMode=cv2.BORDER_REPLICATE)
    scale = rng.uniform(*DRONE_HEIGHT_PX) / height
    if scale < 1:
        view = cv2.resize(view, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    tint = rng.uniform(0.6, 1.3, 3) ** 0.3 * rng.uniform(0.6, 1.3)
    view = np.clip(view.astype(np.float32) * tint + rng.uniform(-15, 15), 0, 255).astype(np.uint8)
    return cv2.GaussianBlur(view, (3, 3), 0) if rng.random() < 0.5 else view


def _scores(svm: cv2.ml.SVM, crops: list[np.ndarray]) -> np.ndarray:
    rows = np.array([features(c) for c in crops], np.float32)
    # RAW_OUTPUT is the decision value, negative on the target side for OpenCV's SVM
    return -svm.predict(rows, flags=cv2.ml.STAT_MODEL_RAW_OUTPUT)[1].ravel()


# --- training --------------------------------------------------------------------------------


def target_photos(photo_dir: Path, detector: PersonDetector) -> list[tuple[np.ndarray, Box, list]]:
    """(photo, the target's box = the biggest person, the other people's boxes), once per photo."""
    seen, photos = set(), []
    for path in sorted(p for p in photo_dir.iterdir() if is_photo(p)):
        image = cv2.imread(str(path))
        if image is None:
            continue
        key = hashlib.md5(image.tobytes()).hexdigest()  # copies of a photo count once
        if key in seen:
            continue
        seen.add(key)
        image = cv2.resize(image, PHOTO_SIZE)
        persons = detector.persons(image)
        if not persons:
            log.warning("nobody found in %s: photo ignored", path.name)
            continue
        target = max(persons, key=lambda p: p.box[2] * p.box[3])
        photos.append((image, target.box, [p.box for p in persons if p is not target]))
    return photos


def _positives(photos: list, rng: np.random.Generator) -> list[np.ndarray]:
    crops = []
    for image, box, _ in photos:
        for _ in range(AUGMENTED_PER_PHOTO):
            crop = crop_box(image, box, BOX_JITTER, rng)
            if crop is not None:
                crops.append(seen_from_above(crop, rng))
    return crops


def _others(scenes: list[tuple[np.ndarray, list]], rng: np.random.Generator) -> list[np.ndarray]:
    crops = []
    for image, boxes in scenes:
        for box in boxes:
            for k in range(NEGATIVE_JITTERS):
                crop = crop_box(image, box, NEGATIVE_JITTER if k else 0.0, rng)
                if crop is not None:
                    crops.append(crop)
    return crops


def _fit(positives: list[np.ndarray], negatives: list[np.ndarray]) -> cv2.ml.SVM:
    rows = np.array([features(c) for c in positives + negatives], np.float32)
    labels = np.array([1] * len(positives) + [0] * len(negatives), np.int32)
    svm = cv2.ml.SVM_create()
    svm.setType(cv2.ml.SVM_C_SVC)
    svm.setKernel(cv2.ml.SVM_RBF)
    svm.setC(SVM_C)
    svm.setGamma(SVM_GAMMA)
    svm.train(rows, cv2.ml.ROW_SAMPLE, labels)
    return svm


def train(photo_dir: Path, negatives: Path | None, seed: int) -> None:
    rng = np.random.default_rng(seed)
    detector = PersonDetector()
    photos = target_photos(photo_dir, detector)
    if not photos:
        raise ValueError(f"no photos of the target in {photo_dir}")
    frames = [(f, [p.box for p in detector.persons(f)]) for f in load_negatives(negatives)]
    scenes = [(image, others) for image, _, others in photos]
    train_scenes, held_people = scenes + frames[::2], _others(frames[1::2], rng)
    if not held_people:
        raise ValueError("no other people to calibrate on: pass --negatives with people")
    log.info("%d photos of the target, %d other people held out", len(photos), len(held_people))

    held = photos[::HELD_OUT_EVERY]
    rest = [p for i, p in enumerate(photos) if i % HELD_OUT_EVERY]
    check_svm = _fit(_positives(rest, rng), _others(train_scenes, rng))
    check_threshold = _threshold(check_svm, held_people)
    recall = np.mean(_scores(check_svm, _positives(held, rng)) >= check_threshold)
    log.info("held-out photos seen from above: %.0f %% recognised", recall * 100)

    svm = _fit(_positives(photos, rng), _others(train_scenes, rng))
    threshold = _threshold(svm, held_people)
    MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    svm.save(str(MODEL_PATH))
    THRESHOLD_PATH.write_text(json.dumps({"threshold": threshold}))
    log.info("saved %s, threshold %.3f (%.0f %% of other people pass)", MODEL_PATH, threshold,
             MAX_FALSE_POSITIVES * 100)  # fmt: skip


def _threshold(svm: cv2.ml.SVM, others: list[np.ndarray]) -> float:
    return float(np.quantile(_scores(svm, others), 1 - MAX_FALSE_POSITIVES))


def check(paths: list[Path], out_dir: Path) -> None:
    """Score every person in images; prints them and saves marked copies (green: the target)."""
    model, detector = BodyModel(), PersonDetector()
    out_dir.mkdir(parents=True, exist_ok=True)
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            continue
        for person in detector.persons(image):
            score = model.score(image, person.box)
            x, y, w, h = person.box
            colour = (0, 255, 0) if score >= model.threshold else (0, 0, 255)
            cv2.rectangle(image, (x, y), (x + w, y + h), colour, 2)
            cv2.putText(image, f"{score:.2f}", (x, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
            print(f"{path.name}: person {person.box} score {score:.2f}"
                  + (" TARGET" if score >= model.threshold else ""))  # fmt: skip
        cv2.imwrite(str(out_dir / f"{path.stem}_body.jpg"), image)


def main() -> None:
    parser = argparse.ArgumentParser(description="recognise the target by their body")
    commands = parser.add_subparsers(dest="command", required=True)
    train_cmd = commands.add_parser("train", help="build the model from the photos in body/")
    train_cmd.add_argument("--photos", type=Path, default=BODY_DIR)
    train_cmd.add_argument("--negatives", type=Path, help="video or folder with other people")
    train_cmd.add_argument("--seed", type=int, default=0)
    check_cmd = commands.add_parser("check", help="score the people in images")
    check_cmd.add_argument("images", type=Path, help="an image or a folder")
    check_cmd.add_argument("--out", type=Path, default=Path("body_check"))
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")
    if args.command == "train":
        train(args.photos, args.negatives, args.seed)
        return
    source = args.images
    paths = sorted(source.iterdir()) if source.is_dir() else [source]
    check([p for p in paths if is_photo(p)], args.out)


if __name__ == "__main__":
    main()
