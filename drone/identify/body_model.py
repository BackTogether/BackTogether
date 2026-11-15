"""Recognise the target by their body (clothes), without the face: from behind, from the side and
from above, where face recognition cannot work.

    uv run --with opencv-contrib-python-headless python -m drone.identify.body_model train \
        --negatives <video or folder with other people>
    uv run --with opencv-contrib-python-headless python -m drone.identify.body_model check <img>

Training photos: drone/identify/body/ (phone), body/drone/ (actual drone views), not in git.
The target must be the biggest person in each. Person boxes (YOLOX, persons.py) are described by
torso and leg colours (HSV histograms, Hellinger-normalised) and an RBF SVM overfitted to this
outfit. Actual drone crops keep their perspective, with mild box, light and blur variations.
Without drone photos, phone crops are projected as views from above. Every original crop is
also fitted unchanged.
Negatives: everybody else in the photos and in the negative video/folder.

A separate fit reports recall on held-out photos. The final demo fit uses all photos and
negative crops; its threshold stays above the negative scores and the SVM decision boundary.
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
PHOTO_MAX_SIDE = 1280  # preserve the aspect ratio, including landscape drone frames
CHECK_IMAGE = MODELS_DIR / "body_check.jpg"
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
SQUASH = (0.3, 1.0)  # height kept when seen from above (0.3: nearly straight down, from high)
NARROW = (0.5, 1.0)  # width kept at the feet
DRONE_HEIGHT_PX = (16, 160)  # flying higher: a child can be 16 px tall in the frame
HELD_OUT_EVERY = 4  # every 4th photo is held out to report how well it generalises
SVM_C = 10.0
SVM_GAMMA = 4.0
MAX_FALSE_POSITIVES = 0.0
DRONE_SCALE = (0.5, 1.0)
DRONE_TINT = (0.95, 1.05)
DRONE_BRIGHTNESS = (0.85, 1.15)
DRONE_OFFSET = (-8, 8)
CHECK_TILE = (240, 210)
CHECK_COLUMNS = 6

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


def augment(crop: np.ndarray, rng: np.random.Generator, native_view: bool = False) -> np.ndarray:
    """Vary the existing view mildly, or project a phone crop to a drone view."""
    if native_view:
        scale = rng.uniform(*DRONE_SCALE)
        view = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        tint = rng.uniform(*DRONE_TINT, 3) * rng.uniform(*DRONE_BRIGHTNESS)
        offset = rng.uniform(*DRONE_OFFSET)
    else:
        view = seen_from_above(crop, rng)
        tint = rng.uniform(0.6, 1.3, 3) ** 0.3 * rng.uniform(0.6, 1.3)
        offset = rng.uniform(-15, 15)
    view = np.clip(view.astype(np.float32) * tint + offset, 0, 255).astype(np.uint8)
    return cv2.GaussianBlur(view, (3, 3), 0) if rng.random() < 0.5 else view


def seen_from_above(crop: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """An eye-level crop projected as a small view from above."""
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
    return view


def _scores(svm: cv2.ml.SVM, crops: list[np.ndarray]) -> np.ndarray:
    rows = np.array([features(c) for c in crops], np.float32)
    # RAW_OUTPUT is the decision value, negative on the target side for OpenCV's SVM
    return -svm.predict(rows, flags=cv2.ml.STAT_MODEL_RAW_OUTPUT)[1].ravel()


# --- training --------------------------------------------------------------------------------


def target_photos(
    photo_dir: Path, detector: PersonDetector, outfit: np.ndarray | None = None
) -> list[tuple[np.ndarray, Box, list]]:
    """(photo, the target's box, the other people's boxes), once per photo. The target is the
    biggest person, or with `outfit` (mean features of known target crops) the person dressed
    most like it: in a drone frame a bystander near the camera is bigger than the target."""
    seen, photos = set(), []
    for path in sorted(p for p in photo_dir.iterdir() if is_photo(p)):
        image = cv2.imread(str(path))
        if image is None:
            continue
        key = hashlib.md5(image.tobytes()).hexdigest()  # copies of a photo count once
        if key in seen:
            continue
        seen.add(key)
        scale = min(1.0, PHOTO_MAX_SIDE / max(image.shape[:2]))
        image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        persons = detector.persons(image)
        if not persons:
            log.warning("nobody found in %s: photo ignored", path.name)
            continue
        target = _pick_target(image, persons, outfit)
        if crop_box(image, target.box) is None:
            log.warning("target too small in %s: photo ignored", path.name)
            continue
        photos.append((image, target.box, [p.box for p in persons if p is not target]))
    return photos


def _pick_target(image: np.ndarray, persons: list, outfit: np.ndarray | None):
    if outfit is None:
        return max(persons, key=lambda p: p.box[2] * p.box[3])
    distances = []
    for person in persons:
        crop = crop_box(image, person.box)
        distance = np.inf if crop is None else float(np.linalg.norm(features(crop) - outfit))
        distances.append(distance)
    return persons[int(np.argmin(distances))]


def _outfit(photos: list) -> np.ndarray | None:
    """Mean features of the target crops: what the target wears."""
    crops = [crop_box(image, box) for image, box, _ in photos]
    vectors = [features(crop) for crop in crops if crop is not None]
    return np.mean(vectors, axis=0) if vectors else None


def _positives(
    photos: list, rng: np.random.Generator, native_view: bool = False
) -> list[np.ndarray]:
    crops = []
    for image, box, _ in photos:
        original = crop_box(image, box)
        if original is not None:
            crops.append(original)
        for _ in range(AUGMENTED_PER_PHOTO):
            crop = crop_box(image, box, BOX_JITTER, rng)
            if crop is not None:
                crops.append(augment(crop, rng, native_view))
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


def train(
    photo_dir: Path, negatives: Path | None, seed: int, drone_photos: Path | None = None
) -> None:
    rng = np.random.default_rng(seed)
    detector = PersonDetector()
    photos = target_photos(photo_dir, detector)
    drone_dir = drone_photos if drone_photos is not None else photo_dir / "drone"
    outfit = _outfit(photos)
    drone = target_photos(drone_dir, detector, outfit) if drone_dir.is_dir() else []
    if drone_photos is not None and not drone:
        raise ValueError(f"no usable drone photos in {drone_photos}")
    if not photos and not drone:
        raise ValueError(f"no photos of the target in {photo_dir} or {drone_dir}")
    frames = [(f, [p.box for p in detector.persons(f)]) for f in load_negatives(negatives)]
    scenes = [(image, others) for image, _, others in photos + drone if others]
    train_scenes = scenes[::2] + frames[::2]
    held_people = _others(scenes[1::2] + frames[1::2], rng)
    other_people = _others(train_scenes, rng)
    if not other_people or not held_people:
        raise ValueError("need other people in training and calibration: pass --negatives")
    log.info("%d phone + %d drone photos; %d negative training and %d calibration crops",
             len(photos), len(drone), len(other_people), len(held_people))  # fmt: skip

    rest = [p for i, p in enumerate(photos) if i % HELD_OUT_EVERY]
    drone_rest = [p for i, p in enumerate(drone) if i % HELD_OUT_EVERY]
    # Actual drone views replace the need for speculative perspective warps of phone photos.
    keep_phone_view = bool(drone)
    validation = {}
    if rest or drone_rest:
        check_svm = _fit(_positives(rest, rng, keep_phone_view)
                         + _positives(drone_rest, rng, True), other_people)  # fmt: skip
        check_threshold = _threshold(check_svm, held_people)
        for name, group, is_drone in [("phone", photos, False), ("drone", drone, True)]:
            if group:
                held = _positives(group[::HELD_OUT_EVERY], rng, is_drone or keep_phone_view)
                recall = float(np.mean(_scores(check_svm, held) >= check_threshold))
                validation[name] = {"recognised_fraction": recall, "crops": len(held)}
                log.info("held-out %s views (including augmentations): %.1f %% recognised",
                         name, recall * 100)  # fmt: skip

    # The demo model memorises all available views and negatives after the held-out check.
    all_negatives = other_people + held_people
    svm = _fit(_positives(photos, rng, keep_phone_view) + _positives(drone, rng, True),
               all_negatives)  # fmt: skip
    threshold = _threshold(svm, all_negatives)
    report = {"threshold": threshold, "phone_photos": len(photos), "drone_photos": len(drone),
              "negative_training_crops": len(all_negatives), "validation": validation,
              "fit_false_positives": int(np.sum(_scores(svm, all_negatives) >= threshold)),
              "svm_c": SVM_C, "svm_gamma": SVM_GAMMA, "seed": seed}  # fmt: skip
    MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    svm.save(str(MODEL_PATH))
    THRESHOLD_PATH.write_text(json.dumps(report, indent=2) + "\n")
    _save_check(photos + drone, svm, threshold)
    log.info("saved %s, threshold %.3f; %d/%d fitted negatives pass; inspect %s",
             MODEL_PATH, threshold, report["fit_false_positives"], len(all_negatives),
             CHECK_IMAGE)  # fmt: skip


def _threshold(svm: cv2.ml.SVM, others: list[np.ndarray]) -> float:
    boundary = np.float32(np.quantile(_scores(svm, others), 1 - MAX_FALSE_POSITIVES))
    # Runtime accepts >=: the highest negative must stay strictly below the threshold.
    return max(0.0, float(np.nextafter(boundary, np.float32(np.inf))))


def _save_check(photos: list, svm: cv2.ml.SVM, threshold: float) -> None:
    tiles = []
    width, height = CHECK_TILE
    recognised = 0
    for i, (image, box, _) in enumerate(photos):
        score = float(_scores(svm, [crop_box(image, box)])[0])
        recognised += score >= threshold
        view = image.copy()
        x, y, w, h = box
        colour = (0, 200, 0) if score >= threshold else (0, 0, 255)
        cv2.rectangle(view, (x, y), (x + w, y + h), colour, 3)
        scale = min(width / view.shape[1], (height - 25) / view.shape[0])
        view = cv2.resize(view, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        tile = np.full((height, width, 3), 240, np.uint8)
        tile[: view.shape[0], : view.shape[1]] = view
        cv2.putText(tile, f"{i + 1}: {score:.2f}", (5, height - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1)  # fmt: skip
        tiles.append(tile)
    while len(tiles) % CHECK_COLUMNS:
        tiles.append(np.full_like(tiles[0], 240))
    sheet = np.vstack([np.hstack(tiles[i : i + CHECK_COLUMNS])
                       for i in range(0, len(tiles), CHECK_COLUMNS)])  # fmt: skip
    if not cv2.imwrite(str(CHECK_IMAGE), sheet):
        raise OSError(f"could not write {CHECK_IMAGE}")
    log.info("fit on original target photos: %d/%d recognised (training fit)",
             recognised, len(photos))  # fmt: skip


def check(paths: list[Path], out_dir: Path) -> None:
    """Score every person in images; prints them and saves marked copies (green: the target)."""
    model, detector = BodyModel(), PersonDetector()
    out_dir.mkdir(parents=True, exist_ok=True)
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            continue
        persons = detector.persons(image)
        scores = [model.score(image, p.box) for p in persons]
        for person, score in zip(persons, scores, strict=True):
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
    train_cmd.add_argument("--drone-photos", type=Path,
                           help="real drone views (default: <photos>/drone/)")  # fmt: skip
    train_cmd.add_argument("--negatives", type=Path, help="video or folder with other people")
    train_cmd.add_argument("--seed", type=int, default=0)
    check_cmd = commands.add_parser("check", help="score the people in images")
    check_cmd.add_argument("images", type=Path, help="an image or a folder")
    check_cmd.add_argument("--out", type=Path, default=Path("body_check"))
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")
    if args.command == "train":
        train(args.photos, args.negatives, args.seed, args.drone_photos)
        return
    source = args.images
    paths = sorted(source.iterdir()) if source.is_dir() else [source]
    check([p for p in paths if is_photo(p)], args.out)


if __name__ == "__main__":
    main()
