"""Find the robot (Bobac: white round base with a black ring, grey box, screen, landing pad on top)
in drone frames, from afar and diagonally, to fly back to it and land (the pad, pad.py, takes
over once close).

    uv run --with opencv-python-headless python -m drone.identify.robot train --negatives <video>
    uv run --with opencv-python-headless python -m drone.identify.robot check <image or folder>

Overfitted to this robot in this hall:
1. proposals: blobs that are not floor. The floor is the frame's own dominant colours (whatever
   the carpet), the rest is cleaned with two opening sizes (thin tape lines on the floor would
   join the robot to other things) and closed into blobs; each blob's box, also grown a little;
2. HOG of the grey crop + its colours (HSV histogram) -> RBF SVM; the best proposal over the
   calibrated threshold is the robot.

Training photos: drone/identify/robot/ (not in git), the robot alone on the floor, seen
diagonally (phone at chest height is close to the drone's view). The robot's box in each photo
is the non-floor blob near the middle with the most white (implausibly wide boxes are left
out); `train` saves models/robot_check.jpg to verify them. Augmented with size (farther away),
in-image rotation (the drone rolls and yaws), light and blur. Negatives: the other blobs of the
photos and all the proposals of the negative video/folder (the hall without the robot).
"""

import argparse
import hashlib
import json
import logging
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .models import MODELS_DIR
from .pad import load_negatives
from .people import is_photo

ROBOT_DIR = Path(__file__).resolve().parent / "robot"
MODEL_PATH = MODELS_DIR / "robot_svm.xml"
THRESHOLD_PATH = MODELS_DIR / "robot_threshold.json"
CHECK_IMAGE = MODELS_DIR / "robot_check.jpg"
PHOTO_SIZE = (540, 720)
# Proposals
WORK_WIDTH = 640  # proposals are found on a copy this wide
FLOOR_BINS = (30, 16)  # hue, saturation
FLOOR_SHARE = 0.6  # the most common colours covering this share of the frame are the floor
OPEN_PX = (9, 17)  # at WORK_WIDTH: removes tape lines and speckle
CLOSE_PX = 11
MIN_BLOB_SHARE = 0.001  # of the frame
MAX_BLOB_SHARE = 0.5
GROW = 0.1  # each box also proposed grown by this fraction on each side
DUPLICATE_IOU = 0.85
# Classifier
CROP_SIZE = (48, 64)
HOG = cv2.HOGDescriptor(CROP_SIZE, (16, 16), (8, 8), (8, 8), 9)
HIST_BINS = (8, 4, 4)
HIST_RANGES = (0, 180, 0, 256, 0, 256)
# Training
MAX_ROBOT_WIDTH = 0.6  # of the photo: a wider "robot" box is a merged blob, the photo is skipped
CENTRE_BAND = (0.2, 0.8)  # the robot's centre is in this horizontal band of the photo...
MIN_BOTTOM = 0.4  # ...and its bottom below this fraction of the height (not the far tables)
WHITE_MAX_SATURATION = 50
WHITE_MIN_VALUE = 150
AUGMENTED_PER_PHOTO = 30
ROBOT_PX = (40, 300)  # height of the robot in the augmented crops (drone at 1-5 m)
ROTATION_DEG = 15.0
BOX_JITTER = 0.06
POSITIVE_IOU = 0.6  # a proposal overlapping the robot's box this much is a positive...
NEGATIVE_IOU = 0.2  # ...and this little a negative
HELD_OUT_EVERY = 4
SVM_C = 10.0
SVM_GAMMA = 0.5
MAX_FALSE_POSITIVES = 0.02
# The threshold never goes below this: on 4 folds of held-out photos vs unseen hall frames, -0.2
# found the robot in 88 % of the photos with no false positive (the 2 % quantile alone let 10 %
# of hall frames through; hard negative mining only cost recall)
MIN_THRESHOLD = -0.2

log = logging.getLogger("identify")

Box = tuple[int, int, int, int]  # left, top, width, height in pixels


@dataclass(frozen=True)
class Robot:
    score: float
    box: Box

    def bbox(self, shape: tuple[int, ...]) -> list[float]:
        """[x0, y0, x1, y1] as fractions of the image, like the mission's detections."""
        h, w = shape[:2]
        x, y, bw, bh = self.box
        return [round(v, 4) for v in (x / w, y / h, (x + bw) / w, (y + bh) / h)]


class RobotDetector:
    def __init__(self) -> None:
        if not MODEL_PATH.is_file() or not THRESHOLD_PATH.is_file():
            raise FileNotFoundError(f"{MODEL_PATH}: train it (python -m drone.identify.robot)")
        self.svm = cv2.ml.SVM_load(str(MODEL_PATH))
        self.threshold = json.loads(THRESHOLD_PATH.read_text())["threshold"]

    def detect(self, image: np.ndarray) -> Robot | None:
        best = best_proposal(image, self.svm)
        return best if best is not None and best.score >= self.threshold else None


def best_proposal(image: np.ndarray, svm: cv2.ml.SVM) -> Robot | None:
    boxes = proposals(image)
    crops = [c for c in (crop_box(image, b) for b in boxes) if c is not None]
    if not crops:
        return None
    kept = [b for b in boxes if crop_box(image, b) is not None]
    scores = _scores(svm, crops)
    best = int(np.argmax(scores))
    return Robot(float(scores[best]), kept[best])


def floor_mask(image: np.ndarray) -> np.ndarray:
    """True where the pixel has one of the frame's dominant colours (the floor)."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    ranges = [0, 180, 0, 256]
    histogram = cv2.calcHist([hsv], [0, 1], None, list(FLOOR_BINS), ranges).ravel()
    order = np.argsort(histogram)[::-1]
    covered = np.cumsum(histogram[order]) / histogram.sum()
    floor = np.zeros(histogram.size, np.float32)
    floor[order[: np.searchsorted(covered, FLOOR_SHARE) + 1]] = 255
    back = cv2.calcBackProject([hsv], [0, 1], floor.reshape(FLOOR_BINS), ranges, 1)
    return back > 0


def proposals(image: np.ndarray) -> list[Box]:
    """Boxes of the non-floor blobs (two cleaning levels), as found and grown, each once."""
    scale = min(1.0, WORK_WIDTH / image.shape[1])
    small = cv2.resize(image, None, fx=scale, fy=scale) if scale < 1 else image
    not_floor = (~floor_mask(small)).astype(np.uint8) * 255
    area = small.shape[0] * small.shape[1]
    boxes: list[Box] = []
    for open_px in OPEN_PX:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_px, open_px))
        mask = cv2.morphologyEx(not_floor, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((CLOSE_PX, CLOSE_PX), np.uint8))
        count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
        for x, y, w, h, pixels in stats[1:count]:
            if not MIN_BLOB_SHARE * area <= pixels <= MAX_BLOB_SHARE * area:
                continue
            for grow in (0.0, GROW):
                box = _grown((x, y, w, h), grow, small.shape)
                if all(iou(box, b) < DUPLICATE_IOU for b in boxes):
                    boxes.append(box)
    return [tuple(round(v / scale) for v in box) for box in boxes]


def _grown(box: Box, grow: float, shape: tuple[int, ...]) -> Box:
    x, y, w, h = box
    dx, dy = w * grow, h * grow
    x0, y0 = max(0, round(x - dx)), max(0, round(y - dy))
    x1, y1 = min(shape[1], round(x + w + dx)), min(shape[0], round(y + h + dy))
    return x0, y0, x1 - x0, y1 - y0


def crop_box(image: np.ndarray, box: Box) -> np.ndarray | None:
    x, y, w, h = box
    crop = image[max(0, y) : y + h, max(0, x) : x + w]
    return crop if crop.shape[0] >= 8 and crop.shape[1] >= 8 else None


def features(crop: np.ndarray) -> np.ndarray:
    small = cv2.resize(crop, CROP_SIZE, interpolation=cv2.INTER_AREA)
    hog = HOG.compute(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)).ravel()
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    counts = cv2.calcHist([hsv], [0, 1, 2], None, list(HIST_BINS), list(HIST_RANGES)).ravel()
    return np.concatenate([hog, np.sqrt(counts / (counts.sum() + 1e-6))])


def _scores(svm: cv2.ml.SVM, crops: list[np.ndarray]) -> np.ndarray:
    rows = np.array([features(c) for c in crops], np.float32)
    # RAW_OUTPUT is the decision value, negative on the robot side for OpenCV's SVM
    return -svm.predict(rows, flags=cv2.ml.STAT_MODEL_RAW_OUTPUT)[1].ravel()


def iou(a: Box, b: Box) -> float:
    x0, y0 = max(a[0], b[0]), max(a[1], b[1])
    x1, y1 = min(a[0] + a[2], b[0] + b[2]), min(a[1] + a[3], b[1] + b[3])
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    union = a[2] * a[3] + b[2] * b[3] - inter
    return inter / union if union > 0 else 0.0


# --- training --------------------------------------------------------------------------------


def robot_photos(photo_dir: Path) -> list[tuple[np.ndarray, Box]]:
    """(photo, the robot's box) per usable photo, copies once."""
    seen, photos = set(), []
    for path in sorted(p for p in photo_dir.iterdir() if is_photo(p)):
        image = cv2.imread(str(path))
        if image is None:
            continue
        key = hashlib.md5(image.tobytes()).hexdigest()
        if key in seen:
            continue
        seen.add(key)
        image = cv2.resize(image, PHOTO_SIZE)
        box = _robot_box(image)
        if box is None:
            log.warning("robot not found in %s: photo skipped", path.name)
            continue
        photos.append((image, box))
    return photos


def _robot_box(image: np.ndarray) -> Box | None:
    """Of the non-floor blobs in the middle, low enough and not implausibly wide, the one with
    the most white pixels (the robot's base; tables and tape lines have little white)."""
    height, width = image.shape[:2]
    lo, hi = CENTRE_BAND
    centred = [
        b
        for b in proposals(image)
        if lo * width < b[0] + b[2] / 2 < hi * width
        and b[1] + b[3] > MIN_BOTTOM * height
        and b[2] <= MAX_ROBOT_WIDTH * width
    ]
    white = _white(image)

    def white_pixels(box: Box) -> int:
        x, y, w, h = box
        return int(white[y : y + h, x : x + w].sum())

    return max(centred, key=white_pixels, default=None)


def _white(image: np.ndarray) -> np.ndarray:
    """1 where the pixel is white (the robot's base and box), else 0."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return ((hsv[..., 1] < WHITE_MAX_SATURATION) & (hsv[..., 2] > WHITE_MIN_VALUE)).astype(np.int32)


def _augmented(image: np.ndarray, box: Box, rng: np.random.Generator) -> np.ndarray | None:
    """The photo rotated about the robot and shrunk like a farther drone, then the robot's crop
    with an imprecise box, light changes and blur."""
    x, y, w, h = box
    centre = (x + w / 2, y + h / 2)
    scale = min(1.0, rng.uniform(*ROBOT_PX) / h)
    matrix = cv2.getRotationMatrix2D(centre, rng.uniform(-ROTATION_DEG, ROTATION_DEG), scale)
    size = (image.shape[1], image.shape[0])
    view = cv2.warpAffine(image, matrix, size, borderMode=cv2.BORDER_REFLECT)
    cx, cy = centre
    jitter = rng.normal(0, BOX_JITTER, 4)
    bw, bh = w * scale * np.exp(jitter[2]), h * scale * np.exp(jitter[3])
    left, top = cx - bw / 2 + jitter[0] * bw, cy - bh / 2 + jitter[1] * bh
    crop = crop_box(view, (round(left), round(top), round(bw), round(bh)))
    if crop is None:
        return None
    light = crop.astype(np.float32) * rng.uniform(0.6, 1.3) + rng.uniform(-20, 20)
    crop = np.clip(light, 0, 255).astype(np.uint8)
    return cv2.GaussianBlur(crop, (3, 3), 0) if rng.random() < 0.5 else crop


def _positives(photos: list[tuple[np.ndarray, Box]], rng: np.random.Generator) -> list:
    crops = []
    for image, box in photos:
        crops.append(crop_box(image, box))
        for _ in range(AUGMENTED_PER_PHOTO):
            crop = _augmented(image, box, rng)
            if crop is not None:
                crops.append(crop)
    return [c for c in crops if c is not None]


def _negatives(photos: list[tuple[np.ndarray, Box]], frames: list[np.ndarray]) -> list:
    crops = []
    for image, box in photos:
        crops += [crop_box(image, b) for b in proposals(image) if iou(b, box) < NEGATIVE_IOU]
    for frame in frames:
        crops += [crop_box(frame, b) for b in proposals(frame)]
    return [c for c in crops if c is not None]


def _fit(positives: list, negatives: list) -> cv2.ml.SVM:
    rows = np.array([features(c) for c in positives + negatives], np.float32)
    labels = np.array([1] * len(positives) + [0] * len(negatives), np.int32)
    svm = cv2.ml.SVM_create()
    svm.setType(cv2.ml.SVM_C_SVC)
    svm.setKernel(cv2.ml.SVM_RBF)
    svm.setC(SVM_C)
    svm.setGamma(SVM_GAMMA)
    svm.train(rows, cv2.ml.ROW_SAMPLE, labels)
    return svm


def _threshold(svm: cv2.ml.SVM, frames: list[np.ndarray]) -> float:
    """The score that at most MAX_FALSE_POSITIVES of robot-free frames reach."""
    best = [r.score for r in (best_proposal(f, svm) for f in frames) if r is not None]
    calibrated = float(np.quantile(best, 1 - MAX_FALSE_POSITIVES)) if best else MIN_THRESHOLD
    return max(calibrated, MIN_THRESHOLD)


def _found(svm: cv2.ml.SVM, photo: tuple[np.ndarray, Box], threshold: float) -> bool:
    image, box = photo
    robot = best_proposal(image, svm)
    return robot is not None and robot.score >= threshold and iou(robot.box, box) >= NEGATIVE_IOU


def train(photo_dir: Path, negatives: Path | None, seed: int) -> None:
    rng = np.random.default_rng(seed)
    photos = robot_photos(photo_dir)
    if not photos:
        raise ValueError(f"no photos of the robot in {photo_dir}")
    _save_check(photos)
    frames = load_negatives(negatives)
    if len(frames) < 2:
        raise ValueError("pass --negatives: frames without the robot, to set the threshold")
    train_frames, held_frames = frames[::2], frames[1::2]
    log.info("%d photos of the robot (check %s), %d negative frames", len(photos), CHECK_IMAGE,
             len(frames))  # fmt: skip

    held = photos[::HELD_OUT_EVERY]
    rest = [p for i, p in enumerate(photos) if i % HELD_OUT_EVERY]
    check_svm = _fit(_positives(rest, rng), _negatives(rest, train_frames))
    check_threshold = _threshold(check_svm, held_frames)
    found = sum(_found(check_svm, p, check_threshold) for p in held)
    log.info("held-out photos: robot found in %d/%d", found, len(held))

    svm = _fit(_positives(photos, rng), _negatives(photos, train_frames))
    threshold = _threshold(svm, held_frames)
    MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    svm.save(str(MODEL_PATH))
    THRESHOLD_PATH.write_text(json.dumps({"threshold": threshold}))
    log.info("saved %s, threshold %.3f (%.0f %% of robot-free frames pass)", MODEL_PATH,
             threshold, MAX_FALSE_POSITIVES * 100)  # fmt: skip


def _save_check(photos: list[tuple[np.ndarray, Box]]) -> None:
    thumbs = []
    for image, (x, y, w, h) in photos:
        marked = cv2.rectangle(image.copy(), (x, y), (x + w, y + h), (0, 255, 0), 4)
        thumbs.append(cv2.resize(marked, (135, 180)))
    while len(thumbs) % 8:
        thumbs.append(np.zeros_like(thumbs[0]))
    rows = [np.hstack(thumbs[k : k + 8]) for k in range(0, len(thumbs), 8)]
    CHECK_IMAGE.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(CHECK_IMAGE), np.vstack(rows))


def check(paths: list[Path], out_dir: Path) -> None:
    """Detect the robot in images; prints each result and saves a marked copy in out_dir."""
    detector = RobotDetector()
    out_dir.mkdir(parents=True, exist_ok=True)
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            continue
        started = time.perf_counter()
        robot = detector.detect(image)
        ms = (time.perf_counter() - started) * 1000
        if robot is None:
            print(f"{path.name}: no robot ({ms:.0f} ms)")
            continue
        x, y, w, h = robot.box
        print(f"{path.name}: robot at {robot.box}, score {robot.score:.2f} ({ms:.0f} ms)")
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.imwrite(str(out_dir / f"{path.stem}_robot.jpg"), image)


def main() -> None:
    parser = argparse.ArgumentParser(description="find the robot in drone frames")
    commands = parser.add_subparsers(dest="command", required=True)
    train_cmd = commands.add_parser("train", help="build the model from the photos in robot/")
    train_cmd.add_argument("--photos", type=Path, default=ROBOT_DIR)
    train_cmd.add_argument("--negatives", type=Path, help="video or folder without the robot")
    train_cmd.add_argument("--seed", type=int, default=0)
    check_cmd = commands.add_parser("check", help="detect the robot in images")
    check_cmd.add_argument("images", type=Path, help="an image or a folder")
    check_cmd.add_argument("--out", type=Path, default=Path("robot_check"))
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
