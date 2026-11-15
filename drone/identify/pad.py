"""Landing pad detector: the white square with the grey circle and the orange H on the robot.

    uv run --with opencv-python-headless python -m drone.identify.pad train
    uv run --with opencv-python-headless python -m drone.identify.pad check <image or folder>

The drone sees the pad mostly diagonally (camera at -25..-75 deg), never from straight above, at
0.6-4 m: 20-150 px wide and squashed. The H has the same hue as the carpet, so colour alone does
not work. The detector, overfitted to this one pad:

1. proposals: convex 4-gons of bright, unsaturated regions (several global and local brightness
   thresholds, so the light level does not matter);
2. each one is rectified with its homography to a 32x32 square (this undoes the diagonal view);
3. HOG of its grey and saturation channels -> an RBF SVM (cv2.ml) scores it; the best proposal
   over the calibrated threshold is the pad.

Training data comes from the photos in base/ (any angle, phone is fine): each photo is taken as
the floor plane and seen by a virtual X1 camera (focal length and frame size of drone/sdk/camera.py)
from random tilts, ranges and headings, with light, blur, noise and JPEG changes. Proposals near
the pad are positives (in 4 rotations), the rest negatives, plus every proposal of the optional
negative video/folder (rooms without the pad). The pad in each photo is found by a synthetic
template, then by the mean of what it found (a robot part can beat the synthetic template up
close); `train` saves models/pad_check.jpg to verify it. The threshold is calibrated on held-out
views and frames: at most MAX_FALSE_POSITIVES of frames without the pad give a detection.

On 7 photos, fresh diagonal views: 91 % detected at 2 % false positives (a linear SVM: 82 %);
all 8 original photos detected, close ups included. ~70 ms per 1280x720 frame on a laptop CPU.
The model (~16 MB) and its threshold live in models/ (not in git): train it once per pad.
"""

import argparse
import glob
import json
import logging
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from drone.sdk.camera import CAMERA_FOCAL_PX, CAMERA_HEIGHT_PX, CAMERA_WIDTH_PX

from .models import MODELS_DIR
from .people import is_photo

BASE_DIR = Path(__file__).resolve().parent / "base"
MODEL_PATH = MODELS_DIR / "pad_svm.xml"
THRESHOLD_PATH = MODELS_DIR / "pad_threshold.json"
CHECK_IMAGE = MODELS_DIR / "pad_check.jpg"  # not in base/: it would be read as a photo
PAD_SIDE_M = 0.20  # side of the white square: measure it, the training ranges depend on it
# Proposals
PATCH_PX = 32
BRIGHT_PERCENTILES = (75, 85, 92, 96, 98.5)
LOCAL_CONTRAST = (12, 25, 40)  # brighter than the surroundings (blur sigma LOCAL_SIGMA) by this
LOCAL_SIGMA = 25
SURROUND_SHRINK = 8
MAX_SATURATION = 90
MIN_AREA_PX = 120
MAX_AREA_FRACTION = 0.3
MIN_SOLIDITY = 0.6
POLY_EPSILONS = (0.03, 0.06)
DUPLICATE_PX = 4.0
HOG = cv2.HOGDescriptor((PATCH_PX, PATCH_PX), (16, 16), (8, 8), (8, 8), 9)
# Training
REFERENCE_WIDTH = 960  # photos are searched for the pad at this width
VIEWS_PER_PHOTO = 80
TILT_DEG = (25.0, 75.0)
RANGE_M = (0.6, 4.0)  # down to the last metre of the landing
AIM_OFFSET_M = 0.4  # the pad is not always in the middle of the frame
BORDER_BGR = (60, 110, 150)  # outside the photo: carpet-like
NEAR_SIDES = 0.2  # proposal centre this close to the pad (in pad sides)...
SAME_SIZE = (0.6, 1.6)  # ...and its side this ratio of the pad's: positive
OUTSIDE_SIDES = 0.1  # proposal centre this far outside the pad (in pad sides): negative
NEGATIVE_FRAMES = 120
SVM_C = 10.0  # RBF beat linear (89 vs 82 % at 2 % false positives) for ~3 ms per frame
SVM_GAMMA = 0.5
VALIDATION_VIEWS = 20  # per photo, for the threshold
MAX_FALSE_POSITIVES = 0.02
HIT_SIDES = 0.3  # a detection this close to the true centre (in pad sides) counts
TEMPLATE_PX = 64


@dataclass(frozen=True)
class Pad:
    score: float
    corners: np.ndarray  # 4x2 pixels, clockwise from the top-left-most
    centre: tuple[float, float]  # pixels

    def bbox(self, shape: tuple[int, ...]) -> list[float]:
        """[x0, y0, x1, y1] as fractions of the image, like the mission's detections."""
        h, w = shape[:2]
        (x0, y0), (x1, y1) = self.corners.min(0), self.corners.max(0)
        return [round(float(v), 4) for v in (x0 / w, y0 / h, x1 / w, y1 / h)]


class PadDetector:
    def __init__(self, model_path: Path = MODEL_PATH) -> None:
        if not model_path.is_file() or not THRESHOLD_PATH.is_file():
            raise FileNotFoundError(f"{model_path}: run `python -m drone.identify.pad train`")
        self.svm = cv2.ml.SVM_load(str(model_path))
        self.min_score = json.loads(THRESHOLD_PATH.read_text())["min_score"]

    def detect(self, image: np.ndarray) -> Pad | None:
        """The best proposal if it scores at least min_score."""
        best = best_proposal(image, self.svm)
        return best if best is not None and best.score >= self.min_score else None


def best_proposal(image: np.ndarray, svm: cv2.ml.SVM) -> Pad | None:
    best = None
    for quad in proposals(image):
        score = _score(svm, image, quad)
        if best is None or score > best.score:
            best = Pad(score, quad, tuple(float(v) for v in quad.mean(0)))
    return best


def _score(svm: cv2.ml.SVM, image: np.ndarray, quad: np.ndarray) -> float:
    row = features(rectify(image, quad, PATCH_PX)).reshape(1, -1)
    # RAW_OUTPUT is the decision value, negative on the pad side for OpenCV's SVM
    return -float(svm.predict(row, flags=cv2.ml.STAT_MODEL_RAW_OUTPUT)[1][0, 0])


def proposals(image: np.ndarray) -> list[np.ndarray]:
    """Convex quadrilaterals of bright unsaturated regions, each once."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    value, saturation = hsv[..., 2], hsv[..., 1]
    grey = saturation < MAX_SATURATION
    masks = [(value > t) & grey for t in np.percentile(value, BRIGHT_PERCENTILES)]
    local = value.astype(np.int16) - _surroundings(value).astype(np.int16)
    masks += [(local > c) & grey for c in LOCAL_CONTRAST]
    max_area = MAX_AREA_FRACTION * image.shape[0] * image.shape[1]
    found: list[np.ndarray] = []
    for mask in masks:
        closed = cv2.morphologyEx(mask.astype(np.uint8) * 255, cv2.MORPH_CLOSE, np.ones((3, 3)))
        for contour in cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            quad = _quad(contour, max_area)
            if quad is not None and all(np.abs(quad - q).max() > DUPLICATE_PX for q in found):
                found.append(quad)
    return found


def _surroundings(value: np.ndarray) -> np.ndarray:
    """The brightness around each pixel: a wide blur, done on a small copy (10x faster, the
    same result for a blur this wide)."""
    small = cv2.resize(value, None, fx=1 / SURROUND_SHRINK, fy=1 / SURROUND_SHRINK,
                       interpolation=cv2.INTER_AREA)  # fmt: skip
    blurred = cv2.GaussianBlur(small, (0, 0), LOCAL_SIGMA / SURROUND_SHRINK)
    return cv2.resize(blurred, (value.shape[1], value.shape[0]), interpolation=cv2.INTER_LINEAR)


def _quad(contour: np.ndarray, max_area: float) -> np.ndarray | None:
    area = cv2.contourArea(contour)
    if not MIN_AREA_PX <= area <= max_area:
        return None
    hull = cv2.convexHull(contour)
    if area < MIN_SOLIDITY * cv2.contourArea(hull):
        return None
    for epsilon in POLY_EPSILONS:
        poly = cv2.approxPolyDP(hull, epsilon * cv2.arcLength(hull, True), True)
        if len(poly) == 4:
            return _clockwise(poly.reshape(4, 2).astype(np.float32))
    return None


def _clockwise(quad: np.ndarray) -> np.ndarray:
    centre = quad.mean(0)
    return quad[np.argsort(np.arctan2(quad[:, 1] - centre[1], quad[:, 0] - centre[0]))]


def rectify(image: np.ndarray, quad: np.ndarray, size: int) -> np.ndarray:
    square = np.float32([[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]])
    return cv2.warpPerspective(image, cv2.getPerspectiveTransform(quad, square), (size, size))


def features(patch: np.ndarray) -> np.ndarray:
    grey = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    saturation = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)[..., 1]
    return np.concatenate([HOG.compute(grey).ravel(), HOG.compute(saturation).ravel()])


# --- training --------------------------------------------------------------------------------


def synthetic_template(size: int = TEMPLATE_PX) -> np.ndarray:
    """The pad drawn from its description: white, grey ring, orange H."""
    pad = np.full((size, size, 3), 235, np.uint8)
    cv2.circle(pad, (size // 2, size // 2), int(size * 0.40), (150, 150, 150), max(2, size // 22))
    orange, a, b, stroke = (120, 190, 240), int(size * 0.30), int(size * 0.70), size // 8
    cv2.rectangle(pad, (a, a), (a + stroke, b), orange, -1)
    cv2.rectangle(pad, (b - stroke, a), (b, b), orange, -1)
    cv2.rectangle(pad, (a, size // 2 - size // 14), (b, size // 2 + size // 14), orange, -1)
    return pad


def _template_vectors(template: np.ndarray) -> list[np.ndarray]:
    return [_unit(np.ascontiguousarray(np.rot90(template, k))) for k in range(4)]


def _unit(patch: np.ndarray) -> np.ndarray:
    """Grey and saturation, each zero-mean and unit-length."""
    channels = [
        cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY).astype(np.float32),
        cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)[..., 1].astype(np.float32),
    ]
    centred = [c - c.mean() for c in channels]
    return np.stack([c / (np.linalg.norm(c) + 1e-6) for c in centred])


def _match(image: np.ndarray, quad: np.ndarray, vectors: list[np.ndarray]) -> tuple[float, int]:
    """(correlation, rotation) of the best-matching rotation of the template."""
    unit = _unit(rectify(image, quad, TEMPLATE_PX))
    scores = [float((unit * v).sum()) / 2 for v in vectors]
    return max(scores), int(np.argmax(scores))


def locate_pads(photos: list[np.ndarray]) -> list[np.ndarray]:
    """The pad's corners in each photo (full-size pixels): synthetic template first, then the
    mean of the pads it found."""
    small = [cv2.resize(p, None, fx=REFERENCE_WIDTH / p.shape[1], fy=REFERENCE_WIDTH / p.shape[1])
             for p in photos]  # fmt: skip
    candidates = [proposals(s) for s in small]
    vectors = _template_vectors(synthetic_template())
    for _ in range(2):
        picks = [
            max(c, key=lambda q, s=s: _match(s, q, vectors)[0])
            for s, c in zip(small, candidates, strict=True)
        ]
        aligned = []
        for s, q in zip(small, picks, strict=True):
            _, rotation = _match(s, q, vectors)
            aligned.append(np.rot90(rectify(s, q, TEMPLATE_PX), -rotation).astype(np.float32))
        vectors = _template_vectors(np.mean(aligned, axis=0).astype(np.uint8))
    return [q * p.shape[1] / REFERENCE_WIDTH for q, p in zip(picks, photos, strict=True)]


def _camera_homography(
    metres_per_px: float, centre: np.ndarray, tilt_deg: float, range_m: float, heading: float,
    aim: np.ndarray,
) -> np.ndarray:  # fmt: skip
    """Photo pixels -> virtual camera pixels. The photo is the floor plane (pad at the origin,
    image y down = world -y); the camera looks at `aim` from `range_m` away, `tilt_deg` down."""
    to_floor = np.array(
        [
            [metres_per_px, 0, -metres_per_px * centre[0]],
            [0, -metres_per_px, metres_per_px * centre[1]],
            [0, 0, 1],
        ]
    )
    target = np.array([aim[0], aim[1], 0.0])
    tilt = np.radians(tilt_deg)
    ahead = np.array([np.cos(heading), np.sin(heading), 0.0])
    position = target - ahead * range_m * np.cos(tilt) + np.array([0, 0, range_m * np.sin(tilt)])
    z = (target - position) / np.linalg.norm(target - position)
    x = np.cross(z, [0, 0, 1])
    x /= np.linalg.norm(x)
    rotation = np.stack([x, np.cross(z, x), z])
    k = np.array(
        [
            [CAMERA_FOCAL_PX, 0, CAMERA_WIDTH_PX / 2],
            [0, CAMERA_FOCAL_PX, CAMERA_HEIGHT_PX / 2],
            [0, 0, 1],
        ]
    )
    extrinsic = np.column_stack([rotation[:, 0], rotation[:, 1], -rotation @ position])
    return k @ extrinsic @ to_floor


def _degrade(image: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    light = image.astype(np.float32) * rng.uniform(0.6, 1.3) + rng.uniform(-20, 20)
    image = np.clip(light, 0, 255).astype(np.uint8)
    blur = int(rng.choice([1, 3, 5]))
    if blur > 1:
        image = cv2.GaussianBlur(image, (blur, blur), 0)
    noisy = image + rng.normal(0, rng.uniform(0, 8), image.shape)
    image = np.clip(noisy, 0, 255).astype(np.uint8)
    quality = int(rng.integers(50, 90))
    return cv2.imdecode(cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])[1], 1)


def diagonal_views(
    photo: np.ndarray, corners: np.ndarray, count: int, rng: np.random.Generator
) -> list[tuple[np.ndarray, np.ndarray]]:
    """(view, the pad's corners in the view) seen by the virtual X1 camera."""
    side_px = _side(corners)
    centre = corners.mean(0)
    views = []
    while len(views) < count:
        homography = _camera_homography(
            PAD_SIDE_M / side_px,
            centre,
            rng.uniform(*TILT_DEG),
            rng.uniform(*RANGE_M),
            rng.uniform(0, 2 * np.pi),
            rng.uniform(-AIM_OFFSET_M, AIM_OFFSET_M, 2),
        )
        size = (CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX)
        view = cv2.warpPerspective(photo, homography, size, borderValue=BORDER_BGR)
        seen = cv2.perspectiveTransform(corners.reshape(1, 4, 2), homography).reshape(4, 2)
        x, y = seen.mean(0)
        if 0 < x < CAMERA_WIDTH_PX and 0 < y < CAMERA_HEIGHT_PX:
            views.append((_degrade(view, rng), seen.astype(np.float32)))
    return views


def _side(quad: np.ndarray) -> float:
    return float(np.mean([np.linalg.norm(quad[k] - quad[(k + 1) % 4]) for k in range(4)]))


def _label(quad: np.ndarray, pad: np.ndarray) -> int | None:
    """1: this proposal is the pad (centre and size match); 0: it lies outside the pad; None:
    a piece of the pad (close up, the white between the ring and the edge is one), not used."""
    side, pad_side = _side(quad), _side(pad)
    distance = np.linalg.norm(quad.mean(0) - pad.mean(0))
    if distance < NEAR_SIDES * pad_side + 3 and SAME_SIZE[0] < side / pad_side < SAME_SIZE[1]:
        return 1
    centre = tuple(float(v) for v in quad.mean(0))
    outside = -cv2.pointPolygonTest(pad.reshape(-1, 1, 2), centre, True)
    return 0 if outside > OUTSIDE_SIDES * pad_side else None


def training_set(
    views: list[tuple[np.ndarray, np.ndarray]], negatives: list[np.ndarray]
) -> tuple[np.ndarray, np.ndarray]:
    rows, labels = [], []
    for view, pad in views:
        for quad in proposals(view):
            label = _label(quad, pad)
            if label is None:
                continue
            patch = rectify(view, quad, PATCH_PX)
            turns = range(4) if label else range(1)  # the pad in every rotation
            for k in turns:
                rows.append(features(np.ascontiguousarray(np.rot90(patch, k))))
                labels.append(label)
    for image in negatives:
        for quad in proposals(image):
            rows.append(features(rectify(image, quad, PATCH_PX)))
            labels.append(0)
    return np.array(rows, np.float32), np.array(labels, np.int32)


def load_negatives(source: Path | None, count: int = NEGATIVE_FRAMES) -> list[np.ndarray]:
    """Frames without the pad: a video or a folder of images, resized to the X1's frame."""
    if source is None:
        return []
    size = (CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX)
    if source.is_dir():
        paths = sorted(p for p in source.iterdir() if is_photo(p))[:count]
        return [cv2.resize(cv2.imread(str(p)), size) for p in paths]
    video = cv2.VideoCapture(str(source))
    frames = []
    total = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    for index in np.linspace(0, max(total - 1, 0), count).astype(int):
        video.set(cv2.CAP_PROP_POS_FRAMES, int(index))
        ok, frame = video.read()
        if ok:
            frames.append(cv2.resize(frame, size))
    return frames


def train(photo_dir: Path, negatives: Path | None, views_per_photo: int, seed: int) -> None:
    log = logging.getLogger("pad")
    paths = sorted(p for p in photo_dir.iterdir() if is_photo(p))
    photos = _unique([cv2.imread(str(p)) for p in paths])
    if not photos:
        raise ValueError(f"no photos of the pad in {photo_dir}")
    corners = locate_pads(photos)
    CHECK_IMAGE.parent.mkdir(parents=True, exist_ok=True)
    _save_check(photos, corners, CHECK_IMAGE)
    log.info("pad found in %d photos: check %s", len(photos), CHECK_IMAGE)
    rng = np.random.default_rng(seed)
    pairs = list(zip(photos, corners, strict=True))
    views = [v for p, c in pairs for v in diagonal_views(p, c, views_per_photo, rng)]
    frames = load_negatives(negatives)
    rows, labels = training_set(views, frames[::2])
    log.info("training on %d positives, %d negatives", labels.sum(), (labels == 0).sum())
    svm = cv2.ml.SVM_create()
    svm.setType(cv2.ml.SVM_C_SVC)
    svm.setKernel(cv2.ml.SVM_RBF)
    svm.setC(SVM_C)
    svm.setGamma(SVM_GAMMA)
    svm.train(rows, cv2.ml.ROW_SAMPLE, labels)
    validation = [v for p, c in pairs for v in diagonal_views(p, c, VALIDATION_VIEWS, rng)]
    threshold = _calibrate(svm, validation, frames[1::2])
    MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    svm.save(str(MODEL_PATH))
    THRESHOLD_PATH.write_text(json.dumps({"min_score": threshold}))
    found = sum(_hit(best_proposal(view, svm), pad, threshold) for view, pad in validation)
    log.info(
        "saved %s; threshold %.3f (%.0f %% false positives): %d/%d fresh diagonal views detected",
        MODEL_PATH,
        threshold,
        MAX_FALSE_POSITIVES * 100,
        found,
        len(validation),
    )


def _calibrate(
    svm: cv2.ml.SVM, views: list[tuple[np.ndarray, np.ndarray]], frames: list[np.ndarray]
) -> float:
    """The score that at most MAX_FALSE_POSITIVES of pad-free images reach: the best score of
    each held-out negative frame, and of each view's proposals away from the pad."""
    worst = [best.score for best in (best_proposal(f, svm) for f in frames) if best is not None]
    for view, pad in views:
        scores = [_score(svm, view, q) for q in proposals(view) if _label(q, pad) == 0]
        if scores:
            worst.append(max(scores))
    return float(np.quantile(worst, 1 - MAX_FALSE_POSITIVES)) if worst else 0.0


def _hit(found: Pad | None, pad: np.ndarray, threshold: float) -> bool:
    """A detection over the threshold, centred on the pad within HIT_SIDES of its side."""
    if found is None or found.score < threshold:
        return False
    return np.hypot(*(np.array(found.centre) - pad.mean(0))) < HIT_SIDES * _side(pad)


def _unique(images: list[np.ndarray | None]) -> list[np.ndarray]:
    """Readable images, exact duplicates once."""
    seen, unique = set(), []
    for image in images:
        if image is None:
            continue
        key = hash(image.tobytes())
        if key not in seen:
            seen.add(key)
            unique.append(image)
    return unique


def _save_check(photos: list[np.ndarray], corners: list[np.ndarray], path: Path) -> None:
    thumbs = []
    for photo, quad in zip(photos, corners, strict=True):
        marked = photo.copy()
        cv2.polylines(marked, [quad.astype(np.int32)], True, (0, 255, 0), 8)
        thumbs.append(cv2.resize(marked, (240, round(240 * photo.shape[0] / photo.shape[1]))))
    height = max(t.shape[0] for t in thumbs)
    row = [cv2.copyMakeBorder(t, 0, height - t.shape[0], 0, 0, cv2.BORDER_CONSTANT) for t in thumbs]
    cv2.imwrite(str(path), np.hstack(row))


def check(paths: list[Path], out_dir: Path) -> None:
    """Detect the pad in images; prints each result and saves a marked copy in out_dir."""
    detector = PadDetector()
    out_dir.mkdir(parents=True, exist_ok=True)
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            continue
        started = time.perf_counter()
        pad = detector.detect(image)
        ms = (time.perf_counter() - started) * 1000
        if pad is None:
            print(f"{path.name}: no pad ({ms:.0f} ms)")
            continue
        print(f"{path.name}: pad at {tuple(round(v) for v in pad.centre)}, score {pad.score:.2f}, "
              f"bbox {pad.bbox(image.shape)} ({ms:.0f} ms)")  # fmt: skip
        cv2.polylines(image, [pad.corners.astype(np.int32)], True, (0, 255, 0), 3)
        cv2.imwrite(str(out_dir / f"{path.stem}_pad.jpg"), image)


def main() -> None:
    parser = argparse.ArgumentParser(description="landing pad detector")
    commands = parser.add_subparsers(dest="command", required=True)
    train_cmd = commands.add_parser("train", help="build the model from the photos in base/")
    train_cmd.add_argument("--photos", type=Path, default=BASE_DIR)
    train_cmd.add_argument("--negatives", type=Path, help="video or folder without the pad")
    train_cmd.add_argument("--views", type=int, default=VIEWS_PER_PHOTO, help="per photo")
    train_cmd.add_argument("--seed", type=int, default=0)
    check_cmd = commands.add_parser("check", help="detect the pad in images")
    check_cmd.add_argument("images", type=Path, help="an image or a folder")
    check_cmd.add_argument("--out", type=Path, default=Path("pad_check"))
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")
    if args.command == "train":
        train(args.photos, args.negatives, args.views, args.seed)
        return
    source = args.images
    paths = sorted(Path(p) for p in glob.glob(str(source / "*"))) if source.is_dir() else [source]
    check([p for p in paths if is_photo(p)], args.out)


if __name__ == "__main__":
    main()
