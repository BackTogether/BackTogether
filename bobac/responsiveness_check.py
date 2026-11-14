"""Check bobac/robot/responsiveness.py on a laptop, without the robot or a camera.

Give it one photo of a person standing, whole body visible, arms not raised. It builds short
videos from it (still, moving, camera noise, lying down) and prints what each check answers
next to what it should answer:

    uv run --no-project --with mediapipe==0.10.9 --with "numpy<2" --with opencv-python-headless \
        python bobac/responsiveness_check.py person.jpg

The photo and the videos are data: keep them out of git.
"""

import argparse
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / "robot"))

import responsiveness  # noqa: E402  (lives in bobac/robot/, which runs on the robot)

FRAME_WIDTH = 640
VIDEO_FRAMES = 120
VIDEO_FPS = 30
SWAY_PX = 25  # how far the "moving" person sways sideways
SWAY_FRAMES = 6.0  # frames per radian of the sway
NOISE_LEVELS = 6  # grey levels of fake camera noise
CHECK_S = 3.0


def write_video(path: Path, frames: list) -> str:
    height, width = frames[0].shape[:2]
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), VIDEO_FPS, (width, height))
    for frame in frames:
        writer.write(frame)
    writer.release()
    return str(path)


def swayed(image: np.ndarray, index: int) -> np.ndarray:
    shift = np.float32([[1, 0, SWAY_PX * np.sin(index / SWAY_FRAMES)], [0, 1, 0]])
    height, width = image.shape[:2]
    return cv2.warpAffine(image, shift, (width, height), borderMode=cv2.BORDER_REPLICATE)


def noisy(image: np.ndarray, random: np.random.Generator) -> np.ndarray:
    noise = random.integers(-NOISE_LEVELS, NOISE_LEVELS + 1, image.shape)
    return np.clip(image.astype(int) + noise, 0, 255).astype(np.uint8)


def main() -> None:
    parser = argparse.ArgumentParser(description="check responsiveness.py with one photo")
    parser.add_argument("photo", help="a person standing, whole body visible, arms not raised")
    args = parser.parse_args()
    image = cv2.imread(args.photo)
    if image is None:
        parser.error("cannot read " + args.photo)
    scale = FRAME_WIDTH / image.shape[1]
    image = cv2.resize(image, (FRAME_WIDTH, int(image.shape[0] * scale)))
    lying = cv2.rotate(image, cv2.ROTATE_180)  # head down: never an upright torso
    random = np.random.default_rng(0)

    folder = Path(tempfile.mkdtemp(prefix="responsiveness_check_"))
    videos = {
        "still": write_video(folder / "still.avi", [image] * VIDEO_FRAMES),
        "moving": write_video(
            folder / "moving.avi", [swayed(image, index) for index in range(VIDEO_FRAMES)]
        ),
        "noise": write_video(
            folder / "noise.avi", [noisy(image, random) for _ in range(VIDEO_FRAMES)]
        ),
        "lying": write_video(folder / "lying.avi", [lying] * VIDEO_FRAMES),
    }
    photo_path = str(folder / "photo.jpg")
    checks = [
        ("is_standing, person standing", "still", responsiveness.is_standing, True),
        ("is_standing, person upside down", "lying", responsiveness.is_standing, False),
        ("is_moving, person still", "still", lambda: responsiveness.is_moving(CHECK_S), False),
        ("is_moving, only camera noise", "noise", lambda: responsiveness.is_moving(CHECK_S), False),
        ("is_moving, person swaying", "moving", lambda: responsiveness.is_moving(CHECK_S), True),
        ("hand_raised, arms down", "still", lambda: responsiveness.hand_raised(CHECK_S), False),
        ("take_photo", "still", lambda: responsiveness.take_photo(photo_path), True),
    ]
    failures = 0
    for name, video, check, expected in checks:
        responsiveness.camera = videos[video]
        answer = check()
        failures += answer != expected
        print(f"{'ok  ' if answer == expected else 'FAIL'} {name}: {answer} (expected {expected})")
    print("cropped photo and videos in", folder)
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
