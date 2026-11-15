"""The three ONNX models, from the OpenCV Zoo. Downloaded once into drone/identify/models/.

    uv run --with opencv-contrib-python-headless python -m drone.identify.models

YuNet (face detection, MIT, 0.2 MB), SFace (face recognition, Apache-2.0, 37 MB) and YOLOX-S
(person detection for the body tracker, Apache-2.0, 34 MB). Download them before the demo: the
venue Wi-Fi is slow.
"""

import urllib.request
from pathlib import Path

MODELS_DIR = Path(__file__).resolve().parent / "models"
ZOO_URL = "https://github.com/opencv/opencv_zoo/raw/main/models"
DETECTOR = "face_detection_yunet/face_detection_yunet_2023mar.onnx"
RECOGNIZER = "face_recognition_sface/face_recognition_sface_2021dec.onnx"
PERSON_DETECTOR = "object_detection_yolox/object_detection_yolox_2022nov.onnx"
DOWNLOAD_TIMEOUT_S = 600


def model_path(zoo_path: str) -> Path:
    return MODELS_DIR / Path(zoo_path).name


def download(zoo_path: str) -> Path:
    """Fetch one model unless it is already there. Returns its local path."""
    target = model_path(zoo_path)
    if target.is_file():
        return target
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    partial = target.with_suffix(".part")  # so an interrupted download is never used
    with urllib.request.urlopen(f"{ZOO_URL}/{zoo_path}", timeout=DOWNLOAD_TIMEOUT_S) as response:
        partial.write_bytes(response.read())
    partial.rename(target)
    return target


def main() -> None:
    for zoo_path in (DETECTOR, RECOGNIZER, PERSON_DETECTOR):
        path = download(zoo_path)
        print(f"{path.name}: {path.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
