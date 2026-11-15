"""Calibrate a face gallery's threshold on other people.

    python -m drone.identify.face_model demo --negatives <video without the target>

The references are the ones people.py keeps (sharp close-ups). The gallery's own threshold is
saved in people/<name>/threshold.json (not in git), above the highest negative face score.
This changes the gallery threshold; the pretrained SFace network is unchanged.
"""

import argparse
import json
import logging
from pathlib import Path

from .faces import MAX_DETECTED, FaceEmbedder
from .pad import load_negatives
from .people import MATCH_THRESHOLD, PEOPLE_DIR, THRESHOLD_FILE, People

CALIBRATION_MARGIN = 0.02


def calibrate(name: str, negatives: Path, folder: Path = PEOPLE_DIR) -> None:
    """Choose a threshold above all faces in the negative video or folder."""
    embedder = FaceEmbedder(detect_width=0, max_faces=MAX_DETECTED)
    people = People(embedder, folder)
    if name not in people.names:
        raise ValueError(f"no usable face references for {name!r} in {folder}")
    scores = [people.score(name, face.embedding)
              for frame in load_negatives(negatives) for face in embedder.faces(frame)]  # fmt: skip
    if not scores:
        raise ValueError(f"no faces in {negatives}: cannot calibrate the gallery")
    threshold = max(MATCH_THRESHOLD, max(scores) + CALIBRATION_MARGIN)
    if threshold >= 1.0:
        raise ValueError("negative faces match the target: check the negative source")
    report = {"threshold": threshold, "negative_faces": len(scores),
              "max_negative_score": float(max(scores)), "margin": CALIBRATION_MARGIN,
              "calibration_false_positives": sum(s >= threshold for s in scores)}  # fmt: skip
    path = folder / name / THRESHOLD_FILE
    path.write_text(json.dumps(report, indent=2) + "\n")
    logging.getLogger("identify").info("saved %s: threshold %.3f, above %d negative faces",
                                       path, threshold, len(scores))  # fmt: skip


def main() -> None:
    parser = argparse.ArgumentParser(description="calibrate a demo face reference gallery")
    parser.add_argument("name", help="the target's subfolder in people/")
    parser.add_argument("--negatives", type=Path, required=True,
                        help="video or folder with other people, without the target")  # fmt: skip
    parser.add_argument("--people", type=Path, default=PEOPLE_DIR)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")
    calibrate(args.name, args.negatives, args.people)


if __name__ == "__main__":
    main()
