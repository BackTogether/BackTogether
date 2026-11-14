"""Search one person of the database in a folder of drone frames.

    uv run --with opencv-python-headless python -m drone.identify maria --frames recordings/frames

POSITIVE (two rising beeps, exit code 0) with the frame, the face and an evidence image, or
FAILED (one low beep, exit code 1) after the retries: the drone should come back.
"""

import argparse
import logging
import sys
from pathlib import Path

from . import announce
from .people import PEOPLE_DIR
from .search import ATTEMPTS, search

EXIT_FOUND = 0
EXIT_FAILED = 1
EXIT_BAD_INPUT = 2


def main() -> None:
    parser = argparse.ArgumentParser(description="find a person in a folder of drone frames")
    parser.add_argument("target", help="folder name of the person in the people database")
    parser.add_argument("--frames", type=Path, required=True, help="folder with the frames")
    parser.add_argument("--people", type=Path, default=PEOPLE_DIR, help="people database")
    parser.add_argument("--attempts", type=int, default=ATTEMPTS)
    parser.add_argument("--evidence", type=Path, default=Path("found.jpg"))
    parser.add_argument("--quiet", action="store_true", help="no sound")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s")
    if not args.frames.is_dir():
        parser.error(f"{args.frames} is not a folder")

    try:
        sighting = search(args.target, args.frames, args.people, args.attempts)
    except ValueError as error:
        print(f"ERROR: {error}")
        sys.exit(EXIT_BAD_INPUT)

    if sighting is None:
        print(f"FAILED: {args.target} not found after {args.attempts} attempts, come back")
    else:
        announce.save_evidence(sighting, args.target, args.evidence)
        print(
            f"POSITIVE: {args.target} in {sighting.frame.name}, face at {sighting.box}, "
            f"similarity {sighting.similarity:.2f}, attempt {sighting.attempt}, "
            f"evidence in {args.evidence}"
        )
    if not args.quiet:
        announce.play(sighting is not None)
    sys.exit(EXIT_FAILED if sighting is None else EXIT_FOUND)


if __name__ == "__main__":
    main()
