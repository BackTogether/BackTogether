"""Find one person of the database in the drone frames. Run it from the repo root.

Live (default): starts watching recordings/frames/ right away and classifies every new frame
the drone saves, until the person is confirmed or the timeout passes:

    uv run --with opencv-python-headless python -m drone.identify leo

Already recorded frames (one pass plus retries over a folder):

    uv run --with opencv-python-headless python -m drone.identify leo --once --frames <folder>

POSITIVE (two rising beeps, exit code 0) with the frame, the face and an evidence image, or
FAILED (one low beep, exit code 1): the drone should come back.
"""

import argparse
import logging
import sys
from pathlib import Path

from . import announce
from .faces import MAX_DETECTED, FaceEmbedder
from .live import SEARCH_TIMEOUT_S, watch
from .people import PEOPLE_DIR, People
from .search import ATTEMPTS, search

DEFAULT_FRAMES = Path("recordings/frames")  # where drone.hoverctl saves the drone frames
FULL_SIZE = 0
EXIT_FOUND = 0
EXIT_FAILED = 1
EXIT_BAD_INPUT = 2


def run_live(args: argparse.Namespace):
    # Reference photos are read once at full size; live frames use the fast settings.
    people = People(FaceEmbedder(detect_width=FULL_SIZE, max_faces=MAX_DETECTED), args.people)
    if args.target not in people.names:
        raise ValueError(f"{args.target} has no usable photo in {args.people} ({people.names})")
    args.frames.mkdir(parents=True, exist_ok=True)
    logging.getLogger("identify").info(
        "live search for %s in %s, up to %.0f s", args.target, args.frames, args.timeout
    )
    return watch(args.target, args.frames, FaceEmbedder(), people, args.timeout)


def main() -> None:
    parser = argparse.ArgumentParser(description="find a person in the drone frames")
    parser.add_argument("target", help="folder name of the person in the people database")
    parser.add_argument("--frames", type=Path, default=DEFAULT_FRAMES, help="frames folder")
    parser.add_argument("--people", type=Path, default=PEOPLE_DIR, help="people database")
    parser.add_argument("--once", action="store_true", help="search existing frames, not live")
    parser.add_argument("--timeout", type=float, default=SEARCH_TIMEOUT_S, help="live, seconds")
    parser.add_argument("--attempts", type=int, default=ATTEMPTS, help="--once retries")
    parser.add_argument("--evidence", type=Path, default=Path("found.jpg"))
    parser.add_argument("--quiet", action="store_true", help="no sound")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")
    if args.once and not args.frames.is_dir():
        parser.error(f"{args.frames} is not a folder")

    try:
        if args.once:
            sighting = search(args.target, args.frames, args.people, args.attempts)
        else:
            sighting = run_live(args)
    except ValueError as error:
        print(f"ERROR: {error}")
        sys.exit(EXIT_BAD_INPUT)
    except KeyboardInterrupt:
        print("stopped")
        sys.exit(EXIT_FAILED)

    if sighting is None:
        print(f"FAILED: {args.target} not found, come back")
    else:
        announce.save_evidence(sighting, args.target, args.evidence)
        print(
            f"POSITIVE: {args.target} in {sighting.frame.parent.name}/{sighting.frame.name}, "
            f"face at {sighting.box}, similarity {sighting.similarity:.2f}, "
            f"evidence in {args.evidence}"
        )
    if not args.quiet:
        announce.play(sighting is not None)
    sys.exit(EXIT_FAILED if sighting is None else EXIT_FOUND)


if __name__ == "__main__":
    main()
