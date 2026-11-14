"""Recognise the people of the face database in the drone frames. Run it from the repo root.

While the drone flies, check every face it sees against the whole database; each person is
announced (beep, evidence image) the first time they are confirmed. Stop it with Ctrl+C:

    uv run --with opencv-python-headless python -m drone.identify

Looking for one person only (live by default): stops at the first confirmed match, or fails
after the timeout:

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
from .live import SEARCH_TIMEOUT_S, recognise_all, watch
from .people import PEOPLE_DIR, People
from .search import ATTEMPTS, search

DEFAULT_FRAMES = Path("recordings/frames")  # where drone.hoverctl saves the drone frames
FULL_SIZE = 0
NO_LIMIT_S = float("inf")
EXIT_FOUND = 0
EXIT_FAILED = 1
EXIT_BAD_INPUT = 2


def load_people(folder: Path) -> People:
    # Reference photos are read once at full size; live frames use the fast settings.
    people = People(FaceEmbedder(detect_width=FULL_SIZE, max_faces=MAX_DETECTED), folder)
    if not people.names:
        raise ValueError(f"no usable photo in {folder}: add <name>/<photo>.jpg")
    return people


def run_database(args: argparse.Namespace) -> None:
    """Recognise everyone in the database until the timeout or Ctrl+C."""
    people = load_people(args.people)
    args.frames.mkdir(parents=True, exist_ok=True)
    logging.getLogger("identify").info("recognising %s in %s", ", ".join(people.names), args.frames)

    def announce_person(name, sighting) -> None:
        evidence = args.evidence.with_name(f"{args.evidence.stem}_{name}{args.evidence.suffix}")
        announce.save_evidence(sighting, name, evidence)
        print(
            f"POSITIVE: {name} in {sighting.frame.parent.name}/{sighting.frame.name}, "
            f"similarity {sighting.similarity:.2f}, evidence in {evidence}",
            flush=True,
        )
        if not args.quiet:
            announce.play(True)

    try:
        found = recognise_all(args.frames, FaceEmbedder(), people, announce_person, args.timeout)
    except KeyboardInterrupt:
        found = None
    print("stopped" if found is None else f"done, recognised: {', '.join(found) or 'nobody'}")


def run_live(args: argparse.Namespace):
    people = load_people(args.people)
    if args.target not in people.names:
        raise ValueError(f"{args.target} has no usable photo in {args.people} ({people.names})")
    args.frames.mkdir(parents=True, exist_ok=True)
    logging.getLogger("identify").info(
        "live search for %s in %s, up to %.0f s", args.target, args.frames, args.timeout
    )
    return watch(args.target, args.frames, FaceEmbedder(), people, args.timeout)


def main() -> None:
    parser = argparse.ArgumentParser(description="find a person in the drone frames")
    parser.add_argument("target", nargs="?", help="look for this person only (folder name)")
    parser.add_argument("--frames", type=Path, default=DEFAULT_FRAMES, help="frames folder")
    parser.add_argument("--people", type=Path, default=PEOPLE_DIR, help="people database")
    parser.add_argument("--once", action="store_true", help="search existing frames, not live")
    parser.add_argument(
        "--timeout",
        type=float,
        help="live, seconds (default: no limit "
        f"for the whole database, {SEARCH_TIMEOUT_S:.0f} for one person)",
    )
    parser.add_argument("--attempts", type=int, default=ATTEMPTS, help="--once retries")
    parser.add_argument("--evidence", type=Path, default=Path("found.jpg"))
    parser.add_argument("--quiet", action="store_true", help="no sound")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")
    if args.once and not args.frames.is_dir():
        parser.error(f"{args.frames} is not a folder")
    if args.target is None:
        if args.once:
            parser.error("--once needs a target")
        args.timeout = args.timeout or NO_LIMIT_S
        try:
            run_database(args)
        except ValueError as error:
            print(f"ERROR: {error}")
            sys.exit(EXIT_BAD_INPUT)
        return
    args.timeout = args.timeout or SEARCH_TIMEOUT_S

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
