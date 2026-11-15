"""Turn a raw video dump (`recordings/hover_video_*.udp`) into an MP4 and JPEG frames.

python -m drone.hoverctl.export recordings/hover_video_<time>.udp            # 2 frames/s
python -m drone.hoverctl.export recordings/*.udp --fps 5 --out recordings/frames

For each dump: `<out>/<dump name>/video.mp4` (stream copied, no re-encode) and
`<out>/<dump name>/frame_00001.jpg`, ... sampled at `--fps`.
"""

import argparse
import shutil
import subprocess
import sys
import tempfile
from collections.abc import Iterator
from pathlib import Path

from .video import DUMP_RECORD
from .video_payload import StreamExtractor, detect_codec

DEFAULT_OUT = Path("recordings/frames")
DEFAULT_FPS = 2.0
JPEG_QUALITY = "2"  # ffmpeg -q:v, 2 (best) .. 31


def datagrams(dump: Path) -> Iterator[bytes]:
    """The datagrams in a dump written by `VideoReceiver`, in arrival order."""
    data = dump.read_bytes()
    offset = 0
    while offset + DUMP_RECORD.size <= len(data):
        _, length = DUMP_RECORD.unpack_from(data, offset)
        offset += DUMP_RECORD.size
        yield data[offset : offset + length]
        offset += length


def elementary_stream(dump: Path) -> bytes:
    """The Annex-B H.264/H.265 stream carried by the dump (RTP or bare Annex-B)."""
    extractor = StreamExtractor()
    return b"".join(extractor.feed(datagram) for datagram in datagrams(dump))


def export(dump: Path, out: Path, fps: float) -> Path | None:
    """Write the MP4 and frames; None if the dump holds no video (e.g. an empty session)."""
    stream = elementary_stream(dump)
    codec = detect_codec(stream)
    if codec is None:
        return None
    target = out / dump.stem
    target.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(suffix=f".{codec}") as raw:
        raw.write(stream)
        raw.flush()
        source = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", codec]
        source += ["-i", raw.name]
        subprocess.run([*source, "-c", "copy", str(target / "video.mp4")], check=True)
        frames = ["-vf", f"fps={fps}", "-q:v", JPEG_QUALITY, str(target / "frame_%05d.jpg")]
        subprocess.run([*source, *frames], check=True)
    return target


def main() -> None:
    parser = argparse.ArgumentParser(prog="hoverctl.export", description=__doc__)
    parser.add_argument("dumps", type=Path, nargs="+", help="hover_video_*.udp files")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="default %(default)s")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS, help="frames per second kept")
    args = parser.parse_args()
    if not shutil.which("ffmpeg"):
        sys.exit("ffmpeg not found: install it (brew / apt / winget install ffmpeg)")
    for dump in args.dumps:
        target = export(dump, args.out, args.fps)
        if target is None:
            print(f"{dump}: no video in it, skipped")
            continue
        count = len(list(target.glob("frame_*.jpg")))
        print(f"{dump} -> {target}/video.mp4 + {count} frames")


if __name__ == "__main__":
    main()
