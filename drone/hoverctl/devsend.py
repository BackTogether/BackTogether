"""Dev tool: send test video to the receiver, without the drone.

python -m drone.hoverctl.devsend               # test pattern, H.264 Annex-B, 2 KB datagrams
python -m drone.hoverctl.devsend --mode rtp    # same pattern as RTP/H.264
python -m drone.hoverctl.devsend --codec hevc  # H.265 Annex-B
python -m drone.hoverctl.devsend --replay recordings/hover_video_<time>.udp  # a real dump
"""

import argparse
import socket
import subprocess
import time
from pathlib import Path

from .video import DUMP_RECORD, VIDEO_PORT

DATAGRAM_BYTES = 2048  # what the app's receive buffer holds
RTP_PACKET_BYTES = 1400
TEST_SOURCE = "testsrc2=size=1280x720:rate=30"
ENCODERS = {"h264": "libx264", "hevc": "libx265"}


def _encoder_args(codec: str, seconds: float) -> list[str]:
    return [
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-re",
        "-f", "lavfi", "-i", TEST_SOURCE, "-t", str(seconds),
        "-c:v", ENCODERS[codec], "-preset", "ultrafast", "-tune", "zerolatency",
        "-g", "30", "-pix_fmt", "yuv420p",
    ]  # fmt: skip


def send_annexb(target: tuple[str, int], codec: str, seconds: float) -> int:
    command = [*_encoder_args(codec, seconds), "-f", codec, "pipe:1"]
    sent = 0
    with (
        subprocess.Popen(command, stdout=subprocess.PIPE) as encoder,
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock,
    ):
        while chunk := encoder.stdout.read1(DATAGRAM_BYTES):
            sock.sendto(chunk, target)
            sent += 1
    return sent


def send_rtp(target: tuple[str, int], seconds: float) -> None:
    url = f"rtp://{target[0]}:{target[1]}?pkt_size={RTP_PACKET_BYTES}"
    subprocess.run([*_encoder_args("h264", seconds), "-f", "rtp", url], check=True)


def replay(target: tuple[str, int], dump: Path) -> int:
    """Send a `video` dump again with its original timing."""
    data = dump.read_bytes()
    offset = sent = 0
    first_arrival = start = None
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        while offset + DUMP_RECORD.size <= len(data):
            arrival, length = DUMP_RECORD.unpack_from(data, offset)
            offset += DUMP_RECORD.size
            if first_arrival is None:
                first_arrival, start = arrival, time.monotonic()
            delay = (arrival - first_arrival) - (time.monotonic() - start)
            if delay > 0:
                time.sleep(delay)
            sock.sendto(data[offset : offset + length], target)
            offset += length
            sent += 1
    return sent


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=VIDEO_PORT)
    parser.add_argument("--mode", choices=["annexb", "rtp"], default="annexb")
    parser.add_argument("--codec", choices=sorted(ENCODERS), default="h264", help="annexb only")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--replay", type=Path, help="dump file written by `hoverctl video`")
    args = parser.parse_args()
    target = (args.host, args.port)
    if args.replay:
        print(f"replayed {replay(target, args.replay)} datagrams")
    elif args.mode == "rtp":
        send_rtp(target, args.seconds)
    else:
        print(f"sent {send_annexb(target, args.codec, args.seconds)} datagrams")


if __name__ == "__main__":
    main()
