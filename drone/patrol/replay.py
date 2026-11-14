"""Replay a recorded flight through the wall perception, to check it without flying.

Reads a video dump (recordings/hover_video_*.udp, arrival time per datagram) and, optionally,
the Wi-Fi push capture with the 1 Hz odometry (recordings/push19099_*.bin: records of
<double t><uint32 len><datagram>, FlightEngineeringInfo in field 7). For every --every seconds
of video it prints the floor-boundary reading (floor_boundary.py) next to the odometry
position, so a distance can be checked against how far the drone then flew.

  HF_HUB_OFFLINE=1 uv run --offline --no-project --with numpy --with pillow --with torch \\
      --with transformers --with av python -m drone.patrol.replay \\
      recordings/hover_video_20261114-215432.udp --push recordings/push19099_20261114-215135.bin \\
      --start 22:07:15 --end 22:07:40 --pitch 30
"""

import argparse
import struct
import time
from collections.abc import Iterator
from datetime import datetime
from pathlib import Path

import numpy as np
from PIL import Image

from drone.hoverctl import pbwire
from drone.hoverctl.video import DUMP_RECORD
from drone.hoverctl.video_payload import StreamExtractor
from drone.patrol.floor_boundary import GEOMETRY_HFOV_DEG, measure_wall

PUSH_RECORD = struct.Struct("<dI")
PUSH_NAME_FIELD, PUSH_PAYLOAD_FIELD = 1, 7
ENGINEERING_PUSH = b"StartReportFlightEngineeringInfo"
ENGINEERING_EVENT = 16  # flight_engineering_info_event inside the payload
CODEC = "h264"


def odometry(path: Path) -> list[tuple[float, tuple[float, float, float]]]:
    """(unix time, position xyz) of every FlightEngineeringInfo push."""
    data, offset, out = path.read_bytes(), 0, []
    while offset + PUSH_RECORD.size <= len(data):
        t, length = PUSH_RECORD.unpack_from(data, offset)
        offset += PUSH_RECORD.size
        fields = pbwire.parse(data[offset : offset + length])
        offset += length
        if fields.get(PUSH_NAME_FIELD, [b""])[0] != ENGINEERING_PUSH:
            continue
        event = pbwire.parse(fields[PUSH_PAYLOAD_FIELD][0])
        out.append((t, pbwire.decode_engineering(event[ENGINEERING_EVENT][0]).position))
    return out


def timed_datagrams(dump: Path) -> Iterator[tuple[float, bytes]]:
    """(arrival time, datagram) of a dump written by hoverctl's VideoReceiver."""
    data, offset = dump.read_bytes(), 0
    while offset + DUMP_RECORD.size <= len(data):
        t, length = DUMP_RECORD.unpack_from(data, offset)
        offset += DUMP_RECORD.size
        yield t, data[offset : offset + length]
        offset += length


def frames(
    dump: Path, start: float, end: float, every: float
) -> Iterator[tuple[float, Image.Image]]:
    """(arrival time, frame) every `every` seconds between start and end (unix)."""
    import av

    extractor, codec = StreamExtractor(), av.CodecContext.create(CODEC, "r")
    due = start
    for t, datagram in timed_datagrams(dump):
        if t > end:
            return
        chunk = extractor.feed(datagram)
        for packet in codec.parse(chunk) if chunk else ():
            try:
                decoded = codec.decode(packet)
            except av.error.InvalidDataError:
                continue  # before the first keyframe
            for frame in decoded:
                if t >= due:
                    due = t + every
                    yield t, frame.to_image()


def position_at(
    track: list[tuple[float, tuple[float, float, float]]], t: float
) -> tuple[float, float, float] | None:
    """Odometry position at `t`, interpolated (z down)."""
    if not track:
        return None
    times = [sample[0] for sample in track]
    x, y, z = (float(np.interp(t, times, [s[1][i] for s in track])) for i in range(3))
    return x, y, z


def _unix(clock: str, day: datetime) -> float:
    hours, minutes, seconds = (int(part) for part in clock.split(":"))
    return day.replace(hour=hours, minute=minutes, second=seconds).timestamp()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("dump", type=Path)
    parser.add_argument("--push", type=Path, help="Wi-Fi push capture with the odometry")
    parser.add_argument("--start", help="HH:MM:SS (local time of the recording)")
    parser.add_argument("--end", help="HH:MM:SS")
    parser.add_argument("--every", type=float, default=0.5, help="seconds between frames")
    parser.add_argument("--pitch", type=float, default=30.0, help="camera deg below level")
    parser.add_argument("--height", type=float, help="camera m above the floor (default: odometry)")
    parser.add_argument("--takeoff-height", type=float, default=0.0, help="m, with odometry")
    parser.add_argument("--hfov", type=float, default=GEOMETRY_HFOV_DEG)
    parser.add_argument("--model", choices=("small", "base"), default="small")
    args = parser.parse_args()

    from drone.patrol.front_distance import DepthModel

    track = odometry(args.push) if args.push else []
    first, _ = next(timed_datagrams(args.dump))
    day = datetime.fromtimestamp(first)
    start = _unix(args.start, day) if args.start else first
    end = _unix(args.end, day) if args.end else float("inf")
    model = DepthModel(model=args.model)
    print(f"depth model {args.model} on {model.device}", flush=True)
    for t, image in frames(args.dump, start, end, args.every):
        position = position_at(track, t)
        height = args.height
        if height is None and position is not None:
            height = -position[2] + args.takeoff_height
        if height is None:
            raise SystemExit("give --height or --push")
        where = "" if position is None else "odometry x={:5.2f} y={:5.2f}".format(*position[:2])
        tick = time.monotonic()
        reading = measure_wall(model.predict(image), height, args.pitch, args.hfov)
        ms = (time.monotonic() - tick) * 1000
        clock = datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-4]
        print(f"{clock} {reading.summary()} h={height:.2f} {where} {ms:.0f} ms", flush=True)


if __name__ == "__main__":
    main()
