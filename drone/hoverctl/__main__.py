"""hoverctl CLI: `python -m drone.hoverctl {sticks,video,web} --help`."""

import argparse
import contextlib
import signal
import time
import webbrowser
from pathlib import Path

from .decoder import MjpegDecoder
from .sticks import DEFAULT_MAX_MAGNITUDE, DRONE_HOST, STICK_PORT, StickSender
from .teleop import run_teleop
from .video import VIDEO_PORT, VideoReceiver, default_dump_path
from .web import WebServer

STATS_PERIOD_S = 1.0
WEB_PORT = 8000
WEB_BIND = "127.0.0.1"


def _raise_interrupt(signum: int, frame: object) -> None:
    raise KeyboardInterrupt  # SIGTERM takes the same path as Ctrl-C, so sticks go neutral


def _add_stick_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--host", default=DRONE_HOST, help="drone IP (default %(default)s)")
    parser.add_argument("--port", type=int, default=STICK_PORT, help="drone stick port")
    parser.add_argument(
        "--local-port", type=int, default=STICK_PORT, help="our source port (the app uses 19091)"
    )
    parser.add_argument(
        "--max",
        type=float,
        default=DEFAULT_MAX_MAGNITUDE,
        help="largest stick value sent, 0-100 (default %(default)s)",
    )
    parser.add_argument("--dry-run", action="store_true", help="print packets, send nothing")


def _add_video_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--video-port", type=int, default=VIDEO_PORT, help="UDP port to listen on")
    parser.add_argument("--dump", type=Path, help="raw datagram dump (default recordings/...)")
    parser.add_argument("--no-dump", action="store_true", help="don't write a dump")
    parser.add_argument("--skip", type=int, default=0, help="strip N header bytes per datagram")
    parser.add_argument("--codec", choices=["auto", "h264", "hevc"], default="auto")
    parser.add_argument("--no-decode", action="store_true", help="don't start ffmpeg")


def _make_sender(args: argparse.Namespace) -> StickSender:
    return StickSender(args.host, args.port, args.local_port, args.max, args.dry_run)


def _make_receiver(args: argparse.Namespace) -> VideoReceiver:
    decoder = None if args.no_decode else MjpegDecoder(None if args.codec == "auto" else args.codec)
    dump = None if args.no_dump else args.dump or default_dump_path()
    return VideoReceiver(args.video_port, dump, args.skip, decoder)


def cmd_sticks(args: argparse.Namespace) -> None:
    sender = _make_sender(args)
    sender.start()
    try:
        run_teleop(sender)
    finally:
        sender.stop()
        print(f"sent {sender.sent} packets ({sender.errors} errors), ended neutral")


def cmd_video(args: argparse.Namespace) -> None:
    receiver = _make_receiver(args)
    receiver.start()
    print(f"listening on UDP {args.video_port}, Ctrl-C to stop")
    try:
        while True:
            time.sleep(STATS_PERIOD_S)
            s = receiver.stats()
            print(
                f"packets {s['packets']}  {s['pps']} pkt/s  {s['bytes_per_s'] / 1024:.0f} KB/s  "
                f"kind {s['kind']}  codec {s['codec']}  frames {s['frames']}  from {s['senders']}",
                flush=True,
            )
    finally:
        receiver.stop()
        if receiver.dump_path:
            print(f"dump: {receiver.dump_path}")


def cmd_web(args: argparse.Namespace) -> None:
    receiver = _make_receiver(args)
    sender = _make_sender(args)
    server = WebServer(args.bind, args.http_port, sender, receiver)
    url = f"http://{args.bind}:{args.http_port}/"
    receiver.start()
    sender.start()
    print(f"web UI on {url}  (sticks -> {args.host}:{args.port}, video on UDP {args.video_port})")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    finally:
        sender.stop()
        server.server_close()
        receiver.stop()
        print(f"sent {sender.sent} stick packets, ended neutral")


def main() -> None:
    parser = argparse.ArgumentParser(prog="hoverctl", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    sticks = commands.add_parser("sticks", help="keyboard teleop in this terminal")
    _add_stick_options(sticks)
    sticks.set_defaults(run=cmd_sticks)

    video = commands.add_parser("video", help="receive, dump and decode UDP video")
    _add_video_options(video)
    video.set_defaults(run=cmd_video)

    web = commands.add_parser("web", help="browser UI: video, virtual sticks, STOP")
    _add_stick_options(web)
    _add_video_options(web)
    web.add_argument("--http-port", type=int, default=WEB_PORT)
    web.add_argument("--bind", default=WEB_BIND, help="HTTP address (default %(default)s)")
    web.add_argument("--no-browser", action="store_true", help="don't open a browser tab")
    web.set_defaults(run=cmd_web)

    args = parser.parse_args()
    signal.signal(signal.SIGTERM, _raise_interrupt)
    with contextlib.suppress(KeyboardInterrupt):
        args.run(args)


if __name__ == "__main__":
    main()
