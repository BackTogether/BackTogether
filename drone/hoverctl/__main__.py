"""hoverctl CLI: `python -m drone.hoverctl {sticks,video,web,status} --help`."""

import argparse
import contextlib
import signal
import sys
import time
import webbrowser
from collections.abc import Callable
from pathlib import Path
from typing import TYPE_CHECKING

from .decoder import MjpegDecoder
from .protocol import CONTROL_SOURCES, DEFAULT_CONTROL_SOURCE
from .sticks import DEFAULT_MAX_MAGNITUDE, DRONE_HOST, STICK_PORT, StickSender
from .teleop import run_teleop
from .video import VIDEO_PORT, VideoReceiver, default_dump_path
from .web import WebServer

if TYPE_CHECKING:
    from .ble import DroneLink

STATS_PERIOD_S = 1.0
WEB_PORT = 8000
WEB_BIND = "127.0.0.1"
NO_BLE_NOTE = "bluetooth off (--no-ble)"
NO_BLEAK_NOTE = "bluetooth unavailable: run with uv run --with bleak"


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
    parser.add_argument(
        "--control-source",
        choices=sorted(CONTROL_SOURCES),
        default=DEFAULT_CONTROL_SOURCE,
        help="control_source_type in each packet: cross-key (6, the app's button pad, the default) "
        "or app (1, the app's analog joystick); try app if the drone ignores the sticks",
    )
    parser.add_argument(
        "--stick-log", type=Path, metavar="PATH", help="append every packet sent as JSONL"
    )


def _add_video_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--video-port", type=int, default=VIDEO_PORT, help="UDP port to listen on")
    parser.add_argument("--dump", type=Path, help="raw datagram dump (default recordings/...)")
    parser.add_argument("--no-dump", action="store_true", help="don't write a dump")
    parser.add_argument("--skip", type=int, default=0, help="strip N header bytes per datagram")
    parser.add_argument("--codec", choices=["auto", "h264", "hevc"], default="auto")
    parser.add_argument("--no-decode", action="store_true", help="don't start ffmpeg")


def _make_sender(args: argparse.Namespace, gate: Callable[[], bool] | None = None) -> StickSender:
    return StickSender(
        args.host,
        args.port,
        args.local_port,
        args.max,
        args.dry_run,
        gate,
        CONTROL_SOURCES[args.control_source],
        args.stick_log,
    )


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


def _always_locked() -> bool:
    return False


def _wifi_options(args: argparse.Namespace) -> dict[str, str | None]:
    """DroneLink keyword arguments: its own default password unless one is given."""
    if args.no_wifi_autostart:
        return {"wifi_password": None}
    return {"wifi_password": args.wifi_password} if args.wifi_password else {}


def _add_wifi_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--wifi-password",
        metavar="PW",
        help="drone Wi-Fi password set on every Bluetooth connect (default: DEFAULT_WIFI_PASSWORD "
        "in ble.py)",
    )
    parser.add_argument(
        "--no-wifi-autostart", action="store_true", help="don't start the drone's Wi-Fi on connect"
    )


def _start_link(args: argparse.Namespace) -> tuple["DroneLink | None", str]:
    """The Bluetooth link and a note for the UI when there is none. Imports ble lazily."""
    if args.no_ble:
        return None, NO_BLE_NOTE
    try:
        from .ble import DroneLink
    except ImportError:  # bleak is not installed
        return None, NO_BLEAK_NOTE
    link = DroneLink(**_wifi_options(args))
    link.start()
    return link, ""


def cmd_web(args: argparse.Namespace) -> None:
    link, ble_note = _start_link(args)
    if link:
        gate = link.is_manual
    elif args.no_ble:
        gate = None
        print(
            "WARNING: --no-ble, the stick interlock is OFF: sticks are sent whatever the "
            "drone's flight mode is.",
            file=sys.stderr,
        )
    else:
        gate = _always_locked  # fail closed: no Bluetooth, no sticks
        print(
            f"WARNING: {NO_BLEAK_NOTE}. Sticks stay locked (use --no-ble to send anyway).",
            file=sys.stderr,
        )
    receiver = _make_receiver(args)
    sender = _make_sender(args, gate)
    server = WebServer(args.bind, args.http_port, sender, receiver, link, ble_note)
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
        if link:
            link.stop()
        print(f"sent {sender.sent} stick packets, ended neutral")


def _format_status(snap: dict) -> str:
    conn, bat, odo = snap["connection"], snap["battery"], snap["odometry"]
    if not conn["connected"]:
        return f"connecting...  {conn['error'] or ''}".rstrip()
    pos, vel = odo["position"], odo["velocity"]
    parts = [
        f"mode {snap['mode']['name']}",
        f"state {snap['flight_state']['value']}",
        f"bat {bat['soc']}% v{bat['voltage']} i{bat['current']} t{bat['temp']}"
        f" chg {'yes' if bat['charging'] else 'no'}",
    ]
    if pos:
        parts.append("pos " + " ".join(f"{pos[a]:+.2f}" for a in "xyz"))
        parts.append("vel " + " ".join(f"{vel[a]:+.2f}" for a in "xyz"))
        parts.append("rpm " + " ".join(str(r) for r in odo["motors_rpm"]))
    parts.append(f"rssi {conn['rssi']}  age {conn['last_message_age_s']}s")
    return "  ".join(parts)


def cmd_status(args: argparse.Namespace) -> None:
    from .ble import DroneLink  # needs bleak: only load it for this command

    link = DroneLink(args.name, **_wifi_options(args))
    link.start()
    commands = [
        ("land", args.land, link.land),
        ("manual", args.manual, link.set_manual),
        ("wifi", args.wifi is not None, lambda: link.start_wifi(args.wifi)),
    ]
    pending = [(name, send) for name, wanted, send in commands if wanted]
    print("connecting to the drone, Ctrl-C to quit")
    try:
        while True:
            if pending and link.snapshot()["connection"]["connected"]:
                for name, send in pending:
                    print(f"{name}: {send()}", flush=True)
                pending = []
            print(_format_status(link.snapshot()), flush=True)
            time.sleep(STATS_PERIOD_S)
    finally:
        link.stop()


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
    web.add_argument(
        "--no-ble",
        action="store_true",
        help="no Bluetooth link: sticks are sent without the MANUAL-mode interlock",
    )
    _add_wifi_options(web)
    web.add_argument("--no-browser", action="store_true", help="don't open a browser tab")
    web.set_defaults(run=cmd_web)

    status = commands.add_parser("status", help="live drone state over Bluetooth (needs bleak)")
    status.add_argument("--name", default="HoverX1", help="drone name prefix (default %(default)s)")
    status.add_argument("--land", action="store_true", help="send land once connected")
    status.add_argument("--manual", action="store_true", help="switch to MANUAL once connected")
    status.add_argument("--wifi", metavar="PASSWORD", help="start the drone's Wi-Fi once connected")
    _add_wifi_options(status)
    status.set_defaults(run=cmd_status)

    args = parser.parse_args()
    signal.signal(signal.SIGTERM, _raise_interrupt)
    with contextlib.suppress(KeyboardInterrupt):
        args.run(args)


if __name__ == "__main__":
    main()
