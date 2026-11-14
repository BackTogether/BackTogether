"""Serve the web UI + HTTP API for the real drone or the simulator.

uv run --with bleak python -m drone.sdk serve --http-port 8077   # real drone (what drone/fly runs)
uv run python -m drone.sdk sim --http-port 8090                  # simulator, nothing leaves the Mac
"""

import argparse
import contextlib
import signal
import webbrowser

from drone.hoverctl.__main__ import add_web_arguments, make_web_parts

from .drone import Drone
from .estimate import DEFAULT_YAW_RATE_DPS, DeadReckoner
from .safety import SafetyLimits
from .server import ApiServer
from .sim import DEFAULT_FRAME_YAW_OFFSET_DEG, DEFAULT_ROOM_HALF_M, connect_sim

SIM_PORT = 8090
SIM_BIND = "127.0.0.1"
SIM_MAX = 60.0


def _raise_interrupt(signum: int, frame: object) -> None:
    raise KeyboardInterrupt  # SIGTERM: same clean exit as Ctrl-C (neutral sticks)


def _add_sdk_arguments(parser: argparse.ArgumentParser) -> None:
    defaults = SafetyLimits()
    parser.add_argument(
        "--yaw-rate",
        type=float,
        default=DEFAULT_YAW_RATE_DPS,
        help="deg/s at full yaw stick, from calibrate_yaw (default %(default)s)",
    )
    parser.add_argument(
        "--fence",
        type=float,
        default=defaults.x_max,
        help="geofence half-size in metres around the takeoff point (default %(default)s)",
    )
    parser.add_argument("--max-alt", type=float, default=defaults.max_alt_m)
    parser.add_argument(
        "--no-video-required",
        action="store_true",
        help="the autopilot keeps flying when the video stops",
    )


def _limits(args: argparse.Namespace) -> SafetyLimits:
    fence = abs(args.fence)
    return SafetyLimits(
        x_min=-fence,
        x_max=fence,
        y_min=-fence,
        y_max=fence,
        max_alt_m=args.max_alt,
        require_video=not args.no_video_required,
    )


def _serve(drone: Drone, args: argparse.Namespace, ble_note: str = "") -> None:
    server = ApiServer(args.bind, args.http_port, drone, _limits(args), ble_note)
    url = f"http://{args.bind}:{args.http_port}/"
    print(f"web UI + API on {url}  (API: drone/sdk/API.md)", flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        drone.close()
        print("drone closed, sticks ended neutral", flush=True)


def cmd_serve(args: argparse.Namespace) -> None:
    link, ble_note, sender, receiver = make_web_parts(args)
    drone = Drone(link, sender, receiver, DeadReckoner(args.yaw_rate))
    drone.start()
    _serve(drone, args, ble_note)


def cmd_sim(args: argparse.Namespace) -> None:
    drone = connect_sim(
        args.max, DeadReckoner(args.yaw_rate), args.frame_yaw_offset, args.seed, args.room
    )
    _serve(drone, args)


def main() -> None:
    parser = argparse.ArgumentParser(prog="drone.sdk", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    serve = commands.add_parser("serve", help="real drone: hoverctl web UI + HTTP API")
    add_web_arguments(serve)
    _add_sdk_arguments(serve)
    serve.set_defaults(run=cmd_serve)

    sim = commands.add_parser("sim", help="simulated drone: same web UI + HTTP API")
    sim.add_argument("--http-port", type=int, default=SIM_PORT)
    sim.add_argument("--bind", default=SIM_BIND, help="HTTP address (default %(default)s)")
    sim.add_argument("--max", type=float, default=SIM_MAX, help="speed cap, stick %%")
    sim.add_argument(
        "--frame-yaw-offset",
        type=float,
        default=DEFAULT_FRAME_YAW_OFFSET_DEG,
        help="angle between the takeoff frame and the drone's heading (unknown to the SDK)",
    )
    sim.add_argument("--seed", type=int, help="noise seed, for repeatable runs")
    sim.add_argument(
        "--room",
        type=float,
        default=DEFAULT_ROOM_HALF_M,
        help="walls at +-this many metres from the start (default %(default)s; e.g. 4 for "
        "drone.patrol.bounce --fake-depth)",
    )
    sim.add_argument("--no-browser", action="store_true", help="don't open a browser tab")
    _add_sdk_arguments(sim)
    sim.set_defaults(run=cmd_sim)

    args = parser.parse_args()
    signal.signal(signal.SIGTERM, _raise_interrupt)
    with contextlib.suppress(KeyboardInterrupt):
        args.run(args)


if __name__ == "__main__":
    main()
