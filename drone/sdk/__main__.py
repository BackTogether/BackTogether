"""Serve the web UI + HTTP API for the real drone or the simulator.

uv run --with bleak python -m drone.sdk serve --http-port 8077   # real drone (what drone/fly runs)
uv run python -m drone.sdk sim --http-port 8090                  # simulator, nothing leaves the Mac
"""

import argparse
import contextlib
import signal
import webbrowser
from pathlib import Path

from drone.hoverctl.__main__ import add_web_arguments, make_web_parts

from .drone import Drone, make_video_yaw_or_note
from .estimate import DEFAULT_YAW_RATE_DPS, DeadReckoner
from .map_image import load_map_yaml
from .map_view import MapView
from .safety import SafetyLimits
from .server import ApiServer
from .sim import DEFAULT_FRAME_YAW_OFFSET_DEG, DEFAULT_ROOM_HALF_M, connect_sim
from .splat import NO_NUMPY_NOTE, UP_AXES, Registration, numpy_missing, render_splat

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
    _add_map_arguments(parser)


def _add_map_arguments(parser: argparse.ArgumentParser) -> None:
    """The page's map view (API.md "Map view"): optional background layers."""
    group = parser.add_argument_group("map view")
    group.add_argument("--map-yaml", type=Path, help="the robot's map_server map (.yaml + .pgm)")
    group.add_argument("--splat", type=Path, help="Gaussian splat of the hall (3DGS .ply)")
    group.add_argument(
        "--splat-up",
        choices=tuple(UP_AXES),
        default="z",
        help="the splat's up axis (%(default)s; write --splat-up=-y for a minus)",
    )
    group.add_argument(
        "--splat-z-max",
        type=float,
        help="drop Gaussians above this height, splat units (default: halfway to the ceiling)",
    )
    group.add_argument("--splat-scale", type=float, help="map metres per splat unit")
    group.add_argument("--splat-rotation", type=float, help="splat -> map rotation, degrees")
    group.add_argument(
        "--splat-offset", type=float, nargs=2, metavar=("X", "Y"), help="splat origin in map"
    )


def _limits(args: argparse.Namespace) -> SafetyLimits:
    fence = abs(args.fence)
    return SafetyLimits(
        x_min=-fence,
        x_max=fence,
        y_min=-fence,
        y_max=fence,
        max_alt_m=args.max_alt,
        max_speed=args.max,  # the autopilot may use the whole cap given to drone/fly
        require_video=not args.no_video_required,
    )


def _map_view(args: argparse.Namespace) -> MapView:
    """The map layers given on the command line; a layer that fails to load is left off."""
    grid = None
    if args.map_yaml:
        try:
            grid = load_map_yaml(args.map_yaml)
            print(f"map: {grid.width} x {grid.height} cells at {grid.resolution} m", flush=True)
        except (OSError, ValueError, KeyError) as error:
            print(f"map: cannot load {args.map_yaml}: {error}", flush=True)
    if not args.splat:
        return MapView(grid)
    registration = Registration.load(Registration.path_for(args.splat))
    if args.splat_scale is not None:
        registration.scale = args.splat_scale
    if args.splat_rotation is not None:
        registration.rotation_deg = args.splat_rotation
    if args.splat_offset is not None:
        registration.x, registration.y = args.splat_offset
    splat, note = None, NO_NUMPY_NOTE if numpy_missing() else None
    if note is None:
        try:
            splat = render_splat(args.splat, args.splat_up, args.splat_z_max)
            print(f"splat: {splat.gaussians} Gaussians drawn, cut at {splat.z_max:.2f}", flush=True)
        except (OSError, ValueError, KeyError) as error:
            note = f"cannot read {args.splat.name}: {error}"
    if note:
        print(f"splat: {note}", flush=True)
    return MapView(grid, splat, note, args.splat, registration)


def _serve(drone: Drone, args: argparse.Namespace, ble_note: str = "") -> None:
    map_view = _map_view(args)
    server = ApiServer(args.bind, args.http_port, drone, _limits(args), ble_note, map_view)
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
    estimator = DeadReckoner(args.yaw_rate)
    drone = Drone(link, sender, receiver, estimator, video_yaw=make_video_yaw_or_note())
    drone.start()
    _serve(drone, args, ble_note)


def cmd_sim(args: argparse.Namespace) -> None:
    drone = connect_sim(
        args.max,
        DeadReckoner(args.yaw_rate),
        args.frame_yaw_offset,
        args.seed,
        args.room,
        video_dir=str(args.video) if args.video else None,
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
    sim.add_argument(
        "--video",
        type=Path,
        help="recorded drone frames (a folder of frame_*.jpg, looped) as the camera, e.g. to run "
        'drone.identify.feed against the simulator; send /sim/child {"detect": false} too',
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
