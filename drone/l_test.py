"""L test without face recognition: where the drone thinks three tape marks are, in the robot's
frame (x ahead, y left), the frame the mission reports the child in.

Tape: START at the robot's centre, A `leg` metres along its front, B `leg` metres to the left
of A. The drone takes off from the robot facing its front; hover over each mark (camera straight
down, set here) with the mark centred in the video and press Enter. START -> A is the robot's
forward, as in the mission's aligning leg (fly it without turning), so A tests the distance and B
the distance, the angle and left / right. Then `drone/robot <leg> <leg>` drives the robot to B.

    drone/ltest 2.22     # with drone/fly running and the drone flying
"""

import argparse
import math

from drone.sdk.client import DEFAULT_URL, DroneApiError, DroneClient
from drone.sdk.mission_geometry import Alignment

CAMERA_DOWN_DEG = -90.0


def ask_pose(client: DroneClient, name: str, where: str) -> tuple[float, float]:
    input(f"\nHover over {name} ({where}), mark centred in the video, then Enter: ")
    pose = client.status()["telemetry"]["pose"]
    print(
        f"  drone frame x {pose['x']:+.2f} y {pose['y']:+.2f}, alt {pose['alt']:.2f} m, "
        f"position sigma {pose.get('position_sigma_m')} m"
    )
    return pose["x"], pose["y"]


def main() -> None:
    parser = argparse.ArgumentParser(description="L test of the drone's position estimate")
    parser.add_argument("leg", type=float, help="length of both legs of the L on the floor, m")
    parser.add_argument("--url", default=DEFAULT_URL, help="drone/fly")
    args = parser.parse_args()
    client = DroneClient(args.url)
    try:
        client.tilt(CAMERA_DOWN_DEG)
    except DroneApiError as error:
        print(f"camera NOT turned down ({error}): MANUAL CONTROL in the page, or tilt it there")
    marks = (
        ("START", "the robot's centre", 0.0, 0.0),
        ("A", f"{args.leg} m ahead", args.leg, 0.0),
        ("B", f"{args.leg} m ahead, {args.leg} m left", args.leg, args.leg),
    )
    points = [ask_pose(client, name, where) for name, where, _, _ in marks]
    (x0, y0), (x1, y1), (x2, y2) = points
    forward = Alignment(math.degrees(math.atan2(y1 - y0, x1 - x0)), math.hypot(x1 - x0, y1 - y0))
    print("\nIn the robot's frame:")
    for (name, _, tape_x, tape_y), (x, y) in zip(marks, points, strict=True):
        ahead, left = forward.launch_from_drone(x - x0, y - y0)
        error = math.hypot(ahead - tape_x, left - tape_y)
        print(
            f"  {name:5s} ahead {ahead:+.2f} left {left:+.2f}   "
            f"(tape {tape_x:+.2f} {tape_y:+.2f})   off by {error:.2f} m"
        )
    ahead, left = forward.launch_from_drone(x2 - x1, y2 - y1)
    print(f"  turn A -> B {math.degrees(math.atan2(left, ahead)):+.0f} deg (tape +90 = left)")


if __name__ == "__main__":
    main()
