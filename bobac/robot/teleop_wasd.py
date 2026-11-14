"""Drive the bobac3 base with the keyboard (WASD), as a basic control check.

Runs ON THE ROBOT with its system Python 3.8 + rospy (not uv). Needs the base node running
(`roslaunch rei_robot_base base.launch`, it provides /cmd_vel).

    source /opt/ros/noetic/setup.bash
    python3 teleop_wasd.py

Keys: w/s forward/back, a/d slide left/right (omni base), q/e turn left/right,
space stop, x quit. The robot only moves while a key keeps arriving: if no key comes for
KEY_TIMEOUT_S it stops by itself, so a laggy SSH link or a closed terminal never leaves it
driving. Keep someone next to the e-stop anyway.
"""

import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

LINEAR_SPEED_MPS = 0.15  # well below the 0.4 m/s the manual recommends as maximum
ANGULAR_SPEED_RADPS = 0.4
KEY_TIMEOUT_S = 0.6  # longer than the keyboard auto-repeat delay, so holding a key keeps moving
PUBLISH_RATE_HZ = 10
STOP_REPEATS = 5

# key -> (linear x, linear y, angular z) as multiples of the speeds above
KEY_BINDINGS = {
    "w": (1, 0, 0),
    "s": (-1, 0, 0),
    "a": (0, 1, 0),
    "d": (0, -1, 0),
    "q": (0, 0, 1),
    "e": (0, 0, -1),
    " ": (0, 0, 0),
}
QUIT_KEY = "x"
HELP = "w/s forward/back, a/d slide, q/e turn, space stop, x quit (releasing keys stops)"


def make_twist(direction):
    twist = Twist()
    twist.linear.x = direction[0] * LINEAR_SPEED_MPS
    twist.linear.y = direction[1] * LINEAR_SPEED_MPS
    twist.angular.z = direction[2] * ANGULAR_SPEED_RADPS
    return twist


def read_key(timeout_s):
    """Return one pressed key, or None if nothing arrives within timeout_s."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
    return sys.stdin.read(1).lower() if ready else None


def stop(publisher):
    for _ in range(STOP_REPEATS):
        publisher.publish(Twist())
        rospy.sleep(1.0 / PUBLISH_RATE_HZ)


def drive(publisher):
    direction = (0, 0, 0)
    last_key_time = rospy.get_time()
    while not rospy.is_shutdown():
        key = read_key(1.0 / PUBLISH_RATE_HZ)
        if key == QUIT_KEY:
            return
        if key in KEY_BINDINGS:
            direction = KEY_BINDINGS[key]
            last_key_time = rospy.get_time()
        if rospy.get_time() - last_key_time > KEY_TIMEOUT_S:
            direction = (0, 0, 0)
        publisher.publish(make_twist(direction))


def main():
    rospy.init_node("teleop_wasd", disable_signals=True)
    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    print(HELP)
    terminal_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        drive(publisher)
    except KeyboardInterrupt:  # Ctrl+C: same as x, stop cleanly
        pass
    finally:
        stop(publisher)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        print("Stopped.")


if __name__ == "__main__":
    main()
