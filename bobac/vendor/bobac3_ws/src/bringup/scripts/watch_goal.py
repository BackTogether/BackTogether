#!/usr/bin/env python3
"""Watch what the robot does with the child the drone sends, to check the transformations.

Runs on the robot next to state_demo.launch (drone/robot_watch starts it over ssh). At each new
move_base goal it prints, in the frame of the robot where it stood when the goal came (X metres
ahead, Y metres to its left, the frame of drone/report_child.py and `drone/robot X Y`):
  the child (child_goal_server's GET /status): compare with what the drone printed;
  the goal (the standoff point short of the child, facing it);
then, twice a second while it moves, where the robot is in that frame, how far from the goal,
and move_base's status. Everything also goes to ~/watch_goal.log.

Ubuntu 20.04, ROS Noetic, system Python 3.8 (no f-string `=`, no walrus).
"""
import json
import math
import os
import time
import urllib.request

import rospy
import tf
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from tf.transformations import euler_from_quaternion

STATUS_URL = "http://localhost:8001/status"  # child_goal_server
MAP_FRAME = "map"
ROBOT_FRAME = "base_footprint"
PERIOD_S = 0.5
QUIET_S = 3.0  # a progress line at least this often, even standing still
MOVED_M = 0.05  # ...else only when it moved this much
LOG_PATH = os.path.expanduser("~/watch_goal.log")
STATUS_NAMES = {0: "pending", 1: "active", 2: "preempted", 3: "succeeded", 4: "aborted",
                5: "rejected", 6: "preempting", 7: "recalling", 8: "recalled", 9: "lost"}

log_file = open(LOG_PATH, "a")
listener = None
watch = {"start": None, "goal": None, "status": "-"}


def say(line):
    line = "%s %s" % (time.strftime("%H:%M:%S"), line)
    print(line, flush=True)
    log_file.write(line + "\n")
    log_file.flush()


def robot_pose():
    """(x, y, yaw) of the robot in the map, or None while tf has none."""
    try:
        (x, y, _z), rotation = listener.lookupTransform(MAP_FRAME, ROBOT_FRAME, rospy.Time(0))
    except (tf.Exception, tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException):
        return None
    return x, y, euler_from_quaternion(rotation)[2]


def in_start(x, y):
    """A map point as (X ahead, Y left) of the robot where it stood when the goal came."""
    sx, sy, syaw = watch["start"]
    dx, dy = x - sx, y - sy
    cos, sin = math.cos(syaw), math.sin(syaw)
    return cos * dx + sin * dy, -sin * dx + cos * dy


def child_in_map():
    try:
        with urllib.request.urlopen(STATUS_URL, timeout=1.0) as reply:
            status = json.load(reply)
    except (OSError, ValueError) as error:
        say("child_goal_server not answering: %s" % error)
        return None, None
    return status.get("child"), status.get("mission_id")


def on_goal(msg):
    target = msg.goal.target_pose.pose
    gyaw = euler_from_quaternion([target.orientation.x, target.orientation.y,
                                  target.orientation.z, target.orientation.w])[2]
    watch["start"] = robot_pose()
    watch["goal"] = (target.position.x, target.position.y)
    if watch["start"] is None:
        say("NEW GOAL map (%.2f, %.2f), but no robot pose from tf" % watch["goal"])
        return
    sx, sy, syaw = watch["start"]
    say("NEW GOAL. robot at map (%.2f, %.2f) yaw %.0f deg: this is X=0 Y=0, X ahead"
        % (sx, sy, math.degrees(syaw)))
    child, mission = child_in_map()
    if child is not None:
        cx, cy = in_start(child["x"], child["y"])
        say("  child  X %+.2f  Y %+.2f  (map %.2f, %.2f; mission %s): compare with the drone's"
            " 'drone/robot X Y'" % (cx, cy, child["x"], child["y"], mission))
    gx, gy = in_start(*watch["goal"])
    say("  goal   X %+.2f  Y %+.2f  heading %.0f deg (map %.2f, %.2f)"
        % (gx, gy, math.degrees(gyaw - syaw), watch["goal"][0], watch["goal"][1]))


def on_status(msg):
    if msg.status_list:
        watch["status"] = STATUS_NAMES.get(msg.status_list[-1].status, "?")


def main():
    global listener
    rospy.init_node("watch_goal", anonymous=True)
    listener = tf.TransformListener()
    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, on_goal)
    rospy.Subscriber("/move_base/status", GoalStatusArray, on_status)
    say("watching move_base goals (log: %s); waiting for the drone's child" % LOG_PATH)
    last_said, last_xy, last_status = 0.0, None, None
    rate = rospy.Rate(1.0 / PERIOD_S)
    while not rospy.is_shutdown():
        rate.sleep()
        pose = robot_pose()
        if watch["start"] is None or pose is None:
            continue
        x, y = in_start(pose[0], pose[1])
        gx, gy = in_start(*watch["goal"])
        moved = last_xy is None or math.hypot(x - last_xy[0], y - last_xy[1]) >= MOVED_M
        if not (moved or watch["status"] != last_status or time.time() - last_said >= QUIET_S):
            continue
        heading = math.degrees(pose[2] - watch["start"][2])
        say("  robot  X %+.2f  Y %+.2f  heading %+4.0f deg | %.2f m to the goal | move_base %s"
            % (x, y, (heading + 180) % 360 - 180, math.hypot(gx - x, gy - y), watch["status"]))
        last_said, last_xy, last_status = time.time(), (x, y), watch["status"]


if __name__ == "__main__":
    main()
