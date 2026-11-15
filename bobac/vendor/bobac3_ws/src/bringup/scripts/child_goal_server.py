#!/usr/bin/env python3
"""HTTP -> move_base. The drone POSTs where the child is; the robot drives to it.

POST /child_found {"x", "y", "frame"?, "mission_id"?}:
  frame "map" (default): x, y in the map frame.
  frame "robot": x metres ahead and y metres to the left of the robot where it stood at the
    drone's takeoff (drone/report_child.py sends this; the robot has not moved since, so its
    pose now is that pose).
The goal stops ~standoff metres short of the child on the line from the robot, facing the
child: move_base cannot drive into the child, whom the lidar sees as an obstacle. The robot's
pose comes from tf (map -> base_footprint): the demo runs gmapping, there is no /amcl_pose.
Arrival publishes /child_goal/reached (face_check then looks for the child's face).

Params: ~port (8001, drone/report_child.py's default), ~standoff (0.8 m), ~robot_frame
(base_footprint). Runs on the robot: Ubuntu 20.04, ROS Noetic, system Python 3.8.
"""
import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler

DEFAULT_PORT = 8001  # drone/report_child.py sends to http://<robot>:8001
DEFAULT_STANDOFF_M = 0.8  # the drone mission's standoff_m
MAP_FRAME = "map"
FRAMES = ("map", "robot")
TF_TIMEOUT_S = 0.5

lock = threading.Lock()
state = {
    "state": "idle",        # idle | navigating | arrived | failed
    "mission_id": None,
    "child": None,          # {"x", "y"}: where the drone saw the child, map frame
    "goal": None,           # {"x", "y", "yaw"}: standoff short of it, facing it
    "pose": None,           # {"x": float, "y": float, "yaw": float}
    "message": "",
    "updated_at": 0.0,
}


def set_state(**kw):
    with lock:
        state.update(kw)
        state["updated_at"] = rospy.get_time()


def robot_pose():
    """The robot in the map frame (tf map -> robot_frame), or None while tf has no pose."""
    try:
        listener.waitForTransform(MAP_FRAME, robot_frame, rospy.Time(0),
                                  rospy.Duration(TF_TIMEOUT_S))
        (x, y, _z), rotation = listener.lookupTransform(MAP_FRAME, robot_frame, rospy.Time(0))
    except (tf.Exception, tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException):
        return None
    pose = {"x": x, "y": y, "yaw": euler_from_quaternion(rotation)[2]}
    with lock:
        state["pose"] = pose
    return pose


def to_map(x, y, frame, pose):
    """The child in the map frame; frame "robot" = ahead / left of the robot now."""
    if frame == MAP_FRAME:
        return x, y
    cos, sin = math.cos(pose["yaw"]), math.sin(pose["yaw"])
    return pose["x"] + x * cos - y * sin, pose["y"] + x * sin + y * cos


def short_of(child_x, child_y, pose):
    """(x, y, yaw): standoff metres short of the child on the line from the robot, facing it;
    the robot's own spot (turned to the child) when it is already that close."""
    dx, dy = child_x - pose["x"], child_y - pose["y"]
    distance = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    if distance <= standoff:
        return pose["x"], pose["y"], yaw
    keep = (distance - standoff) / distance
    return pose["x"] + dx * keep, pose["y"] + dy * keep, yaw


def on_done(status, _result):
    if status == GoalStatus.SUCCEEDED:
        set_state(state="arrived", message="reached the child")
        reached_pub.publish(Bool(data=True))     # bringup picks it up and starts the care flow
        rospy.loginfo("arrived at the child")
    else:
        set_state(state="failed", message="move_base status %d" % status)
        rospy.logwarn("move_base failed, status %d", status)


def send_goal(x, y, yaw):
    """Drive to (x, y, yaw) in the map frame."""
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = MAP_FRAME
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    q = quaternion_from_euler(0.0, 0.0, yaw)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    move_base.send_goal(goal, done_cb=on_done)
    rospy.loginfo("goal -> map (%.2f, %.2f) yaw %.2f", x, y, yaw)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def reply(self, code, obj):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def read_json(self):
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_GET(self):
        if self.path.startswith("/status"):
            robot_pose()
            with lock:
                return self.reply(200, dict(state))
        self.reply(404, {"error": "unknown path"})

    def do_POST(self):
        try:
            body = self.read_json()
        except ValueError:
            return self.reply(400, {"error": "body is not valid json"})

        if self.path == "/child_found":
            try:
                x = float(body["x"])
                y = float(body["y"])
            except (KeyError, TypeError, ValueError):
                return self.reply(400, {"error": "need numeric x and y"})
            if not (math.isfinite(x) and math.isfinite(y)):
                return self.reply(400, {"error": "x or y is not finite"})
            frame = body.get("frame", MAP_FRAME)
            if frame not in FRAMES:
                return self.reply(400, {"error": "frame must be map or robot"})
            pose = robot_pose()
            if pose is None:
                return self.reply(503, {"error": "robot pose unknown (no tf map -> robot)"})

            mission_id = body.get("mission_id")
            with lock:
                if mission_id is not None and mission_id == state["mission_id"]:
                    return self.reply(200, {"ok": True, "duplicate": True,
                                            "state": state["state"]})
                state["mission_id"] = mission_id

            child_x, child_y = to_map(x, y, frame, pose)
            goal_x, goal_y, yaw = short_of(child_x, child_y, pose)
            child = {"x": round(child_x, 3), "y": round(child_y, 3)}
            goal = {"x": round(goal_x, 3), "y": round(goal_y, 3), "yaw": round(yaw, 3)}
            set_state(state="navigating", child=child, goal=goal, message="driving to the child")
            send_goal(goal_x, goal_y, yaw)
            return self.reply(202, {"accepted": True, "child": child, "goal": goal})

        if self.path == "/cancel":
            move_base.cancel_all_goals()
            set_state(state="idle", goal=None, mission_id=None, message="cancelled")
            return self.reply(200, {"ok": True})

        self.reply(404, {"error": "unknown path"})

    def log_message(self, *args):
        pass   # keep the roslaunch console readable


if __name__ == "__main__":
    rospy.init_node("child_goal_server")
    port = int(rospy.get_param("~port", DEFAULT_PORT))
    standoff = float(rospy.get_param("~standoff", DEFAULT_STANDOFF_M))
    robot_frame = str(rospy.get_param("~robot_frame", "base_footprint"))
    listener = tf.TransformListener()
    reached_pub = rospy.Publisher("/child_goal/reached", Bool, queue_size=1, latch=True)

    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("waiting for move_base...")
    move_base.wait_for_server()

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    set_state(state="idle", message="ready")
    rospy.loginfo("child_goal_server listening on 0.0.0.0:%d", port)

    rospy.spin()
    server.shutdown()
