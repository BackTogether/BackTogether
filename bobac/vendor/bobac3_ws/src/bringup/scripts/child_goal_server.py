#!/usr/bin/env python3
"""HTTP -> move_base. The drone POSTs the child's (x, y) in map frame; we drive there.

Runs on the robot: Ubuntu 20.04, ROS Noetic, system Python 3.8, stdlib + rospy only.
"""
import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler

PORT = 8000

lock = threading.Lock()
state = {
    "state": "idle",        # idle | navigating | arrived | failed
    "mission_id": None,
    "goal": None,           # {"x": float, "y": float}
    "pose": None,           # {"x": float, "y": float, "yaw": float}
    "message": "",
    "updated_at": 0.0,
}


def set_state(**kw):
    with lock:
        state.update(kw)
        state["updated_at"] = rospy.get_time()


def on_amcl(msg):
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y ** 2 + o.z ** 2))
    with lock:
        state["pose"] = {"x": p.x, "y": p.y, "yaw": yaw}


def on_done(status, _result):
    if status == GoalStatus.SUCCEEDED:
        set_state(state="arrived", message="reached the child")
        reached_pub.publish(Bool(data=True))     # bringup picks it up and starts the care flow
        rospy.loginfo("arrived at the child")
    else:
        set_state(state="failed", message="move_base status %d" % status)
        rospy.logwarn("move_base failed, status %d", status)


def send_goal(x, y):
    """Drive to (x, y) in map frame, arriving facing the child."""
    with lock:
        pose = state["pose"]
    yaw = math.atan2(y - pose["y"], x - pose["x"]) if pose else 0.0

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
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

            mission_id = body.get("mission_id")
            with lock:
                if mission_id is not None and mission_id == state["mission_id"]:
                    return self.reply(200, {"ok": True, "duplicate": True,
                                            "state": state["state"]})
                state["mission_id"] = mission_id

            set_state(state="navigating", goal={"x": x, "y": y},
                      message="driving to the child")
            send_goal(x, y)
            return self.reply(202, {"accepted": True, "goal": {"x": x, "y": y}})

        if self.path == "/cancel":
            move_base.cancel_all_goals()
            set_state(state="idle", goal=None, mission_id=None, message="cancelled")
            return self.reply(200, {"ok": True})

        self.reply(404, {"error": "unknown path"})

    def log_message(self, *args):
        pass   # keep the roslaunch console readable


if __name__ == "__main__":
    rospy.init_node("child_goal_server")
    reached_pub = rospy.Publisher("/child_goal/reached", Bool, queue_size=1, latch=True)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, on_amcl)

    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("waiting for move_base...")
    move_base.wait_for_server()

    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    set_state(state="idle", message="ready")
    rospy.loginfo("child_goal_server listening on 0.0.0.0:%d", PORT)

    rospy.spin()
    server.shutdown()
