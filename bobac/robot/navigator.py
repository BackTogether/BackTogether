"""Navigation (P1, contract in interfaces/ROBOT_API.md): give it a goal and the robot gets there.

Runs ON THE ROBOT with its system Python 3.8 + rospy. Needs `slam_nav.launch` running (it
brings up move_base, which plans around obstacles and drives /cmd_vel).

Try it from a terminal on the robot, with someone at the e-stop:

    python3 navigator.py --x 1.0 --y 0.0 --yaw 0.0     # metres and radians, frame `map`

As a module (the bridge imports it) it never calls rospy.init_node and creates its ROS clients
on first use.
"""

import argparse
import math

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

MAP_FRAME = "map"
SERVER_WAIT_S = 10.0
GOAL_TIMEOUT_S = 120.0
ATTEMPTS = 2  # one retry after clearing the costmaps
# The vendor launch names the node move_base_node, so its services live under that name.
CLEAR_COSTMAPS_SERVICE = "/move_base_node/clear_costmaps"
CLEAR_COSTMAPS_WAIT_S = 2.0
# Why move_base ended a goal without reaching it, in words a person can act on.
FAILURES = {
    GoalStatus.ABORTED: "no path to the goal, or the robot got stuck",
    GoalStatus.REJECTED: "goal rejected (outside the map or inside an obstacle?)",
    GoalStatus.PREEMPTED: "stopped",
    GoalStatus.RECALLED: "stopped",
}

_client = None


def _result(success: bool, message: str) -> dict:
    return {"success": success, "message": message}


def _move_base():
    """The move_base action client, created on first use; None if move_base is not running."""
    global _client
    if _client is None:
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        if not client.wait_for_server(rospy.Duration(SERVER_WAIT_S)):
            return None
        _client = client
    return _client


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def go_to_pose(x: float, y: float, yaw: float = 0.0, timeout_s: float = GOAL_TIMEOUT_S) -> dict:
    """Drive to (x, y) facing yaw, in frame `map`. Blocks until arrival, failure or timeout."""
    client = _move_base()
    if client is None:
        return _result(False, "move_base is not running (start slam_nav.launch)")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = MAP_FRAME
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation = _yaw_to_quaternion(yaw)
    deadline = rospy.Time.now() + rospy.Duration(timeout_s)
    for attempt in range(1, ATTEMPTS + 1):
        goal.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(goal)
        if not client.wait_for_result(deadline - rospy.Time.now()):
            client.cancel_goal()
            return _result(False, "timeout")
        state = client.get_state()
        if state == GoalStatus.SUCCEEDED:
            return _result(True, "arrived")
        if state != GoalStatus.ABORTED or attempt == ATTEMPTS:
            break
        # Aborted is often a stale obstacle left in the costmap: wipe it and plan again.
        rospy.logwarn("navigator: goal aborted, clearing costmaps and retrying")
        _clear_costmaps()
    return _result(False, FAILURES.get(state, f"move_base ended with state {state}"))


def _clear_costmaps() -> None:
    try:
        rospy.wait_for_service(CLEAR_COSTMAPS_SERVICE, timeout=CLEAR_COSTMAPS_WAIT_S)
        rospy.ServiceProxy(CLEAR_COSTMAPS_SERVICE, Empty)()
    except (rospy.ROSException, rospy.ServiceException) as error:
        rospy.logwarn("navigator: could not clear costmaps: %s", error)


def main() -> None:
    parser = argparse.ArgumentParser(description="send the robot to a point of the map")
    parser.add_argument("--x", type=float, required=True, help="metres, frame map")
    parser.add_argument("--y", type=float, required=True, help="metres, frame map")
    parser.add_argument("--yaw", type=float, default=0.0, help="radians, 0 = map x axis")
    parser.add_argument("--timeout", type=float, default=GOAL_TIMEOUT_S, help="seconds")
    args = parser.parse_args()
    rospy.init_node("navigator_cli", anonymous=True)
    print(go_to_pose(args.x, args.y, args.yaw, args.timeout))


if __name__ == "__main__":
    main()
