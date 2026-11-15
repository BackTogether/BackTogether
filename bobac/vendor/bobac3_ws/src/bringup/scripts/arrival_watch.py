#!/usr/bin/env python3
"""home -> arrived: watches the goal sent by hand and says when the robot got there.

In home the robot waits for a goal (rviz, or /move_base_simple/goal). When move_base reports
that goal reached, the machine goes to arrived, which is what starts face_check. Only the
first drive of the run counts: once out of home this node does nothing.

Topics:
  /move_base/status (actionlib_msgs/GoalStatusArray)  watched for a reached goal
  /system_state/set (std_msgs/String)                 arrived, once

Runs on the robot: ROS Noetic, system Python 3.8, stdlib + rospy only.
"""

import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from bringup.state import StateClient

START_STATE = "home"
NEXT_STATE = "arrived"


class ArrivalWatch:
    def __init__(self):
        self.machine = StateClient()
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.on_status, queue_size=1)
        rospy.loginfo("arrival_watch: in %s, waiting for move_base to reach a goal", START_STATE)

    def on_status(self, status_array):
        if self.machine.state != START_STATE:
            return
        if not any(goal.status == GoalStatus.SUCCEEDED for goal in status_array.status_list):
            return
        rospy.loginfo("arrival_watch: goal reached, asking for %s", NEXT_STATE)
        self.machine.set(NEXT_STATE)


if __name__ == "__main__":
    rospy.init_node("arrival_watch")
    ArrivalWatch()
    rospy.spin()
