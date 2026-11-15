#!/usr/bin/env python3
"""The one node that owns the state of the machine: it publishes it and it changes it.

Every other node reads /system_state instead of guessing where the demo is. The topic is
latched, so a node that starts late (or an rqt / rostopic opened mid demo) gets the current
state right away instead of waiting for the next change.

To change the state, publish the new one on /system_state/set. This node is the only
publisher of /system_state: nobody else writes it, so there is a single source of truth.

Params (set in a launch file):
  ~initial_state  the state published on startup (default home)
  ~states         comma separated list of the states this machine may be in; a request for
                  a state outside the list is refused and logged. Empty accepts any state
                  (default home,arrived,searching,care,done)

Topics:
  /system_state     (std_msgs/String, latched)  the current state, published on every change
  /system_state/set (std_msgs/String)           asks for a new state

  rostopic echo /system_state                                   # follow the demo
  rostopic pub -1 /system_state/set std_msgs/String "data: care" # change it by hand

The flow of state_demo.launch: home (waiting for a goal) -> arrived (move_base reached it,
arrival_watch) -> searching (face_check turning to recognise the child) -> care. This node
does not drive the flow: it only keeps and announces where it is. The other half is
StateClient in bringup/state.py, what a node uses to read the state and ask for the next one.

Runs on the robot: ROS Noetic, system Python 3.8, stdlib + rospy only.
"""

import rospy
from bringup.state import SET_TOPIC, STATE_TOPIC
from std_msgs.msg import String

DEFAULT_INITIAL_STATE = "home"
DEFAULT_STATES = "home,arrived,searching,care,done"


def parse_states(raw):
    """The ~states param as a list: comma separated, blanks dropped. [] means accept any."""
    return [state.strip() for state in str(raw).split(",") if state.strip()]


class SystemState:
    def __init__(self):
        self.states = parse_states(rospy.get_param("~states", DEFAULT_STATES))
        self.state = str(rospy.get_param("~initial_state", DEFAULT_INITIAL_STATE)).strip()
        if self.states and self.state not in self.states:
            raise ValueError(
                f"~initial_state '{self.state}' is not one of ~states "
                f"({', '.join(self.states)})"
            )
        self.publisher = rospy.Publisher(STATE_TOPIC, String, queue_size=1, latch=True)
        self.publisher.publish(String(data=self.state))
        rospy.Subscriber(SET_TOPIC, String, self.on_set, queue_size=10)
        rospy.loginfo(
            "system_state: '%s', waiting for %s (states: %s)",
            self.state,
            SET_TOPIC,
            ", ".join(self.states) if self.states else "any",
        )

    def on_set(self, message):
        requested = str(message.data).strip()
        if not requested:
            rospy.logwarn("system_state: ignoring an empty state request")
            return
        if self.states and requested not in self.states:
            rospy.logwarn(
                "system_state: refusing '%s', not one of ~states (%s); staying in '%s'",
                requested,
                ", ".join(self.states),
                self.state,
            )
            return
        if requested == self.state:
            rospy.logdebug("system_state: already '%s'", self.state)
            return
        rospy.loginfo("system_state: %s -> %s", self.state, requested)
        self.state = requested
        self.publisher.publish(String(data=self.state))


if __name__ == "__main__":
    rospy.init_node("system_state")
    SystemState()
    rospy.spin()
