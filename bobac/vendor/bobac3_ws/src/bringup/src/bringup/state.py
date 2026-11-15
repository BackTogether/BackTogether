"""The state of the machine, the half that every node needs.

system_state.py (the node) owns /system_state and is its only publisher. Every other node
uses StateClient here: it reads the state to know whether it is its turn, and asks for the
next one when its task is done.

This is a module, not a script: catkin_install_python puts a wrapper in the devel space for
every script it installs, and a wrapper exports nothing, so scripts cannot import each other.
"""

import threading
import time

import rospy
from std_msgs.msg import String

STATE_TOPIC = "/system_state"
SET_TOPIC = "/system_state/set"
SET_WAIT_S = 1.0  # time given to system_state to subscribe before asking for a state


class StateClient:
    """A node's side of the machine: which state it is in, and asking for the next one.

    Used by the nodes that only work in one state (arrival_watch, face_check): they check
    `state` to know whether it is their turn and call `set` when their task is done.
    """

    def __init__(self, on_change=None):
        self.on_change = on_change
        self._state = None
        self._lock = threading.Lock()
        self._setter = rospy.Publisher(SET_TOPIC, String, queue_size=1)
        rospy.Subscriber(STATE_TOPIC, String, self._on_state, queue_size=1)

    @property
    def state(self):
        """The state the machine is in, None while system_state has not published one."""
        with self._lock:
            return self._state

    def _on_state(self, message):
        state = str(message.data).strip()
        with self._lock:
            if state == self._state:
                return
            self._state = state
        if self.on_change:
            self.on_change(state)

    def set(self, state):
        """Ask system_state for the next state.

        A no-op when no system_state node is running (the launches that do not start one):
        /system_state is latched, so a running machine has always told us its state by now.
        """
        if self.state is None:
            return
        deadline = time.monotonic() + SET_WAIT_S
        while self._setter.get_num_connections() == 0 and not rospy.is_shutdown():
            if time.monotonic() > deadline:
                rospy.logwarn("state: nothing listens on %s, asked for '%s'", SET_TOPIC, state)
                break
            time.sleep(0.05)
        self._setter.publish(String(data=state))
