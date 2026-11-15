#!/usr/bin/env python3
"""`care`: the search is over, the robot is with the child. Start the episode.

face_check recognised the child and put the machine in `care` (bringup/scripts/face_check.py).
This node waits for that state and then does what the old care_trigger.py did: hide the full
screen image, free the head camera the video call needs, and POST /care/start to the care
server, which greets the child, lets the voice agent talk with them, opens the video call on
the robot's screen and sends the parents the link.

Params (set in kid_care.launch):
  ~start_state   the state that starts the episode (default care). Empty: start as soon as the
                 care server answers, for running this package without a state machine
  ~care_url      the care server (default http://localhost:8090)
  ~delay         seconds between the trigger and the episode, so the screen and the camera are
                 free before the call window opens
  ~server_wait   seconds to wait for the care server before giving up
  ~hide_splash   hide bringup's full screen image, which would cover the call (default true)
  ~free_camera   stop the nodes holding the head camera, the call needs it (default true)
  ~camera_topic  the head camera topic (default /head_camera/image_raw)

Runs on the robot: ROS Noetic, system Python 3.8, stdlib + rospy only.
"""

import json
import threading
import time
import urllib.error
import urllib.request

import rosgraph
import rosnode
import rospy
from bringup.state import StateClient
from std_msgs.msg import Bool

DEFAULT_START_STATE = "care"
DEFAULT_CARE_URL = "http://localhost:8090"
DEFAULT_DELAY_S = 2.0
DEFAULT_SERVER_WAIT_S = 60.0
DEFAULT_CAMERA_TOPIC = "/head_camera/image_raw"
SPLASH_TOPIC = "/splash/show"
STATUS_PATH = "/status"
START_PATH = "/care/start"
HTTP_TIMEOUT_S = 5.0
POLL_S = 0.5


class KidCareStart:
    def __init__(self):
        self.start_state = str(rospy.get_param("~start_state", DEFAULT_START_STATE)).strip()
        self.care_url = str(rospy.get_param("~care_url", DEFAULT_CARE_URL)).rstrip("/")
        self.delay = float(rospy.get_param("~delay", DEFAULT_DELAY_S))
        self.server_wait = float(rospy.get_param("~server_wait", DEFAULT_SERVER_WAIT_S))
        self.hide_splash = bool(rospy.get_param("~hide_splash", True))
        self.free_camera = bool(rospy.get_param("~free_camera", True))
        self.camera_topic = str(rospy.get_param("~camera_topic", DEFAULT_CAMERA_TOPIC))
        self.triggered = threading.Event()
        self.splash = rospy.Publisher(SPLASH_TOPIC, Bool, queue_size=1, latch=True)
        # the machine's own client: it tells us when face_check has found the child
        self.machine = StateClient(self.on_state) if self.start_state else None

    def on_state(self, state):
        if state == self.start_state:
            self.triggered.set()

    def wait_for_trigger(self):
        """Block until the machine is in ~start_state. True unless the launch was stopped."""
        if not self.start_state:
            rospy.loginfo("kid_care_start: no state to wait for, starting the episode now")
            return True
        rospy.loginfo("kid_care_start: waiting for the machine to be in '%s'", self.start_state)
        while not rospy.is_shutdown():
            if self.triggered.wait(POLL_S):
                return True
        return False

    def clear_the_screen(self):
        """The image bringup shows is always on top: it would hide the video call."""
        if self.hide_splash:
            self.splash.publish(Bool(data=False))

    def stop_camera_nodes(self):
        """The call's browser cannot open the head camera while a ROS node holds it."""
        if not self.free_camera:
            return
        publishers = rosgraph.Master(rospy.get_name()).getSystemState()[0]
        nodes = [n for topic, names in publishers if topic == self.camera_topic for n in names]
        if not nodes:
            return
        rospy.loginfo("kid_care_start: freeing the head camera, stopping %s", ", ".join(nodes))
        rosnode.kill_nodes(nodes)

    def server_is_up(self):
        try:
            with urllib.request.urlopen(self.care_url + STATUS_PATH, timeout=HTTP_TIMEOUT_S) as a:
                return a.status == 200
        except (urllib.error.URLError, OSError):
            return False

    def wait_for_server(self):
        deadline = time.time() + self.server_wait
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.server_is_up():
                return True
            rospy.sleep(POLL_S)
        return False

    def start_care(self):
        body = json.dumps({"found_time": time.time()}).encode("utf-8")
        request = urllib.request.Request(
            self.care_url + START_PATH, data=body, headers={"Content-Type": "application/json"}
        )
        try:
            with urllib.request.urlopen(request, timeout=HTTP_TIMEOUT_S) as answer:
                rospy.loginfo("kid_care_start: care server answered %d", answer.status)
        except urllib.error.HTTPError as error:
            rospy.logwarn("kid_care_start: care server said %d (409 = already busy)", error.code)
        except (urllib.error.URLError, OSError) as error:
            rospy.logerr("kid_care_start: no care server at %s (%s)", self.care_url, error)

    def run(self):
        if not self.wait_for_trigger():
            return
        rospy.loginfo("kid_care_start: with the child, starting the care flow")
        self.clear_the_screen()
        self.stop_camera_nodes()
        if not self.wait_for_server():
            if not rospy.is_shutdown():
                rospy.logerr("kid_care_start: no care server after %.0f s", self.server_wait)
            return
        rospy.sleep(self.delay)  # let the screen and the camera be free before the call opens
        if not rospy.is_shutdown():
            self.start_care()


if __name__ == "__main__":
    try:
        rospy.init_node("kid_care_start")
        KidCareStart().run()
        rospy.spin()  # stay up so the launch does not report this node as finished mid demo
    except rospy.ROSInterruptException:
        pass  # the launch was stopped while this node was waiting
