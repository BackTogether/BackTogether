#!/usr/bin/env python3
"""When the robot reaches a navigation goal, turn on the spot until the expected person is
recognised by the head camera, then stop facing them.

Arrival = move_base reports its goal SUCCEEDED (/move_base/result). While searching, the robot
turns slowly; once the face is seen it turns towards it until the face is in the middle of the
image, stops and publishes the name on /face_check/found. A new goal, or not finding the person
within ~timeout, stops the search. The robot is always left stopped.

Params (set in bringup.launch):
  ~person           folder name of the person's photos in $FACE_DIRECTORY (default jorge)
  ~turn_speed       rad/s while searching (default 0.3, slow so the recogniser keeps up)
  ~timeout          seconds before giving up (default: two full turns)
  ~search_on_start  true: search right away, without waiting for a goal (to test it alone)
"""

import math
import threading
import time

import rospy
from actionlib_msgs.msg import GoalStatus
from face_rec.msg import face_results
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from std_msgs.msg import String

DEFAULT_PERSON = "jorge"
DEFAULT_TURN_SPEED = 0.3  # rad/s
FULL_TURNS = 2
RATE_HZ = 10
IMAGE_WIDTH_PX = 640  # head camera (ar_pose usbcam_head.launch)
CENTRED_PX = 60  # face this close to the image centre = the robot is facing the person
CENTRE_GAIN = 0.002  # rad/s per pixel off-centre while turning towards the face
FACE_FRESH_S = 0.7  # a sighting older than this means the face left the image


class FaceCheck:
    def __init__(self):
        self.person = str(rospy.get_param("~person", DEFAULT_PERSON)).lower()
        self.turn_speed = float(rospy.get_param("~turn_speed", DEFAULT_TURN_SPEED))
        default_timeout = FULL_TURNS * 2 * math.pi / self.turn_speed
        self.timeout = float(rospy.get_param("~timeout", default_timeout))
        self.lock = threading.Lock()
        self.seen_at = 0.0
        self.face_x = 0.0
        self.face_area = 0.0  # px^2 of the face box: grows as the person gets closer
        self.search_requested = threading.Event()
        self.new_goal = threading.Event()
        if rospy.get_param("~search_on_start", False):
            self.search_requested.set()

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.found = rospy.Publisher("/face_check/found", String, queue_size=1, latch=True)
        rospy.Subscriber("/face_results", face_results, self.on_faces, queue_size=1)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.on_result, queue_size=1)
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.on_goal, queue_size=1)
        rospy.loginfo("face_check: looking for '%s' after each arrival", self.person)

    def on_faces(self, results):
        for face in results.face_data:
            if face.name.lower() == self.person:
                with self.lock:
                    self.seen_at = time.monotonic()
                    self.face_x = (face.xmin + face.xmax) / 2.0
                    self.face_area = (face.xmax - face.xmin) * (face.ymax - face.ymin)
                return

    def on_result(self, result):
        if result.status.status == GoalStatus.SUCCEEDED:
            rospy.loginfo("face_check: arrived, searching for '%s'", self.person)
            self.new_goal.clear()
            self.search_requested.set()

    def on_goal(self, _goal):
        self.new_goal.set()  # move_base drives again: never turn against it

    def face(self):
        """(offset from the image centre in px, + = right; box area in px^2), None if unseen."""
        with self.lock:
            if time.monotonic() - self.seen_at > FACE_FRESH_S:
                return None
            return self.face_x - IMAGE_WIDTH_PX / 2.0, self.face_area

    def offset_px(self):
        """How far the person's face is from the image centre (px, + = right), None if unseen."""
        face = self.face()
        return None if face is None else face[0]

    def turn(self, angular_z):
        twist = Twist()
        twist.angular.z = angular_z
        self.cmd_vel.publish(twist)

    def search(self):
        """Turn until facing the person. True if found, False on timeout or a new goal."""
        rate = rospy.Rate(RATE_HZ)
        deadline = time.monotonic() + self.timeout
        try:
            while time.monotonic() < deadline and not rospy.is_shutdown():
                if self.new_goal.is_set():
                    rospy.loginfo("face_check: new goal, search cancelled")
                    return False
                offset = self.offset_px()
                if offset is None:
                    self.turn(self.turn_speed)  # keep looking
                elif abs(offset) <= CENTRED_PX:
                    return True
                else:
                    # image x grows to the right, +angular.z turns left: turn towards the face
                    speed = min(self.turn_speed, CENTRE_GAIN * abs(offset))
                    self.turn(-speed if offset > 0 else speed)
                rate.sleep()
            return False
        finally:
            self.turn(0.0)

    def spin(self):
        while not rospy.is_shutdown():
            if not self.search_requested.wait(0.5):
                continue
            self.search_requested.clear()
            if self.search():
                rospy.loginfo("face_check: found '%s', facing them", self.person)
                self.found.publish(String(data=self.person))
            elif not self.new_goal.is_set():
                rospy.logwarn("face_check: '%s' not found in %.0f s", self.person, self.timeout)


if __name__ == "__main__":
    rospy.init_node("face_check")
    FaceCheck().spin()
