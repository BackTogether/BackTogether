#!/usr/bin/env python3
"""When the robot reaches the child's goal, look for the expected person around it, then follow
them: face left/right of the image = turn towards it, face bigger = back up, face smaller =
move forward, so it stays centred and at the same size.

The drone's odometry is not exact, so its goal is only roughly where the child is: quite often
the child is not in the camera when the robot gets there, and turning on the spot until the
timeout finds nothing. The search is therefore three steps, with the camera watching for the
face through all of them:

  1. a full turn on the spot, at the point the robot arrived at;
  2. wider and wider circles around that point (~ring_step apart, up to ~search_radius = 0.6 m),
     driven sideways on the omni base so the robot always faces out of the circle: each lap
     looks at the same ground from a new place, which is what brings out a child hidden behind
     something, and never turns the camera back into the circle it has already seen;
  3. nothing out to the widest circle: back to the map origin (~home_x, ~home_y), where the
     robot started, still watching for the child on the way there.

The lidar (~scan_topic, /fuse_scan: both lidars, all around the robot) keeps the robot off
anything in the way: a blocked step out is turned away from, a blocked lap is reversed, and a
leg that stays blocked is given up instead of forced. Only the expected person counts: the
recogniser's name must match ~person, like before.

Arrival = the machine reached the arrived state (state_demo.launch, arrival_watch) or
child_goal_server reached the drone's goal (~arrival_topic, /child_goal/reached). The search
puts the machine in searching, and finding the child in care, with the name on
/face_check/found. A new move_base goal stops the search. The robot is always left stopped.

Params (set in bringup.launch):
  ~person           folder name of the person's photos in $FACE_DIRECTORY (default jorge)
  ~turn_speed       rad/s while turning (default 0.3, slow so the recogniser keeps up)
  ~timeout          seconds for the whole search, the way home apart (default: what the turn
                    on the spot and every circle need, with margin)
  ~search_radius    widest circle around the arrival point, m (default 0.6; under 0.1 the
                    robot never leaves the spot, which is what it did before)
  ~ring_step        how much wider each circle is, m (default 0.3)
  ~search_speed     m/s while stepping out and going round (default 0.12)
  ~clearance        stop this far from what the lidar sees in the way, m (default 0.4)
  ~return_home      false: stay where the search ended instead of driving home (default true)
  ~home_x, ~home_y  where home is in the map frame (default 0, 0: where the robot started)
  ~home_timeout     seconds given to the drive home (default 120)
  ~search_on_start  true: search right away, without waiting for a goal (to test it alone)
  ~arrival_topic    std_msgs/Bool that starts a search (default /child_goal/reached)
  ~scan_topic       laser scan used to keep clear of obstacles (default /fuse_scan)
  ~odom_frame       frame the search geometry is measured in (default odom: unlike map it does
                    not jump when gmapping corrects itself, and 0.6 m of driving needs that)
  ~robot_frame      the robot's own frame (default base_footprint)
  ~max_linear       m/s forward / back while following (default 0.15)
  ~target_area      face box to keep while following, px^2 (default 20736 = 144 x 144, the
                    measured box; 0 = its size when found)

States (only when a system_state node is running):
  arrived   starts the search
  searching published when the search starts; the machine stays there when the child is
            nowhere (send the goal again to retry)
  care      published once the child is found, before following them

Runs on the robot: Ubuntu 20.04, ROS Noetic, system Python 3.8.
"""


import math
import threading
import time

import rospy
import tf
from actionlib_msgs.msg import GoalID
from bringup.state import StateClient
from face_rec.msg import face_results
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionGoal
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion

START_STATE = "arrived"
SEARCHING_STATE = "searching"
CARE_STATE = "care"
DEFAULT_PERSON = "jorge"
DEFAULT_ARRIVAL_TOPIC = "/child_goal/reached"  # child_goal_server.py
DEFAULT_SCAN_TOPIC = "/fuse_scan"  # both lidars merged: the robot sees all around itself
MAP_FRAME = "map"
DEFAULT_ODOM_FRAME = "odom"
DEFAULT_ROBOT_FRAME = "base_footprint"
DEFAULT_TURN_SPEED = 0.3  # rad/s
RATE_HZ = 10
TF_WAIT_S = 2.0  # time given to tf for the first pose of a search

# What each leg of the search returns.
FOUND = "found"  # the child is recognised and in the middle of the image
DONE = "done"  # this leg is over and found nothing: the next one may start
STOP = "stop"  # the search itself must end: a new move_base goal, or shutdown

# Face boxes measured on the robot's head camera (640x480), person standing in front:
#   centre    x 260..404  y 232..376  -> face centre x = 332, box 144 x 144 px
#   far right x 452..596  y 248..392  -> centre x 524
#   far left  x   4..148  y 248..392  -> centre x 76
IMAGE_CENTRE_X_PX = 332.0  # where a face straight ahead really lands (not 640 / 2)
TARGET_AREA_PX2 = 144.0 * 144.0  # the measured box: keep the person at that distance
FACE_REACH_PX = 224.0  # farthest a face centre gets from it (524 - 332 = 192, 332 - 76 = 256)
CENTRED_PX = 60  # face this close to the image centre = the robot is facing the person
FACE_FRESH_S = 0.7  # a sighting older than this means the face left the image
FACE_LOST_S = 3.0  # following: stop following after this long without the face
CENTRE_TIMEOUT_S = 8.0  # turning towards a face seen mid search: give up after this
CENTRE_LOST_S = 1.5  # ...or as soon as the face has been gone this long, and keep searching
DEFAULT_MAX_LINEAR = 0.15  # m/s forward or back while following: slow, a person is close
AREA_GAIN = 0.3  # m/s per unit of relative area error
AREA_DEADBAND = 0.15  # within 15 % of the target size: hold the distance, do not jitter
AREA_SMOOTHING = 0.3  # weight of the newest box size in the running average

# The widening search: circles around the point the robot arrived at.
DEFAULT_SEARCH_RADIUS_M = 0.6  # the widest circle: the robot never goes further from the goal
DEFAULT_RING_STEP_M = 0.3  # 0.3 then 0.6 m: two laps, ~50 s, and clearly different viewpoints
DEFAULT_SEARCH_SPEED = 0.12  # m/s stepping out and going round: the pace of a careful look
DEFAULT_CLEARANCE_M = 0.4  # a lidar range shorter than this is something in the way
CLEAR_CONE_RAD = math.radians(70)  # width of the cone checked in the direction of travel
RADIUS_GAIN = 1.0  # m/s per metre off the circle: brings the robot back onto it
YAW_GAIN = 1.2  # rad/s per radian of heading error
AT_CENTRE_M = 0.05  # closer than this to the centre there is no "out of the circle" yet
AIM_RAD = math.radians(25)  # heading error above which the robot turns before it drives
LAP_RAD = 2 * math.pi - math.radians(10)  # a full turn, less what a controller never closes
BLOCKED_TURN_RAD = math.radians(90)  # way out blocked: try this much further round
MAX_BLOCKED_TURNS = 3  # ...this many times, then give up on stepping out
MAX_REVERSALS = 2  # a lap blocked both ways this often: give that circle up
MIN_ORBIT_RADIUS_M = 0.1  # a circle smaller than this is not worth a lap
TIME_MARGIN = 1.6  # a leg gets the time it needs times this, then it is over
DEFAULT_HOME_TIMEOUT_S = 120.0  # the drive home is move_base's, it may have to replan
HOME_REACHED_M = 0.3  # this close to home in the map: home
HOME_GOAL_M = 0.1  # a move_base goal this close to home is this node's own, not a new one
CANCEL_SETTLE_S = 0.4  # after cancelling the drive home, before driving the base by hand


def wrap(angle):
    """An angle folded into -pi..pi, so a difference of headings is the short way round."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def clamp(value, limit):
    """`value` kept within -limit..limit."""
    return max(-limit, min(limit, value))


class FaceCheck:
    def __init__(self):
        self.person = str(rospy.get_param("~person", DEFAULT_PERSON)).lower()
        self.turn_speed = float(rospy.get_param("~turn_speed", DEFAULT_TURN_SPEED))
        self.max_linear = float(rospy.get_param("~max_linear", DEFAULT_MAX_LINEAR))
        # face box size to keep while following, px^2 (0 = the size when the face was found)
        self.target_area = float(rospy.get_param("~target_area", TARGET_AREA_PX2))
        self.max_radius = float(rospy.get_param("~search_radius", DEFAULT_SEARCH_RADIUS_M))
        self.ring_step = float(rospy.get_param("~ring_step", DEFAULT_RING_STEP_M))
        self.search_speed = float(rospy.get_param("~search_speed", DEFAULT_SEARCH_SPEED))
        self.clearance = float(rospy.get_param("~clearance", DEFAULT_CLEARANCE_M))
        self.go_home = bool(rospy.get_param("~return_home", True))
        self.home = (float(rospy.get_param("~home_x", 0.0)),
                     float(rospy.get_param("~home_y", 0.0)))
        self.home_timeout = float(rospy.get_param("~home_timeout", DEFAULT_HOME_TIMEOUT_S))
        self.odom_frame = str(rospy.get_param("~odom_frame", DEFAULT_ODOM_FRAME))
        self.robot_frame = str(rospy.get_param("~robot_frame", DEFAULT_ROBOT_FRAME))
        self.timeout = float(rospy.get_param("~timeout", self.default_timeout()))

        self.lock = threading.Lock()
        self.seen_at = 0.0
        self.face_x = 0.0
        self.face_area = 0.0  # px^2 of the face box: grows as the person gets closer
        self.scan = None
        self.home_goal = False  # the goal this node sends to go home is not a new goal
        self.search_requested = threading.Event()
        self.new_goal = threading.Event()
        if rospy.get_param("~search_on_start", False):
            self.search_requested.set()

        self.tf = tf.TransformListener()
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.cancel = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.found = rospy.Publisher("/face_check/found", String, queue_size=1, latch=True)
        rospy.Subscriber("/face_results", face_results, self.on_faces, queue_size=1)
        arrival_topic = str(rospy.get_param("~arrival_topic", DEFAULT_ARRIVAL_TOPIC))
        rospy.Subscriber(arrival_topic, Bool, self.on_arrival, queue_size=1)
        scan_topic = str(rospy.get_param("~scan_topic", DEFAULT_SCAN_TOPIC))
        rospy.Subscriber(scan_topic, LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.on_goal, queue_size=1)
        self.machine = StateClient(self.on_state)
        rospy.loginfo("face_check: looking for '%s' after each arrival, out to %.2f m in steps"
                      " of %.2f m (%.0f s)", self.person, self.max_radius, self.ring_step,
                      self.timeout)

    def default_timeout(self):
        """Seconds the whole search needs: the turn on the spot, then stepping out onto each
        circle and going round it, with margin. The way home has its own ~home_timeout."""
        total = 2 * math.pi / self.turn_speed
        for radius in self.rings():
            total += self.ring_step / self.search_speed
            total += 2 * math.pi * radius / self.orbit_speed(radius)
        return total * TIME_MARGIN

    def rings(self):
        """The radii to search, smallest first: ~ring_step apart, the last one exactly
        ~search_radius, and none at all when ~search_radius says not to leave the spot."""
        if self.max_radius < MIN_ORBIT_RADIUS_M:
            return []
        step = max(self.ring_step, MIN_ORBIT_RADIUS_M)
        count = max(1, int(math.ceil(self.max_radius / step)))
        return [min(self.max_radius, (ring + 1) * step) for ring in range(count)]

    def orbit_speed(self, radius):
        """m/s round a circle of `radius`: slow enough that the robot, which turns as it goes
        to keep facing out, never turns faster than ~turn_speed (the recogniser needs that)."""
        return min(self.search_speed, self.turn_speed * radius)

    # ------------------------------------------------------------- what the robot can tell

    def on_faces(self, results):
        for face in results.face_data:
            if face.name.lower() == self.person:
                with self.lock:
                    self.seen_at = time.monotonic()
                    self.face_x = (face.xmin + face.xmax) / 2.0
                    self.face_area = (face.xmax - face.xmin) * (face.ymax - face.ymin)
                return

    def on_scan(self, scan):
        with self.lock:
            self.scan = scan

    def on_arrival(self, arrived):
        if arrived.data:
            self.start_search("at the child")

    def on_state(self, state):
        if state == START_STATE:
            self.start_search("arrived")

    def on_goal(self, message):
        """move_base drives again: never turn against it. The goal this node sends to go home
        is part of the search, not a new one."""
        position = message.goal.target_pose.pose.position
        if self.home_goal and math.hypot(position.x - self.home[0],
                                         position.y - self.home[1]) <= HOME_GOAL_M:
            return
        self.new_goal.set()

    def start_search(self, reason):
        if self.machine.state == CARE_STATE:
            return  # already with the child: never turn away from them
        rospy.loginfo("face_check: %s, searching for '%s'", reason, self.person)
        self.new_goal.clear()
        self.search_requested.set()

    def face(self):
        """(offset from the image centre in px, + = right; box area in px^2), None if unseen."""
        with self.lock:
            if time.monotonic() - self.seen_at > FACE_FRESH_S:
                return None
            return self.face_x - IMAGE_CENTRE_X_PX, self.face_area

    def offset_px(self):
        """How far the person's face is from the image centre (px, + = right), None if unseen."""
        face = self.face()
        return None if face is None else face[0]

    def pose(self, frame=None):
        """(x, y, yaw) of the robot in `frame`, by default the odom frame: the search measures
        itself there because, unlike the map, it does not jump when gmapping corrects itself.
        None while tf has no such transform."""
        try:
            (x, y, _z), rotation = self.tf.lookupTransform(frame or self.odom_frame,
                                                           self.robot_frame, rospy.Time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            return None
        return x, y, euler_from_quaternion(rotation)[2]

    def wait_for_pose(self):
        """The robot's pose at the start of a search, giving tf a moment to have one."""
        deadline = time.monotonic() + TF_WAIT_S
        while not rospy.is_shutdown():
            pose = self.pose()
            if pose is not None or time.monotonic() > deadline:
                return pose
            rospy.sleep(1.0 / RATE_HZ)
        return None

    def blocked(self, direction):
        """True when the lidar sees something within ~clearance in the cone the robot is about
        to drive into (`direction` in the robot frame: 0 ahead, +pi/2 to its left). False while
        no scan has arrived: the robot then drives as it did before there was a lidar check."""
        with self.lock:
            scan = self.scan
        if scan is None:
            return False
        angle = scan.angle_min
        for distance in scan.ranges:
            if (scan.range_min <= distance <= self.clearance
                    and abs(wrap(angle - direction)) <= CLEAR_CONE_RAD / 2.0):
                return True
            angle += scan.angle_increment
        return False

    # ----------------------------------------------------------------- driving the base

    def drive(self, forward=0.0, sideways=0.0, turn=0.0):
        """m/s ahead, m/s to the robot's left (the base is omni), rad/s counter clockwise."""
        twist = Twist()
        twist.linear.x = forward
        twist.linear.y = sideways
        twist.angular.z = turn
        self.cmd_vel.publish(twist)

    def stop(self):
        self.drive()

    def centring_speed(self, offset):
        """rad/s that turns the robot towards a face `offset` px from the centre (+ = right)."""
        if abs(offset) <= CENTRED_PX / 2.0:
            return 0.0  # close enough: do not jitter
        # proportional: full turn speed only when the face is at the edge of the image
        speed = min(self.turn_speed, self.turn_speed * abs(offset) / FACE_REACH_PX)
        return -speed if offset > 0 else speed  # image x to the right = turn right (negative z)

    def aiming_speed(self, error):
        """rad/s that turns the robot onto a heading `error` radians away (+ = to its left)."""
        return clamp(YAW_GAIN * error, self.turn_speed)

    # ------------------------------------------------------------------------ the search

    def interrupted(self):
        """True when the search must stop right away: a new goal for move_base, or shutdown."""
        if rospy.is_shutdown():
            return True
        if self.new_goal.is_set():
            rospy.loginfo("face_check: new goal, search cancelled")
            return True
        return False

    def leg_end(self, seconds, deadline):
        """When a leg of the search is over: the time it needs plus margin, never past the
        search's own `deadline`."""
        return min(deadline, time.monotonic() + seconds * TIME_MARGIN)

    def look(self):
        """Run at every step of the search: FOUND once the child is recognised and in the middle
        of the image, None to carry on looking. Leaves the robot stopped whenever it saw the
        face, even if it then lost it again."""
        if self.face() is None:
            return None
        self.stop()
        rospy.loginfo("face_check: saw '%s', turning to face them", self.person)
        if self.centre_on_face():
            return FOUND
        rospy.logwarn("face_check: lost '%s' again, back to the search", self.person)
        return None

    def centre_on_face(self):
        """Turn on the spot until the child's face is in the middle of the image. True once it
        is, False if the face stays gone for CENTRE_LOST_S (the search then carries on)."""
        rate = rospy.Rate(RATE_HZ)
        end = time.monotonic() + CENTRE_TIMEOUT_S
        last_seen = time.monotonic()
        try:
            while not self.interrupted() and time.monotonic() < end:
                offset = self.offset_px()
                if offset is None:
                    self.stop()  # never turn on an old sighting
                    if time.monotonic() - last_seen > CENTRE_LOST_S:
                        return False
                elif abs(offset) <= CENTRED_PX:
                    return True
                else:
                    last_seen = time.monotonic()
                    self.drive(turn=self.centring_speed(offset))
                rate.sleep()
            return False
        finally:
            self.stop()

    def sweep(self, deadline):
        """A full turn on the spot where the robot stands. FOUND / DONE / STOP. Measured on tf
        when there is a pose, on the clock when there is not."""
        rospy.loginfo("face_check: turning on the spot")
        end = self.leg_end(2 * math.pi / self.turn_speed, deadline)
        rate = rospy.Rate(RATE_HZ)
        last = self.pose()
        turned = 0.0
        try:
            while turned < LAP_RAD:
                if self.interrupted():
                    return STOP
                if time.monotonic() > end:
                    return DONE  # the clock closes the turn when there is no pose to measure
                pose = self.pose()
                if pose is not None:
                    if last is not None:
                        turned += abs(wrap(pose[2] - last[2]))
                    last = pose
                self.drive(turn=self.turn_speed)
                if self.look() == FOUND:
                    return FOUND
                rate.sleep()
            rospy.loginfo("face_check: nothing on the spot, widening the search")
            return DONE
        finally:
            self.stop()

    def step_out(self, centre, radius, deadline):
        """Drive straight out from `centre` until `radius` away from it, facing out. FOUND /
        DONE / STOP. DONE also when something in the way stops it short: the lap then runs on
        the circle it did reach, and the next circle tries again from somewhere else."""
        end = self.leg_end(self.ring_step / self.search_speed + 2 * math.pi / self.turn_speed,
                           deadline)
        rate = rospy.Rate(RATE_HZ)
        aim = None  # the way out, fixed once chosen so the robot does not wander
        turns = 0
        try:
            while True:
                if self.interrupted():
                    return STOP
                if time.monotonic() > end:
                    rospy.logwarn("face_check: no time left to reach %.2f m out", radius)
                    return DONE
                pose = self.pose()
                if pose is None:
                    rospy.logwarn("face_check: no pose from tf, stopping the circles")
                    return DONE
                x, y, yaw = pose
                out = math.hypot(x - centre[0], y - centre[1])
                if out >= radius:
                    return DONE
                if aim is None:
                    # from the centre, "out" is wherever the robot looks: after the turn on the
                    # spot that is the way the drone's goal pointed it
                    aim = math.atan2(y - centre[1], x - centre[0]) if out > AT_CENTRE_M else yaw
                error = wrap(aim - yaw)
                forward = self.search_speed if abs(error) <= AIM_RAD else 0.0
                if forward > 0.0 and self.blocked(0.0):
                    if out > AT_CENTRE_M or turns >= MAX_BLOCKED_TURNS:
                        rospy.logwarn("face_check: the way out is blocked %.2f m from the goal,"
                                      " circling there instead", out)
                        return DONE
                    turns += 1
                    aim = wrap(aim + BLOCKED_TURN_RAD)
                    error = wrap(aim - yaw)
                    forward = 0.0
                    rospy.loginfo("face_check: way out blocked, trying %.0f deg round",
                                  math.degrees(BLOCKED_TURN_RAD * turns))
                self.drive(forward, 0.0, self.aiming_speed(error))
                if self.look() == FOUND:
                    return FOUND
                rate.sleep()
        finally:
            self.stop()

    def orbit(self, centre, radius, deadline):
        """One lap around `centre` at `radius`, sliding sideways and turning as it goes so the
        camera faces out of the circle the whole way round. FOUND / DONE / STOP. Something in
        the way reverses the lap; blocked both ways MAX_REVERSALS times gives the circle up."""
        rospy.loginfo("face_check: circling %.2f m round the goal, looking outwards", radius)
        speed = self.orbit_speed(radius)
        end = self.leg_end(2 * math.pi * radius / speed, deadline)
        rate = rospy.Rate(RATE_HZ)
        way = 1.0  # +1 = to the robot's left, reversed when something is in the way
        reversals = 0
        angle = None  # where round the circle the robot is
        covered = 0.0  # how far round it has gone, and the arc that has covered
        least = most = 0.0
        try:
            while True:
                if self.interrupted():
                    return STOP
                if time.monotonic() > end:
                    rospy.logwarn("face_check: no time left on the %.2f m circle", radius)
                    return DONE
                pose = self.pose()
                if pose is None:
                    rospy.logwarn("face_check: no pose from tf, stopping the circles")
                    return DONE
                x, y, yaw = pose
                here = math.atan2(y - centre[1], x - centre[0])
                if angle is not None:
                    covered += wrap(here - angle)  # going back covers no new ground
                    least, most = min(least, covered), max(most, covered)
                    if most - least >= LAP_RAD:
                        rospy.loginfo("face_check: nothing on the %.2f m circle", radius)
                        return DONE
                angle = here
                sideways = way * speed
                off = radius - math.hypot(x - centre[0], y - centre[1])
                outwards = clamp(RADIUS_GAIN * off, self.search_speed)
                if self.blocked(math.atan2(sideways, outwards)):
                    if reversals >= MAX_REVERSALS:
                        rospy.logwarn("face_check: the %.2f m circle is blocked both ways,"
                                      " widening the search", radius)
                        return DONE
                    reversals += 1
                    way = -way
                    rospy.loginfo("face_check: something in the way, going round the other way")
                    self.stop()
                else:
                    # turning at sideways / radius keeps the robot's nose pointing straight out
                    self.drive(outwards, sideways,
                               sideways / radius + self.aiming_speed(wrap(here - yaw)))
                if self.look() == FOUND:
                    return FOUND
                rate.sleep()
        finally:
            self.stop()

    def send_home(self):
        """Ask move_base for home: the map origin, the pose the robot started the run in."""
        goal = PoseStamped()
        goal.header.frame_id = MAP_FRAME
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = self.home[0]
        goal.pose.position.y = self.home[1]
        goal.pose.orientation.w = 1.0  # along the map's x axis, the way it started
        self.home_goal = True
        self.goal.publish(goal)

    def return_home(self):
        """Nothing out to the widest circle: let move_base drive back home, watching for the
        child all the way. A face on the way stops the drive and wins. FOUND / DONE / STOP."""
        rospy.loginfo("face_check: '%s' nowhere around the goal, going home to (%.2f, %.2f)",
                      self.person, self.home[0], self.home[1])
        self.send_home()
        end = time.monotonic() + self.home_timeout
        rate = rospy.Rate(RATE_HZ)
        try:
            while not self.interrupted() and time.monotonic() < end:
                if self.face() is not None:
                    self.cancel.publish(GoalID())  # move_base is driving: take the base back
                    rospy.sleep(CANCEL_SETTLE_S)
                    if self.look() == FOUND:
                        return FOUND
                    self.send_home()  # lost again: carry on home
                pose = self.pose(MAP_FRAME)
                if pose is not None and math.hypot(pose[0] - self.home[0],
                                                   pose[1] - self.home[1]) <= HOME_REACHED_M:
                    rospy.loginfo("face_check: home, waiting for a new goal")
                    return DONE
                rate.sleep()
            rospy.logwarn("face_check: gave up on the way home")
            return DONE
        finally:
            self.home_goal = False

    def search(self):
        """The whole search around the goal the robot was sent to: a turn on the spot, then
        wider and wider circles, then home. True once the child is found and centred."""
        deadline = time.monotonic() + self.timeout
        centre = self.wait_for_pose()
        if centre is None:
            rospy.logwarn("face_check: no tf %s -> %s, turning on the spot only",
                          self.odom_frame, self.robot_frame)
            return self.sweep(deadline) == FOUND

        result = self.sweep(deadline)
        for radius in self.rings():
            if result != DONE or time.monotonic() > deadline:
                break
            result = self.step_out(centre, radius, deadline)
            if result != DONE:
                break
            pose = self.pose()
            reached = 0.0 if pose is None else math.hypot(pose[0] - centre[0],
                                                          pose[1] - centre[1])
            if reached < max(MIN_ORBIT_RADIUS_M, radius - self.ring_step / 2.0):
                continue  # stopped well short of this circle: try the next one from here
            result = self.orbit(centre, reached, deadline)

        if result == DONE and self.go_home:
            result = self.return_home()
        return result == FOUND

    # ---------------------------------------------------------------- with the child, at last

    def distance_speed(self, area, target_area):
        """m/s that keeps the face box at target_area: bigger face (closer) = back up, smaller
        face (farther) = move forward."""
        error = (target_area - area) / target_area  # + = face too small = person too far
        if abs(error) <= AREA_DEADBAND:
            return 0.0
        return clamp(AREA_GAIN * error, self.max_linear)

    def follow(self):
        """Keep the person's face in the middle of the image and at the same size (so the robot
        stays at the same distance), until it is lost for FACE_LOST_S or a new goal arrives.
        The size to keep is ~target_area, or the face's size when it was found. The robot is
        always left stopped."""
        rate = rospy.Rate(RATE_HZ)
        last_seen = time.monotonic()
        target_area = self.target_area
        smooth_area = None
        try:
            while not rospy.is_shutdown() and not self.new_goal.is_set():
                face = self.face()
                if face is None:
                    self.stop()  # never move on an old sighting
                    if time.monotonic() - last_seen > FACE_LOST_S:
                        rospy.logwarn("face_check: lost '%s'", self.person)
                        return
                else:
                    last_seen = time.monotonic()
                    offset, area = face
                    # the box size jumps from frame to frame: follow its running average
                    smooth_area = (
                        area
                        if smooth_area is None
                        else AREA_SMOOTHING * area + (1 - AREA_SMOOTHING) * smooth_area
                    )
                    if target_area <= 0:
                        target_area = smooth_area
                        rospy.loginfo("face_check: keeping the face at %.0f px^2", target_area)
                    self.drive(self.distance_speed(smooth_area, target_area), 0.0,
                               self.centring_speed(offset))
                rate.sleep()
        finally:
            self.stop()

    def spin(self):
        while not rospy.is_shutdown():
            if not self.search_requested.wait(0.5):
                continue
            self.search_requested.clear()
            self.machine.set(SEARCHING_STATE)
            found = self.search()
            # one search per arrival: the arrival comes twice (/child_goal/reached and the
            # arrived state), and the second request must not start a new search after this one
            self.search_requested.clear()
            if found:
                rospy.loginfo("face_check: found '%s', following them", self.person)
                self.found.publish(String(data=self.person))
                self.machine.set(CARE_STATE)
                self.follow()
            elif not self.new_goal.is_set():
                rospy.logwarn("face_check: '%s' not found around the goal", self.person)


if __name__ == "__main__":
    rospy.init_node("face_check")
    FaceCheck().spin()
