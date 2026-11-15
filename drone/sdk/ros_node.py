#!/usr/bin/env python3
"""Example rospy node for the drone: runs on the robot (ROS1, Python 3.8) and bridges the
drone's HTTP API (drone/sdk/API.md, served by the Mac) to ROS topics. Copy it with client.py.

    rosrun <pkg> ros_node.py _url:=http://<mac address>:8077

Publishes
  /drone/odom              nav_msgs/Odometry      estimated pose, frame "drone_takeoff"
  /drone/battery           sensor_msgs/BatteryState
  /drone/image/compressed  sensor_msgs/CompressedImage  JPEG frames; header.stamp = the frame's
                           time (the Mac's X-Frame-Time moved onto this clock), frame "drone_camera"
  /drone/state             std_msgs/String        JSON of GET /status (10 Hz)
  Lost-child mission (drone/sdk/mission.py, API.md "Mission"):
  /drone/mission           std_msgs/String (latched)  JSON of GET /mission, on every change
  /drone/child_pose        geometry_msgs/PoseStamped (latched)  the child, frame "map"
                           (orientation: facing the child from the robot); "drone_takeoff" while
                           the robot's launch pose is unknown
  /move_base_simple/goal   geometry_msgs/PoseStamped  once per child report, ~standoff short of
                           the child, facing it (from the robot's pose now, tf map->base_footprint)
  /drone/fix               geometry_msgs/PoseArray (latched)  once per child report, same frame as
                           /drone/child_pose: poses[0] = the drone when it took the fix, exactly
                           over the child with the camera straight down (z = height above the
                           floor, orientation = its yaw), poses[1] = the child (z = 0, as in
                           /drone/child_pose); header.stamp = the fix's frame time on this clock
  /drone/pose_map          geometry_msgs/PoseStamped  the drone in "map", once aligned (10 Hz)
  tf  map -> drone_takeoff (once aligned) and drone_takeoff -> drone_base (always)
Subscribes
  /drone/cmd_vel  geometry_msgs/Twist   velocity, stops 0.5 s after the last message
  /drone/patrol   std_msgs/String       JSON body of POST /patrol: {"waypoints": [...], ...}
  /drone/stop     std_msgs/Empty        STOP (cancels the autopilot and the mission, hovers)
  /drone/resume   std_msgs/Empty        allow cmd_vel again after a STOP
  /drone/mission/start  std_msgs/String  JSON body of POST /mission/start
  /drone/mission/abort  std_msgs/Empty   back to the takeoff point and land
  /drone/detection      std_msgs/String  JSON {"stamp": <header.stamp of the image, seconds>,
                        "bbox": [x0, y0, x1, y1] (fractions of the image, y down),
                        "confidence": 0..1, "label"?, "anchor"?: "feet" for a whole body,
                        "centre" (default) for a head}; "frame_time" (the Mac's) instead of
                        "stamp" also works
  /drone/launch_pose    geometry_msgs/PoseStamped  the robot's pose in "map" at takeoff, if tf
                        cannot give it (the node posts it by itself from tf otherwise)
  /map (~map_topic)     nav_msgs/OccupancyGrid (latched)  sent once to the Mac's map view
                        (POST /map/grid), again whenever it changes or the Mac lost it
Map view (the web page's map, API.md "Map view"): the robot's tf pose map -> base_footprint is
posted at ~map_pose_rate Hz (POST /map/robot_pose).

Axes: ROS REP 103 (x forward, y left, z up, yaw counter-clockwise); the drone's takeoff frame
is x forward-ish, y right, z down, heading clockwise, so y, z and yaw change sign here.
Params: ~url, ~max_linear (m/s sent as full stick, default 1.0), ~max_angular (rad/s, 1.0),
~map_frame ("map"), ~robot_frame ("base_footprint"), ~send_goal (true), ~goal_topic
("/move_base_simple/goal"), ~standoff (m; default: the mission's standoff_m), ~auto_launch_pose
(true: post the robot's tf pose as the launch pose while the mission launches), ~map_pose_rate
(Hz, default 2; 0 = off), ~send_map (true), ~map_topic ("/map").
"""

from __future__ import annotations

import array
import json
import math
import threading
from collections import deque

import rospy
import tf2_ros
from client import DroneApiError, DroneClient, standoff_goal
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import BatteryState, CompressedImage
from std_msgs.msg import Empty, String

DEFAULT_URL = "http://127.0.0.1:8077"
CMD_VEL_HOLD_S = 0.5  # the server's watchdog: the drone stops this long after the last one
RETRY_S = 1.0
WARN_PERIOD_S = 2.0
ODOM_FRAME = "drone_takeoff"
BASE_FRAME = "drone_base"
CAMERA_FRAME = "drone_camera"
PERCENT = 100.0
LAUNCH_STATES = ("launching", "climbing", "aligning")  # the robot stays put until reporting
CLOCK_SAMPLES = 100  # the Mac -> robot clock offset is the smallest of these (least delay)
STAMP_HISTORY = 300  # image stamps remembered, to give detections the Mac's frame time back
STAMP_MATCH_S = 0.002
TF_TIMEOUT_S = 0.5
MAP_POSE_HZ = 2.0  # the robot's pose for the Mac's map view


def _unit(value: float) -> float:
    return max(-1.0, min(1.0, value))


def _yaw_quaternion(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class DroneNode:
    def __init__(self) -> None:
        self.client = DroneClient(rospy.get_param("~url", DEFAULT_URL))
        self.max_linear = float(rospy.get_param("~max_linear", 1.0))
        self.max_angular = float(rospy.get_param("~max_angular", 1.0))
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_footprint")
        self.send_goal = bool(rospy.get_param("~send_goal", True))
        self.standoff = rospy.get_param("~standoff", None)
        self.auto_launch_pose = bool(rospy.get_param("~auto_launch_pose", True))
        self.resume = False
        self.clock_offsets = deque(maxlen=CLOCK_SAMPLES)  # robot time - Mac time, seconds
        self.stamps = deque(maxlen=STAMP_HISTORY)  # (image stamp, Mac frame time)
        self.last_mission = None  # (state, step, child id): /drone/mission on change only
        self.child_id = None
        self.launch_posted = None  # started_at of the mission we posted the launch pose for
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.odom = rospy.Publisher("/drone/odom", Odometry, queue_size=10)
        self.battery = rospy.Publisher("/drone/battery", BatteryState, queue_size=10)
        self.image = rospy.Publisher("/drone/image/compressed", CompressedImage, queue_size=2)
        self.state = rospy.Publisher("/drone/state", String, queue_size=10)
        self.mission = rospy.Publisher("/drone/mission", String, queue_size=10, latch=True)
        self.child = rospy.Publisher("/drone/child_pose", PoseStamped, queue_size=1, latch=True)
        self.fix = rospy.Publisher("/drone/fix", PoseArray, queue_size=1, latch=True)
        self.pose_map = rospy.Publisher("/drone/pose_map", PoseStamped, queue_size=10)
        goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.goal = rospy.Publisher(goal_topic, PoseStamped, queue_size=1)
        rospy.Subscriber("/drone/cmd_vel", Twist, self.on_cmd_vel, queue_size=1)
        rospy.Subscriber("/drone/patrol", String, self.on_patrol, queue_size=1)
        rospy.Subscriber("/drone/stop", Empty, self.on_stop, queue_size=1)
        rospy.Subscriber("/drone/resume", Empty, self.on_resume, queue_size=1)
        rospy.Subscriber("/drone/mission/start", String, self.on_mission_start, queue_size=1)
        rospy.Subscriber("/drone/mission/abort", Empty, self.on_mission_abort, queue_size=1)
        rospy.Subscriber("/drone/detection", String, self.on_detection, queue_size=10)
        rospy.Subscriber("/drone/launch_pose", PoseStamped, self.on_launch_pose, queue_size=1)
        self.grid = None  # the last /map, as the arguments of DroneClient.map_grid
        if rospy.get_param("~send_map", True):
            map_topic = rospy.get_param("~map_topic", "/map")
            rospy.Subscriber(map_topic, OccupancyGrid, self.on_map, queue_size=1)
        map_pose_hz = float(rospy.get_param("~map_pose_rate", MAP_POSE_HZ))
        if map_pose_hz > 0:
            rospy.Timer(rospy.Duration(1.0 / map_pose_hz), self.post_robot_pose)
        for target in (self.stream_status, self.stream_frames):
            threading.Thread(target=self.keep_running, args=(target,), daemon=True).start()

    # --- commands -----------------------------------------------------------------------------

    def on_cmd_vel(self, twist: Twist) -> None:
        try:
            self.client.move(
                fwd=_unit(twist.linear.x / self.max_linear),
                right=_unit(-twist.linear.y / self.max_linear),
                up=_unit(twist.linear.z / self.max_linear),
                turn=_unit(-twist.angular.z / self.max_angular),
                seconds=CMD_VEL_HOLD_S,
                resume=self.resume,
            )
            self.resume = False
        except DroneApiError as error:
            rospy.logwarn_throttle(WARN_PERIOD_S, f"drone cmd_vel refused: {error}")

    def on_patrol(self, message: String) -> None:
        try:
            body = json.loads(message.data)
            self.client.patrol(body.pop("waypoints"), **body)
            rospy.loginfo("drone patrol started")
        except (ValueError, KeyError, DroneApiError) as error:
            rospy.logwarn(f"drone patrol refused: {error}")

    def on_stop(self, _message: Empty) -> None:
        try:
            self.client.stop()
        except DroneApiError as error:
            rospy.logerr(f"drone STOP failed: {error}")

    def on_resume(self, _message: Empty) -> None:
        self.resume = True

    def on_mission_start(self, message: String) -> None:
        try:
            self.client.mission_start(**json.loads(message.data or "{}"))
            rospy.loginfo("drone mission started: press the drone's button to take off")
        except (ValueError, TypeError, DroneApiError) as error:
            rospy.logwarn(f"drone mission refused: {error}")

    def on_mission_abort(self, _message: Empty) -> None:
        try:
            self.client.mission_abort()
        except DroneApiError as error:
            rospy.logerr(f"drone mission abort failed: {error}")

    def on_detection(self, message: String) -> None:
        try:
            body = json.loads(message.data)
            frame_time = body.get("frame_time")
            if frame_time is None and body.get("stamp") is not None:
                frame_time = self.mac_time(float(body["stamp"]))
            reply = self.client.mission_target(
                frame_time,
                body["bbox"],
                body.get("confidence", 1.0),
                body.get("label"),
                body.get("anchor"),
            )
            if not reply.get("accepted"):
                rospy.logdebug_throttle(WARN_PERIOD_S, f"detection not used: {reply.get('reason')}")
        except (ValueError, KeyError, TypeError, DroneApiError) as error:
            rospy.logwarn_throttle(WARN_PERIOD_S, f"drone detection refused: {error}")

    def on_launch_pose(self, message: PoseStamped) -> None:
        position, q = message.pose.position, message.pose.orientation
        self.post_launch_pose(position.x, position.y, _yaw_of(q))

    # --- map view ---------------------------------------------------------------------------

    def on_map(self, grid: OccupancyGrid) -> None:
        info, origin = grid.info, grid.info.origin
        self.grid = (
            info.width,
            info.height,
            info.resolution,
            (origin.position.x, origin.position.y, _yaw_of(origin.orientation)),
            array.array("b", grid.data).tobytes(),
            f"ros {grid.header.frame_id or 'map'}",
        )
        self.post_grid()

    def post_grid(self) -> None:
        try:
            self.client.map_grid(*self.grid)
            rospy.loginfo(f"map sent to the drone's map view ({self.grid[0]} x {self.grid[1]})")
        except DroneApiError as error:
            rospy.logwarn_throttle(WARN_PERIOD_S, f"map not sent: {error}")

    def post_robot_pose(self, _event) -> None:
        pose = self.robot_pose()
        if pose is None:
            return
        try:
            reply = self.client.map_robot_pose(*pose)
        except DroneApiError as error:
            rospy.logwarn_throttle(WARN_PERIOD_S, f"robot pose not sent: {error}")
            return
        if self.grid is not None and not reply.get("has_grid"):
            self.post_grid()  # the Mac restarted: /map is latched, so it would never come again

    # --- streams ------------------------------------------------------------------------------

    def keep_running(self, stream) -> None:
        """Run a stream, reconnecting after errors, until ROS shuts down."""
        while not rospy.is_shutdown():
            try:
                stream()
            except (DroneApiError, OSError, ValueError) as error:
                rospy.logwarn_throttle(WARN_PERIOD_S, f"drone stream: {error}")
            rospy.sleep(RETRY_S)

    def stream_status(self) -> None:
        for status in self.client.events():
            if rospy.is_shutdown():
                return
            self.clock_offsets.append(rospy.get_time() - status["updated_at"])
            self.state.publish(String(json.dumps(status)))
            telemetry = status["telemetry"]
            if telemetry["pose"]:
                self.publish_odom(telemetry["pose"])
            if telemetry["battery_pct"] is not None:
                self.publish_battery(telemetry["battery_pct"])
            if status.get("mission"):
                self.on_mission(status["mission"])

    def stream_frames(self) -> None:
        for jpeg, meta in self.client.frames_with_meta():
            if rospy.is_shutdown():
                return
            message = CompressedImage()
            if meta["frame_time"] is not None and self.clock_offsets:
                stamp = meta["frame_time"] + min(self.clock_offsets)
                message.header.stamp = rospy.Time.from_sec(stamp)
                self.stamps.append((message.header.stamp.to_sec(), meta["frame_time"]))
            else:
                message.header.stamp = rospy.Time.now()
            message.header.frame_id = CAMERA_FRAME
            message.format = "jpeg"
            message.data = jpeg
            self.image.publish(message)

    def mac_time(self, stamp: float) -> float:
        """The Mac's frame time of an image we published with header.stamp `stamp`."""
        for published, frame_time in reversed(self.stamps):
            if abs(published - stamp) <= STAMP_MATCH_S:
                return frame_time
        return stamp - (min(self.clock_offsets) if self.clock_offsets else 0.0)

    # --- mission ------------------------------------------------------------------------------

    def on_mission(self, mission: dict) -> None:
        child = mission.get("child") or {}
        key = (mission["state"], mission["step"], child.get("id"))
        if key != self.last_mission:
            self.last_mission = key
            self.mission.publish(String(json.dumps(mission)))
        transform = mission.get("map_from_drone")
        if transform:
            self.publish_map_transform(transform)
        if mission.get("pose_map"):
            pose = mission["pose_map"]
            self.pose_map.publish(
                self.pose_stamped(self.map_frame, pose["x"], pose["y"], pose["yaw"])
            )
        if self.auto_launch_pose and mission["state"] in LAUNCH_STATES:
            self.maybe_post_launch_pose(mission)
        if child and child.get("id") != self.child_id:
            self.child_id = child["id"]
            self.publish_child(child, mission)

    def maybe_post_launch_pose(self, mission: dict) -> None:
        if mission.get("launch_pose") or self.launch_posted == mission["started_at"]:
            return
        pose = self.robot_pose()
        if pose is None:
            rospy.logwarn_throttle(
                WARN_PERIOD_S,
                f"no tf {self.map_frame} -> {self.robot_frame}: publish /drone/launch_pose",
            )
            return
        if self.post_launch_pose(*pose):
            self.launch_posted = mission["started_at"]

    def post_launch_pose(self, x: float, y: float, yaw: float) -> bool:
        try:
            self.client.mission_launch_pose(x, y, yaw)
        except DroneApiError as error:
            rospy.logwarn(f"drone launch pose refused: {error}")
            return False
        rospy.loginfo(f"drone launch pose: x {x:.2f} y {y:.2f} yaw {yaw:.2f} in {self.map_frame}")
        return True

    def publish_child(self, child: dict, mission: dict) -> None:
        if not child.get("map"):
            drone = child["drone"]  # drone frame -> "drone_takeoff" (ROS axes: y flips)
            self.child.publish(self.pose_stamped(ODOM_FRAME, drone["x"], -drone["y"], 0.0))
            self.publish_fix(child, ODOM_FRAME, (drone["x"], -drone["y"], 0.0))
            rospy.logwarn("child found, but no launch pose: no map position and no goal")
            return
        x, y = child["map"]["x"], child["map"]["y"]
        robot = self.robot_pose() or _launch_xy_yaw(mission)
        standoff = self.standoff
        if standoff is None:
            standoff = (mission.get("config") or {}).get("standoff_m", 0.0)
        goal_x, goal_y, goal_yaw = standoff_goal(robot[0], robot[1], x, y, float(standoff))
        self.child.publish(self.pose_stamped(self.map_frame, x, y, goal_yaw))
        self.publish_fix(child, self.map_frame, (x, y, goal_yaw))
        rospy.loginfo(f"child at x {x:.2f} y {y:.2f} in {self.map_frame}")
        if self.send_goal:
            self.goal.publish(self.pose_stamped(self.map_frame, goal_x, goal_y, goal_yaw))
            rospy.loginfo(f"goal sent: x {goal_x:.2f} y {goal_y:.2f} yaw {goal_yaw:.2f}")

    def publish_fix(self, child: dict, frame: str, child_xy_yaw: tuple) -> None:
        """/drone/fix: [the drone at the fix, the child], in `frame` (map, or drone_takeoff
        while the launch pose is unknown). Nothing without a fix (an older Mac server)."""
        fix = child.get("fix")
        if not fix:
            return
        if frame == ODOM_FRAME:
            drone = fix["drone"]  # drone frame -> "drone_takeoff": y and the heading flip
            pose = {"x": drone["x"], "y": -drone["y"], "z": drone["alt"]}
            pose["yaw"] = -math.radians(drone["heading_deg"])
        elif fix.get("map"):
            pose = fix["map"]
        else:
            return
        message = PoseArray()
        message.header.stamp = rospy.Time.from_sec(self.robot_time(fix["time"]))
        message.header.frame_id = frame
        message.poses = [
            _pose(pose["x"], pose["y"], pose["z"], pose["yaw"]),
            _pose(child_xy_yaw[0], child_xy_yaw[1], 0.0, child_xy_yaw[2]),
        ]
        self.fix.publish(message)
        rospy.loginfo(
            f"drone fix: drone at x {pose['x']:.2f} y {pose['y']:.2f} z {pose['z']:.2f} in "
            f"{frame}, {fix['centre_error_px']:.0f} px from centred, {fix['samples']} frames"
        )

    def robot_time(self, mac_time: float) -> float:
        """A Mac unix time on this node's clock (the learned offset)."""
        return mac_time + (min(self.clock_offsets) if self.clock_offsets else 0.0)

    def robot_pose(self) -> tuple[float, float, float] | None:
        """(x, y, yaw) of the robot in the map frame from tf, or None."""
        try:
            stamped = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rospy.Time(0), rospy.Duration(TF_TIMEOUT_S)
            )
        except tf2_ros.TransformException:
            return None
        translation = stamped.transform.translation
        return translation.x, translation.y, _yaw_of(stamped.transform.rotation)

    def publish_map_transform(self, transform: dict) -> None:
        message = TransformStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.map_frame
        message.child_frame_id = ODOM_FRAME
        message.transform.translation.x = transform["x"]
        message.transform.translation.y = transform["y"]
        message.transform.translation.z = transform["z"]
        message.transform.rotation = _yaw_quaternion(transform["yaw"])
        self.tf_broadcaster.sendTransform(message)

    @staticmethod
    def pose_stamped(frame: str, x: float, y: float, yaw: float) -> PoseStamped:
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = frame
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.orientation = _yaw_quaternion(yaw)
        return message

    def publish_odom(self, pose: dict) -> None:
        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = ODOM_FRAME
        message.child_frame_id = BASE_FRAME
        message.pose.pose.position.x = pose["x"]
        message.pose.pose.position.y = -pose["y"]
        message.pose.pose.position.z = pose["alt"]
        message.pose.pose.orientation = _yaw_quaternion(-math.radians(pose["heading_deg"]))
        position_var = pose["position_sigma_m"] ** 2
        heading_var = math.radians(pose["heading_sigma_deg"]) ** 2
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = covariance[14] = position_var
        covariance[35] = heading_var
        message.pose.covariance = covariance
        vx, vy, vz = pose["velocity"]  # takeoff frame; ROS wants the body frame
        heading = math.radians(pose["heading_deg"])
        message.twist.twist.linear.x = vx * math.cos(heading) + vy * math.sin(heading)
        message.twist.twist.linear.y = -(-vx * math.sin(heading) + vy * math.cos(heading))
        message.twist.twist.linear.z = -vz
        self.odom.publish(message)
        body = TransformStamped()
        body.header = message.header
        body.child_frame_id = BASE_FRAME
        position = message.pose.pose.position
        body.transform.translation.x = position.x
        body.transform.translation.y = position.y
        body.transform.translation.z = position.z
        body.transform.rotation = message.pose.pose.orientation
        self.tf_broadcaster.sendTransform(body)

    def publish_battery(self, percent: int) -> None:
        message = BatteryState()
        message.header.stamp = rospy.Time.now()
        message.percentage = percent / PERCENT
        message.present = True
        message.voltage = float("nan")
        self.battery.publish(message)


def _pose(x: float, y: float, z: float, yaw: float) -> Pose:
    message = Pose()
    message.position.x, message.position.y, message.position.z = x, y, z
    message.orientation = _yaw_quaternion(yaw)
    return message


def _yaw_of(q: Quaternion) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _launch_xy_yaw(mission: dict) -> tuple[float, float, float]:
    launch = mission.get("launch_pose") or {}
    return launch.get("x", 0.0), launch.get("y", 0.0), launch.get("yaw", 0.0)


def main() -> None:
    rospy.init_node("drone")
    DroneNode()
    rospy.spin()


if __name__ == "__main__":
    main()
