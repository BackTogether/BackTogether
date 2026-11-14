#!/usr/bin/env python3
"""Example rospy node for the drone: runs on the robot (ROS1, Python 3.8) and bridges the
drone's HTTP API (drone/sdk/API.md, served by the Mac) to ROS topics. Copy it with client.py.

    rosrun <pkg> ros_node.py _url:=http://<mac address>:8077

Publishes
  /drone/odom              nav_msgs/Odometry      estimated pose, frame "drone_takeoff"
  /drone/battery           sensor_msgs/BatteryState
  /drone/image/compressed  sensor_msgs/CompressedImage  JPEG frames
  /drone/state             std_msgs/String        JSON of GET /status (10 Hz)
Subscribes
  /drone/cmd_vel  geometry_msgs/Twist   velocity, stops 0.5 s after the last message
  /drone/patrol   std_msgs/String       JSON body of POST /patrol: {"waypoints": [...], ...}
  /drone/stop     std_msgs/Empty        STOP (cancels the autopilot, hovers)
  /drone/resume   std_msgs/Empty        allow cmd_vel again after a STOP

Axes: ROS REP 103 (x forward, y left, z up, yaw counter-clockwise); the drone's takeoff frame
is x forward-ish, y right, z down, heading clockwise, so y, z and yaw change sign here.
Params: ~url, ~max_linear (m/s sent as full stick, default 1.0), ~max_angular (rad/s, 1.0).
"""

from __future__ import annotations

import json
import math
import threading

import rospy
from client import DroneApiError, DroneClient
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, CompressedImage
from std_msgs.msg import Empty, String

DEFAULT_URL = "http://127.0.0.1:8077"
CMD_VEL_HOLD_S = 0.5  # the server's watchdog: the drone stops this long after the last one
RETRY_S = 1.0
WARN_PERIOD_S = 2.0
ODOM_FRAME = "drone_takeoff"
BASE_FRAME = "drone_base"
PERCENT = 100.0


def _unit(value: float) -> float:
    return max(-1.0, min(1.0, value))


def _yaw_quaternion(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class DroneNode:
    def __init__(self) -> None:
        self.client = DroneClient(rospy.get_param("~url", DEFAULT_URL))
        self.max_linear = float(rospy.get_param("~max_linear", 1.0))
        self.max_angular = float(rospy.get_param("~max_angular", 1.0))
        self.resume = False
        self.odom = rospy.Publisher("/drone/odom", Odometry, queue_size=10)
        self.battery = rospy.Publisher("/drone/battery", BatteryState, queue_size=10)
        self.image = rospy.Publisher("/drone/image/compressed", CompressedImage, queue_size=2)
        self.state = rospy.Publisher("/drone/state", String, queue_size=10)
        rospy.Subscriber("/drone/cmd_vel", Twist, self.on_cmd_vel, queue_size=1)
        rospy.Subscriber("/drone/patrol", String, self.on_patrol, queue_size=1)
        rospy.Subscriber("/drone/stop", Empty, self.on_stop, queue_size=1)
        rospy.Subscriber("/drone/resume", Empty, self.on_resume, queue_size=1)
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
            self.state.publish(String(json.dumps(status)))
            telemetry = status["telemetry"]
            if telemetry["pose"]:
                self.publish_odom(telemetry["pose"])
            if telemetry["battery_pct"] is not None:
                self.publish_battery(telemetry["battery_pct"])

    def stream_frames(self) -> None:
        for jpeg in self.client.frames():
            if rospy.is_shutdown():
                return
            message = CompressedImage()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = BASE_FRAME
            message.format = "jpeg"
            message.data = jpeg
            self.image.publish(message)

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

    def publish_battery(self, percent: int) -> None:
        message = BatteryState()
        message.header.stamp = rospy.Time.now()
        message.percentage = percent / PERCENT
        message.present = True
        message.voltage = float("nan")
        self.battery.publish(message)


def main() -> None:
    rospy.init_node("drone")
    DroneNode()
    rospy.spin()


if __name__ == "__main__":
    main()
