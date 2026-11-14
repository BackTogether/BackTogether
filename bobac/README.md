# bobac/: Bobac_d mobile robot

Everything for the wheeled display robot: a **Bobac_d** "Robot ROS Mobile Platform" (Dobot,
made by Shenzhen Yuanchuangxing Technology). Code that talks to the robot, its adapters and its
notes live here. Board label: `area: bobac`.

## What we know (from the robot and its label)
- Round differential-drive base with front sensor windows; ROS-based.
- Mast with a touchscreen display and a USB webcam on top.
- Red **emergency stop** button on the back box; Wi-Fi antennas on the sides.
- Size 377 × 377 × 798 mm, 26 kg. Manufactured 2025-12-18, serial MR18-4523-0015.

## Still to find out
- ROS version, how to reach it (Wi-Fi IP, SSH user), and which topics move the base
  (`/cmd_vel`?) and read the sensors.
- What the front sensors are (lidar, depth camera, ultrasonic) and how the screen is driven.

Add findings here as you confirm them. Every motion command needs a speed limit and a stop path.
