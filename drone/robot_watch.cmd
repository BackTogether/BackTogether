@echo off
rem Windows twin of drone/robot_watch (keep the two in step): watch, on the robot, what it does
rem with the child the drone sends (watch_goal.py over ssh). Needs the venue Wi-Fi.
rem Usage: drone\robot_watch [ssh target, default bobac3@<ROBOT_IP>]
setlocal
cd /d "%~dp0.."
set "TARGET=%~1"
if not defined TARGET set "TARGET=bobac3@<ROBOT_IP>"
scp -q bobac\vendor\bobac3_ws\src\bringup\scripts\watch_goal.py %TARGET%:watch_goal.py
ssh -t %TARGET% "bash -ic \"python3 -u ~/watch_goal.py\""
