@echo off
rem Windows twin of drone/dock (keep the two in step): wait for the parent to hang up (or low battery, or the mission flying home), then land on
rem the robot's pad (drone/dock_robot.py). Needs drone\fly running and the pad model.
rem Usage: drone\dock --care http://<robot>:8000 [--battery 35] [--robot X Y]
rem        drone\dock --now     land on the robot right away
setlocal
cd /d "%~dp0.."
rem UTF-8 for the page, the logs and the console, whatever the Windows code page is
set PYTHONUTF8=1
rem Offline like the bash launchers (the drone's Wi-Fi has no internet); drone\setup turns it off
set "OFFLINE=--offline"
if defined DRONE_ONLINE set "OFFLINE="
uv run %OFFLINE% --no-project --python 3.11 --with opencv-contrib-python-headless python -m drone.dock_robot %*
