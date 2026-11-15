@echo off
rem Windows twin of drone/land (keep the two in step): find the landing pad and land on it (drone/land_on_pad.py).
rem Needs drone\fly running and the pad model (python -m drone.identify.pad train).
rem Usage: drone\land [--alt 2] [--pad-height 0.4] [--takeoff-height 0] [--tilt -50]
rem        drone\land --watch     no flight: is the pad seen on the live video?
setlocal
cd /d "%~dp0.."
rem UTF-8 for the page, the logs and the console, whatever the Windows code page is
set PYTHONUTF8=1
rem Offline like the bash launchers (the drone's Wi-Fi has no internet); drone\setup turns it off
set "OFFLINE=--offline"
if defined DRONE_ONLINE set "OFFLINE="
uv run %OFFLINE% --no-project --python 3.11 --with opencv-contrib-python-headless python -m drone.land_on_pad %*
