@echo off
rem Windows twin of drone/find (keep the two in step): patrol, find the child, hover over them (drone/find_child.py). Needs drone\fly running.
rem Usage: drone\find <photos folder | name in drone\identify\people> [--alt 2] [--tilt -40]
rem        drone\find <name> --watch     no flight: recognise and track on the live video only
setlocal
cd /d "%~dp0.."
rem UTF-8 for the page, the logs and the console, whatever the Windows code page is
set PYTHONUTF8=1
rem Offline like the bash launchers (the drone's Wi-Fi has no internet); drone\setup turns it off
set "OFFLINE=--offline"
if defined DRONE_ONLINE set "OFFLINE="
uv run %OFFLINE% --no-project --python 3.11 --with opencv-contrib-python-headless python -m drone.find_child %*
