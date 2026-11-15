@echo off
rem Windows twin of drone/robot (keep the two in step): talk to the robot through ngrok.
rem Usage: drone\robot             is the robot reachable and ready?
rem        drone\robot 2 0         a child 2 m ahead, 0 m left of the robot: it drives there
rem        drone\robot --watch     with drone\find: send the child the mission finds
setlocal
cd /d "%~dp0.."
set PYTHONUTF8=1
set "OFFLINE=--offline"
if defined DRONE_ONLINE set "OFFLINE="
set "ARGS=%*"
if "%~1"=="" set "ARGS=--ping"
if not "%~2"=="" if "%~3"=="" (
  echo %~1| findstr /r "^-*[0-9][0-9.]*$" >nul && set "ARGS=--x %~1 --y %~2 --follow"
)
uv run %OFFLINE% --no-project --python 3.11 python -m drone.report_child %ARGS%