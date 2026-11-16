@echo off
rem Windows twin of drone/demo (keep the two in step): take off from the floor LAUNCH_AHEAD_M in
rem front of the robot, sweep at 3 m, find the child, hover over them, send them to the robot
rem through ngrok and stay there; PAD_WAIT_S after the robot's pad came in view, land on it.
rem Needs drone\fly running and, on the robot, roslaunch bringup state_demo.launch.
rem Without a child it first waits for the parent's request from the web page.
rem Usage: drone\demo [child] [more drone\find options, e.g. --alt 2.2 --length 10 --width 6]
setlocal
set "CHILD=--from-web"
set "FIRST=%~1"
if defined FIRST if not "%FIRST:~0,1%"=="-" (
  set "CHILD=%~1"
  shift
)
rem the drone takes off from the floor this far in front of the robot
set "LAUNCH_AHEAD_M=0.5"
rem it lands on the robot's pad this long after the pad came in view
set "PAD_WAIT_S=30"
set "REST="
:rest
if "%~1"=="" goto run
set "REST=%REST% %1"
shift
goto rest
:run
call "%~dp0find.cmd" %CHILD% --alt 3 --orbit 0 --sweep --collect --robot --launch-ahead %LAUNCH_AHEAD_M%%REST%
call "%~dp0dock.cmd" --pad-wait %PAD_WAIT_S%
