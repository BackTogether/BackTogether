@echo off
rem Windows twin of drone/demo (keep the two in step): the whole demo, drone side. Take off from
rem the robot's pad, lawnmower sweep at 3 m, find the child, hover over them with the camera down, send
rem the child to the robot through ngrok and follow it. Needs drone\fly running and, on the robot,
rem roslaunch bringup state_demo.launch.
rem Usage: drone\demo [child, default jorge] [more drone\find options, e.g. --length 10 --width 6]
setlocal
set "CHILD=jorge"
set "FIRST=%~1"
if defined FIRST if not "%FIRST:~0,1%"=="-" (
  set "CHILD=%~1"
  shift
)
rem takeoff from the pad on the robot's top (land_on_pad.PAD_HEIGHT_M)
set "PAD_HEIGHT_M=0.4"
set "REST="
:rest
if "%~1"=="" goto run
set "REST=%REST% %1"
shift
goto rest
:run
call "%~dp0find.cmd" %CHILD% --alt 3 --orbit 0 --takeoff-height %PAD_HEIGHT_M% --sweep --robot%REST%