@echo off
rem Windows twin of drone/ltest (keep the two in step): L test of the drone's position estimate.
rem Needs drone\fly running and the drone flying. Then drone\robot <leg> <leg> for the robot.
rem Usage: drone\ltest 2.22     both legs of the taped L, metres
setlocal
cd /d "%~dp0.."
set PYTHONUTF8=1
set "OFFLINE=--offline"
if defined DRONE_ONLINE set "OFFLINE="
uv run %OFFLINE% --no-project --python 3.11 python -m drone.l_test %*