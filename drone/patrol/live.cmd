@echo off
rem Windows twin of drone/patrol/live (keep the two in step; the modes are explained there):
rem live patrol tests against the running app (drone\fly = drone.sdk serve), offline-safe.
rem Usage: drone\patrol\live feed <name> | mission | mission-sweep | status | abort | spin [sticks]
rem          | tilt <deg> | alt <m> | distance | dry | bounce | dvd-dry | dvd | clear-dry | clear
rem          | sweep-dry | sweep  [extra args]
rem        drone\patrol\live model    download the depth model (drone\setup does it)
rem Settings, as environment variables: SDK (http://127.0.0.1:8077), TAKEOFF_HEIGHT,
rem DVD_TAKEOFF_HEIGHT, STOP_AT, CLEAR_AT, ORBIT_TURNS.
setlocal
cd /d "%~dp0..\.."
rem UTF-8 for the page, the logs and the console, whatever the Windows code page is
set PYTHONUTF8=1
rem Offline like the bash launchers (the drone's Wi-Fi has no internet); drone\setup turns it off
set "OFFLINE=--offline"
if defined DRONE_ONLINE set "OFFLINE="
if not defined SDK set "SDK=http://127.0.0.1:8077"
if not defined TAKEOFF_HEIGHT set TAKEOFF_HEIGHT=1.0
if not defined DVD_TAKEOFF_HEIGHT set DVD_TAKEOFF_HEIGHT=0
if not defined STOP_AT set STOP_AT=2.5
if not defined CLEAR_AT set CLEAR_AT=3.3
if not defined ORBIT_TURNS set ORBIT_TURNS=1
rem the depth model is cached; the drone's Wi-Fi has no internet
if not defined HF_HUB_OFFLINE set HF_HUB_OFFLINE=1
set "PY=uv run %OFFLINE% --no-project --python 3.11"
set "DEPTH=%PY% --with numpy --with pillow --with torch --with transformers python -m"
set "JSON=Content-Type: application/json"
set "BOUNCE=--sdk %SDK% --takeoff-height %TAKEOFF_HEIGHT% --geofence 4 --tilt -30 --stop-at %STOP_AT% --clear-at %CLEAR_AT%"
set "PATROL=--sdk %SDK% --takeoff-height %DVD_TAKEOFF_HEIGHT% --tilt -30"
set "MISSION_END=\"orbit_turns\": %ORBIT_TURNS%, \"launch_height_m\": %DVD_TAKEOFF_HEIGHT%}"

set "SELF=%~f0"
set "MODE=%~1"
set "ARG=%~2"
rem the extra arguments: everything after the mode
set "REST="
shift
:rest
if "%~1"=="" goto dispatch
set REST=%REST% %1
shift
goto rest

:dispatch
for %%m in (feed mission mission-sweep abort status spin tilt alt distance dry bounce dvd-dry dvd clear-dry clear sweep-dry sweep model) do if "%MODE%"=="%%m" goto %%m
findstr /b /c:"rem Usage" /c:"rem     " /c:"rem Settings" /c:"rem DVD_" "%SELF%"
exit /b 1

:feed
if not defined ARG echo child name: a folder in drone\identify\people& exit /b 1
%PY% --with opencv-contrib-python-headless --with numpy python -m drone.identify.feed %REST% --url %SDK%
exit /b
:mission
rem small test mission: square 2-4 m ahead of the launch, until found, then overhead fix, orbit, land
curl -s -X POST "%SDK%/mission/start" -H "%JSON%" -d "{\"waypoints\": [{\"x\": 2, \"y\": -1.5}, {\"x\": 4, \"y\": -1.5}, {\"x\": 4, \"y\": 1.5}, {\"x\": 2, \"y\": 1.5}], \"loops\": 0, %MISSION_END%"
echo.
exit /b
:mission-sweep
rem the mission watches for the child while `live sweep` flies the lawnmower
curl -s -X POST "%SDK%/mission/start" -H "%JSON%" -d "{\"waypoints\": [{\"x\": 2, \"y\": 0}], \"strategy\": \"external\", \"patrol_tilt_deg\": -40, %MISSION_END%"
echo.
exit /b
:abort
curl -s -X POST "%SDK%/mission/abort"
echo.
exit /b
:status
%PY% python -c "import sys; from drone.sdk.client import DroneClient as C; m = C(sys.argv[1]).mission(); c = m.get('child') or {}; print(m['state'], '|', m['step'], '|', m.get('message'), '| child:', c.get('drone'), c.get('map'))" %SDK%
exit /b
:spin
%PY% --with numpy --with pillow python -m drone.calibration.spin %REST%
exit /b
:tilt
if not defined ARG echo degrees& exit /b 1
curl -s -X POST -H "%JSON%" -d "{\"deg\": %ARG%}" "%SDK%/tilt"
echo.
exit /b
:alt
if not defined ARG echo metres above takeoff& exit /b 1
%PY% python -c "import sys; from drone.sdk.client import DroneClient as C; c = C(sys.argv[1]); p = c.status()['telemetry']['pose']; print(c.goto(p['x'], p['y'], float(sys.argv[2])))" %SDK% %ARG%
exit /b
:distance
%DEPTH% drone.patrol.front_distance --sdk %SDK% --takeoff-height %TAKEOFF_HEIGHT% %REST%
exit /b
:dry
%DEPTH% drone.patrol.bounce %BOUNCE% --dry-run %REST%
exit /b
:bounce
%DEPTH% drone.patrol.bounce %BOUNCE% --speed 0.3 %REST%
exit /b
:dvd-dry
%DEPTH% drone.patrol.dvd %PATROL% --dry-run %REST%
exit /b
:dvd
%DEPTH% drone.patrol.dvd %PATROL% %REST%
exit /b
:clear-dry
%DEPTH% drone.patrol.clear %PATROL% --dry-run %REST%
exit /b
:clear
%DEPTH% drone.patrol.clear %PATROL% %REST%
exit /b
:sweep-dry
%DEPTH% drone.patrol.sweep %PATROL% --dry-run %REST%
exit /b
:sweep
%DEPTH% drone.patrol.sweep %PATROL% %REST%
exit /b
:model
%PY% --with numpy --with pillow --with torch --with transformers python -c "from drone.patrol.front_distance import DepthModel; DepthModel(); print('depth model ready')"
exit /b
