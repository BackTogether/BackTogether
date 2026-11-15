@echo off
rem Windows twin of drone/fly (keep the two in step): start the drone web app with Bluetooth,
rem then open http://127.0.0.1:8077/ in one browser tab. Close the Hover app first.
rem Usage: drone\fly [max stick speed 0-100, default 100] [unused, kept for old commands]
rem                  [options of `python -m drone.sdk serve`, e.g. --fence 15, --bind 0.0.0.0]
rem First time, with internet: drone\setup
setlocal
cd /d "%~dp0.."
rem UTF-8 for the page, the logs and the console, whatever the Windows code page is
set PYTHONUTF8=1
rem Offline like the bash launchers (the drone's Wi-Fi has no internet); drone\setup turns it off
set "OFFLINE=--offline"
if defined DRONE_ONLINE set "OFFLINE="
if not exist recordings mkdir recordings
rem PHOTO / REC from the page land here
if not defined DRONE_MEDIA_DIR set "DRONE_MEDIA_DIR=%USERPROFILE%\Downloads"
set "MAX=%~1"
if not defined MAX set MAX=100
for /f %%t in ('powershell -NoProfile -Command "Get-Date -Format HHmm"') do set "STAMP=%%t"
rem the serve options: everything from the third argument on
set "REST="
shift
shift
:rest
if "%~1"=="" goto run
set REST=%REST% %1
shift
goto rest
:run
rem --no-project: the link needs bleak and numpy only, not the project's simulator packages
uv run %OFFLINE% --no-project --python 3.11 --with bleak --with numpy python -m drone.sdk serve ^
    --http-port 8077 --no-browser --max %MAX% --video-quality sd ^
    --stick-log "recordings\sticks-%STAMP%.jsonl" %REST%
