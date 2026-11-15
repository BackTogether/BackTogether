@echo off
rem One-time setup on Windows, WITH INTERNET (the drone's Wi-Fi has none): checks uv and ffmpeg,
rem then downloads what the launchers run offline afterwards: Python 3.11, the packages of
rem drone\fly, drone\find / land / dock and drone\patrol\live, and the depth model (about 1 GB).
rem Usage: drone\setup
setlocal
cd /d "%~dp0.."
where uv >nul 2>nul || goto no_uv
where ffmpeg >nul 2>nul || goto no_ffmpeg
set DRONE_ONLINE=1
set HF_HUB_OFFLINE=0
echo [1/4] drone\fly: Python 3.11, bleak, numpy
call drone\fly.cmd 100 sd --help >nul || goto failed
echo [2/4] drone\find, land, dock: OpenCV
call drone\find.cmd --help >nul || goto failed
echo [3/4] drone\patrol\live: torch, transformers (large)
call drone\patrol\live.cmd sweep --help >nul || goto failed
echo [4/4] depth model
call drone\patrol\live.cmd model || goto failed
echo Ready. Next: drone\fly, then http://127.0.0.1:8077 (drone\sdk\README.md, "Windows").
exit /b 0

:no_uv
echo uv not found. Install it, open a new terminal and run drone\setup again:
echo   powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
exit /b 1
:no_ffmpeg
echo ffmpeg not found. Install it, open a new terminal and run drone\setup again:
echo   winget install Gyan.FFmpeg
exit /b 1
:failed
echo Setup failed: see the error above (is there internet?).
exit /b 1
