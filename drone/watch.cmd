@echo off
rem Windows twin of drone/watch (keep the two in step): fly by hand from the page while the
rem child is recognised and tracked. From each takeoff to its landing: the camera video, the
rem video with the boxes (recordings\boxes-<child>-<time>.mp4) and frames for the body model
rem (recordings\collect-<time>\). Sends no flight command.
rem Usage: drone\watch [child, default jorge] [more drone\find options]
setlocal
set "CHILD=jorge"
set "FIRST=%~1"
if defined FIRST if not "%FIRST:~0,1%"=="-" (
  set "CHILD=%~1"
  shift
)
set "REST="
:rest
if "%~1"=="" goto run
set "REST=%REST% %1"
shift
goto rest
:run
call "%~dp0find.cmd" %CHILD% --watch --collect%REST%
