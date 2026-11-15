@echo off
rem Windows twin of drone/inbox (keep the two in step): save the web page photos where
rem drone\find reads faces, drone\identify\people\<name>\. Start it before the parent sends the
rem request, then: drone\find <name>.
rem Usage: drone\inbox
setlocal
cd /d "%~dp0.."
set PYTHONUTF8=1
uv run --no-project --python 3.11 python scripts\web_inbox.py --out drone\identify\people %*
