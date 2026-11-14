"""Record a drone session: the Hover app screen (video) and its telemetry, in sync.

The video is the phone screen recorded by scrcpy on the laptop (the app's live preview),
the telemetry is the app log captured by drone.telemetry.capture. Telemetry `t` is the
phone's clock, so the phone clock is read the moment scrcpy opens the video file (the
laptop clock is not used: under WSL2 it drifts about 3 ms/s against the phone).

Run from the repo root: uv run python -m drone.telemetry.record
Stop with Ctrl+C. Output in captures/session_<time>/:
  video.mkv        phone screen (mkv stays readable even if scrcpy is killed)
  applog.log       redacted app log
  telemetry.jsonl  parsed telemetry (see drone.telemetry.parse)
  sync.json        video time (s) = telemetry t - video_start_phone_epoch
"""

import json
import os
import shutil
import subprocess
import threading
import time
from pathlib import Path

from drone.telemetry.capture import (
    CAPTURES_DIR,
    adb_binary,
    capture_until_interrupted,
    is_wsl,
    print_stats,
)
from drone.telemetry.parse import parse_capture, print_summary, write_jsonl

# The phone shell may lack one of these; the first that prints a number wins.
PHONE_CLOCK_COMMANDS = ("echo $EPOCHREALTIME", "date +%s.%N")
RECORDING_STARTED = "Recording started"  # scrcpy log line once the file is open
SCRCPY_STOP_TIMEOUT_S = 5
WINDOW_TITLE = "Hover X1 recording"


def scrcpy_binary() -> str:
    if os.environ.get("SCRCPY"):
        return os.environ["SCRCPY"]
    return "scrcpy.exe" if is_wsl() else "scrcpy"


def host_path(path: Path) -> str:
    """Path as the scrcpy binary sees it: Windows exes under WSL need a Windows path."""
    if not is_wsl():
        return str(path)
    return subprocess.run(
        ["wslpath", "-w", str(path)], capture_output=True, text=True, check=True
    ).stdout.strip()


def phone_epoch(adb: str) -> float:
    for command in PHONE_CLOCK_COMMANDS:
        out = subprocess.run([adb, "shell", command], capture_output=True, text=True).stdout
        try:
            return float(out.strip())
        except ValueError:
            continue
    raise RuntimeError("Could not read the phone clock over adb")


class ScreenRecorder:
    """scrcpy recording of the phone screen; notes the phone time the file starts."""

    def __init__(self, adb: str, video_path: Path) -> None:
        self.started_phone_epoch: float | None = None
        self.clock_round_trip_s: float | None = None
        self._adb = adb
        self._proc = subprocess.Popen(
            [
                scrcpy_binary(),
                f"--record={host_path(video_path)}",
                "--no-audio",
                f"--window-title={WINDOW_TITLE}",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
        )
        threading.Thread(target=self._watch_log, daemon=True).start()

    def _watch_log(self) -> None:
        for line in self._proc.stdout:
            if self.started_phone_epoch is None and RECORDING_STARTED in line:
                self._read_start_time()
            print(f"[scrcpy] {line.rstrip()}", flush=True)

    def _read_start_time(self) -> None:
        """Phone clock at the moment the log line arrived, minus half the adb round trip."""
        seen = time.time()
        phone = phone_epoch(self._adb)
        replied = time.time()
        self.clock_round_trip_s = replied - seen
        self.started_phone_epoch = phone - self.clock_round_trip_s / 2

    def stop(self) -> None:
        """scrcpy gets Ctrl+C too and closes the file itself; kill it if it hangs."""
        try:
            self._proc.wait(timeout=SCRCPY_STOP_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            self._proc.terminate()
            self._proc.wait()


def write_sync(session: Path, recorder: ScreenRecorder) -> None:
    sync = {
        "video_file": "video.mkv",
        "video_start_phone_epoch": recorder.started_phone_epoch,
        "clock_round_trip_s": recorder.clock_round_trip_s,
        "note": "video time (s) = telemetry t - video_start_phone_epoch",
    }
    (session / "sync.json").write_text(json.dumps(sync, indent=2) + "\n")


def main() -> None:
    adb = adb_binary()
    if shutil.which(scrcpy_binary()) is None:
        raise SystemExit(f"'{scrcpy_binary()}' not found in PATH (set the SCRCPY env var)")
    session = CAPTURES_DIR / f"session_{time.strftime('%Y%m%d_%H%M%S')}"
    session.mkdir(parents=True)
    print(f"Session {session}  (Ctrl+C to stop)\nWaiting for phone...", flush=True)
    subprocess.run([adb, "wait-for-device"], check=True)

    recorder = ScreenRecorder(adb, session / "video.mkv")
    applog = session / "applog.log"
    stats = capture_until_interrupted(adb, applog)
    recorder.stop()

    write_sync(session, recorder)
    if recorder.started_phone_epoch is None:
        print("WARNING: scrcpy never reported 'Recording started': video missing or unsynced")
    print_stats(applog, stats)
    records = parse_capture(applog)
    write_jsonl(records, session / "telemetry.jsonl")
    print_summary(records, session / "telemetry.jsonl")


if __name__ == "__main__":
    main()
