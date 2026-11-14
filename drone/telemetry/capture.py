"""Record the Hover app's drone RPC log from the phone, with secrets redacted.

The app logs every RPC it exchanges with the drone to logcat as protobuf text (tag
mxlogger_info). It also logs the account password in plain text, so every line is
redacted before it touches the disk.

Run from the repo root: uv run python -m drone.telemetry.capture
Stop with Ctrl+C. Survives USB drops: it waits for the phone and resumes.
"""

import os
import platform
import re
import subprocess
import time
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

CAPTURES_DIR = Path(__file__).parent / "captures"
APP_LOG_TAG = "mxlogger_info"
SECRET_FIELD = re.compile(
    r"""(?ix)
    (\b\w*(?:passw|pwd|token|secret)\w*\b["']?\s*[:=]\s*)  # key, then : or =
    ("[^"]*"|'[^']*'|[^\s,}]+)                             # value
    """
)
APP_TAG = re.compile(r"CUSTOM (\w+)")
TOP_TAGS = 25


@dataclass
class CaptureStats:
    redactions: int = 0
    tags: Counter = field(default_factory=Counter)


def is_wsl() -> bool:
    return "microsoft" in platform.release().lower()


def adb_binary() -> str:
    """Windows adb.exe under WSL2 (USB is not visible there), plain adb elsewhere."""
    if os.environ.get("ADB"):
        return os.environ["ADB"]
    return "adb.exe" if is_wsl() else "adb"


def redact(line: str) -> tuple[str, int]:
    return SECRET_FIELD.subn(r'\1"<redacted>"', line)


def capture(adb: str, out_path: Path, stats: CaptureStats) -> None:
    """Stream logcat into out_path until the phone disconnects, updating stats as it goes."""
    with (
        subprocess.Popen(
            [adb, "logcat", "-v", "epoch", "-T", "1"],
            stdout=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
        ) as proc,
        out_path.open("a", encoding="utf-8") as out,
    ):
        for raw in proc.stdout:
            if APP_LOG_TAG not in raw:
                continue
            line, hits = redact(raw)
            stats.redactions += hits
            stats.tags.update(APP_TAG.findall(line))
            out.write(line)


def capture_until_interrupted(adb: str, out_path: Path) -> CaptureStats:
    """Capture into out_path, reconnecting after USB drops, until Ctrl+C."""
    stats = CaptureStats()
    try:
        while True:
            print("Waiting for phone...", flush=True)
            subprocess.run([adb, "wait-for-device"], check=True)
            print(f"{time.strftime('%H:%M:%S')} phone connected, recording", flush=True)
            capture(adb, out_path, stats)
            print(f"{time.strftime('%H:%M:%S')} phone DISCONNECTED", flush=True)
    except KeyboardInterrupt:
        pass
    return stats


def print_stats(out_path: Path, stats: CaptureStats) -> None:
    print(f"\nSaved {out_path}  ({stats.redactions} secret values redacted)")
    print("Most frequent app log sources:")
    for tag, count in stats.tags.most_common(TOP_TAGS):
        print(f"  {count:6d}  {tag}")


def main() -> None:
    adb = adb_binary()
    CAPTURES_DIR.mkdir(exist_ok=True)
    out_path = CAPTURES_DIR / f"applog_{time.strftime('%Y%m%d_%H%M%S')}.log"
    print(f"Writing {out_path}  (Ctrl+C to stop)")
    print_stats(out_path, capture_until_interrupted(adb, out_path))


if __name__ == "__main__":
    main()
