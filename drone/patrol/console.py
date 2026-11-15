"""Live terminal panel for the patrols: stdout is redrawn in place as the last few lines.

A patrol prints several lines a second; a scrolling terminal is unreadable in flight. In a
terminal, `install()` keeps only the newest PANEL_LINES lines on screen (redrawn in place with
ANSI codes) and writes everything to recordings/<name>_<time>.log. When stdout is not a
terminal (piped, tests, Docker logs) nothing changes. PATROL_PANEL_LINES=0 turns it off.
"""

import atexit
import os
import shutil
import sys
import time
from collections import deque
from pathlib import Path
from typing import TextIO

PANEL_LINES = int(os.environ.get("PATROL_PANEL_LINES", "10"))
LOG_DIR = Path("recordings")
CURSOR_UP_LINES = "\x1b[{}F"  # to the start of the line N lines up
CLEAR_TO_END = "\x1b[J"


class LivePanel:
    """A stdout replacement: complete lines go to the log and to a redrawn panel."""

    def __init__(self, stream: TextIO, lines: int, log: TextIO) -> None:
        self._stream = stream
        self._lines: deque[str] = deque(maxlen=lines)
        self._log = log
        self._shown = 0  # lines currently drawn, to move back over them
        self._partial = ""

    def write(self, text: str) -> int:
        self._log.write(text)
        self._partial += text
        *complete, self._partial = self._partial.split("\n")
        if complete:
            self._lines.extend(line for line in complete if line.strip())
            self._redraw()
        return len(text)

    def flush(self) -> None:
        self._log.flush()
        self._stream.flush()

    def isatty(self) -> bool:
        return True

    def _redraw(self) -> None:
        width = max(20, shutil.get_terminal_size().columns - 1)  # no wrapping: keeps the count
        up = CURSOR_UP_LINES.format(self._shown) if self._shown else ""
        body = "".join(line[:width] + "\n" for line in self._lines)
        self._stream.write(up + CLEAR_TO_END + body)
        self._stream.flush()
        self._shown = len(self._lines)


def install(name: str) -> Path | None:
    """Turn stdout into a PANEL_LINES-line live panel and log it all; the log path, or None."""
    if PANEL_LINES <= 0 or not sys.stdout.isatty():
        return None
    if sys.platform == "win32":
        os.system("")  # the old Windows console reads ANSI codes only after this
    LOG_DIR.mkdir(exist_ok=True)
    path = LOG_DIR / f"{name}_{time.strftime('%Y%m%d-%H%M%S')}.log"
    log = path.open("w", buffering=1)
    sys.__stdout__.write(f"full log: {path}  (last {PANEL_LINES} lines below)\n")  # fixed header
    sys.stdout = LivePanel(sys.__stdout__, PANEL_LINES, log)
    atexit.register(_restore, log)
    return path


def _restore(log: TextIO) -> None:
    """At exit: the real stdout back first, so the final flush never hits the closed log."""
    sys.stdout = sys.__stdout__
    log.close()
