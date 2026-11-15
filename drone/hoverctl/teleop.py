"""Terminal keyboard teleop.

A terminal sends no key-up events, only the auto-repeat of the last key held. A key counts as
held until HOLD_S after its last repeat. The first press waits longer, because macOS waits
about 0.45 s (default InitialKeyRepeat; Windows 0.25-1 s, by its keyboard settings) before it
starts repeating. Only the last key held repeats, so the terminal can't do diagonal moves. The
web UI can.
"""

import contextlib
import os
import select
import sys
import time
from collections.abc import Callable, Iterator
from dataclasses import asdict, dataclass

from .protocol import AXIS_DIRECTIONS, AXIS_NAMES, Axes
from .sticks import StickSender

# key -> (axis, direction), with the signs of `Axes` (+ = fwd, right, up, turn right, camera up).
# Also used by the web UI.
KEY_BINDINGS: dict[str, tuple[str, int]] = {
    "w": ("pitch", 1),
    "s": ("pitch", -1),
    "a": ("roll", -1),
    "d": ("roll", 1),
    "i": ("thrust", 1),
    "k": ("thrust", -1),
    "j": ("yaw", -1),
    "l": ("yaw", 1),
    "u": ("gimbal", 1),
    "o": ("gimbal", -1),
}
NEUTRAL_KEY = " "
QUIT_KEY = "q"
HOLD_S = 0.15
FIRST_PRESS_HOLD_S = 0.55
POLL_S = 0.01
STATUS_PERIOD_S = 0.1
_WINDOWS_KEY_PREFIXES = ("\x00", "\xe0")  # msvcrt: the next character is a key code
HELP = (
    "W/S fwd/back  A/D left/right  I/K up/down  J/L turn left/right  U/O camera up/down\n"
    "SPACE neutral  Q quit (Ctrl-C works too)"
)


@dataclass
class _Press:
    first: float
    last: float

    def held(self, now: float) -> bool:
        window = FIRST_PRESS_HOLD_S if self.last == self.first else HOLD_S
        return now - self.last <= window


class HeldKeys:
    def __init__(self) -> None:
        self._presses: dict[str, _Press] = {}

    def press(self, key: str, now: float) -> None:
        press = self._presses.get(key)
        if press and press.held(now):
            press.last = now
        else:
            self._presses[key] = _Press(now, now)

    def clear(self) -> None:
        self._presses.clear()

    def axes(self, now: float) -> Axes:
        totals = dict.fromkeys(AXIS_NAMES, 0.0)
        for key, press in self._presses.items():
            if press.held(now):
                axis, direction = KEY_BINDINGS[key]
                totals[axis] += direction
        return Axes(**{axis: max(-1.0, min(1.0, value)) for axis, value in totals.items()})


def _status_line(sender: StickSender) -> str:
    axes = "  ".join(
        f"{AXIS_DIRECTIONS[name][0]} {value:+5.0f}" for name, value in asdict(sender.output).items()
    )
    return f"\r{axes}  sent {sender.sent}  errors {sender.errors} "


def _read_keys(fd: int) -> str:
    ready, _, _ = select.select([fd], [], [], POLL_S)
    return os.read(fd, 64).decode(errors="ignore").lower() if ready else ""


def _read_keys_windows() -> str:
    import msvcrt

    keys = ""
    while msvcrt.kbhit():
        key = msvcrt.getwch()
        if key in _WINDOWS_KEY_PREFIXES:
            msvcrt.getwch()  # an arrow or function key: its code is a letter, not a key we bind
        else:
            keys += key
    if not keys:
        time.sleep(POLL_S)
    return keys.lower()


@contextlib.contextmanager
def _key_reader() -> Iterator[Callable[[], str]]:
    """A function returning the keys typed since its last call (waits up to POLL_S for one)."""
    if sys.platform == "win32":  # the console gives single keys as it is: nothing to restore
        yield _read_keys_windows
        return
    # Unix-only modules, imported here so the web app (which imports this file) runs on Windows.
    import termios
    import tty

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield lambda: _read_keys(fd)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


def run_teleop(sender: StickSender) -> None:
    """Read keys until Q. The caller owns `sender` and must stop it (which sends neutral)."""
    if not sys.stdin.isatty():
        raise SystemExit("sticks needs an interactive terminal")
    print(HELP)
    with _key_reader() as read_keys:
        try:
            _fly_keys(sender, read_keys)
        finally:
            print()  # leave the status line, which is redrawn in place


def _fly_keys(sender: StickSender, read_keys: Callable[[], str]) -> None:
    held = HeldKeys()
    next_status = 0.0
    while True:
        now = time.monotonic()
        for key in read_keys():
            if key == QUIT_KEY:
                return
            if key == NEUTRAL_KEY:
                held.clear()
            elif key in KEY_BINDINGS:
                held.press(key, now)
        sender.set_input(held.axes(time.monotonic()))
        if not sender.dry_run and now >= next_status:
            print(_status_line(sender), end="", flush=True)
            next_status = now + STATUS_PERIOD_S
