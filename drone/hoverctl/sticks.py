"""30 Hz stick sender with the app's ramp and a neutral-on-silence watchdog."""

import socket
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import asdict, astuple

from .protocol import AXIS_LIMIT, NEUTRAL, Axes, encode_rocker_command, to_packet

DRONE_HOST = "<DRONE_IP>"
STICK_PORT = 19091
TICK_S = 0.033  # the app sends every 33 ms
DEFAULT_MAX_MAGNITUDE = 50.0
# Same ramp as the app: first non-zero tick jumps to 28, then +3 per tick.
RAMP_START = 28.0
RAMP_STEP = 3.0
INPUT_TIMEOUT_S = 0.3  # no fresh input for this long -> neutral
EXIT_NEUTRAL_S = 0.5


def ramp_axis(current: float, target: float) -> float:
    """Move one axis toward `target`; reducing or releasing is immediate, speeding up is not."""
    if not target:
        return 0.0
    same_sign = current * target > 0
    if not same_sign:
        return min(abs(target), RAMP_START) * (1 if target > 0 else -1)
    if abs(target) <= abs(current):
        return target
    step = min(RAMP_STEP, abs(target) - abs(current))
    return current + step * (1 if target > 0 else -1)


def ramp(current: Axes, target: Axes) -> Axes:
    return Axes(*(ramp_axis(c, t) for c, t in zip(astuple(current), astuple(target), strict=True)))


class StickSender:
    """Sends `RockerControlCommand` packets every tick from a background thread.

    Inputs are -1..1 per axis with our signs (`Axes`), scaled to `max_magnitude`; the encoder
    converts them to the packet's signs. If no input arrives for INPUT_TIMEOUT_S the target
    becomes neutral. `stop()` sends EXIT_NEUTRAL_S of neutral.
    `gate` is the stick interlock: while it returns False (or raises) the sticks are neutral and
    `locked` is set. Without a session gate nothing is sent at all while locked, not even neutral.
    With one (`set_session_gate`) the rule is: no open session, nothing is sent; session open,
    neutral is sent all the time (the app streams neutral on its joystick screen, and the drone
    may need that stream to stay in MANUAL) and values only while `gate` passes. In dry-run
    packets are still printed.
    """

    def __init__(
        self,
        host: str = DRONE_HOST,
        port: int = STICK_PORT,
        local_port: int = STICK_PORT,
        max_magnitude: float = DEFAULT_MAX_MAGNITUDE,
        dry_run: bool = False,
        gate: Callable[[], bool] | None = None,
    ) -> None:
        self.address = (host, port)
        self.max_magnitude = min(abs(max_magnitude), AXIS_LIMIT)
        self.dry_run = dry_run
        self.locked = False
        self.silent = gate is not None  # locked and no session: not even neutral goes out
        self.interlocked = gate is not None
        self._gate = gate
        self._session: Callable[[], bool] | None = None
        self.output = NEUTRAL
        self.sent = 0
        self.errors = 0
        self.last_error = ""
        self._input = NEUTRAL
        self._input_time = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="sticks", daemon=True)
        self._socket: socket.socket | None = None
        if not dry_run:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._socket.bind(("", local_port))

    def start(self) -> None:
        self._thread.start()

    def set_session_gate(self, session: Callable[[], bool]) -> None:
        """While `session()` is True a locked sender still sends neutral packets."""
        self._session = session

    def set_input(self, axes: Axes) -> None:
        with self._lock:
            self._input = axes
            self._input_time = time.monotonic()

    def stop(self) -> None:
        """Stop the loop after sending neutral for EXIT_NEUTRAL_S. Safe to call twice."""
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join()
        if self._socket:
            self._socket.close()
            self._socket = None

    def _target(self) -> Axes:
        with self._lock:
            if time.monotonic() - self._input_time > INPUT_TIMEOUT_S:
                return NEUTRAL
            return self._input.scaled(self.max_magnitude, self.max_magnitude)

    def _run(self) -> None:
        self._every_tick(lambda: not self._stop.is_set(), self._tick)
        deadline = time.monotonic() + EXIT_NEUTRAL_S
        self._every_tick(lambda: time.monotonic() < deadline, self._send_neutral)

    def _every_tick(self, keep_going: Callable[[], bool], action: Callable[[], None]) -> None:
        next_tick = time.monotonic()
        while keep_going():
            try:
                action()
            except Exception as error:  # a bug must never leave the last stick value latched
                self.output = NEUTRAL
                self._note_error(error)
            next_tick += TICK_S
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    def _tick(self) -> None:
        self._read_gates()
        self.output = NEUTRAL if self.locked else ramp(self.output, self._target())
        self._send(self.output)

    def _send_neutral(self) -> None:
        self._read_gates()
        self.output = NEUTRAL
        self._send(NEUTRAL)

    def _read_gates(self) -> None:
        gate_open = self._passes(self._gate, default=True)
        if self._session is None:
            self.locked = self.silent = not gate_open
            return
        session_open = self._passes(self._session, default=False)
        self.locked = not (gate_open and session_open)
        self.silent = not session_open

    def _passes(self, check: Callable[[], bool] | None, default: bool) -> bool:
        if check is None:
            return default
        try:
            return check()
        except Exception as error:  # an interlock that fails stays closed
            self._note_error(error)
            return False

    def _send(self, axes: Axes) -> None:
        if self.silent and not self.dry_run:
            return
        packet = encode_rocker_command(self.sent, int(time.time() * 1000), axes)
        self.sent += 1
        if self.dry_run:
            wire = asdict(to_packet(axes))
            values = " ".join(f"{name}={value:+4.0f}" for name, value in wire.items())
            print(f"seq={self.sent - 1:<6} packet {values}  {packet.hex()}", flush=True)
            return
        try:
            self._socket.sendto(packet, self.address)
        except OSError as error:  # e.g. not on the drone's Wi-Fi yet: keep trying
            self._note_error(error)

    def _note_error(self, error: Exception) -> None:
        self.errors += 1
        message = f"{type(error).__name__}: {error}"
        if message != self.last_error:
            print(f"\nstick sender: {message}", file=sys.stderr, flush=True)
        self.last_error = message
