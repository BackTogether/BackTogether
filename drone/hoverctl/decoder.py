"""H.264/H.265 Annex-B stream -> JPEG frames, using the ffmpeg CLI as a subprocess."""

import os
import shutil
import struct
import subprocess
import sys
import threading
import time
from collections import deque

from .video_payload import detect_codec

JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"
JPEG_QUALITY = "5"  # ffmpeg -q:v, 2 (best) .. 31
READ_CHUNK = 1 << 16
MAX_PENDING_BYTES = 4 << 20  # stream kept while waiting for the first parameter set
RESTART_DELAY_S = 2.0
CLOSE_TIMEOUT_S = 1.0
FPS_WINDOW_S = 2.0
# JPEG start-of-frame markers (baseline, extended, progressive) hold the frame size
JPEG_SOF_MARKERS = (b"\xff\xc0", b"\xff\xc1", b"\xff\xc2")
JPEG_SOF_SIZE = struct.Struct(">xxxHH")  # length (2), precision (1), height, width


def _ffmpeg_command(codec: str) -> list[str]:
    return [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-flags",
        "low_delay",
        # Probe as little as possible. Not `-fflags nobuffer`: it drops the probed packets,
        # which hold the SPS/PPS and first keyframe, so a whole GOP is lost.
        "-probesize",
        "32",
        "-analyzeduration",
        "0",
        "-f",
        codec,  # raw demuxer named like the codec: h264 or hevc
        "-i",
        "pipe:0",
        "-f",
        "mjpeg",
        "-q:v",
        JPEG_QUALITY,
        "-flush_packets",
        "1",
        "pipe:1",
    ]


class MjpegDecoder:
    """Feed Annex-B bytes; read the latest JPEG with `wait_frame`.

    With `codec=None` the codec is detected from the first SPS/VPS, and data before it is kept
    (up to MAX_PENDING_BYTES) so the decoder starts on a parameter set.
    """

    def __init__(self, codec: str | None = None) -> None:
        if not shutil.which("ffmpeg"):
            raise SystemExit("ffmpeg not found: brew install ffmpeg (or use --no-decode)")
        self._configured_codec = codec
        self.codec = codec
        self.frames = 0
        self.size: tuple[int, int] | None = None  # (width, height) of the last frame
        self._frame_times: deque[float] = deque()
        self._pending = bytearray()
        self._process: subprocess.Popen | None = None
        self._last_start = 0.0
        self._frame = b""
        self._frame_ready = threading.Condition()

    def feed(self, data: bytes) -> None:
        if not data:
            return
        if self.codec is None:
            self._buffer_until_codec(data)
            return
        if not self._running() and not self._start():
            return
        process = self._process  # restart() may clear it from another thread
        if process is None:
            return
        try:
            process.stdin.write(data)
            process.stdin.flush()
        except (OSError, ValueError):  # broken pipe, or stdin closed by restart()
            self._process = None

    @property
    def fps(self) -> float:
        """Decoded frames per second over the last FPS_WINDOW_S."""
        with self._frame_ready:
            self._drop_old_frame_times(time.monotonic())
            return len(self._frame_times) / FPS_WINDOW_S

    def restart(self) -> None:
        """Start over at the next parameter set, e.g. after the drone changed resolution: one
        ffmpeg process keeps the frame size it started with."""
        self.close()
        self.codec = self._configured_codec
        self._pending = bytearray()
        self._last_start = 0.0

    def wait_frame(self, after: int, timeout: float) -> tuple[int, bytes]:
        """Block until a frame newer than number `after` exists (or timeout)."""
        with self._frame_ready:
            self._frame_ready.wait_for(lambda: self.frames > after, timeout)
            return self.frames, self._frame

    def close(self) -> None:
        """Let ffmpeg finish on end of input (killing it mid-write spams broken-pipe errors)."""
        if not self._process:
            return
        try:
            self._process.stdin.close()
            self._process.wait(CLOSE_TIMEOUT_S)
        except (BrokenPipeError, subprocess.TimeoutExpired):
            self._process.kill()
        self._process = None

    def _buffer_until_codec(self, data: bytes) -> None:
        self._pending += data
        self.codec = detect_codec(bytes(self._pending))
        if self.codec:
            pending, self._pending = bytes(self._pending), bytearray()
            self.feed(pending)
        elif len(self._pending) > MAX_PENDING_BYTES:
            del self._pending[: len(self._pending) - MAX_PENDING_BYTES]

    def _running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def _start(self) -> bool:
        if time.monotonic() - self._last_start < RESTART_DELAY_S:
            return False
        self._last_start = time.monotonic()
        print(f"decoder: starting ffmpeg ({self.codec})", file=sys.stderr)
        self._process = subprocess.Popen(
            _ffmpeg_command(self.codec), stdin=subprocess.PIPE, stdout=subprocess.PIPE
        )
        threading.Thread(target=self._read_frames, args=(self._process,), daemon=True).start()
        return True

    def _read_frames(self, process: subprocess.Popen) -> None:
        buffer = b""
        while chunk := os.read(process.stdout.fileno(), READ_CHUNK):
            buffer += chunk
            while (start := buffer.find(JPEG_SOI)) >= 0:
                end = buffer.find(JPEG_EOI, start + len(JPEG_SOI))
                if end < 0:
                    break
                self._publish(buffer[start : end + len(JPEG_EOI)])
                buffer = buffer[end + len(JPEG_EOI) :]
            if JPEG_SOI not in buffer:
                buffer = buffer[-1:]  # a marker may be split across reads

    def _publish(self, jpeg: bytes) -> None:
        now = time.monotonic()
        with self._frame_ready:
            self._frame = jpeg
            self.frames += 1
            self.size = _jpeg_size(jpeg) or self.size
            self._frame_times.append(now)
            self._drop_old_frame_times(now)
            self._frame_ready.notify_all()

    def _drop_old_frame_times(self, now: float) -> None:
        while self._frame_times and now - self._frame_times[0] > FPS_WINDOW_S:
            self._frame_times.popleft()


def _jpeg_size(jpeg: bytes) -> tuple[int, int] | None:
    """(width, height) from the JPEG start-of-frame header."""
    for marker in JPEG_SOF_MARKERS:
        offset = jpeg.find(marker)
        if offset >= 0 and offset + 2 + JPEG_SOF_SIZE.size <= len(jpeg):
            height, width = JPEG_SOF_SIZE.unpack_from(jpeg, offset + 2)
            return width, height
    return None
