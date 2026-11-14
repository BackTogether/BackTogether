"""H.264/H.265 Annex-B stream -> JPEG frames, using the ffmpeg CLI as a subprocess."""

import os
import shutil
import subprocess
import sys
import threading
import time

from .video_payload import detect_codec

JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"
JPEG_QUALITY = "5"  # ffmpeg -q:v, 2 (best) .. 31
READ_CHUNK = 1 << 16
MAX_PENDING_BYTES = 4 << 20  # stream kept while waiting for the first parameter set
RESTART_DELAY_S = 2.0


def _ffmpeg_command(codec: str) -> list[str]:
    return [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-fflags",
        "nobuffer",
        "-flags",
        "low_delay",
        "-probesize",
        "32768",
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
        self.codec = codec
        self.frames = 0
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
        try:
            self._process.stdin.write(data)
            self._process.stdin.flush()
        except (BrokenPipeError, ValueError):
            self._process = None

    def wait_frame(self, after: int, timeout: float) -> tuple[int, bytes]:
        """Block until a frame newer than number `after` exists (or timeout)."""
        with self._frame_ready:
            self._frame_ready.wait_for(lambda: self.frames > after, timeout)
            return self.frames, self._frame

    def close(self) -> None:
        if self._process:
            self._process.stdin.close()
            self._process.terminate()
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
        with self._frame_ready:
            self._frame = jpeg
            self.frames += 1
            self._frame_ready.notify_all()
