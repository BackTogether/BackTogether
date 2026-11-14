"""H.264/H.265 Annex-B stream -> JPEG frames, using the ffmpeg CLI as a subprocess.

Optionally the same ffmpeg also writes a small grayscale copy of every frame (raw bytes) to a
second pipe, for image processing (drone/sdk/flow.py): one decode, no JPEG round trip.
"""

import contextlib
import os
import shutil
import struct
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from collections.abc import Callable

from .video_payload import PictureCounter, find_parameter_set

JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"
JPEG_QUALITY = "5"  # ffmpeg -q:v, 2 (best) .. 31
READ_CHUNK = 1 << 16
MAX_PENDING_BYTES = 4 << 20  # stream kept while waiting for the first parameter set
RESTART_DELAY_S = 2.0
CLOSE_TIMEOUT_S = 1.0
FPS_WINDOW_S = 2.0  # also the window of the decode lag average
# More pictures than this inside ffmpeg means it dropped some (undecodable): the lag matching
# is lost, so it starts over at the next keyframe
RESYNC_QUEUED = 60
# JPEG start-of-frame markers (baseline, extended, progressive) hold the frame size
JPEG_SOF_MARKERS = (b"\xff\xc0", b"\xff\xc1", b"\xff\xc2")
JPEG_SOF_SIZE = struct.Struct(">xxxHH")  # length (2), precision (1), height, width


# (raw frame bytes, unix time it was read): runs on the reader thread, must be quick
GrayCallback = Callable[[bytes, float], None]


def _gray_output(size: tuple[int, int], fd: int) -> list[str]:
    """A second output: every frame scaled to `size`, 8-bit gray, raw, to file descriptor fd."""
    width, height = size
    return [
        "-vf",
        f"scale={width}:{height},format=gray",
        "-f",
        "rawvideo",
        "-flush_packets",
        "1",
        f"pipe:{fd}",
    ]


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
        # One encoder thread: ffmpeg frame-threads the JPEG encoder on many-core machines, and
        # that holds 4 frames (measured on a replayed flight: 135 -> 41 ms, 7 ms with the
        # access unit delimiters of video_payload.py). One thread encodes 720p30 in ~15 % of a core.
        "-threads",
        "1",
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
        # Decode lag: arrival time of every picture written to ffmpeg, matched in order with
        # the frames it publishes (one frame per picture: no B-frames, low_delay)
        self._pictures: PictureCounter | None = None
        self._arrivals: deque[float] = deque()
        self._lags: deque[tuple[float, float]] = deque()  # (published at, lag s)
        self._pending = bytearray()
        self._process: subprocess.Popen | None = None
        self._last_start = 0.0
        self._frame = b""
        self._frame_ready = threading.Condition()
        self._restart_wanted = False  # set by restart(), done in feed()
        self._gray: tuple[tuple[int, int], GrayCallback] | None = None

    def set_gray_output(self, width: int, height: int, callback: GrayCallback) -> bool:
        """Also get every frame as `width` x `height` 8-bit gray bytes: `callback(frame, time)`
        on a reader thread. Takes effect at the next (re)start of ffmpeg. False where a second
        pipe cannot be passed to ffmpeg (Windows)."""
        if os.name != "posix":
            return False
        self._gray = ((width, height), callback)
        if self._process is not None:
            self.restart()
        return True

    def feed(self, data: bytes) -> None:
        if not data:
            return
        if self._restart_wanted:
            data = self._restart_at_parameter_set(data)
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
        arrived = time.time()
        if self._pictures is not None:
            if len(self._arrivals) > RESYNC_QUEUED:
                self._arrivals.clear()
                self._pictures.wait_for_keyframe()
            self._arrivals.extend([arrived] * self._pictures.feed(data))
        try:
            process.stdin.write(data)
            process.stdin.flush()
        except (OSError, ValueError):  # broken pipe, or stdin closed by restart()
            self._process = None

    @property
    def fps(self) -> float:
        """Decoded frames per second over the last FPS_WINDOW_S."""
        with self._frame_ready:
            self._drop_old_frame_times(time.time())
            return len(self._frame_times) / FPS_WINDOW_S

    @property
    def lag_ms(self) -> float | None:
        """Mean time from a picture's arrival to its decoded frame over the last FPS_WINDOW_S;
        None without recent frames."""
        with self._frame_ready:
            self._drop_old_frame_times(time.time())
            if not self._lags:
                return None
            return 1000 * sum(lag for _, lag in self._lags) / len(self._lags)

    @property
    def queued(self) -> int:
        """Pictures written to ffmpeg and not decoded yet."""
        return len(self._arrivals)

    def restart(self) -> None:
        """Start a fresh ffmpeg at the next parameter set, e.g. after asking the drone for another
        resolution: one ffmpeg process keeps the frame size it started with.

        Safe from any thread: it only sets a flag. The thread that calls `feed` does the restart,
        so ffmpeg is never closed while it is being written to, and the new one starts on an
        SPS/VPS (the drone sends one with every keyframe, about once a second).
        """
        self._restart_wanted = True

    def _restart_at_parameter_set(self, data: bytes) -> bytes:
        """Keep the old decoder until a parameter set arrives, then swap. Returns what to feed."""
        found = find_parameter_set(data)
        if found is None:
            return data  # the old decoder keeps showing frames until the swap
        offset, codec = found
        self._restart_wanted = False
        self.close()
        self.codec = self._configured_codec or codec
        self._pending = bytearray()
        self._last_start = 0.0
        return data[offset:]

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
        found = find_parameter_set(bytes(self._pending))
        if found:
            # Start ffmpeg on the parameter set: with the short probe, slices before it leave
            # ffmpeg without a frame size, and it never outputs a frame
            offset, self.codec = found
            pending, self._pending = bytes(self._pending[offset:]), bytearray()
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
        self._pictures = PictureCounter(self.codec)
        self._arrivals.clear()
        command = _ffmpeg_command(self.codec)
        if self._gray is None:
            self._process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        else:
            self._start_with_gray(command, *self._gray)
        threading.Thread(target=self._read_frames, args=(self._process,), daemon=True).start()
        return True

    def _start_with_gray(
        self, command: list[str], size: tuple[int, int], callback: GrayCallback
    ) -> None:
        read_end, write_end = os.pipe()
        try:
            self._process = subprocess.Popen(
                [*command, *_gray_output(size, write_end)],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                pass_fds=(write_end,),
            )
        finally:
            os.close(write_end)  # ffmpeg holds its own copy: EOF reaches us when it exits
        frame_bytes = size[0] * size[1]
        threading.Thread(
            target=_read_gray, args=(read_end, frame_bytes, callback), daemon=True
        ).start()

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
        now = time.time()
        with self._frame_ready:
            self._frame = jpeg
            self.frames += 1
            self.size = _jpeg_size(jpeg) or self.size
            self._frame_times.append(now)
            with contextlib.suppress(IndexError):  # the feed thread may have just cleared it
                self._lags.append((now, now - self._arrivals.popleft()))
            self._drop_old_frame_times(now)
            self._frame_ready.notify_all()

    def _drop_old_frame_times(self, now: float) -> None:
        while self._frame_times and now - self._frame_times[0] > FPS_WINDOW_S:
            self._frame_times.popleft()
        while self._lags and now - self._lags[0][0] > FPS_WINDOW_S:
            self._lags.popleft()


def _read_gray(fd: int, frame_bytes: int, callback: GrayCallback) -> None:
    """Cut the raw gray pipe into frames until ffmpeg exits. ffmpeg blocks (and with it the
    JPEG output) while this pipe is full, so the callback must stay quick."""
    buffer = bytearray()
    with os.fdopen(fd, "rb", buffering=0) as pipe:
        while chunk := pipe.read(READ_CHUNK):
            buffer += chunk
            while len(buffer) >= frame_bytes:
                frame = bytes(buffer[:frame_bytes])
                del buffer[:frame_bytes]
                try:
                    callback(frame, time.time())
                except Exception:  # a bad frame handler must not stop the video
                    print(f"decoder: gray frame callback:\n{traceback.format_exc()}", flush=True)


def _jpeg_size(jpeg: bytes) -> tuple[int, int] | None:
    """(width, height) from the JPEG start-of-frame header."""
    for marker in JPEG_SOF_MARKERS:
        offset = jpeg.find(marker)
        if offset >= 0 and offset + 2 + JPEG_SOF_SIZE.size <= len(jpeg):
            height, width = JPEG_SOF_SIZE.unpack_from(jpeg, offset + 2)
            return width, height
    return None
