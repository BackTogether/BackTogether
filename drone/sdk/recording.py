"""Photos and MP4 recordings of the live video, on the laptop.

The MP4 is the H.264 we already receive, remuxed by ffmpeg (`-c copy`, no re-encode, so no
quality loss and almost no CPU). It starts at the next parameter set (the drone sends one with
every keyframe, about once a second) so the file opens on a decodable frame.
"""

import shutil
import subprocess
import threading
import time
from pathlib import Path

from drone.hoverctl.video_payload import find_parameter_set

RECORDINGS_DIR = Path("recordings")  # gitignored
RECORD_FPS = 30  # the drone's live stream; raw H.264 carries no timestamps
FINALIZE_TIMEOUT_S = 5.0


def timestamped_path(prefix: str, suffix: str) -> Path:
    """recordings/<prefix>_<date>-<time>_<ms><suffix>, folder created."""
    now = time.time()
    millis = int(now * 1000) % 1000
    name = f"{prefix}_{time.strftime('%Y%m%d-%H%M%S', time.localtime(now))}_{millis:03d}{suffix}"
    RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
    return RECORDINGS_DIR / name


def save_jpeg(jpeg: bytes, path: Path | None = None) -> Path:
    """Write a JPEG frame; default path recordings/photo_<time>.jpg."""
    target = path or timestamped_path("photo", ".jpg")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(jpeg)
    return target


class Mp4Recorder:
    """Feed Annex-B bytes (VideoReceiver stream listener); stop() finalizes the MP4."""

    def __init__(self, path: Path | None = None, fps: int = RECORD_FPS) -> None:
        """Raises RuntimeError without the ffmpeg CLI."""
        if not shutil.which("ffmpeg"):
            raise RuntimeError("ffmpeg not found: install it (brew / apt / winget install ffmpeg)")
        self.path = path or timestamped_path("video", ".mp4")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.fps = fps
        self.started_at = time.time()
        self.bytes = 0
        self.error: str | None = None
        self._process: subprocess.Popen | None = None
        self._closed = False
        self._lock = threading.Lock()

    @property
    def writing(self) -> bool:
        """True once the first parameter set arrived and ffmpeg is writing."""
        return self._process is not None

    def feed(self, data: bytes) -> None:
        with self._lock:
            if self._closed or self.error:
                return
            if self._process is None:
                found = find_parameter_set(data)
                if found is None:
                    return  # wait for a keyframe's parameter set
                offset, codec = found
                data = data[offset:]
                self._process = subprocess.Popen(
                    self._command(codec), stdin=subprocess.PIPE, stderr=subprocess.PIPE
                )
            try:
                self._process.stdin.write(data)
                self.bytes += len(data)
            except (OSError, ValueError) as error:  # ffmpeg died: keep the error, stop feeding
                self.error = f"ffmpeg: {error}"

    def stop(self) -> Path | None:
        """Close the file properly; the path, or None if nothing was recorded."""
        with self._lock:
            self._closed = True
            process, self._process = self._process, None
        if process is None:
            return None
        try:  # closes stdin: ffmpeg writes the MP4 index and exits
            _, stderr = process.communicate(timeout=FINALIZE_TIMEOUT_S)
        except (OSError, subprocess.TimeoutExpired) as error:
            process.kill()
            self.error = f"ffmpeg did not finish: {error}"
            return None
        if process.returncode != 0:
            self.error = stderr.decode(errors="replace").strip() or f"ffmpeg {process.returncode}"
            return None
        return self.path

    def _command(self, codec: str) -> list[str]:
        return [
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-fflags", "+genpts", "-f", codec, "-framerate", str(self.fps), "-i", "pipe:0",
            "-c", "copy", str(self.path),
        ]  # fmt: skip
