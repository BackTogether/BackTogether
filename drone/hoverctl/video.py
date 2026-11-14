"""UDP 7078 video receiver: dump raw datagrams, keep stats and feed the decoder.

Dump format (read it back with `devsend --replay`): repeated records of
`<d I` (arrival unix time in seconds, payload length) followed by the payload.
"""

import socket
import struct
import threading
import time
from collections import deque
from pathlib import Path
from typing import BinaryIO

from .decoder import MjpegDecoder
from .video_payload import StreamExtractor, start_code_offset

VIDEO_PORT = 7078
RECV_BUFFER = 65535  # the app reads 2048, but take anything the drone sends
SOCKET_RCVBUF = 4 << 20
RECV_TIMEOUT_S = 0.5
HEX_PREVIEW_PACKETS = 10
HEX_PREVIEW_BYTES = 32
RATE_WINDOW_S = 2.0
DUMP_RECORD = struct.Struct("<dI")
DEFAULT_DUMP_DIR = Path("recordings")  # gitignored


def default_dump_path() -> Path:
    return DEFAULT_DUMP_DIR / time.strftime("hover_video_%Y%m%d-%H%M%S.udp")


class VideoReceiver:
    def __init__(
        self,
        port: int = VIDEO_PORT,
        dump_path: Path | None = None,
        skip: int = 0,
        decoder: MjpegDecoder | None = None,
    ) -> None:
        self.port = port
        self.dump_path = dump_path
        self.decoder = decoder
        self.extractor = StreamExtractor(skip)
        self.packets = 0
        self.bytes = 0
        self.senders: set[str] = set()
        self.previews: list[str] = []
        self._recent: deque[tuple[float, int]] = deque()
        self._stop = threading.Event()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, SOCKET_RCVBUF)
        self._socket.bind(("", port))
        self._socket.settimeout(RECV_TIMEOUT_S)
        self._thread = threading.Thread(target=self._run, name="video", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join()
        self._socket.close()
        if self.decoder:
            self.decoder.close()

    def stats(self) -> dict:
        now = time.monotonic()
        recent = [(t, n) for t, n in list(self._recent) if now - t <= RATE_WINDOW_S]
        return {
            "port": self.port,
            "packets": self.packets,
            "bytes": self.bytes,
            "pps": round(len(recent) / RATE_WINDOW_S, 1),
            "bytes_per_s": round(sum(n for _, n in recent) / RATE_WINDOW_S),
            "senders": sorted(self.senders),
            "kind": self.extractor.kind,
            "codec": self.decoder.codec if self.decoder else None,
            "frames": self.decoder.frames if self.decoder else 0,
            "dump": str(self.dump_path) if self.dump_path else None,
            "previews": self.previews,
        }

    def _run(self) -> None:
        dump = self._open_dump()
        try:
            while not self._stop.is_set():
                try:
                    datagram, sender = self._socket.recvfrom(RECV_BUFFER)
                except TimeoutError:
                    continue
                self._handle(datagram, f"{sender[0]}:{sender[1]}", dump)
        finally:
            if dump:
                dump.close()

    def _open_dump(self) -> BinaryIO | None:
        if not self.dump_path:
            return None
        self.dump_path.parent.mkdir(parents=True, exist_ok=True)
        return self.dump_path.open("wb")

    def _handle(self, datagram: bytes, sender: str, dump: BinaryIO | None) -> None:
        arrival = time.time()
        if dump:
            dump.write(DUMP_RECORD.pack(arrival, len(datagram)) + datagram)
        self.packets += 1
        self.bytes += len(datagram)
        self.senders.add(sender)
        self._recent.append((time.monotonic(), len(datagram)))
        while self._recent and time.monotonic() - self._recent[0][0] > RATE_WINDOW_S:
            self._recent.popleft()
        if len(self.previews) < HEX_PREVIEW_PACKETS:
            self.previews.append(_preview(datagram, sender))
            print(f"video: {self.previews[-1]}", flush=True)
        stream = self.extractor.feed(datagram)
        if self.decoder:
            self.decoder.feed(stream)


def _preview(datagram: bytes, sender: str) -> str:
    offset = start_code_offset(datagram)
    hint = "no start code" if offset is None else f"start code at {offset}"
    return f"{sender} len={len(datagram)} {hint} {datagram[:HEX_PREVIEW_BYTES].hex(' ')}"
