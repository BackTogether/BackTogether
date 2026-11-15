"""Guess what the drone's video datagrams contain and turn them into an Annex-B stream.

We don't know the real format yet. This file holds every guess, so it is the only place to
change once we have a capture (`video` dumps one; `devsend --replay` plays it back).
Supported now:
- bare Annex-B byte stream cut into datagrams (optionally after a fixed `skip`-byte header)
- RTP carrying H.264 (RFC 6184: single NAL, STAP-A, FU-A)
"""

from enum import StrEnum

START_CODE = b"\x00\x00\x01"
LONG_START_CODE = b"\x00\x00\x00\x01"
HEADER_SEARCH_BYTES = 64  # how far into a datagram we look for a start code (stats hint)

RTP_VERSION = 2
RTP_HEADER_LEN = 12
RTP_DYNAMIC_PAYLOAD_TYPES = range(96, 128)
RTP_LOCK_PACKETS = 3  # this many packets in a row with the same SSRC -> it's RTP

H264_NAL_TYPE_MASK = 0x1F
H264_SLICE_TYPES = (1, 5)  # non-IDR, IDR
H264_IDR = 5
H264_SPS = 7
H264_STAP_A = 24
H264_FU_A = 28
FU_START_BIT = 0x80
FU_END_BIT = 0x40
RTP_MARKER_BIT = 0x80  # second header byte: the last packet of a picture
# An access unit delimiter right after a picture's last packet: ffmpeg's H.264 parser otherwise
# holds each picture until the next one starts (one frame, ~33 ms, measured 68 -> 35 ms lag)
H264_ACCESS_UNIT_DELIMITER = LONG_START_CODE + b"\x09\xf0"
HEVC_VPS_HEADER = b"\x40\x01"
HEVC_VCL_TYPES = range(32)
HEVC_IRAP_TYPES = range(16, 24)
FIRST_SLICE_BIT = 0x80  # H.264 first_mb_in_slice == 0 / HEVC first_slice_segment_in_pic_flag
NAL_HEADER_BYTES = {"h264": 1, "hevc": 2}


class PayloadKind(StrEnum):
    ANNEXB = "annexb"
    RTP = "rtp"


def start_code_offset(data: bytes) -> int | None:
    """Offset of the first Annex-B start code near the datagram start, if any."""
    offset = data.find(START_CODE, 0, HEADER_SEARCH_BYTES + len(START_CODE))
    if offset < 0:
        return None
    return offset - 1 if offset and data[offset - 1] == 0 else offset


def find_parameter_set(stream: bytes) -> tuple[int, str] | None:
    """(offset of its start code, codec) of the first H.264 SPS or HEVC VPS in the stream."""
    offset = stream.find(START_CODE)
    while 0 <= offset < len(stream) - len(START_CODE) - 1:
        nal = stream[offset + len(START_CODE) :]
        start = offset - 1 if offset and stream[offset - 1] == 0 else offset  # 4-byte code
        if nal.startswith(HEVC_VPS_HEADER):
            return start, "hevc"
        if nal[0] & H264_NAL_TYPE_MASK == H264_SPS:
            return start, "h264"
        offset = stream.find(START_CODE, offset + 1)
    return None


class PictureCounter:
    """Counts pictures (first slices) in an Annex-B stream fed in pieces of any size, from the
    first keyframe on: a decoder drops the pictures before it."""

    def __init__(self, codec: str) -> None:
        self._header = NAL_HEADER_BYTES[codec]
        self._is_slice = _h264_slice if codec == "h264" else _hevc_slice
        self._is_keyframe = _h264_keyframe if codec == "h264" else _hevc_keyframe
        self._need = len(START_CODE) + self._header + 1  # start code, header, first slice byte
        self._tail = b""
        self._started = False

    def wait_for_keyframe(self) -> None:
        """Count nothing until the next keyframe (after the decoder lost track)."""
        self._started = False

    def feed(self, data: bytes) -> int:
        """New pictures that start in `data` (a start code split from the last piece counts)."""
        stream = self._tail + data
        count = 0
        offset = stream.find(START_CODE)
        while offset >= 0 and offset + self._need <= len(stream):
            complete_before = offset + self._need <= len(self._tail)  # counted last time
            nal = offset + len(START_CODE)
            if not complete_before and self._is_slice(stream[nal]):
                self._started = self._started or self._is_keyframe(stream[nal])
                first_slice = stream[nal + self._header] & FIRST_SLICE_BIT != 0
                count += self._started and first_slice
            offset = stream.find(START_CODE, offset + 1)
        self._tail = stream[-self._need :]
        return count


def _h264_slice(header: int) -> bool:
    return header & H264_NAL_TYPE_MASK in H264_SLICE_TYPES


def _hevc_slice(header: int) -> bool:
    return (header >> 1) & 0x3F in HEVC_VCL_TYPES


def _h264_keyframe(header: int) -> bool:
    return header & H264_NAL_TYPE_MASK == H264_IDR


def _hevc_keyframe(header: int) -> bool:
    return (header >> 1) & 0x3F in HEVC_IRAP_TYPES


def detect_codec(stream: bytes) -> str | None:
    """'h264' or 'hevc' from the first parameter set (H.264 SPS or HEVC VPS) in the stream."""
    found = find_parameter_set(stream)
    return found[1] if found else None


def _rtp_ssrc(packet: bytes) -> bytes | None:
    """SSRC if the packet looks like RTP with a dynamic payload type, else None."""
    if len(packet) <= RTP_HEADER_LEN or packet[0] >> 6 != RTP_VERSION:
        return None
    if packet[1] & 0x7F not in RTP_DYNAMIC_PAYLOAD_TYPES:
        return None
    return packet[8:12]


def _rtp_payload(packet: bytes) -> bytes:
    csrc_count = packet[0] & 0x0F
    start = RTP_HEADER_LEN + 4 * csrc_count
    if packet[0] & 0x10:  # header extension: 4-byte header, length in 32-bit words
        start += 4 + 4 * int.from_bytes(packet[start + 2 : start + 4], "big")
    end = len(packet) - packet[-1] if packet[0] & 0x20 else len(packet)  # padding
    return packet[start:end]


def depacketize_h264(payload: bytes) -> bytes:
    """One RTP H.264 payload -> Annex-B bytes. FU-A fragments go out as they come."""
    if not payload:
        return b""
    nal_type = payload[0] & H264_NAL_TYPE_MASK
    if nal_type == H264_STAP_A:
        out, offset = bytearray(), 1
        while offset + 2 <= len(payload):
            size = int.from_bytes(payload[offset : offset + 2], "big")
            out += LONG_START_CODE + payload[offset + 2 : offset + 2 + size]
            offset += 2 + size
        return bytes(out)
    if nal_type == H264_FU_A:
        if len(payload) < 2:
            return b""
        fu_header = payload[1]
        if not fu_header & FU_START_BIT:
            return payload[2:]
        nal_header = (payload[0] & ~H264_NAL_TYPE_MASK & 0xFF) | (fu_header & H264_NAL_TYPE_MASK)
        return LONG_START_CODE + bytes([nal_header]) + payload[2:]
    return LONG_START_CODE + payload


class StreamExtractor:
    """Datagrams in, Annex-B elementary stream out. Drops data until the kind is known."""

    def __init__(self, skip: int = 0) -> None:
        self.skip = skip
        self.kind: PayloadKind | None = None
        self._ssrc: bytes | None = None
        self._held: list[bytes] = []  # RTP candidates, kept so the first SPS/PPS isn't lost

    def feed(self, datagram: bytes) -> bytes:
        data = datagram[self.skip :]
        if self.kind is PayloadKind.ANNEXB:
            return data
        if self.kind is PayloadKind.RTP:
            return _rtp_to_annexb(data)
        return self._detect(data)

    def _detect(self, data: bytes) -> bytes:
        if start_code_offset(data) == 0:
            self.kind = PayloadKind.ANNEXB
            return data
        ssrc = _rtp_ssrc(data)
        if not ssrc or ssrc != self._ssrc:
            self._held.clear()
        self._ssrc = ssrc
        if ssrc:
            self._held.append(data)
        if len(self._held) < RTP_LOCK_PACKETS:
            return b""
        self.kind = PayloadKind.RTP
        held, self._held = self._held, []
        return b"".join(_rtp_to_annexb(packet) for packet in held)


def _rtp_to_annexb(packet: bytes) -> bytes:
    """Depacketize, and close the picture with an access unit delimiter on its last packet."""
    payload = _rtp_payload(packet)
    stream = depacketize_h264(payload)
    if packet[1] & RTP_MARKER_BIT and _ends_slice(payload):
        stream += H264_ACCESS_UNIT_DELIMITER
    return stream


def _ends_slice(payload: bytes) -> bool:
    """The payload is a whole slice NAL or the last fragment of one (not a parameter set: the
    drone marks those too, and a delimiter must not split them from their picture)."""
    if not payload:
        return False
    nal_type = payload[0] & H264_NAL_TYPE_MASK
    if nal_type == H264_FU_A:
        return (
            len(payload) > 1
            and bool(payload[1] & FU_END_BIT)
            and (payload[1] & H264_NAL_TYPE_MASK in H264_SLICE_TYPES)
        )
    return nal_type in H264_SLICE_TYPES
