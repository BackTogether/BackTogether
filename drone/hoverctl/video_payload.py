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
H264_SPS = 7
H264_STAP_A = 24
H264_FU_A = 28
FU_START_BIT = 0x80
HEVC_VPS_HEADER = b"\x40\x01"


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
            return depacketize_h264(_rtp_payload(data))
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
        return b"".join(depacketize_h264(_rtp_payload(packet)) for packet in held)
