"""ADGP framing of the HoverAir X1 Bluetooth LE link (decoded from a capture of the owner's phone).

One frame is one GATT write or notification value:

    FD | seq | len_lo | type<<2 | len_hi2 | frag_count | frag_idx
    | crc8(bytes 0-5) | payload | crc8(payload)

`len` is the 10-bit payload length of that frame. A message longer than one frame is split in
fragments that share `seq` and carry their own checksums. crc8 is poly 0x07, init 0, no reflection.
"""

from typing import NamedTuple

MAGIC = 0xFD
TYPE_PUSH = 2  # PushMessage, drone -> phone, own seq counter
TYPE_RPC = 3  # RpcRequest (phone -> drone) and RpcResponse (drone -> phone, echoes the request seq)
MAX_FRAME = 512  # drone -> phone frames never exceed this (504 payload bytes)

_HEADER_SIZE = 7
_OVERHEAD = _HEADER_SIZE + 1
_CRC_POLY = 0x07
_TYPE_SHIFT = 2
_LEN_HIGH_MASK = 0x03


class Message(NamedTuple):
    seq: int
    type: int
    payload: bytes


class _Frame(NamedTuple):
    message: Message
    count: int
    index: int


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ _CRC_POLY if crc & 0x80 else crc << 1) & 0xFF
    return crc


def _encode_frame(payload: bytes, seq: int, msg_type: int, count: int, index: int) -> bytes:
    length = len(payload)
    header = bytes(
        [
            MAGIC,
            seq & 0xFF,
            length & 0xFF,
            msg_type << _TYPE_SHIFT | length >> 8,
            count,
            index,
        ]
    )
    return header + bytes([crc8(header)]) + payload + bytes([crc8(payload)])


def encode(payload: bytes, seq: int, msg_type: int = TYPE_RPC) -> list[bytes]:
    """Frames to send, in order. Splitting at the drone's frame size is inferred: the app never
    sent more than 50 bytes in one write."""
    step = MAX_FRAME - _OVERHEAD
    chunks = [payload[i : i + step] for i in range(0, len(payload), step)] or [b""]
    return [_encode_frame(c, seq, msg_type, len(chunks), i) for i, c in enumerate(chunks)]


def _decode_frame(frame: bytes) -> _Frame:
    if len(frame) < _OVERHEAD or frame[0] != MAGIC:
        raise ValueError("not an ADGP frame")
    seq, len_lo, type_len, count, index, header_crc = frame[1:_HEADER_SIZE]
    length = len_lo | (type_len & _LEN_HIGH_MASK) << 8
    if header_crc != crc8(frame[: _HEADER_SIZE - 1]):
        raise ValueError("header checksum mismatch")
    if len(frame) != length + _OVERHEAD:
        raise ValueError("length mismatch")
    payload = frame[_HEADER_SIZE:-1]
    if frame[-1] != crc8(payload):
        raise ValueError("payload checksum mismatch")
    return _Frame(Message(seq, type_len >> _TYPE_SHIFT, payload), count, index)


class Reassembler:
    """Feed frames as they arrive; a complete message comes back, otherwise None."""

    def __init__(self) -> None:
        self._parts: dict[tuple[int, int], dict[int, bytes]] = {}

    def feed(self, frame: bytes) -> Message | None:
        """Raises ValueError on a malformed frame."""
        decoded = _decode_frame(frame)
        message = decoded.message
        if decoded.count <= 1:
            return message
        parts = self._parts.setdefault((message.seq, message.type), {})
        parts[decoded.index] = message.payload
        if len(parts) < decoded.count:
            return None
        del self._parts[(message.seq, message.type)]
        payload = b"".join(parts[i] for i in range(decoded.count))
        return Message(message.seq, message.type, payload)
