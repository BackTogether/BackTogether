"""Stick datagram: a hand-encoded `h130.RockerControlCommand` protobuf.

Only two tiny messages are needed, so we write the protobuf wire format directly instead of
pulling in protoc and the protobuf runtime. Field numbers come from drone/proto/h130.proto.
The schema is proto2 (the app's generated code tracks field presence), and the app sets every
axis on every tick, so zero axes go on the wire too: the packet matches the app's byte for byte.
"""

import struct
from dataclasses import astuple, dataclass, fields

AXIS_LIMIT = 100.0
# RawManualControlCommand.control_source_type (enum ConnectType). The app's button pad sends
# CROSS_KEY with the 28 / +3 ramp we copy (session1 logs); its analog joystick sends APP.
CONTROL_SOURCE_APP = 1
CONTROL_SOURCE_CROSS_KEY = 6
CONTROL_SOURCES = {"cross-key": CONTROL_SOURCE_CROSS_KEY, "app": CONTROL_SOURCE_APP}
DEFAULT_CONTROL_SOURCE = "cross-key"

_WIRE_VARINT = 0
_WIRE_LEN = 2
_WIRE_FIXED32 = 5

# RockerControlCommand
_FIELD_SEQ_NUM = 1
_FIELD_TIMESTAMP = 2
_FIELD_COMMAND = 3
# RawManualControlCommand (roll=3, pitch=4, thrust=5, yaw=6, gimbal=7, in Axes field order)
_FIELD_CONTROL_SOURCE = 1
_FIRST_AXIS_FIELD = 3


@dataclass(frozen=True)
class Axes:
    """A stick command: -1..1 from inputs, -100..100 once scaled.

    Signs are ours, relative to the camera: pitch = forward(+)/back(-), roll = right(+)/left(-),
    thrust = up(+)/down(-), yaw = turn right(+)/left(-), gimbal = camera up(+)/down(-).
    The packet uses other signs; `to_packet` converts.
    """

    roll: float = 0.0
    pitch: float = 0.0
    thrust: float = 0.0
    yaw: float = 0.0
    gimbal: float = 0.0

    def scaled(self, factor: float, limit: float) -> "Axes":
        """Multiply every axis by `factor` and clamp it to +-`limit`."""
        return Axes(*(max(-limit, min(limit, value * factor)) for value in astuple(self)))

    def is_neutral(self) -> bool:
        return not any(astuple(self))


NEUTRAL = Axes()
AXIS_NAMES = tuple(field.name for field in fields(Axes))

# Packet sign of each + direction above. The drone's body frame has z down (like NED), so
# forward and up are negative. Evidence in drone/README.md, "Stick signs".
PITCH_FORWARD = -1.0
ROLL_RIGHT = 1.0
THRUST_UP = -1.0
YAW_RIGHT = 1.0
GIMBAL_UP = -1.0
PACKET_SIGNS = Axes(
    roll=ROLL_RIGHT, pitch=PITCH_FORWARD, thrust=THRUST_UP, yaw=YAW_RIGHT, gimbal=GIMBAL_UP
)
# Names of the + and - direction of each axis, for help text and readouts.
AXIS_DIRECTIONS = {
    "roll": ("right", "left"),
    "pitch": ("fwd", "back"),
    "thrust": ("up", "down"),
    "yaw": ("turn r", "turn l"),
    "gimbal": ("cam up", "cam down"),
}


def to_packet(axes: Axes) -> Axes:
    """Our signs to the packet's signs (and back: it is its own inverse)."""
    pairs = zip(astuple(PACKET_SIGNS), astuple(axes), strict=True)
    return Axes(*(sign * value or 0.0 for sign, value in pairs))  # `or 0.0`: no -0.0 printed


def _varint(value: int) -> bytes:
    out = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        if not value:
            out.append(byte)
            return bytes(out)
        out.append(byte | 0x80)


def _key(field: int, wire_type: int) -> bytes:
    return _varint(field << 3 | wire_type)


def _encode_manual_control(axes: Axes, control_source: int) -> bytes:
    out = _key(_FIELD_CONTROL_SOURCE, _WIRE_VARINT) + _varint(control_source)
    for field, value in enumerate(astuple(to_packet(axes)), start=_FIRST_AXIS_FIELD):
        out += _key(field, _WIRE_FIXED32) + struct.pack("<f", value)
    return out


def encode_rocker_command(
    seq_num: int, timestamp_ms: int, axes: Axes, control_source: int = CONTROL_SOURCE_CROSS_KEY
) -> bytes:
    """One UDP payload for <DRONE_IP>:19091. `axes` are in -100..100, with our signs."""
    command = _encode_manual_control(axes, control_source)
    return (
        _key(_FIELD_SEQ_NUM, _WIRE_VARINT)
        + _varint(seq_num)
        + _key(_FIELD_TIMESTAMP, _WIRE_VARINT)
        + _varint(timestamp_ms)
        + _key(_FIELD_COMMAND, _WIRE_LEN)
        + _varint(len(command))
        + command
    )
