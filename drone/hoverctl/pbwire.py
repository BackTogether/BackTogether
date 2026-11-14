"""The few `h130` protobuf messages the Bluetooth link needs, on the raw wire format.

Like protocol.py this avoids the protobuf runtime. Field numbers come from drone/proto/h130.proto.
Decoders ignore unknown fields. Fields that are absent are proto3 defaults (0, empty).
"""

import math
import socket
import struct
import urllib.request
from dataclasses import dataclass, field
from enum import IntEnum

from .sticks import DRONE_HOST

_WIRE_VARINT = 0
_WIRE_FIXED64 = 1
_WIRE_LEN = 2
_WIRE_FIXED32 = 5
_INT32_BITS = 32
_INT32_SIGN = 1 << (_INT32_BITS - 1)
_MASK32 = (1 << _INT32_BITS) - 1
_MASK64 = (1 << 64) - 1  # negative ints go on the wire as 64-bit two's complement varints

# RpcRequest: id plus one request in the `request` oneof
REQUEST_ID = 6
REQ_WIFI_START = 25
REQ_SET_TIME_UTC = 37  # RealTimeMessage
REQ_BATTERY_STATUS = 27
REQ_CHARGER_STATE = 28
REQ_GET_FLIGHT_MODE = 38
REQ_GET_FLIGHT_STATUS = 39
REQ_ABORT_FLIGHT = 40  # land
REQ_GET_STORAGE_CAPACITY = 41
REQ_SET_FLIGHT_MODE_TO_MANUAL = 46  # bool
REQ_HEARTBEAT_INFO = 70  # Empty, the app sends it at 1 Hz over Wi-Fi
REQ_GET_BLE_ENCRY_LIST = 71
REQ_CONNECT_TYPE = 75  # ConnectTypeRequest: tells the drone an app is the controller
REQ_SET_PREVIEW_RESOLUTION = 148  # PreviewResolutionParams; the app never sends it
REQ_GET_PREVIEW_RESOLUTION = 149  # FlightModeConfig
REQ_CAMERA_START_PREVIEW = 240  # StartPreviewParams
REQ_SET_UDP_SERVER_IP = 245  # string: where the drone pushes video
REQ_SWITCH_TO_MANUAL_CAPTURE = 249  # bool
REQ_EXIT_PREVIEW_MODE = 250  # Empty
REQ_ENTER_PREVIEW_MODE = 254  # EnterPreview

# RpcResponse: id plus one response in the `response` oneof
RESPONSE_ID = 7
RESP_ERROR = 1
RESP_BATTERY_STATUS = 37
RESP_CHARGER_STATE = 38
RESP_GET_FLIGHT_MODE = 48
RESP_GET_FLIGHT_STATUS = 49
RESP_GET_STORAGE_CAPACITY = 51
RESP_GET_PREVIEW_RESOLUTION = 159
RESP_ERROR_RESPONSE = 220

# ErrorResponse
_ERROR_CODE = 1
_ERROR_DESCRIPTION = 2

# PushMessage.event (EventPb) and the EventPb events we use
PUSH_EVENT = 7
EVENT_CHARGER = 1
EVENT_FLIGHT_MODE_CHANGE = 3
EVENT_FLIGHT_STATUS_CHANGE = 4
EVENT_FLIGHT_EXCEPTION = 5
EVENT_BATTERY_STATUS = 8
EVENT_FLIGHT_ENGINEERING_INFO = 16
EVENT_FLIGHT_ANALYTICS_LOG = 31  # the app's only live gimbal angle; the X1 never sent it to us

# BleEncryListRequest
_HANDSHAKE_PHONE_ID = 1
_HANDSHAKE_REQUEST_TYPE = 2
HANDSHAKE_REQUEST_CONNECT = 1

# WifiParams, as the app sends it
_WIFI_TYPE = 1
_WIFI_SSID = 2
_WIFI_PASSWORD = 3
_WIFI_COUNTRY = 5
_WIFI_FREQUENCY_STRATEGY = 6
_WIFI_RATE_STRATEGY = 7
WIFI_TYPE_AP = 1
WIFI_FREQUENCY_BEST_AVAILABLE = 0
WIFI_RATE_DOWNLOAD = 2

# EnterPreview: the drone pushes live video as UDP to `udp_server_ip` (the port is fixed at 7078)
_PREVIEW_UDP_SERVER_IP = 1
_PREVIEW_CAMERA_START = 3
_PREVIEW_CONNECT_TYPE = 4
CONNECT_TYPE_APP = 1
# ConnectTypeRequest
_CONNECT_TYPE = 1
# RealTimeMessage (time_zone: whole hours, the app's raw offset without daylight saving)
_TIME_UTC = 1
_TIME_ZONE = 2
# StartPreviewParams
_START_PREVIEW_ENABLE = 1
_START_PREVIEW_CODEC = 3
VIDEO_CODEC_AVC = 1
VIDEO_CODEC_HEVC = 2
# FlightModeConfig.type, and PreviewResolutionParams {flight_mode = 1, res = 2}
_FLIGHT_MODE_TYPE = 1
_PREVIEW_RES_FLIGHT_MODE = 1
_PREVIEW_RES = 2
PREVIEW_RESOLUTION_LOW = 1
PREVIEW_RESOLUTION_MEDIUM = 2
# Preview quality as hoverctl offers it. "sd" sends nothing extra, like the app (1280x720 AVC
# 30 fps on our X1). "hd" asks for PREVIEW_RESOLUTION_MEDIUM, which the app never does.
VIDEO_QUALITY_SD = "sd"
VIDEO_QUALITY_HD = "hd"
VIDEO_QUALITIES = (VIDEO_QUALITY_SD, VIDEO_QUALITY_HD)
# FlightAnalyticsLog { repeated FlightRealtimeLog realtime = 1 }, FlightRealtimeLog.gimbal_pitch = 8
_ANALYTICS_REALTIME = 1
_REALTIME_GIMBAL_PITCH = 8

# The Wi-Fi RPC the app uses for everything but Bluetooth: raw RpcRequest in an HTTP POST
WIFI_RPC_URL = f"http://{DRONE_HOST}:1234/rpc"
WIFI_RPC_TIMEOUT_S = 3.0
WIFI_REQUEST_ID = 1
DRONE_NETWORK_PREFIX = DRONE_HOST.rsplit(".", 1)[0] + "."
_ROUTE_PROBE_PORT = 9  # connect() on UDP sends nothing, it only picks the local interface

# FlightEngineeringInfo
_ENG_VELOCITY = (1, 2, 3)
_ENG_POSITION = (4, 5, 6)
_ENG_MOTORS_RPM = 7
_ENG_BATTERY_VOLTAGE = 50
_ENG_TOTAL_CURRENT = 51
_ENG_PM_VOLTAGE = 52
_ENG_PM_CURRENT = 53

# BatteryStatusResponse
_BATTERY_SOC = 1
_BATTERY_VOLTAGE = 2
_BATTERY_TEMP = 3
_BATTERY_CURRENT = 4

# ChargerStateResponse
_CHARGER_CONNECTED = 1
_CHARGER_FULL = 2

# StorageCapacity
_STORAGE_AVAILABLE = 1
_STORAGE_TOTAL = 2

# FlightException { FlightStatusError error = 1 { repeated errors = 1 };
#                   FlightStatusEvent event = 2 { event = 1 } }
_EXC_ERROR = 1
_EXC_EVENT = 2
_EXC_LIST = 1
_EXC_EVENT_CODE = 1


class FlightMode(IntEnum):
    """FlightModeConfig.c"""

    UNSET = 0
    HOVER = 1
    REVEAL = 2
    FOLLOW = 3
    ORBIT = 4
    OVERHEAD = 5
    CUSTOM = 6
    HOVER_SNAPSHOT = 7
    WILDERNESS_FOLLOW = 8
    RETREAT_FOLLOW = 9
    GOLD_FOLLOW = 10
    DIRECT_FOLLOW = 11
    SKIING_FOLLOW = 12
    RIDING_FOLLOW = 13
    HITCHCOCK = 14
    ROOM_TOUR = 15
    PARTY_MODE = 16
    SPIRAL = 17
    CANOE_FOLLOW = 18
    PADDLE_BOARD_FOLLOW = 19
    SURFING_FOLLOW = 20
    ORBIT_FOLLOW = 21
    OVERHEAD_FOLLOW = 22
    FIXED_MODE = 23
    FOIL_BOARD_FOLLOW = 24
    SNORKEL_MODE = 25
    PILOT_MOTION_FOLLOW = 26
    HANDHELD_MODE = 27
    INCEPTION = 28
    UNIFIED_FOLLOW = 29
    SPACE3D_MODE = 30
    HOVER_SPAS = 31
    KITE_SURFING = 32
    MANUAL = 255


# CaptainInfo oneof field number -> state name
FLIGHT_STATES = {
    1: "taking_off",
    2: "landing",
    3: "idle",
    4: "flying_manual",
    5: "flying_traj",
    6: "pre_takeoff",
    7: "fly_back",
    8: "dynamic_control",
    9: "turtle_roll",
    10: "no_flying_traj",
}

# States where the drone is in the air. Position is only meaningful here: its origin resets
# during taking_off, and after landing -> idle the values jump to another frame (issue #13).
IN_FLIGHT_STATES = frozenset(
    {"taking_off", "flying_manual", "flying_traj", "no_flying_traj", "fly_back", "landing"}
)

Fields = dict[int, list[int | bytes]]


def _varint(value: int) -> bytes:
    out = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        if not value:
            out.append(byte)
            return bytes(out)
        out.append(byte | 0x80)


def _key(number: int, wire_type: int) -> bytes:
    return _varint(number << 3 | wire_type)


def _int_field(number: int, value: int) -> bytes:
    return _key(number, _WIRE_VARINT) + _varint(value & _MASK64)


def _bytes_field(number: int, value: bytes) -> bytes:
    return _key(number, _WIRE_LEN) + _varint(len(value)) + value


def _str_field(number: int, value: str) -> bytes:
    return _bytes_field(number, value.encode())


def parse(data: bytes) -> Fields:
    """Field number -> values in order. Varints are ints; length-delimited and fixed are bytes."""
    fields: Fields = {}
    pos = 0
    while pos < len(data):
        tag, pos = _read_varint(data, pos)
        number, wire_type = tag >> 3, tag & 7
        value: int | bytes
        if wire_type == _WIRE_VARINT:
            value, pos = _read_varint(data, pos)
        elif wire_type == _WIRE_LEN:
            size, pos = _read_varint(data, pos)
            value, pos = _take(data, pos, size)
        elif wire_type == _WIRE_FIXED32:
            value, pos = _take(data, pos, 4)
        elif wire_type == _WIRE_FIXED64:
            value, pos = _take(data, pos, 8)
        else:
            raise ValueError(f"unsupported wire type {wire_type}")
        fields.setdefault(number, []).append(value)
    return fields


def _read_varint(data: bytes, pos: int) -> tuple[int, int]:
    result = shift = 0
    while True:
        if pos >= len(data):
            raise ValueError("truncated varint")
        byte = data[pos]
        pos += 1
        result |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return result, pos
        shift += 7


def _take(data: bytes, pos: int, size: int) -> tuple[bytes, int]:
    if pos + size > len(data):
        raise ValueError("truncated field")
    return data[pos : pos + size], pos + size


def _int(fields: Fields, number: int) -> int:
    """Last varint of a field as a signed int32 (negative int32s arrive as 64-bit varints)."""
    values = fields.get(number)
    if not values or not isinstance(values[-1], int):
        return 0
    value = values[-1] & _MASK32
    return value - (1 << _INT32_BITS) if value & _INT32_SIGN else value


def _int64(fields: Fields, number: int) -> int:
    values = fields.get(number)
    return values[-1] if values and isinstance(values[-1], int) else 0


def _bool(fields: Fields, number: int) -> bool:
    return bool(_int64(fields, number))


def _float(fields: Fields, number: int) -> float:
    values = fields.get(number)
    if not values or not isinstance(values[-1], bytes):
        return 0.0
    return struct.unpack("<f", values[-1])[0]


def _message(fields: Fields, number: int) -> Fields | None:
    values = fields.get(number)
    if not values or not isinstance(values[-1], bytes):
        return None
    return parse(values[-1])


def _repeated_ints(fields: Fields, number: int) -> list[int]:
    """Repeated int32, packed (one length-delimited value) or not."""
    out: list[int] = []
    for value in fields.get(number, []):
        if isinstance(value, int):
            out.append(_int({number: [value]}, number))
            continue
        pos = 0
        while pos < len(value):
            item, pos = _read_varint(value, pos)
            out.append(_int({number: [item]}, number))
    return out


# --- requests -------------------------------------------------------------------------------


def encode_rpc_request(request_id: int, request_field: int, body: bytes = b"") -> bytes:
    """RpcRequest with an (often empty) message in `request_field`."""
    return _int_field(REQUEST_ID, request_id) + _bytes_field(request_field, body)


def encode_set_manual_request(request_id: int) -> bytes:
    return _int_field(REQUEST_ID, request_id) + _int_field(REQ_SET_FLIGHT_MODE_TO_MANUAL, 1)


def encode_handshake_request(request_id: int, phone_id: str) -> bytes:
    body = _str_field(_HANDSHAKE_PHONE_ID, phone_id) + _int_field(
        _HANDSHAKE_REQUEST_TYPE, HANDSHAKE_REQUEST_CONNECT
    )
    return encode_rpc_request(request_id, REQ_GET_BLE_ENCRY_LIST, body)


def encode_wifi_start_request(request_id: int, ssid: str, password: str, country: str) -> bytes:
    """Same fields as the app, including its explicit frequency_strategy = 0."""
    body = (
        _int_field(_WIFI_TYPE, WIFI_TYPE_AP)
        + _str_field(_WIFI_SSID, ssid)
        + _str_field(_WIFI_PASSWORD, password)
        + _str_field(_WIFI_COUNTRY, country)
        + _int_field(_WIFI_FREQUENCY_STRATEGY, WIFI_FREQUENCY_BEST_AVAILABLE)
        + _int_field(_WIFI_RATE_STRATEGY, WIFI_RATE_DOWNLOAD)
    )
    return encode_rpc_request(request_id, REQ_WIFI_START, body)


def encode_connect_type_request(request_id: int) -> bytes:
    """connect_type_request(APP): the app sends it after every Bluetooth and Wi-Fi connect."""
    body = _int_field(_CONNECT_TYPE, CONNECT_TYPE_APP)
    return encode_rpc_request(request_id, REQ_CONNECT_TYPE, body)


def encode_set_time_request(request_id: int, utc_s: int, zone_hours: int) -> bytes:
    """set_time_utc_request, as the app sends it after every Bluetooth connect."""
    body = _int_field(_TIME_UTC, utc_s) + _int_field(_TIME_ZONE, zone_hours)
    return encode_rpc_request(request_id, REQ_SET_TIME_UTC, body)


def _preview_params(enable: bool, codec: int) -> bytes:
    """StartPreviewParams as the app sends it (session_id 0, proto3 omits false and 0)."""
    return (_int_field(_START_PREVIEW_ENABLE, 1) if enable else b"") + _int_field(
        _START_PREVIEW_CODEC, codec
    )


def encode_enter_preview_request(request_id: int, udp_server_ip: str, codec: int) -> bytes:
    """Start live video: the drone pushes it to udp_server_ip:7078."""
    body = (
        _str_field(_PREVIEW_UDP_SERVER_IP, udp_server_ip)
        + _bytes_field(_PREVIEW_CAMERA_START, _preview_params(True, codec))
        + _int_field(_PREVIEW_CONNECT_TYPE, CONNECT_TYPE_APP)
    )
    return encode_rpc_request(request_id, REQ_ENTER_PREVIEW_MODE, body)


def encode_camera_start_preview_request(request_id: int, codec: int) -> bytes:
    """Start the video push, as the Manual Control screen does after set_udp_server_ipaddress."""
    return encode_rpc_request(request_id, REQ_CAMERA_START_PREVIEW, _preview_params(True, codec))


def encode_set_udp_server_request(request_id: int, udp_server_ip: str) -> bytes:
    return _int_field(REQUEST_ID, request_id) + _str_field(REQ_SET_UDP_SERVER_IP, udp_server_ip)


def encode_switch_to_manual_capture_request(request_id: int) -> bytes:
    return _int_field(REQUEST_ID, request_id) + _int_field(REQ_SWITCH_TO_MANUAL_CAPTURE, 1)


def _manual_mode_config() -> bytes:
    return _int_field(_FLIGHT_MODE_TYPE, FlightMode.MANUAL)


def encode_get_preview_resolution_request(request_id: int) -> bytes:
    """get_preview_resolution_request for MANUAL (the drone answers PreviewResolutionParams)."""
    return encode_rpc_request(request_id, REQ_GET_PREVIEW_RESOLUTION, _manual_mode_config())


def encode_set_preview_resolution_request(request_id: int, resolution: int) -> bytes:
    """set_preview_resolution_request {flight_mode MANUAL, res} (PREVIEW_RESOLUTION_*)."""
    body = _bytes_field(_PREVIEW_RES_FLIGHT_MODE, _manual_mode_config()) + _int_field(
        _PREVIEW_RES, resolution
    )
    return encode_rpc_request(request_id, REQ_SET_PREVIEW_RESOLUTION, body)


def encode_stop_preview_request(request_id: int, codec: int) -> bytes:
    """First half of stopping video (the app then sends exit_preview_mode_request)."""
    return encode_rpc_request(request_id, REQ_CAMERA_START_PREVIEW, _preview_params(False, codec))


# --- Wi-Fi RPC (HTTP, only reachable on the drone's own network) -----------------------------


def drone_network_ip() -> str | None:
    """Our address on the drone's Wi-Fi: the interface that routes to the drone, if it is one."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((DRONE_HOST, _ROUTE_PROBE_PORT))
        address = str(probe.getsockname()[0])
    except OSError:
        return None  # no route at all
    finally:
        probe.close()
    return address if address.startswith(DRONE_NETWORK_PREFIX) else None


def post_wifi_rpc(request: bytes) -> "RpcResponse":
    """POST one RpcRequest to the drone; raises OSError (URLError included) if unreachable."""
    http_request = urllib.request.Request(
        WIFI_RPC_URL, request, {"Content-Type": "application/octet-stream"}
    )
    with urllib.request.urlopen(http_request, timeout=WIFI_RPC_TIMEOUT_S) as reply:  # noqa: S310
        return decode_rpc_response(reply.read())


# --- responses and pushes --------------------------------------------------------------------


@dataclass(frozen=True)
class RpcResponse:
    request_id: int
    kind: int | None  # field number of the response oneof, None if the response had none
    body: bytes
    error: str | None  # set when the drone answered with an error


def decode_rpc_response(payload: bytes) -> RpcResponse:
    fields = parse(payload)
    request_id = _int(fields, RESPONSE_ID)
    if RESP_ERROR in fields:
        return RpcResponse(request_id, RESP_ERROR, b"", f"rpc error {_int(fields, RESP_ERROR)}")
    detail = _message(fields, RESP_ERROR_RESPONSE)
    if detail is not None:
        code = _int(detail, _ERROR_CODE)
        text = bytes(detail.get(_ERROR_DESCRIPTION, [b""])[-1]).decode(errors="replace")
        return RpcResponse(request_id, RESP_ERROR_RESPONSE, b"", f"error {code}: {text}")
    for kind, values in fields.items():
        if kind != RESPONSE_ID and isinstance(values[-1], bytes):
            return RpcResponse(request_id, kind, values[-1], None)
    # Responses made of one scalar (bool abort_flight_response) or nothing
    kind = next((k for k in fields if k != RESPONSE_ID), None)
    return RpcResponse(request_id, kind, b"", None)


def decode_push(payload: bytes) -> tuple[int, bytes] | None:
    """(EventPb field number, event body) of a PushMessage event, None for other pushes."""
    event = _message(parse(payload), PUSH_EVENT)
    if not event:
        return None
    for number, values in event.items():
        if isinstance(values[-1], bytes):
            return number, values[-1]
    return None


@dataclass(frozen=True)
class Engineering:
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    motors_rpm: list[int]
    battery_voltage: int
    total_current: int
    pm_voltage: int
    pm_current: int


def decode_engineering(body: bytes) -> Engineering:
    f = parse(body)
    return Engineering(
        position=tuple(_float(f, n) for n in _ENG_POSITION),
        velocity=tuple(_float(f, n) for n in _ENG_VELOCITY),
        motors_rpm=_repeated_ints(f, _ENG_MOTORS_RPM),
        battery_voltage=_int(f, _ENG_BATTERY_VOLTAGE),
        total_current=_int(f, _ENG_TOTAL_CURRENT),
        pm_voltage=_int(f, _ENG_PM_VOLTAGE),
        pm_current=_int(f, _ENG_PM_CURRENT),
    )


@dataclass(frozen=True)
class Battery:
    soc: int
    voltage: int
    temp: int
    current: int


def decode_battery(body: bytes) -> Battery:
    f = parse(body)
    return Battery(
        soc=_int(f, _BATTERY_SOC),
        voltage=_int(f, _BATTERY_VOLTAGE),
        temp=_int(f, _BATTERY_TEMP),
        current=_int(f, _BATTERY_CURRENT),
    )


@dataclass(frozen=True)
class Charger:
    connected: bool
    fully_charged: bool


def decode_charger(body: bytes) -> Charger:
    f = parse(body)
    return Charger(_bool(f, _CHARGER_CONNECTED), _bool(f, _CHARGER_FULL))


@dataclass(frozen=True)
class Storage:
    available_bytes: int
    total_bytes: int


def decode_storage(body: bytes) -> Storage:
    f = parse(body)
    return Storage(_int64(f, _STORAGE_AVAILABLE), _int64(f, _STORAGE_TOTAL))


def decode_preview_resolution(body: bytes) -> int:
    """PreviewResolutionParams.res (0 = unset)."""
    return _int64(parse(body), _PREVIEW_RES)


def decode_camera_tilt(body: bytes) -> float | None:
    """Gimbal pitch in degrees from a FlightAnalyticsLog (the last sample, like the app)."""
    samples = parse(body).get(_ANALYTICS_REALTIME, [])
    if not samples or not isinstance(samples[-1], bytes):
        return None
    fields = parse(samples[-1])
    if _REALTIME_GIMBAL_PITCH not in fields:
        return None
    return math.degrees(_float(fields, _REALTIME_GIMBAL_PITCH))  # the drone sends radians


def decode_flight_mode(body: bytes) -> int:
    """FlightModeConfig.type (a FlightMode value, or an unknown number)."""
    return _int64(parse(body), 1)


def mode_name(mode: int) -> str:
    try:
        return FlightMode(mode).name
    except ValueError:
        return f"MODE_{mode}"


def decode_flight_state(body: bytes) -> str:
    """CaptainInfo: the name of the state that is set."""
    for number in parse(body):
        if number in FLIGHT_STATES:
            return FLIGHT_STATES[number]
    return "unknown"


@dataclass(frozen=True)
class FlightException:
    errors: list[int] = field(default_factory=list)  # FlightStatusError.d codes
    event: int = 0  # FlightStatusEvent.c code


def decode_flight_exception(body: bytes) -> FlightException:
    f = parse(body)
    errors = _message(f, _EXC_ERROR)
    event = _message(f, _EXC_EVENT)
    return FlightException(
        errors=_repeated_ints(errors, _EXC_LIST) if errors else [],
        event=_int64(event, _EXC_EVENT_CODE) if event else 0,
    )
