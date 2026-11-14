"""Convert a capture log from drone.telemetry.capture into JSONL telemetry records.

Each output line is one JSON object with a `t` (epoch seconds) and a `type`:
engineering, state, heartbeat, stick, battery or flight_mode. Records are sorted by t.

Engineering frame conventions: z points down (altitude_m = -z); x/y are a fixed world
frame NOT aligned with the drone heading. The origin is reset during TAKING_OFF (the first
record after it can still be in the old frame) and the values jump to another frame after
LANDING -> IDLE, so position and velocity are only meaningful while `in_flight` is true:
from the first FLYING_* state until IDLE. Stick records are the pilot's commands
(-100..100), not the drone attitude.

Run from the repo root: uv run python -m drone.telemetry.parse <capture.log> [-o out.jsonl]
"""

import argparse
import json
import re
import time
from collections import Counter
from collections.abc import Iterable, Iterator
from dataclasses import dataclass, field
from pathlib import Path

LOGCAT_LINE = re.compile(r"^\s*(\d+\.\d+)\s.*?\[mxlogger_info\]: ?(.*)$")
SINK_PREFIX = "plain_text_sink::"
HEADER_SINK_PREFIX = "plain_text_sink::plain_text_sink "
HEADER_TIMESTAMP = re.compile(r"^\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d{1,3} ")  # ms not zero-padded
PROTO_FIELD = re.compile(r"^(\w+): (.*)$")
PROTO_OPEN = re.compile(r"^(\w+) \{$")
PROTO_CLOSE = "}"

ENGINEERING_MARKER = "FlightEngineeringInfo@"
BATTERY_MESSAGE = "battery_status_response"
FLIGHT_MODE_MESSAGE = "get_flight_mode_response"
STATE_CHANGE = re.compile(r"DroneStateManager captainInfo changed from: (\w+) to: (\w+)")
HEARTBEAT = re.compile(
    r"HeartBeatInfo SyncPrimaryState:.*?flightMode=(\w+), flightStatus=(\w+)"
    r".*?isRecording=(\w+).*?batteryPercent=(\w+).*?charging=(\w+)"
)
NUMBER = r"(-?[\d.]+)"
STICK = re.compile(
    rf"ManualControl yaw:{NUMBER},thrust:{NUMBER},roll:{NUMBER},pitch:{NUMBER},gimbal:{NUMBER}"
)

TAKEOFF_STATE = "TAKING_OFF"
FLYING_STATE_PREFIX = "FLYING"  # FLYING_MANUAL, FLYING_TRAJ (Follow, Orbit...)
LANDING_STATE = "LANDING"
IDLE_STATE = "IDLE"

# Text-format messages: every key maps to the list of its values (repeated fields and
# nested messages stay in order).
Fields = dict[str, list]


@dataclass
class Block:
    """A header line plus the protobuf text lines logged after it."""

    t: float
    header: str
    body: list[str] = field(default_factory=list)


def read_bodies(path: Path) -> Iterator[tuple[float, str]]:
    """Yield (epoch seconds, app log text) for each logcat line of the capture."""
    with path.open(encoding="utf-8", errors="replace") as lines:
        for line in lines:
            match = LOGCAT_LINE.match(line.rstrip("\r\n"))
            if match:
                yield float(match.group(1)), match.group(2)


def header_text(body: str) -> str | None:
    """Return the header without the sink prefix, or None if body is not a header."""
    text = body.removeprefix(HEADER_SINK_PREFIX)
    return text if HEADER_TIMESTAMP.match(text) else None


def group_blocks(bodies: Iterable[tuple[float, str]]) -> Iterator[Block]:
    """Group each header with its following lines, skipping sink noise lines."""
    block: Block | None = None
    for t, body in bodies:
        header = header_text(body)
        if header is not None:
            if block:
                yield block
            block = Block(t, header)
        elif block and not body.startswith(SINK_PREFIX):
            block.body.append(body)
    if block:
        yield block


def parse_value(raw: str) -> int | float | str:
    for convert in (int, float):
        try:
            return convert(raw)
        except ValueError:
            pass
    return raw.strip('"')


def parse_text_proto(lines: Iterable[str]) -> Fields:
    """Parse protobuf text format; lines that are not proto fields are ignored."""
    stack: list[Fields] = [{}]
    for line in lines:
        text = line.strip()
        if opened := PROTO_OPEN.match(text):
            child: Fields = {}
            stack[-1].setdefault(opened.group(1), []).append(child)
            stack.append(child)
        elif text == PROTO_CLOSE and len(stack) > 1:
            stack.pop()
        elif scalar := PROTO_FIELD.match(text):
            stack[-1].setdefault(scalar.group(1), []).append(parse_value(scalar.group(2)))
    return stack[0]


def first(fields: Fields, key: str, default: int | float | str = 0) -> int | float | str:
    """First value of a field; proto3 omits defaults, so missing means 0."""
    return fields.get(key, [default])[0]


def engineering_record(t: float, fields: Fields) -> dict:
    position = [float(first(fields, f"position_{axis}")) for axis in "xyz"]
    return {
        "t": t,
        "type": "engineering",
        "in_flight": False,
        "position": position,
        "velocity": [float(first(fields, f"velocity_{axis}")) for axis in "xyz"],
        "altitude_m": 0.0 - position[2],  # 0.0 - z avoids writing -0.0
        "motors_rpm": fields.get("motors_rpm", []),
        "battery_mv": first(fields, "battery_voltage"),
        "current_ma": first(fields, "total_current"),
        "pm_voltage": first(fields, "pm_voltage"),
        "pm_current": first(fields, "pm_current"),
    }


def battery_record(t: float, fields: Fields) -> dict:
    return {
        "t": t,
        "type": "battery",
        "soc_pct": first(fields, "soc"),
        "temp_c": first(fields, "temp"),
        "current_ma": first(fields, "current"),
        "voltage_mv": first(fields, "voltage"),
    }


def flight_mode_record(t: float, fields: Fields) -> dict:
    return {"t": t, "type": "flight_mode", "mode": first(fields, "type", "")}


def proto_records(block: Block) -> Iterator[dict]:
    """Records carried by the protobuf text body of a block."""
    if not block.body:
        return
    fields = parse_text_proto(block.body)
    if ENGINEERING_MARKER in block.header:
        yield engineering_record(block.t, fields)
    for battery in fields.get(BATTERY_MESSAGE, []):
        yield battery_record(block.t, battery)
    for mode in fields.get(FLIGHT_MODE_MESSAGE, []):
        yield flight_mode_record(block.t, mode)


def header_records(block: Block) -> Iterator[dict]:
    """Records carried by the header line itself."""
    if state := STATE_CHANGE.search(block.header):
        yield {"t": block.t, "type": "state", "from": state.group(1), "to": state.group(2)}
    elif beat := HEARTBEAT.search(block.header):
        mode, status, recording, battery, charging = beat.groups()
        yield {
            "t": block.t,
            "type": "heartbeat",
            "flight_mode": mode,
            "flight_status": status,
            "battery_pct": parse_value(battery),
            "charging": charging == "true",
            "recording": recording == "true",
        }
    elif stick := STICK.search(block.header):
        yaw, thrust, roll, pitch, gimbal = (float(value) for value in stick.groups())
        yield {
            "t": block.t,
            "type": "stick",
            "yaw": yaw,
            "thrust": thrust,
            "roll": roll,
            "pitch": pitch,
            "gimbal": gimbal,
        }


def content(record: dict) -> tuple:
    """Record values without the timestamp, used to detect repeats."""
    return tuple(json.dumps(value) for key, value in record.items() if key != "t")


def drop_repeats(records: Iterable[dict]) -> Iterator[dict]:
    """Drop engineering blocks seen before and unchanged heartbeat/battery/flight_mode."""
    seen_engineering: set[tuple] = set()
    last_by_type: dict[str, tuple] = {}
    for record in records:
        key = content(record)
        if record["type"] == "engineering":
            if key in seen_engineering:
                continue
            seen_engineering.add(key)
        elif record["type"] in ("heartbeat", "battery", "flight_mode"):
            if last_by_type.get(record["type"]) == key:
                continue
            last_by_type[record["type"]] = key
        yield record


def mark_in_flight(records: list[dict]) -> None:
    """Set in_flight on engineering records from the first FLYING_* state to the next IDLE.

    Not from TAKING_OFF: the frame reset lands somewhere during takeoff, and by FLYING_*
    it has always happened in our captures.
    """
    in_flight = False
    for record in records:
        if record["type"] == "state":
            if record["to"].startswith(FLYING_STATE_PREFIX):
                in_flight = True
            elif record["to"] == IDLE_STATE:
                in_flight = False
        elif record["type"] == "engineering":
            record["in_flight"] = in_flight


def parse_capture(path: Path) -> list[dict]:
    """All telemetry records of a capture log, sorted by t."""
    records: list[dict] = []
    for block in group_blocks(read_bodies(path)):
        records.extend(header_records(block))
        records.extend(proto_records(block))
    records.sort(key=lambda record: record["t"])
    records = list(drop_repeats(records))
    mark_in_flight(records)
    return records


def flight_spans(records: list[dict]) -> list[tuple[float, float]]:
    """(start, end) of each flight: TAKING_OFF until the following LANDING -> IDLE."""
    spans = []
    start: float | None = None
    for record in records:
        if record["type"] != "state":
            continue
        if record["to"] == TAKEOFF_STATE and start is None:
            start = record["t"]
        elif record["from"] == LANDING_STATE and record["to"] == IDLE_STATE and start:
            spans.append((start, record["t"]))
            start = None
    return spans


def write_jsonl(records: list[dict], out_path: Path) -> None:
    with out_path.open("w", encoding="utf-8") as out:
        for record in records:
            out.write(json.dumps(record) + "\n")


def print_summary(records: list[dict], out_path: Path) -> None:
    for record_type, count in sorted(Counter(r["type"] for r in records).items()):
        print(f"  {count:6d}  {record_type}")
    spans = flight_spans(records)
    if not spans:
        print("No complete flight (TAKING_OFF -> LANDING -> IDLE)")
    for start, end in spans:
        clock = time.strftime("%H:%M:%S", time.localtime(start))
        print(f"Flight from {clock}: {end - start:.1f} s")
    print(f"Saved {out_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("capture", type=Path, help="capture log from drone.telemetry.capture")
    parser.add_argument("-o", "--output", type=Path, help="default: capture path with .jsonl")
    args = parser.parse_args()
    out_path = args.output or args.capture.with_suffix(".jsonl")
    records = parse_capture(args.capture)
    write_jsonl(records, out_path)
    print_summary(records, out_path)


if __name__ == "__main__":
    main()
