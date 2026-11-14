# drone/: HoverAir X1

Everything for the **HoverAir X1** (Zero Zero Robotics): the small foldable white drone with caged propellers.
Code that talks to the drone, its adapters and its notes all live here.

**Owners: `@rorosaga` and `@andresgilvicente`.** Tag both of them on anything drone-related: board issues use label `area: drone`, assigned to one of them; questions use the `request` label.

## Goal
Control the drone from a laptop instead of the phone app: manual sticks, Follow mode, land, status and live video. The CLI and a small web app live in `drone/hoverctl/`.

## Protocol (reverse-engineered from app v3.12.0, 2026-11-14)
The drone is the original X1 ("HOVER CAMERA X1"), Wi-Fi SSID `HoverX1_<last 6 of SN>`, drone IP `<DRONE_IP>`.

| What | Channel | Status |
|---|---|---|
| Commands and queries (flight mode, land, takeoff, Wi-Fi start, status) | BLE, `RpcRequest` protobuf inside the "ADGP" framing (header + crc8) | Messages known; BLE UUIDs and framing bytes still to confirm |
| Manual sticks | UDP → `<DRONE_IP>:19091` from local port 19091, about 30 Hz, raw protobuf `RockerControlCommand` | ✅ Known |
| Live video | UDP, **the drone sends to the phone's port 7078**; packets of 2 KB or less; H.264 or H.265 | Payload format to confirm |
| Heartbeat or state | HTTP to `<DRONE_IP>`, about 1 Hz | Seen, not decoded |

The full schema is `proto/h130.proto`, generated from the app.

Key messages:
- `RockerControlCommand { int64 seq_num=1; int64 timestamp=2 (ms); RawManualControlCommand command=3 }`
- `RawManualControlCommand { control_source_type=1 (APP=1); is_sport_mode=2; roll=3; pitch=4; thrust=5; yaw=6; gimbal=7 }`. The axes are floats from -100 to 100.
- `RpcRequest { int32 id=6; oneof request {...} }`:
  - `abort_flight_request=40`: this is **land**, and the app uses it
  - `set_flight_mode_request=54` (a `FlightModeConfig`): HOVER=1, REVEAL=2, FOLLOW=3, ORBIT=4, MANUAL=255
  - `takeoff_request=1023` (`TakeOffParams`, `trigger_source` APP=1). Untested on the X1.
  - `wifi_start_request=25` (`WifiParams`: ssid, password, type=AP). **The app sets the Wi-Fi password over BLE on each connection**, so it isn't printed anywhere.

## Behaviour learned in test flights
- Every takeoff so far used the **drone's button**. The state sequence is PRE_TAKEOFF, then TAKING_OFF, then FLYING_TRAJ (Follow) or FLYING_MANUAL.
- **Follow needs a person in view.** It refuses takeoff with `TAKEOFF_NOT_TRACKING` unless the camera sees someone. Hold it on your palm, camera facing you.
- **Follow tracking runs on the drone.** The app sends no steering during Follow.
- **The app's joystick screen switches the drone to MANUAL.**
- **Manual Control is unlocked** on our drone.

## Gotchas
- **Don't use VPN-based capture (PCAPdroid).** The app binds its sockets to the drone's Wi-Fi, a VPN blocks that, and the video goes black.
- **USB tethering from the Samsung test phone doesn't work on macOS.** The phone only offers RNDIS, which macOS can't use. When the Mac joins the drone's Wi-Fi it loses internet, so script the switch.
- **The Hover app logs the account password in plain text** to logcat. Redact logs before sharing them.

Raw captures and the decompiled app live outside git, in `../drone-re/` on rorosaga's machine.

## Usage (`drone/hoverctl/`)
Stdlib only, plus the `ffmpeg` CLI for video (`brew install ffmpeg`). Nothing to download on the drone's Wi-Fi. Run from the repo root:

```bash
uv run python -m drone.hoverctl web            # browser UI on http://127.0.0.1:8000: video, two sticks, STOP
uv run python -m drone.hoverctl sticks         # keyboard teleop in the terminal
uv run python -m drone.hoverctl video          # receive UDP 7078: stats + dump to recordings/ (gitignored)
uv run python -m drone.hoverctl sticks --dry-run   # print packets, send nothing
```

- **Keys:** W/S pitch, A/D roll, I/K thrust, J/L yaw, U/O gimbal, Space = neutral (web: STOP), Q quits the terminal teleop. Web UI: left stick is thrust/yaw, right stick is pitch/roll.
- **Safety:** `--max` (default 50 of 100) caps every axis. Ramp like the app: start at 28, then +3 per tick, and release is instant. Sticks go neutral 0.3 s after the last input: browser tab closed, key released (terminal: 0.15 s after the last auto-repeat). Exit, Ctrl-C, SIGTERM or an error send 0.5 s of neutral.
- **Terminal limits:** it only sees auto-repeat, so a single tap moves for about 0.5 s, and only one key works at a time. Use the web UI for diagonals.
- **Video:** the payload format is still a guess (`video_payload.py`). We handle bare Annex-B (H.264/H.265) and RTP/H.264. The first 10 datagrams print in hex, and the UI shows them while nothing decodes. If a fixed header shows up, try `--skip N`. Force the codec with `--codec h264|hevc`.
- **Testing without the drone:** `uv run python -m drone.hoverctl.devsend [--mode rtp | --codec hevc]` sends an ffmpeg test pattern to 127.0.0.1:7078. `--replay recordings/<dump>.udp` replays a real capture with its original timing.
