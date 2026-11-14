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
| Commands, queries, telemetry (mode, state, battery, land, Wi-Fi start) | BLE service `0xFE45`: write `6e400002-b5a3-f393-e0a9-e50e24dcca9e`, notify `6e400003-…`. `RpcRequest`/`RpcResponse`/`PushMessage` protobuf in ADGP framing (`adgp.py`) | ✅ **Works from the Mac** (`hoverctl status`), 2026-11-14 |
| Telemetry push | BLE `PushMessage`, `flight_engineering_info_event=16`: position/velocity xyz, motor rpm, battery V/A | ✅ Fixed **1 Hz** (614 events measured) |
| Manual sticks | UDP → `<DRONE_IP>:19091` from local port 19091, about 30 Hz, raw protobuf `RockerControlCommand` | Format known; not yet flown from the Mac |
| Wi-Fi RPC | HTTP POST `http://<DRONE_IP>:1234/rpc`, body = raw `RpcRequest`, reply = raw `RpcResponse`. The app sends `heartbeat_info_request=70` here at 1 Hz | From app code; not yet confirmed on the wire |
| Live video | UDP, the drone pushes to the client's **port 7078** after `enter_preview_mode_request=254` over the Wi-Fi RPC | Request built (`start_video()`); not yet confirmed on the drone |

**Connecting over BLE.** No pairing and no encryption. Right after connecting, the app sends `get_ble_encry_list_request` with its phone id. The drone replies with its list of bound phone ids. The Mac reuses the owner's phone id (`ble.py`), and the drone accepts it. The drone takes **one BLE client at a time**, so close the Hover app before using `hoverctl`.

**Wi-Fi.** The drone's access point only runs after a `wifi_start_request` over BLE, and that request sets the password. `hoverctl` sends it on every BLE connect with the team test password **`<DRONE_WIFI_PASSWORD>`** (`--wifi-password` to change it; rotate after the hackathon). A drone restart turns the access point off until the next BLE connect.

The full schema is `proto/h130.proto`, generated from the app.

Key messages:
- `RockerControlCommand { int64 seq_num=1; int64 timestamp=2 (ms); RawManualControlCommand command=3 }`
- `RawManualControlCommand { control_source_type=1 (APP=1); is_sport_mode=2; roll=3; pitch=4; thrust=5; yaw=6; gimbal=7 }`. The axes are floats from -100 to 100.
- `RpcRequest { int32 id=6; oneof request {...} }`:
  - `abort_flight_request=40`: this is **land**, and the app uses it
  - `set_flight_mode_request=54` (a `FlightModeConfig`): HOVER=1, REVEAL=2, FOLLOW=3, ORBIT=4, MANUAL=255
  - `takeoff_request=1023` (`TakeOffParams`, `trigger_source` APP=1). Untested on the X1.
  - `wifi_start_request=25` (`WifiParams`: ssid, password, type=AP, country). It sets the Wi-Fi password; see above.
  - `enter_preview_mode_request=254` (`EnterPreview`: `udp_server_ip`, `flight_mode_manual`, `camera_start_preview{enable, codec}`, `connect_type`): starts live video to `udp_server_ip:7078`. Stop it with `camera_start_preview_request=240` (enable=false), then `exit_preview_mode_request=250`.
  - `set_flight_mode_to_manual_request=46`: on its own, the drone acks it and then **falls back to the mode set on the drone** (seen 2026-11-14). See "Manual Control in the app" below.
- No live roll/pitch/yaw: the app has a `flight_analytics_log_event=31` path with attitude, but the X1 never sent it in our captures.

## Behaviour learned in test flights
- Every takeoff so far used the **drone's button**. The state sequence is PRE_TAKEOFF, then TAKING_OFF, then FLYING_TRAJ (Follow) or FLYING_MANUAL.
- **Follow needs a person in view.** It refuses takeoff with `TAKEOFF_NOT_TRACKING` unless the camera sees someone. Hold it on your palm, camera facing you.
- **Follow tracking runs on the drone.** The app sends no steering during Follow.
- **The app's joystick screen switches the drone to MANUAL.**
- **Manual Control is unlocked** on our drone.

### Manual Control in the app (observed 2026-11-14)
1. "Searching for Hover": hold the drone's power button. To pair a new phone, press mode + power together and the light flashes blue. Then tap **Connect** (BLE), then "Got it".
2. On the home screen, tap **Manual Control**. The app shows "Connecting to Hover's Wi-Fi" (`wifi_start` over BLE, then the phone joins `HoverX1_…`).
3. The joystick screen opens with the low-res live video, and the drone shows **"Waiting to take off"**. Take off with the drone's button.

So Manual Control is a **session**: Wi-Fi joined, then preview started with `flight_mode_manual`, then a stick stream. It is more than one mode request. `hoverctl` is being changed to run the same sequence from its MANUAL button.

## Gotchas
- **Don't use VPN-based capture (PCAPdroid).** The app binds its sockets to the drone's Wi-Fi, a VPN blocks that, and the video goes black.
- **USB tethering from the Samsung test phone doesn't work on macOS.** The phone only offers RNDIS, which macOS can't use. When the Mac joins the drone's Wi-Fi it loses internet. To stay online, use an iPhone hotspot over USB or Bluetooth tethering. A Wi-Fi hotspot can't work, because the Mac has one Wi-Fi card.
- **macOS may switch back** from the drone's Wi-Fi (it has no internet) to a known network. Turn off Auto-Join on your usual network while testing.
- **The Hover app logs the account password in plain text** to logcat. Redact logs before sharing them.

Raw captures and the decompiled app live outside git, in `../drone-re/` on rorosaga's machine.

## Usage (`drone/hoverctl/`)
Stdlib only, plus the `ffmpeg` CLI for video (`brew install ffmpeg`). Nothing to download on the drone's Wi-Fi. Run from the repo root:

```bash
uv run --with bleak python -m drone.hoverctl web   # browser UI on http://127.0.0.1:8000: video, sticks, STOP, drone state over Bluetooth
uv run python -m drone.hoverctl web --no-ble   # same without Bluetooth: no state, no interlock
uv run python -m drone.hoverctl sticks         # keyboard teleop in the terminal
uv run python -m drone.hoverctl video          # receive UDP 7078: stats + dump to recordings/ (gitignored)
uv run python -m drone.hoverctl sticks --dry-run   # print packets, send nothing
```

- **Keys:** W/S pitch, A/D roll, I/K thrust, J/L yaw, U/O gimbal, Space = neutral (web: STOP), Q quits the terminal teleop. Web UI: left stick is thrust/yaw, right stick is pitch/roll.
- **Web UI:** video on the left, telemetry on the right (stick output per axis against the `--max` cap, packets, errors, video stats, the hex of the first datagrams while nothing decodes). The top bar shows DRY RUN, the time since the last state update (red after 1 s), and the video state (red when frames stop for 2 s). **V** (or the top-right button, or a click on the small window) shrinks the video to a picture-in-picture so the sticks get bigger. Keys work together (diagonals), the on-screen sticks follow them, and every key is released when the tab loses focus. Mouse or touch dragging works too.
- **Safety:** `--max` (default 50 of 100) caps every axis. Ramp like the app: start at 28, then +3 per tick, and release is instant. Sticks go neutral 0.3 s after the last input: browser tab closed, key released (terminal: 0.15 s after the last auto-repeat). Exit, Ctrl-C, SIGTERM or an error send 0.5 s of neutral.
- **Bluetooth in the web UI:** `web` starts the `DroneLink` (ble.py, needs `bleak`; close the Hover app first, the drone takes one Bluetooth client). The DRONE section on top of the right panel shows link, mode (large), flight state, battery, odometry, storage and the last exception, each with its age (grey after 5 s), plus the mode and battery in the top bar. Buttons: MANUAL, LAND (click twice within 3 s) and WIFI (password of 8+ characters, SET). They are POST endpoints `/api/drone/{manual,land,wifi}` returning `{ok, error}`; `/api/state` has the `drone` snapshot (`null` with `--no-ble`). Without `bleak` the page says so and the sticks stay locked.
- **Drone Wi-Fi autostart:** a drone restart turns its Wi-Fi off, so after every Bluetooth (re)connect `DroneLink` sends `wifi_start_request` like the app does, with the team's test password `<DRONE_WIFI_PASSWORD>` (`DEFAULT_WIFI_PASSWORD` in `ble.py`, to be rotated). Change it with `web --wifi-password PW` (also on `status`), skip it with `--no-wifi-autostart`. The DRONE panel shows `wifi: on · HoverX1_060039` or the error. The manual WIFI field still works. Join the laptop to that Wi-Fi (SSID `HoverX1_060039`, the password above) to get video and sticks.
- **Live video start:** the drone only pushes video after the app asks. `enter_preview_mode_request` (RpcRequest field 254, `EnterPreview{udp_server_ip = our <DRONE_SUBNET>.x address, camera_start_preview{enable, H.264}, connect_type APP}`) goes over the Wi-Fi RPC (HTTP POST `http://<DRONE_IP>:1234/rpc`), because the app never sends it over Bluetooth. The destination port is not in the request (fixed 7078). VIDEO START / VIDEO STOP buttons (`/api/drone/{video,video-stop}`, `DroneLink.start_video()/stop_video()`); the page also starts video once by itself when the laptop has a <DRONE_SUBNET>.x address and no video packets arrived. While video runs a 1 Hz heartbeat RPC is sent, as the app does. Needs the Bluetooth link (`web` without `--no-ble`) and the laptop on the drone's Wi-Fi.
- **Stick interlock:** with Bluetooth active (and not `--dry-run`) the sender sends **no UDP packet at all**, not even neutral, unless the drone reports MANUAL (reading under 5 s old, link up). It stops within one tick when the mode leaves MANUAL, and the UI shows the sticks as LOCKED. `--dry-run` still prints packets and shows what would happen. `--no-ble` turns the interlock off (warning at startup).
- **Terminal limits:** it only sees auto-repeat, so a single tap moves for about 0.5 s, and only one key works at a time. Use the web UI for diagonals.
- **Video:** the payload format is still a guess (`video_payload.py`). We handle bare Annex-B (H.264/H.265) and RTP/H.264. The first 10 datagrams print in hex, and the UI shows them while nothing decodes. If a fixed header shows up, try `--skip N`. Force the codec with `--codec h264|hevc`.
- **Testing without the drone:** `uv run python -m drone.hoverctl.devsend [--mode rtp | --codec hevc]` sends an ffmpeg test pattern to 127.0.0.1:7078. `--replay recordings/<dump>.udp` replays a real capture with its original timing.
