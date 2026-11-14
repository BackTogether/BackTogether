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
| Manual sticks | UDP → `<DRONE_IP>:19091` from local port 19091, every 33 ms, raw protobuf `RockerControlCommand` (no header). Same in the app's old (`f40/d`, the X1's `PreviewActivity`) and new (`y30/y`) screens | 2026-11-14: sent from the Mac, the drone ignored them. Fix pending a test, see "Why the sticks were ignored" |
| Wi-Fi RPC | HTTP POST `http://<DRONE_IP>:1234/rpc`, body = raw `RpcRequest`, reply = raw `RpcResponse`. The app sends `heartbeat_info_request=70` here at 1 Hz | From app code; not yet confirmed on the wire |
| Live video | UDP, the drone pushes **RTP/H.264, 1280×720, 30 fps** (payload type 96) to the client's **port 7078** after `set_udp_server_ipaddress=245` + `camera_start_preview=240` (the Manual control session) | ✅ **Works on the Mac** (2026-11-14, received during the Manual control session) |

**Connecting over BLE.** No pairing and no encryption. Right after connecting, the app sends `get_ble_encry_list_request` with its phone id. The drone replies with its list of bound phone ids. The Mac reuses the owner's phone id (`ble.py`), and the drone accepts it. Then the app sends `set_time_utc_request=37` (`RealTimeMessage`: UTC seconds, whole-hour zone) and `connect_type_request=75` (`CONNECT_TYPE_APP`), on every connect (logcat 14:55:32, logcat2 15:33 and 15:41). `DroneLink` does the same. The drone takes **one BLE client at a time**, so close the Hover app before using `hoverctl`.

**Wi-Fi.** The drone's access point only runs after a `wifi_start_request` over BLE, and that request sets the password. `hoverctl` sends it on every BLE connect with the team test password **`<DRONE_WIFI_PASSWORD>`** (`--wifi-password` to change it; rotate after the hackathon). A drone restart turns the access point off until the next BLE connect.

The full schema is `proto/h130.proto`, generated from the app.

Key messages:
- `RockerControlCommand { int64 seq_num=1; int64 timestamp=2 (ms); RawManualControlCommand command=3 }`
- `RawManualControlCommand { control_source_type=1; is_sport_mode=2; roll=3; pitch=4; thrust=5; yaw=6; gimbal=7 }`. The axes are floats (fixed32) from -100 to 100. `control_source_type` is `ConnectType`: the app's button pad sends `APP_CROSS_KEY=6` (with the 28, then +3 per tick ramp seen in `manual_control.log`), its analog joystick sends `APP=1`. The schema is **proto2**: the app's generated code tracks presence, and it sets roll, pitch, thrust and yaw on every tick, so **zero axes are on the wire** (`1d 00000000`...). `seq_num` starts at 2 per screen, `timestamp` is `System.currentTimeMillis()`.
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

So Manual Control is a **session**, not one mode request. From `logcat2` (15:42:38), everything after the Wi-Fi start goes over the **Wi-Fi RPC**, not BLE:
0. `connect_type_request=75` (`CONNECT_TYPE_APP`) as soon as the phone joins the Wi-Fi (`WifiConnectController sendConnectType success`, 15:42:38.395).
1. `heartbeat_info_request=70` every 1 s. The first reply means "drone connected", and the 30 Hz stick stream starts, neutral included.
2. `switch_to_manual_capture_request=249` (true).
3. `set_udp_server_ipaddress_request=245` (the client's IP, so video goes there on port 7078).
4. `set_flight_mode_to_manual_request=46` (true).
5. `camera_start_preview_request=240` (enable, session 0, AVC).
6. About 0.5 s later the drone pushes `FLIGHT_MODE_CHANGE_EVENT MANUAL`.

### Why the sticks were ignored (2026-11-14)
First Mac flight: Manual session on, mode MANUAL, 5,728 packets sent with no error, and the drone never moved. Three differences with the app, all fixed in `hoverctl`; the next test tells which one mattered:
1. **No `connect_type_request(APP)`.** The app sends it after every Bluetooth connect and every Wi-Fi connect (see above). We never did. It is the only request in the app that names the controlling client, so it is the most likely cause. `DroneLink` now sends it over Bluetooth on connect (the panel shows "app announced ✓") and over Wi-Fi as the first Manual step (`connect ✓`).
2. **`control_source_type` 1 instead of 6.** Every manual flight in our logs used the button pad (`manual_control.log`: 28, 31, 34...), which sends `APP_CROSS_KEY=6`. `hoverctl` now sends 6. `--control-source app` sends 1 again, as the analog joystick does.
3. **Zero axes left out.** We wrote proto3 style; the app writes all axes every tick. Now we do too.

The UDP transport itself was already right: in both app screens a `DatagramSocket` bound to port 19091 sends the raw `RockerControlCommand` to `<DRONE_IP>:19091`, bound to the drone's Wi-Fi network, with no header and no handshake. It sends only while the Manual screen is open (`setSendEnable`) and the heartbeat says "connected" (`setConnectState`).

To exit: `camera_start_preview(false)`, then `set_flight_mode_to_manual(false)`, then `exit_preview_mode_request`. `enter_preview_mode_request=254` belongs to the home-page preview, not this screen. `hoverctl`'s **Manual control** button runs this sequence (`DroneLink.enter_manual()`).

## Gotchas
- **Don't use VPN-based capture (PCAPdroid).** The app binds its sockets to the drone's Wi-Fi, a VPN blocks that, and the video goes black.
- **USB tethering from the Samsung test phone doesn't work on macOS.** The phone only offers RNDIS, which macOS can't use. When the Mac joins the drone's Wi-Fi it loses internet. To stay online, use an iPhone hotspot over USB or Bluetooth tethering. A Wi-Fi hotspot can't work, because the Mac has one Wi-Fi card.
- **macOS may switch back** from the drone's Wi-Fi (it has no internet) to a known network. Turn off Auto-Join on your usual network while testing.
- **The Hover app logs the account password in plain text** to logcat. Redact logs before sharing them.

Raw captures and the decompiled app live outside git, in `../drone-re/` on rorosaga's machine.

## Usage (`drone/hoverctl/`)
Stdlib only, plus the `ffmpeg` CLI for video (`brew install ffmpeg`). Nothing to download on the drone's Wi-Fi. Run from the repo root:

```bash
drone/fly                                          # the usual way: web UI on http://127.0.0.1:8077, Bluetooth on, cap 60, stick log in recordings/
drone/fly 100                                      # same with the full stick range
uv run --with bleak python -m drone.hoverctl web   # browser UI on http://127.0.0.1:8000: video, sticks, STOP, drone state over Bluetooth
uv run python -m drone.hoverctl web --no-ble   # same without Bluetooth: no state, no interlock
uv run python -m drone.hoverctl sticks         # keyboard teleop in the terminal
uv run python -m drone.hoverctl video          # receive UDP 7078: stats + dump to recordings/ (gitignored)
uv run python -m drone.hoverctl sticks --dry-run   # print packets, send nothing
```

- **Keys:** W/S forward/back, A/D left/right, I/K up/down, J/L turn left/right, U/O camera up/down, Space = neutral (web: STOP), Q quits the terminal teleop. Web UI: left stick up = climb, right = turn right; right stick up = forward, right = fly right. Readouts name the direction (`fwd 28`, `up 40`), never a packet sign. The DRONE panel shows **altitude = −position z** (the drone frame has z down) only in flight (`taking_off`, `flying_*`, `fly_back`, `landing`). On the ground it says "on ground" and dims the raw position: its origin resets at every takeoff, and after landing the values jump to another frame (#13).
- **Web UI:** video on the left, telemetry on the right (stick output per axis against the `--max` cap, packets, errors, video stats, the hex of the first datagrams while nothing decodes). The top bar shows DRY RUN, the time since the last state update (red after 1 s), and the video state (red when frames stop for 2 s). **V** (or the top-right button, or a click on the small window) shrinks the video to a picture-in-picture so the sticks get bigger. Keys work together (diagonals), the on-screen sticks follow them, and every key is released when the tab loses focus. Mouse or touch dragging works too.
- **Safety:** `--max` (default 50 of 100) caps every axis. Ramp like the app: start at 28, then +3 per tick, and release is instant. Sticks go neutral 0.3 s after the last input: browser tab closed, key released (terminal: 0.15 s after the last auto-repeat). Exit, Ctrl-C, SIGTERM or an error send 0.5 s of neutral.
- **Bluetooth in the web UI:** `web` starts the `DroneLink` (ble.py, needs `bleak`; close the Hover app first, the drone takes one Bluetooth client). The DRONE section on top of the right panel shows link, mode (large), flight state, battery, odometry, storage and the last exception, each with its age (grey after 5 s), plus the mode and battery in the top bar. Buttons: MANUAL CONTROL, LAND (click twice within 3 s) and WIFI (password of 8+ characters, SET). They are POST endpoints `/api/drone/{manual,land,wifi}` returning `{ok, error}`; `/api/state` has the `drone` snapshot (`null` with `--no-ble`) including `manual_entry` (`session`, `steps`, `error`). Without `bleak` the page says so and the sticks stay locked.
- **Drone Wi-Fi autostart:** a drone restart turns its Wi-Fi off, so after every Bluetooth (re)connect `DroneLink` sends `wifi_start_request` like the app does, with the team's test password `<DRONE_WIFI_PASSWORD>` (`DEFAULT_WIFI_PASSWORD` in `ble.py`, to be rotated). Change it with `web --wifi-password PW` (also on `status`), skip it with `--no-wifi-autostart`. The DRONE panel shows `wifi: on · HoverX1_060039` or the error. The manual WIFI field still works. Join the laptop to that Wi-Fi (SSID `HoverX1_060039`, the password above) to get video and sticks.
- **Manual Control (`DroneLink.enter_manual()`):** the MANUAL CONTROL button runs what the app does when you tap Manual Control, in the same order, all over the Wi-Fi RPC (so the laptop must be on `HoverX1_060039`, else the error is "join HoverX1_060039 (password <DRONE_WIFI_PASSWORD>) first"): `connect_type_request(APP)` (if it fails the error shows and the sequence goes on, like in the app), `heartbeat_info_request` (then 1 Hz), `switch_to_manual_capture_request(true)`, `set_udp_server_ipaddress_request(our ip)`, `set_flight_mode_to_manual_request(true)` (over Wi-Fi, not Bluetooth), `camera_start_preview_request(enable, session 0)`. That last one starts the video, so no separate VIDEO START is needed. The panel shows `connect / wifi / preview / manual / video` with a check mark each, plus the drone's mode. Takeoff is still the drone's button.
- **Live video start:** the drone only pushes video after the app asks. `enter_preview_mode_request` (RpcRequest field 254, `EnterPreview{udp_server_ip = our <DRONE_SUBNET>.x address, camera_start_preview{enable, H.264}, connect_type APP}`) goes over the Wi-Fi RPC (HTTP POST `http://<DRONE_IP>:1234/rpc`), because the app never sends it over Bluetooth. The destination port is not in the request (fixed 7078). VIDEO START / VIDEO STOP buttons (`/api/drone/{video,video-stop}`, `DroneLink.start_video()/stop_video()`); the page also starts video once by itself when the laptop has a <DRONE_SUBNET>.x address and no video packets arrived. While video runs a 1 Hz heartbeat RPC is sent, as the app does. Needs the Bluetooth link (`web` without `--no-ble`) and the laptop on the drone's Wi-Fi.
- **Stick interlock:** with Bluetooth active (and not `--dry-run`) the sender sends **no UDP packet at all** outside a manual session (before `enter_manual()` succeeded, after VIDEO STOP, or after 5 missed heartbeats). Inside the session it sends neutral packets all the time, like the app's joystick screen, and non-neutral values only while the drone reports MANUAL (reading under 5 s old, link up). STOP or a mode change away from MANUAL zero the sticks within one tick, and the UI shows the sticks as LOCKED. `--dry-run` still prints packets and shows what would happen. `--no-ble` turns the interlock off (warning at startup).
- **Stick diagnostics (STICKS panel):** `gate` is `no session` (nothing sent), `session-neutral` (neutral only: no fresh MANUAL reading) or `live`; it turns red while you hold a key and the gate is not live. `non-neutral sent` counts packets with a stick value; `last packet sent` shows it with our signs, its packet values and its hex; `control source` shows 6 or 1. The top bar's **keys** cell lights while the page holds a flight key, so a dark cell while you press means the page doesn't have the keyboard focus. `--stick-log PATH` appends every packet sent as a JSON line (`seq`, `time`, `axes`, `packet`, `control_source`, `hex`).
- **Stick signs** (see table): hoverctl's own signs (`Axes`) are + = forward, right, up, turn right, camera up. The packet uses other signs, and `protocol.py` converts in one place (`PITCH_FORWARD`, `ROLL_RIGHT`, `THRUST_UP`, `YAW_RIGHT`, `GIMBAL_UP`). `--dry-run` prints the packet values. Packet value → motion, relative to the camera:

  | Packet field | −100 | +100 | Evidence |
  |---|---|---|---|
  | pitch (4) | forward | back | app: the button labelled Forward sends pitch −; flights in #13 |
  | roll (3) | left | right | flights in #13; our flight 3: roll + moved 90° clockwise of pitch − (z-down frame) |
  | thrust (5) | up | down | app: "Up" sends −; our flight 3: thrust − took z from −0.58 to −1.42 m, thrust + back to −0.05 |
  | yaw (6) | turn left | turn right | flights in #13 only |
  | gimbal (7) | camera up | camera down | app only: its gimbal-up button sends −100 (not seen in flight) |

  The app's `ManualControl` log line is read back from the builder that was just sent, so the log is the packet. The app's pad is drawn for someone facing the camera, so its "Right" and "Turn right" send roll − and yaw −. The drone has a Head/Back `manual_control_heading_type` setting, which the app only shows as swapped Forward/Backward labels, so the drone probably applies it itself. Everything above is with `HeadingTypeHead` (the default, seen in our logcat); Back is untested.
- **Terminal limits:** it only sees auto-repeat, so a single tap moves for about 0.5 s, and only one key works at a time. Use the web UI for diagonals.
- **Video:** the payload format is still a guess (`video_payload.py`). We handle bare Annex-B (H.264/H.265) and RTP/H.264. The first 10 datagrams print in hex, and the UI shows them while nothing decodes. If a fixed header shows up, try `--skip N`. Force the codec with `--codec h264|hevc`.
- **Testing without the drone:** `uv run python -m drone.hoverctl.devsend [--mode rtp | --codec hevc]` sends an ffmpeg test pattern to 127.0.0.1:7078. `--replay recordings/<dump>.udp` replays a real capture with its original timing.
