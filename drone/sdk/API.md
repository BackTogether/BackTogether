# drone/sdk/API.md: the drone's HTTP API

Served on the Mac by the SDK, together with the hoverctl web page, by one process that owns the
drone (Bluetooth, sticks, video):

```bash
drone/fly                                              # real drone, http://127.0.0.1:8077
drone/fly 60 sd --bind 0.0.0.0                         # same, reachable from the robot
uv run python -m drone.sdk sim --http-port 8090        # simulator, same API
```

Same style as the robot bridge (`interfaces/ROBOT_API.md`): JSON in and out, every reply has
`Access-Control-Allow-Origin: *`, bad input is `400 {"error": str}`, a conflict is
`409 {"error": str}` (`"busy"` while an autopilot action runs: send `/stop` first). Clients:
[client.py](client.py) (Python 3.8, stdlib) and [ros_node.py](ros_node.py).

Units: metres, seconds, degrees. Positions are in the drone's **takeoff frame**: it resets at
every takeoff, x and y horizontal (y = 90 degrees clockwise of x seen from above), `alt` up.
`heading_deg` is the estimated heading: 0 = the frame's +x, + = turned right. It is an
estimate (see README.md, "For navigation/positioning work"), not a compass.

## Read
| Method | Reply |
|---|---|
| `GET /status` | the status below |
| `GET /events` | Server-Sent Events: `event: status` + `data: <the status JSON>`, 10 Hz |
| `GET /frame.jpg` | the latest frame (JPEG); `503` before the first |
| `GET /video.mjpg` | MJPEG stream (`multipart/x-mixed-replace`, each part has `Content-Length`) |
| `GET /sim/truth` | simulator only: true `x`, `y`, `alt`, `heading_deg`, `front_m` (wall ahead), `wall_hits` |

`GET /status`:
```json
{"state": "patrol", "message": null,
 "task": {"name": "patrol", "state": "running", "message": null, "started_at": 1763160000.0,
          "patrol": {"state": "flying", "loop": 1, "index": 0, "visits": 0, "waypoints": [], "options": {}}},
 "telemetry": {"time": 1763160000.0, "connected": true, "link_age_s": 0.2, "mode": "MANUAL",
   "flight_state": "flying_manual", "in_flight": true, "battery_pct": 82,
   "position": [1.2, 0.4, -1.5], "velocity": [0.3, 0.0, 0.0], "odometry_time": 1763159999.6,
   "odometry_age_s": 0.4, "altitude_m": 1.5, "camera_tilt_deg": null, "manual_session": true,
   "sticks_gate": "live", "stick_output": {"roll": 0, "pitch": 21, "thrust": 0, "yaw": 0, "gimbal": 0},
   "video_age_s": 0.03,
   "pose": {"x": 1.3, "y": 0.4, "z": -1.5, "alt": 1.5, "heading_deg": -12.0,
            "heading_sigma_deg": 14.0, "position_sigma_m": 0.13, "velocity": [0.3, 0.0, 0.0], "in_flight": true},
   "tilt": {"deg": -45.0, "low_deg": -45.0, "high_deg": -45.0, "anchored": true, "source": "estimated"}},
 "speed": {"max": 60.0, "sport": false},
 "recording": {"on": false, "last": null},
 "target": [3.0, 0.0, 1.5],
 "limits": {"x_min": -8, "x_max": 8, "y_min": -8, "y_max": 8, "min_alt_m": 0.5, "max_alt_m": 3,
            "low_battery_pct": 25, "max_speed": 60, "...": "..."},
 "estimator": {"name": "DeadReckoner", "yaw_rate_dps": 60.0},
 "remote_locked": false, "simulated": false, "updated_at": 1763160000.0}
```
- `state`: `idle`, or the running action: `goto`, `turn`, `tilt`, `patrol`, `remote` (a `/move`
  stream under the safety rules). `task.state`: `running`, `done`, `stopped` (STOP), `landed`
  (safety landing), `failed` (`message` says why).
- `telemetry.position`/`velocity` are the raw 1 Hz odometry (z down); `pose` is the estimate
  (10 Hz, extrapolated), `tilt` the camera tilt estimate (`deg` null until anchored).
- `sticks_gate`: `no session` (nothing sent), `session-neutral` (not MANUAL: sticks locked), `live`.

## Commands
| Method | Body | Reply |
|---|---|---|
| `POST /manual` | - | `200 {"ok", "error"}`: the app's Manual Control sequence. Then take off with the **drone's button** (the X1 has no takeoff request) |
| `POST /move` | `{"fwd", "right", "up", "turn"}` each -1..1 (fraction of the speed cap), `"seconds"` 0..5 (default 0.5), `"resume"` (bool) | `200 {"ok": true}`. Velocity command held for `seconds`, then neutral (the watchdog): stream it faster than that. `409` while an action runs, after a STOP (until `"resume": true`), or while the safety rules hold or fly home |
| `POST /goto` | `{"x", "y", "alt"?}` (alt default: current) | `202 {"accepted": true}`; `400` outside the geofence/altitude limits; `409` not flying in MANUAL or busy |
| `POST /turn` | `{"heading"}` or `{"by"}` (degrees, + = right) | `202` / `409` |
| `POST /tilt` | `{"deg"}` -90..15 (0 = level, -90 = down) | `202`: moves the camera to that estimated tilt, anchoring the estimate at a stop first if unknown (up to ~20 s); `409` if the sticks are locked |
| `POST /patrol` | `{"waypoints": [{"x", "y", "alt"?, "dwell_s"?, "scan"?}], "loops"? (1, 0 = forever), "dwell_s"? (2), "scan"? (false), "return_home"? (true), "land"? (false)}` | `202` / `400` / `409` |
| `POST /stop` | - | `200 {"ok": true}`: STOP. Cancels any action at once, sticks neutral, the drone hovers |
| `POST /land` | - | `200 {"ok", "error"}`: STOP, then land |
| `POST /speed` | `{"max"?: 0..100, "sport"?: bool}` | `200 {"ok", "max", "sport"}`: speed cap in stick %, sport flag |
| `POST /photo` | - | `200 {"ok": true, "path"}`: saves the latest frame on the Mac (`recordings/photo_*.jpg`); `409` before the first frame |
| `POST /record` | `{"on": bool}` | `200 {"ok", "on", "path", "error"?}`: MP4 of the live stream on the Mac (`recordings/video_*.mp4`, no re-encode, starts at the next keyframe) |
| `POST /drone/photo` | - | `200 {"ok", "error"}`: full-quality photo on the drone's SD card (the app's shutter: `camera_take_photo_request`) |
| `POST /drone/record` | `{"on": bool}` | `200 {"ok", "error"}`: recording on the drone's SD card (`camera_start_video_request`) |

Simulator only: `POST /sim/button` (the takeoff button), `/sim/battery {"pct"}`,
`/sim/cut {"what": "link" | "telemetry" | "video", "seconds"}`, `/sim/push {"dx", "dy"}` (a gust,
in the takeoff frame).

## Safety (all commands)
- **STOP always wins**: `/stop`, `/land`, the web page's STOP, LAND or any flight key cancel the
  running action within one tick (0.1 s) and lock `/move` until a client resumes.
- Targets outside the geofence box or the altitude limits are refused (`400`).
- Low battery, or the estimated position leaving the fence or going above the ceiling: the
  drone flies back to the takeoff point and lands.
- Bluetooth link, telemetry (odometry) or video lost: hover, resume when it comes back, land
  after `loss_land_s` (8 s).
- Every autopilot or `/move` command is capped at `limits.max_speed` (stick %).
Limits are set when the server starts (`--fence`, `--max-alt`, `--no-video-required`).
