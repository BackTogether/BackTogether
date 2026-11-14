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
| `GET /frame.jpg` | the latest frame (JPEG); `503` before the first. Headers `X-Frame-Time` and `X-Drone-Pose` (below) |
| `GET /video.mjpg` | MJPEG stream (`multipart/x-mixed-replace`, each part has `Content-Length`, `X-Frame-Time` and `X-Drone-Pose`) |
| `GET /mission` | the lost-child mission's state (see [Mission](#mission-lost-child-search)) |
| `GET /sim/truth` | simulator only: true `x`, `y`, `alt`, `heading_deg`, `front_m` (wall ahead), `wall_hits`, and `child` (the fake child: `fwd`, `left` in the launch frame, `x`, `y` in the takeoff frame, `map` once the launch pose is known, `in_view`, true `bbox`) |

**Frame metadata** (every frame, for detectors): `X-Frame-Time` = unix time (the Mac's clock,
3 decimals) when the picture reached the Mac; send it back as `frame_time` with a detection.
`X-Drone-Pose` = compact JSON `{"x", "y", "alt", "heading_deg", "tilt_deg"}` of the estimate at
that time (takeoff frame; `tilt_deg` null until anchored). Browsers: the headers are exposed
(`Access-Control-Expose-Headers`). Python: `DroneClient.frames_with_meta()` / `frame_with_meta()`.

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
   "video_age_s": 0.03, "video_lag_ms": 9,
   "pose": {"x": 1.3, "y": 0.4, "z": -1.5, "alt": 1.5, "heading_deg": -12.0,
            "heading_sigma_deg": 14.0, "position_sigma_m": 0.13, "velocity": [0.3, 0.0, 0.0], "in_flight": true,
            "heading_source": "flow", "flow_yaw_rate_dps": 0.1, "flow_confidence": 0.95},
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
- `pose.heading_source`: `flow` (yaw measured from the video, drone/sdk/flow.py) or `sticks`
  (yaw-stick model); `flow_yaw_rate_dps` (+ = right) and `flow_confidence` (0..1) are null while
  the video measures nothing. `estimator.yaw_rate_dps` is re-calibrated online by the video.
- `video_lag_ms`: mean time from a picture's arrival to its decoded frame (null without frames).
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
in the takeoff frame), `/sim/child {"fwd"?, "left"?, "detect"?, "miss"?}` (the fake child, metres
ahead of / left of the launch point, default 3.5 / 1.5; `detect` false hides it from the fake
detector, `miss` = fraction of frames it misses, 0.2).

## Mission (lost-child search)
`drone/sdk/mission.py`. One autopilot action that flies the whole demo: **launching** (waits
for the button takeoff; enters MANUAL itself if needed) → **climbing** (to `patrol_alt_m`, camera
to `patrol_tilt_deg`) → **aligning** (flies `align_m` straight ahead: the odometry displacement
is the robot's forward axis in the drone frame; the drone must be launched facing the robot's
forward) → **patrolling** → **approaching** (stops the patrol, flies over the detection, camera to
`approach_tilt_deg`, then stop-and-correct visual servo until the bbox centre is at the image
centre; no detection for `lost_timeout_s` or over `approach_timeout_s` → back to patrolling) →
**reporting** (the child's position is in `child`, a photo is saved, hovers 3 s) → **orbiting**
(down to `orbit_alt_m`, `orbit_turns` circles of `orbit_radius_m` facing the child, camera at
atan(height / radius) down) → **returning** (to the takeoff point) → **landed**. Other ends:
**stopped** (any STOP, LAND or flight key: the drone hovers, `stopped_in` says where), **aborted**
(`/mission/abort`: back to the takeoff point and land), **failed** (`message`; lands at the
takeoff point if it flew). Patrol timeout without a detection: returns and lands ("child not
found"). Safety rules (geofence, battery, link loss) apply throughout.

| Method | Body | Reply |
|---|---|---|
| `POST /mission/start` | the fields below, all optional except the waypoints | `202 {"accepted": true}`; `400` bad field or waypoint outside the fence; `409` busy |
| `POST /mission/target` | `{"frame_time"?, "bbox": [x0, y0, x1, y1], "confidence"? (1), "label"?}`: bbox as fractions of the image (0..1, y down); `frame_time` = the frame's `X-Frame-Time` (default: now) | `200 {"accepted", "state", "reason"}`: accepted while searching (launching..approaching) with `confidence >= min_confidence`; a fresh one (frame < 3 s old) during the patrol starts the approach |
| `POST /mission/launch_pose` | `{"x", "y", "yaw"}`: the robot's pose in `map` at takeoff (metres, radians, ROS) | `200 {"ok": true}`: enables the `map` coordinates and the goal. The ROS node sends it from tf |
| `POST /mission/abort` | - | `200 {"ok": true}`: back to the takeoff point and land; `409` no mission running |

`POST /mission/start` fields (defaults): `waypoints` `[{"x", "y", "alt"?, "dwell_s"?, "scan"?}]`
in `frame`: `"launch"` (default: x metres ahead of the robot at launch, y metres to its left),
`"drone"` (the takeoff frame) or `"map"` (needs the launch pose by the end of the aligning leg);
`strategy` `"waypoints"` (the SDK's Patrol, from the nearest waypoint) or `"external"` (the
mission hovers under the safety rules and accepts `/move` from an outside patrol, e.g.
`drone.patrol.dvd --sdk`; when the approach starts `/move` answers `409 "stopped: the mission
flies the drone now"`, which ends that patrol); `loops` (0 = until found), `dwell_s` (1), `scan`
(false), `patrol_alt_m` (2.0, above the takeoff point), `patrol_tilt_deg` (-40; null = leave it),
`align_m` (1.5; 0 = no leg, then only frame "drone"), `approach_tilt_deg` (-90), `orbit_alt_m`
(1.2), `orbit_radius_m` (1.5), `orbit_turns` (2; 0 = none), `standoff_m` (0.8), `launch_height_m`
(0: the takeoff point above the floor, the robot's top), `target_height_m` (0.5: the detected
point above the floor), `min_confidence` (0.5), `takeoff_timeout_s` (120), `patrol_timeout_s`
(600), `approach_timeout_s` (90), `lost_timeout_s` (10).

`GET /mission` (also `"mission"` in `/status` and every `/events` message):
```json
{"state": "reporting", "step": "hovering over the child", "message": null, "stopped_in": null,
 "started_at": 1794694020.6, "since": 1794694117.8, "states": [{"state": "launching", "at": 1794694020.6}],
 "detections": 104, "attempts": 1, "config": {"...": "the start fields"},
 "last_detection": {"frame_time": 1794694117.5, "bbox": [0.45, 0.4, 0.55, 0.6], "confidence": 0.9,
                    "label": "leo", "received_at": 1794694117.8, "age_s": 0.3},
 "alignment": {"forward_deg": -19.1, "leg_m": 1.69},
 "launch_pose": {"x": 1.0, "y": 2.0, "yaw": 0.5},
 "map_from_drone": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.167},
 "pose_map": {"x": 3.5, "y": 4.7, "z": 2.0, "yaw": 0.84},
 "child": {"id": 1, "time": 1794694117.8, "photo": "recordings/photo_....jpg",
           "drone": {"x": 2.9, "y": -2.57}, "launch": {"x": 3.6, "y": 1.4},
           "map": {"x": 3.44, "y": 5.02}, "goal": {"x": 2.94, "y": 4.39, "yaw": 0.89}},
 "patrol": null, "safety": null, "accepts_moves": false, "updated_at": 1794694118.0}
```
- `child.drone` is in the takeoff frame; `launch` (x ahead, y left of the robot at launch) needs
  the aligning leg; `map` and `goal` (`standoff_m` short of the child on the line from the launch
  pose, facing it) need the launch pose too. `null` until known.
- `map_from_drone`: the ROS frame `drone_takeoff` (the takeoff frame with y flipped, as in
  ros_node's `/drone/odom`) in `map`. `pose_map`: the drone in `map` now.
- `safety`: why the safety rules hold the drone (returning home, riding out a loss), or null.

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
