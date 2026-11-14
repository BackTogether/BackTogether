# drone/sdk: Python SDK, autopilot and HTTP API for the HoverAir X1

Fly the X1 from code on the laptop (macOS, Linux; Windows natively), from the simulator, or
from the robot over HTTP. It builds on `drone/hoverctl` (Bluetooth link, 30 Hz sticks, live
video) and adds: a `Drone` object with telemetry and video streams, a pose estimate, a camera
tilt estimate, a slow closed-loop navigator, waypoint patrols, safety rules, photos and MP4
recording, a simulator, an HTTP API and a ROS client.

| File | What |
|---|---|
| `drone.py` | `Drone`: connection, motion, telemetry/video streams, photos, recording, tilt |
| `estimate.py` | `Estimator` interface and `DeadReckoner` (the default pose estimate) |
| `tilt.py` | `TiltEstimator`: camera tilt from the gimbal stick, calibrated on our drone |
| `camera.py` | approximate camera intrinsics (focal length, FOV) |
| `navigate.py` | `Navigator` (goto, turn_to, climb_to, hover, supervise) and `calibrate_yaw` |
| `patrol.py` | `Patrol` of waypoints with dwell, 360 scan and per-waypoint callbacks |
| `safety.py` | `SafetyLimits`: geofence, altitude, battery, loss timeouts, max speed |
| `sim.py` | the simulator (`connect_sim`): same interface, nothing leaves the laptop |
| `server.py`, `__main__.py` | the web page + HTTP API ([API.md](API.md)) |
| `client.py`, `ros_node.py` | Python 3.8 stdlib client and an example rospy node, for the robot |
| `recording.py`, `telemetry.py` | MP4/photo helpers; Telemetry, Pose, Tilt and streams |

## Quick start
Run from the repo root. The drone takes **one** controller: one process owns its Bluetooth link,
sticks and video. Close the Hover app first.

**The usual way: `drone/fly`.** It now runs `python -m drone.sdk serve`: the same web page as
before (plus the compass, map, tilt, PHOTO and REC), and the HTTP API on the same port.
```bash
drone/fly                          # http://127.0.0.1:8077: page + API, cap 60, SD video
drone/fly 60 sd --bind 0.0.0.0     # reachable from the robot (anyone on that network can fly it!)
drone/fly 60 sd --yaw-rate 68      # after calibrate_yaw (below)
```

**Simulator** (no drone, no Bluetooth; needs `ffmpeg`): the same page and API.
```bash
uv run python -m drone.sdk sim --http-port 8090    # open http://127.0.0.1:8090
curl -XPOST localhost:8090/manual; curl -XPOST localhost:8090/sim/button   # MANUAL + takeoff button
```

**Python on the laptop** (in-process; not while drone/fly runs):
```python
from drone.sdk import Drone, Navigator
from drone.sdk.sim import connect_sim

with Drone.connect(max_speed=50) as drone:  # or: with connect_sim() as drone:
    drone.wait_connected()
    drone.enter_manual()  # sim: then drone.link.press_button()
    drone.wait_for_takeoff()  # press the drone's button: PRE_TAKEOFF, TAKING_OFF, flying
    drone.move(fwd=0.4, seconds=1.0)  # -1..1 of the speed cap; STOP raises Interrupted
    drone.move(turn=0.5, seconds=1.0)
    for telemetry in drone.telemetry_stream(timeout=2):  # 10 Hz; or drone.on_telemetry(callback)
        print(telemetry.pose, telemetry.tilt)
        break
    jpeg = drone.latest_frame()  # or drone.frames() / drone.on_frame(callback)
    drone.take_photo()
    drone.start_recording()
    Navigator(drone).goto(1.0, 0.0)
    drone.stop_recording()
    drone.land()
```
Leaving the `with` block always sends neutral, and lands if it left with an exception
(Ctrl-C included). Without an exception the drone is left hovering.

**From the robot** (ROS1, Python 3.8): copy `client.py` (and `ros_node.py`) to the robot.
```python
from client import DroneClient

drone = DroneClient("http://<mac>:8077")
drone.patrol([{"x": 2, "y": 0}, {"x": 2, "y": 2}], loops=0, scan=True)
for status in drone.events():  # SSE, 10 Hz
    print(status["state"], status["telemetry"]["pose"])
```
`rosrun <pkg> ros_node.py _url:=http://<mac>:8077` publishes `/drone/odom`, `/drone/battery`,
`/drone/image/compressed` and `/drone/state`, and takes `/drone/cmd_vel` (Twist),
`/drone/patrol` (JSON String), `/drone/stop` and `/drone/resume` (see its docstring). **Network:**
the Mac's Wi-Fi is on the drone's access point, so the robot must reach the Mac another way:
the robot joins `HoverX1_060039` too, or an Ethernet/USB cable between them.

## Patrol example
```python
from drone.sdk import Drone, Navigator, Patrol, PatrolOptions, SafetyLimits, Waypoint


def seen(index, waypoint, pose, jpeg):  # on arrival at each waypoint
    open(f"recordings/wp{index}.jpg", "wb").write(jpeg or b"")


with Drone.connect(max_speed=50) as drone:
    drone.enter_manual()
    drone.wait_for_takeoff()
    navigator = Navigator(drone, SafetyLimits(x_min=-4, x_max=4, y_min=-4, y_max=4, max_alt_m=2.5))
    navigator.climb_to(1.5)
    patrol = Patrol(
        navigator,
        [Waypoint(2, 0), Waypoint(2, 2, alt=2.0), Waypoint(0, 2, scan=True)],
        PatrolOptions(loops=2, dwell_s=2, return_home=True, land=True),
        on_waypoint=seen,
    )
    print(patrol.run())  # "done: 2 loop(s)", or "returned home and landed: low battery 24%"
```
Over HTTP: `POST /patrol` with the same fields (API.md). Waypoints are in the takeoff frame:
the frame of the 1 Hz odometry, which starts at the takeoff point and whose axes are **not**
aligned with where the camera points (the estimator learns that angle as it flies).

## Safety rules
Checked every 0.1 s by the Navigator for every autopilot action (goto, turn, tilt, patrol) and
for outside `/move` streams (ROS cmd_vel, the bounce patrol), which run under a `remote` watcher:
- **STOP always wins:** `Drone.stop()`, `POST /stop` or `/land`, the page's STOP or LAND, or any
  flight key in the page cancel the running action at once (STOP pre-empted a patrol in 0.21 s in
  the sim) and lock `/move` until a client sends `"resume": true`. The page shows "STOPPED ·
  hovering" for 4 s (also for a STOP from the API or ROS: `/api/state` → `estimate.stopped_at`),
  and says so in red if the server never confirmed it. The bounce patrol over HTTP ends when STOP
  comes while it sets the camera tilt (before, its first `/move` resumed and it flew on).
- Targets outside the geofence box (default ±8 m) or the altitude limits (0.5-3 m) are refused.
- Low battery (25 %), or the estimated position leaving the fence or above the ceiling: fly back
  to the takeoff point and land.
- Bluetooth link, odometry or video lost: hover; resume if it comes back; land after 8 s.
- Every command is capped at `max_speed` (60 % stick) whatever the page's speed cap is.
- The stick interlock of hoverctl stays underneath: nothing moves unless the drone reports MANUAL.
- Losing focus in the page only releases the page's own keys: a ROS patrol keeps flying when
  the browser window is in the back.

## Photos and recording
- On the laptop: `Drone.take_photo()` saves the latest frame (`recordings/photo_<time>.jpg`);
  `start_recording()` / `stop_recording()` remux the received H.264 into an MP4
  (`recordings/video_<time>.mp4`, `ffmpeg -c copy`, no re-encode, starting at the next keyframe;
  finalised on stop and on close). Page: **P** / **R** or the PHOTO / REC buttons over the video
  (REC shows a red dot and the time). API: `POST /photo`, `POST /record {"on": bool}`.
- On the drone's SD card, full quality: `Drone.drone_photo()` / `drone_record(on)`, the page's
  "photo on drone" / "rec on drone". They send what the app's shutter and record buttons send
  (decompiled `com.zerozerorobotics.preview.c` → `d40.x0.x()` / `w(bool)`, over the Wi-Fi RPC
  `http://<DRONE_IP>:1234/rpc`): `camera_take_photo_request=244` (Empty) and
  `camera_start_video_request=243` (bool); logcat shows `cameraStartVideoRequest success`.
  Untested from the Mac. The drone does not tell us whether it records: the button shows what
  we last asked.

## Camera tilt (`tilt.py`)
The X1 never reports the gimbal angle, so `TiltEstimator` integrates the gimbal stick actually
sent (after the ramp, cap and interlock): rate = 0.124 deg/s per stick % above a 13.5 % dead
zone, clamped at the stops. It starts **unknown** (the page says "unknown — hold U or O to a
stop") and keeps the range of tilts that still fits; holding U or O long enough to reach a stop
from anywhere squeezes the range onto that stop: the estimate is then **anchored**, and every
later push into a stop re-anchors it. `Drone.set_tilt(deg)` / `POST /tilt` drive it to a target
(anchoring first if needed). `Drone.camera_tilt()`, `/status` → `telemetry.tilt`.

**Calibration (2026-11-14, stick log `recordings/sticks-2026.jsonl` + video motion):**
bottom stop -90° (straight down, seen in the video); top stop +15° ±4 (a frame at the top stop,
camera on the floor: vanishing point of two vertical column edges vs the horizon); travel
about 105°; stop to stop 10.0 s at stick 100 (ramp included) and 32 s at stick 40, the same up and
down; fit 0.124 deg/s per % above 13.5 %. Constants: `GIMBAL_*` in `tilt.py`.

**Recalibrating:** fly (or hold the drone) in MANUAL with the web page and `--stick-log`.
1. Hold U for 15 s: the camera is at the top stop.
2. Hold O at speed 100 and time the full sweep to the bottom stop in the video; hold U back up.
3. Repeat at speed 40.
4. Rate per stick % = travel / (sweep time) / (stick - dead zone): two speeds give both unknowns.
   Check against the stick log (`gimbal` of each packet) and the video's vertical motion (frame to
   frame shift of the horizon / f, see camera.py). Put the numbers in `tilt.py`.

**Camera intrinsics (`camera.py`):** focal length about 850 px at 1280×720, principal point at the
image centre, so 74° horizontal and 46° vertical FOV. From one measurement: ±10 %. No distortion
model.

## Bounce patrol (`drone/patrol`, by @andresgilvicente) on the SDK
The random-bounce patrol (fly straight until a wall is close, turn a random angle) now reads
frames, state, altitude, camera tilt and the calibrated yaw rate from the SDK and sends its sticks
through it (`drone/patrol/sdkdrone.py`, which replaced `hoverweb.py`), so the safety rules above
apply. Its front distance comes from `front_distance.py` (Depth Anything V2, optional heavy deps)
or, in the simulator, from the room's walls (`--fake-depth`).
```bash
uv run python -m drone.patrol.bounce --sim --fake-depth --duration 120          # in-process sim
uv run python -m drone.sdk sim --http-port 8090 --room 4 &                       # or over HTTP:
curl -XPOST localhost:8090/manual; curl -XPOST localhost:8090/sim/button
uv run python -m drone.patrol.bounce --sdk http://127.0.0.1:8090 --fake-depth
# real drone: drone/fly, MANUAL + takeoff by the button, then
uv run --no-project --with numpy --with pillow --with torch --with transformers \
    python -m drone.patrol.bounce --sdk http://127.0.0.1:8077 --dry-run          # then without --dry-run
```
It sets the camera to `--tilt` (-45°) first and uses the live tilt estimate as the depth
geometry's pitch, the measured 74° HFOV, and `estimator.yaw_rate_dps` for its timed turns. STOP or
LAND in the page ends it. Windows: `drone/patrol/winfly` (it now starts `drone.sdk serve`).

## Operating systems
- **macOS**: as above. `enter_manual()` joins the drone's Wi-Fi with `networksetup`.
- **Linux**: Bluetooth through BlueZ + bleak (`uv run --with bleak`; your user may need the
  `bluetooth` group; no `setcap` needed). Wi-Fi auto-join through NetworkManager (`nmcli`, in
  hoverctl's macwifi.py). `ffmpeg` from the distribution.
- **Windows**: native Python (WSL2 has no Bluetooth and does not receive the drone's UDP video:
  see `drone/patrol/winfly`, which runs the server on Windows from WSL). Join the drone's Wi-Fi by
  hand or with `winfly wifi`. `ffmpeg` on the PATH (`winget install ffmpeg`).
- The SDK itself has no OS-specific calls: paths through pathlib, `ffmpeg` found with
  `shutil.which` (a clear error if missing), stdlib HTTP.

## First flight with the SDK (real drone)
Open space (no people within 3 m), fresh battery, Hover app closed, `drone/fly` running, the page
open in **one** tab, a hand on STOP (Space).
1. MANUAL CONTROL in the page, then the drone's button: wait for `flying_manual`.
2. `curl -XPOST localhost:8077/move -d '{"fwd": 0.3, "seconds": 1}'`: it should move forward
   ~0.5 m and stop. Then `-d '{"turn": 0.5, "seconds": 1}'`: it turns right, and the page's compass
   ring turns with it.
3. `curl -XPOST localhost:8077/tilt -d '{"deg": -30}'`: the camera sweeps to a stop, then to -30°.
4. `curl -XPOST localhost:8077/goto -d '{"x": 1, "y": 0}'`, watch the map; STOP if it heads off.
5. Calibrate the yaw rate (needs ~4 m clear ahead; stop drone/fly first, it owns the drone):
   ```python
   from drone.sdk import Drone, Navigator, calibrate_yaw

   with Drone.connect() as drone:
       drone.enter_manual()
       drone.wait_for_takeoff()
       print(calibrate_yaw(Navigator(drone)))  # deg/s at full stick: drone/fly ... --yaw-rate N
       drone.land()
   ```
6. A 2-waypoint patrol, small and slow:
   `curl -XPOST localhost:8077/patrol -d '{"waypoints": [{"x": 1.5, "y": 0}, {"x": 1.5, "y": 1.5}], "loops": 1, "land": true}'`.
7. `POST /land` (or LAND twice in the page).

## For navigation/positioning work
**What the drone gives us:** position and velocity from `FlightEngineeringInfo` at a fixed
**1 Hz**, in a frame that **resets at every takeoff**, z down, and whose axes are **not aligned with
the heading**. No heading/yaw, no attitude, no gimbal angle. Video 1280×720 at 30 fps.

**The `Estimator` interface** (`estimate.py`), what the Drone feeds and the Navigator flies on:
```python
class Estimator(Protocol):
    def update(self, telemetry: Telemetry) -> None: ...  # 10 Hz (odometry inside changes at 1 Hz)
    def on_command(self, sticks: Axes, dt: float) -> None: ...  # sticks actually sent, 30 Hz
    def pose(self) -> Pose:
        ...  # x, y, z (takeoff frame), heading_deg, heading_sigma_deg,
        # position_sigma_m, velocity, in_flight
```
Plug a new one in with `Drone(..., estimator=MyEstimator())`, `connect_sim(estimator=...)` or
`Drone.connect(estimator=...)`. Frames are there for it: `drone.on_frame(callback)` (JPEG bytes,
30 fps) or `drone.frames()`. The page's compass and map draw whatever `pose()` says.

**The default, `DeadReckoner`:**
- heading = integral of the yaw stick sent × `yaw_rate_dps` (60 deg/s at full stick until
  `calibrate_yaw` measures it; the sim's true rate is 66), uncertainty growing with each turn and
  0.5 deg/s of drift;
- heading fix from the direction of travel: when the drone has flown a steady horizontal
  command without yaw for 1.5 s, heading = velocity direction − command direction (1-D Kalman);
  this is also how it learns the unknown angle between the takeoff frame and the camera;
- position = last odometry sample + its velocity × age (at most 1 s);
- a new takeoff starts a new frame (heading reset, uncertainty 45°).

**Its limits:** position only at 1 Hz and lags; extrapolation overshoots when the drone stops;
heading drifts while hovering and turning (no sensor), and only improves when flying straight;
odometry drift accumulates (the sim adds a random walk); no absolute position: the frame is
lost at landing.

**The Navigator** (`navigate.py`) flies on `pose()`: far from a target it sends a proportional
command turned into body sticks with the estimated heading; within 1.5 m it does
stop-and-correct (neutral, wait for a fresh odometry sample, short pulse). Its gains and the
pulse length are constants at the top. `turn_to` is open loop on the estimated heading.
`Navigator.supervise()` applies the safety rules to commands from elsewhere.

**Suggested next steps for a teammate:**
1. **Optical-flow heading from the video:** between frames, the mean horizontal flow dx (pixels)
   of distant features gives the yaw: yaw_rate ≈ atan(dx_px / f) × 30 fps with f ≈ 850 px
   (`camera.py`). Fuse it in `on_command`/`update` instead of the stick model: no calibration,
   no drift while hovering. Translation also causes flow: use the top of the image, or subtract
   the odometry velocity.
2. **Teach-and-repeat patrol:** fly the route once by hand, store keyframes (JPEG + odometry +
   heading) every metre; on replay, match the live frame to the next keyframe (ORB features,
   homography) to correct heading and lateral offset. Robust indoors, no markers.
3. **Visual localisation without markers:** a map of keyframes with features (or a small VO/SLAM
   such as ORB-SLAM3 monocular, scale from the odometry) gives an absolute position that survives
   the takeoff-frame reset.
Each one is an `Estimator`: the navigator, patrols, API, ROS node and compass work unchanged.

## Known limits
- The SDK has only flown in the simulator: the real-drone path reuses hoverctl's tested link,
  sticks and video, but goto/patrol gains, m/s per stick %, and the yaw rate are untested on the X1.
- Takeoff is by the drone's button only (the X1 refuses takeoff requests).
- Heading is an estimate (see above); waypoints far apart accumulate drift.
- `/move` and patrols need the drone in MANUAL and the laptop on its Wi-Fi.
- The API has no authentication: bind to 127.0.0.1 unless the robot needs it.
- Drone-side photo/record and the HD preview are untested.
