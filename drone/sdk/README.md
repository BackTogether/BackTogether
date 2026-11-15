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
| `flow.py` | `VideoYaw`: yaw measured from the live video (optical flow), optional numpy |
| `tilt.py` | `TiltEstimator`: camera tilt from the gimbal stick, calibrated on our drone |
| `camera.py` | approximate camera intrinsics (focal length, FOV) |
| `navigate.py` | `Navigator` (goto, turn_to, climb_to, hover, supervise) and `calibrate_yaw` |
| `patrol.py` | `Patrol` of waypoints with dwell, 360 scan and per-waypoint callbacks |
| `mission.py` | the lost-child mission: launch, align, patrol, approach, report, orbit, return ([below](#for-the-robot-team-ros1)) |
| `approach_track.py` | the mission's default approach: camera-steered close-in, then camera at -90 and centred over the child, the fix |
| `mission_geometry.py` | its frames (drone, launch, map) and the camera model: bbox → floor point (a body's feet), and back for the sim |
| `vision.py` | the detector's boxes per frame, for the web page's video overlay (`/vision`) |
| `safety.py` | `SafetyLimits`: geofence, altitude, battery, loss timeouts, max speed |
| `sim.py` | the simulator (`connect_sim`): same interface, nothing leaves the laptop |
| `server.py`, `__main__.py` | the web page + HTTP API ([API.md](API.md)) |
| `map_view.py`, `map_image.py`, `splat.py` | the page's map view: drone, robot, child on the robot's map and a splat ([below](#map-view)) |
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
`/drone/patrol` (JSON String), `/drone/stop` and `/drone/resume` (see its docstring), plus the
mission topics of [For the robot team](#for-the-robot-team-ros1). **Network:** the Mac's Wi-Fi is
on the drone's access point, so the robot must reach the Mac another way: the robot joins
`HoverX1_060039` too, or an Ethernet/USB cable between them.

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

**Camera intrinsics (`camera.py`):** focal length 627 px at 1280×720, principal point at the
image centre, so 91° horizontal and 60° vertical FOV (pinhole, no distortion model). Fitted on
2026-11-14 to three exact 360° spins (the video yaw integrates to 360° per revolution at sticks
100/80/60, within 0.6 %); the earlier 850 px (one vanishing-point measurement) made video yaw
~1.3× too small.

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

## DVD patrol (`drone/patrol/dvd.py`): reflection turns, floor-boundary distance
Flies straight and, at a wall, turns by the reflection angle (incidence = reflection, off both
walls in a corner; a random turn when the wall's direction is unclear), so the path sweeps the
room. Front distance: `floor_boundary.py` takes the depth model's map, finds in each column
where the floor ends (the column's farthest point: floor depth grows upward, an obstacle's
grows downward), and turns that row into metres with the camera height and tilt. It never uses
the model's metric scale. A line through those contact points gives the wall's direction.
`camera_height.py` filters the altitude (jumps, out-of-range values) and re-scales the height
from each straight approach: the distance must shrink as fast as the odometry moves.
```bash
uv run --offline --no-project --with numpy --with pillow \
    python -m drone.patrol.dvd --sim --duration 300 --pillars --plot /tmp/dvd.png   # sim room
# real drone: drone/fly 60 sd --fence 15, MANUAL + takeoff, then
drone/patrol/live dvd-dry      # decisions only;  drone/patrol/live dvd   to fly
# a recorded flight through the perception, with its odometry:
HF_HUB_OFFLINE=1 uv run --offline --no-project --with numpy --with pillow --with torch \
    --with transformers --with av python -m drone.patrol.replay recordings/<dump>.udp \
    --push recordings/<push>.bin --start 22:07:15 --end 22:07:40
```
Why not the old front distance (2026-11-14 wall approach, odometry as ground truth): it read
2.1-2.8 m whenever the drone was within ~1.3 m of the wall. Three causes: `--takeoff-height 1.0`
after a floor takeoff (camera 2.72 m instead of 1.72: x1.6 on every distance); the floor band
used for the scale check was wall, not floor, once the floor left the image (so the reading
could never drop below the floor distance of the image's bottom rows); and the model's own
metric scale was 1.3-2x too far and jumped up to 5x on the plain white wall. The floor boundary
measured the same approach within 0.1 m (median) at 72 fps (Small on the Mac's GPU, MPS).

## Clear patrol (`drone/patrol/clear.py`): always turn the same way until clear
The DVD patrol's perception, SDK plumbing, holds and panel, with a deterministic state machine:
FORWARD until the front distance is under `--stop-at` (1.0 m), HOVER `--hover-s` (1 s), TURN in
place always `--turn` (right) at `--turn-input` until every fresh reading for `--clear-for-s`
(0.5 s) is beyond `--clear-at` (4 m) or a clear view; unreadable or low-confidence readings
(< 80 % of the corridor readable) are not clear. A full 360° (SDK heading) without that: hover,
turn on to the heading with the largest distance seen, and go if it is beyond stop-at + 0.5 m,
else hover "boxed in" until STOP. The panel shows phase, front distance, degrees turned, decision.
```bash
uv run --offline --no-project --with numpy --with pillow \
    python -m drone.patrol.clear --sim --duration 300 --pillars --plot /tmp/clear.png
drone/patrol/live clear-dry    # decisions only;  drone/patrol/live clear   to fly
```
In the 20 m sim hall (300 s) it never touched a wall or pillar (34-35 % coverage, DVD 40 %), but
each turn ends at the first clear heading, ~90° off the wall, so it loops the room ~1 m from the
walls (perimeter only, the pillars in the middle are never met).

## Lawnmower sweep (`drone/patrol/sweep.py`): parallel lanes over a big hall
The predictable way to search a sports hall from the video: straight lanes along the hall, one
lane spacing apart (right, right, then left, left at the walls). Start anywhere, facing along
the hall. Same perception, SDK plumbing, holds and panel as dvd.py.
- **Align**: 3 m straight ahead without yaw; the odometry displacement is the lane axis (the
  odometry frame's axes are not the drone's), and the SDK heading learns it meanwhile.
- **Lane**: holds the heading (SDK heading, video yaw when `flow`) and the lane's line (odometry,
  roll <= 0.12), climbs to and holds `--alt` (2 m). It ends where the floor boundary sees
  something closer than `--stop-at` (1.0), but at least 0.15 m before the floor leaves the
  image (1.51 m at 2 m and -30°, so 1.66 m: nothing nearer can be measured), or at `--length`. A wall (a >= 2 m wide floor-contact line, or where an end wall
  was seen before) ends the lane; anything else (person, pillar) is waited for `--wait` (3 s),
  then the lane ends there, the spot is reported, and the next lane goes on the same way past
  it. Obstacles count within `--corridor` (0.8 m) of the path.
- **Turn**: hover 1 s, 90° toward the sweep side (relative, measured heading), probe that way: a
  wall closer than spacing + 0.5 m ends the side (or one last lane 1.5 m off it when half a
  lane fits); else cross one spacing (odometry), 90° again, next lane back.
- **Other side**: back across to the start lane, flown again in full (the first time it only
  covered what was ahead of the start), then lanes on the other side; `--width` stops early.
- **Coverage** (`coverage.py`): 0.5 m floor cells the camera saw (footprint from height, tilt,
  74° HFOV, heading, position; up to 4 m ahead, cut at the obstacle in front). The panel shows
  m² seen, % of the hall as far as its walls are known, and a strip across the hall (`@` =
  seen). At the end the 3 largest unseen patches (>= 2 m²) get a straight revisit leg
  (`--no-revisit`), the rest is reported.
- **Lane spacing**: the footprint's width at the bottom of the image x (1 - `--overlap` 0.2):
  3.48 m at 2 m and -30° -> **2.78 m** (printed at start; `--lane` overrides).
- **Safety**: the SDK's rules; STOP / LAND ends it for good; it hovers without fresh depth, tilt
  or altitude; it slows to `--sticks-speed` (0.7x) after 10 s of heading on the stick model.

```bash
uv run --offline --no-project --with numpy --with pillow python -m drone.patrol.sweep \
    --sim --sim-hall 30 20 --pillars --plot /tmp/sweep.png    # sim hall (+ --sim-frame-offset 5
                                                             #   --sim-yaw-error 0.1: heading errors)
# real drone: drone/fly 60 sd --fence 20 (hall), MANUAL + takeoff from the floor, facing along it
drone/patrol/live sweep-dry                      # decisions only, sends nothing
drone/patrol/live sweep --length 5 --width 5     # first test: 2 lanes of 5 m
drone/patrol/live sweep                          # the hall; --width / --length bound the area
```
As the mission's patrol: over HTTP start the mission with `"strategy": "external",
"patrol_tilt_deg": -30`, then `drone/patrol/live sweep --pitch 30` once it is patrolling (the
mission owns the tilt). A detection hands the drone to the mission; if it loses the child and
patrols again, the sweep resumes where it was (back to its lane line first). In Python:
`MissionConfig(patrol=MissionSweep([...flags]), patrol_tilt_deg=-30)`.

Simulator, 30 x 20 m hall, start in the middle facing along it (`--sim-hall 30 20`):

| run | time | flown | coverage (2 / 5 / 8 / 11 min) | overlap | wall / pillar hits | lanes off-line rms (max) | lane direction error |
|---|---|---|---|---|---|---|---|
| plain | 661 s | 228 m | 40 / 57 / 82 / 100 % | 89 % | 0 / - | 0.03 m (0.19) | <= 2.6° |
| 5° frame offset + 10 % yaw-rate error | 667 s | 226 m | 39 / 58 / 83 / 100 % | 87 % | 0 / - | 0.05 m (0.22) | <= 1.8° |
| 5 pillars (people) | 690 s | 207 m | 35 / 54 / 92 / 98 % | 67 % | 0 / 2 (*) | 0.03 m (0.17) | <= 3.4° |

(*) Both were passes ~0.5 m from a pillar at the edge of the obstacle corridor; fixed since by
stopping as soon as a contact is 0.15 m from the image's near edge (offline: any pillar
within 0.7 m of the path now stops the drone); the rerun was clean until stopped at lane 3.

Overlap = cells seen by 2+ lanes: the 4 m-deep footprint (6.7 m wide at 4 m) is much wider than
its near edge, which sets the spacing. To finish faster, `--lane 4` (6 lanes instead of 8) or
`--alt 2.5` (spacing 3.5 m) still covers everything within 4 m ahead. A full 30 x 20 m sweep takes ~11 min at `--speed 0.6` (0.72 m/s in the sim):
more than half a battery, so for the demo bound it (`--width`, `--length`) or fly faster.

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
        # position_sigma_m, velocity, in_flight, heading_source, flow_yaw_rate_dps, flow_confidence
```
Optional: `on_yaw_step(step: YawStep)`, the yaw measured from the video (`flow.py`).
Plug a new one in with `Drone(..., estimator=MyEstimator())`, `connect_sim(estimator=...)` or
`Drone.connect(estimator=...)`. Frames are there for it: `drone.on_frame(callback)` (JPEG bytes,
30 fps) or `drone.frames()`. The page's compass and map draw whatever `pose()` says.

**The default, `DeadReckoner`:**
- heading = integral of the yaw stick sent × `yaw_rate_dps` (24.2 deg/s at full stick, from the
  360° spins; the sim's true rate is 66), uncertainty growing with each turn and 0.5 deg/s of drift;
- **yaw measured from the video** (`flow.py`, needs numpy: `uv run --with numpy`, e.g.
  `uv run --with bleak --with numpy python -m drone.sdk serve ...`; without numpy the SDK says so
  at start and uses the sticks only). The decoder's ffmpeg also writes a 320×180 gray copy of every
  frame; every 6 frames six horizontal strips are phase-correlated with the same strips 0.2 s
  earlier, and the row profile of the sideways shift gives the yaw for any camera tilt. A trusted
  step (confidence ≥ 0.3, no gimbal or sideways stick and speed < 1 m/s in the last second)
  replaces the stick model's turn over the same interval, so decoding delay does not matter; steady
  turns nudge `yaw_rate_dps` toward what the video saw (online calibration). `pose().heading_source`
  is `"flow"` while trusted steps arrive (the page's compass says "estimated · video"), `"sticks"`
  otherwise; `flow_yaw_rate_dps` and `flow_confidence` are in `/status`, SSE and `/api/state`. Not
  on Windows (no second ffmpeg pipe) and not in the simulator (its test pattern does not turn);
- heading fix from the direction of travel: when the drone has flown a steady horizontal
  command without yaw for 1.5 s, heading = velocity direction − command direction (1-D Kalman);
  this is also how it learns the unknown angle between the takeoff frame and the camera;
- position = last odometry sample + its velocity × age (at most 1 s);
- a new takeoff starts a new frame (heading reset, uncertainty 45°).

**Its limits:** position only at 1 Hz and lags; extrapolation overshoots when the drone stops;
without the video yaw (no numpy, blank or blurred images, sideways flight, gimbal moving) the
heading follows the stick model and drifts while hovering and turning; the video yaw has a scale
error of about ±5-8 % (focal length, lens distortion) and no absolute reference;
odometry drift accumulates (the sim adds a random walk); no absolute position: the frame is
lost at landing.

**The Navigator** (`navigate.py`) flies on `pose()`: far from a target it sends a proportional
command turned into body sticks with the estimated heading; within 1.5 m it does
stop-and-correct (neutral, wait for a fresh odometry sample, short pulse). Its gains and the
pulse length are constants at the top. `turn_to` is open loop on the estimated heading.
`Navigator.supervise()` applies the safety rules to commands from elsewhere.

**Suggested next steps for a teammate:**
1. Optical-flow heading: done (`flow.py`, see above).
2. **Teach-and-repeat patrol:** fly the route once by hand, store keyframes (JPEG + odometry +
   heading) every metre; on replay, match the live frame to the next keyframe (ORB features,
   homography) to correct heading and lateral offset. Robust indoors, no markers.
3. **Visual localisation without markers:** a map of keyframes with features (or a small VO/SLAM
   such as ORB-SLAM3 monocular, scale from the odometry) gives an absolute position that survives
   the takeoff-frame reset.
Each one is an `Estimator`: the navigator, patrols, API, ROS node and compass work unchanged.

## For the robot team (ROS1)
The lost-child demo is one mission on the Mac (`mission.py`, API.md "Mission"); the robot runs
`ros_node.py` (Python 3.8, rospy + tf2_ros + `client.py`, stdlib only) which turns it into topics.

**Network.** The Mac's Wi-Fi is on the drone's access point (`HoverX1_060039`, drone
`<DRONE_IP>`, the Mac gets a `<DRONE_SUBNET>.x` by DHCP: `ipconfig getifaddr en0` on the Mac, or
`local_ip` in the page's video panel). The robot reaches the Mac either by joining
`HoverX1_060039` too (password in drone/README.md; the robot's own AP may stop while its Wi-Fi
is a client) or, better, by an Ethernet cable Mac ↔ robot with static addresses (e.g. Mac
`<LAPTOP_A_IP>`, robot `<LAPTOP_B_IP>`, netmask 255.255.255.0): the drone link stays on the Mac's Wi-Fi.
On the Mac the server must listen on every interface:
```bash
drone/fly 60 sd --bind 0.0.0.0                      # port 8077; anyone on those networks can fly it
curl http://<mac ip>:8077/mission                   # from the robot: must answer JSON
```

**Run the node on the robot** (copy `drone/sdk/client.py` and `drone/sdk/ros_node.py` into one
folder, e.g. `~/drone/`):
```bash
cd ~/drone && python3 ros_node.py _url:=http://<mac ip>:8077              # rosrun works too
# params: _send_goal:=true _standoff:=0.8 _map_frame:=map _robot_frame:=base_footprint
#         _goal_topic:=/move_base_simple/goal _auto_launch_pose:=true
#         _map_pose_rate:=2 _send_map:=true _map_topic:=/map           (the page's map view)
```

**Topics** (all in the node's docstring)
| Topic | Type | What |
|---|---|---|
| `/drone/image/compressed` | `sensor_msgs/CompressedImage` | JPEG, `header.stamp` = when the picture reached the Mac, on the robot's clock (offset learned from the 10 Hz status), frame `drone_camera` |
| `/drone/detection` (in) | `std_msgs/String` | JSON `{"stamp": <the image's header.stamp, s>, "bbox": [x0, y0, x1, y1], "confidence": 0.9, "label": "leo", "anchor": "feet"}`, bbox as fractions of the image (y down); `anchor` `"feet"` for a whole body, `"centre"` (default) for a head |
| `/drone/mission` | `std_msgs/String`, latched | JSON of `GET /mission` on every change (`state`, `step`, `child`...) |
| `/drone/child_pose` | `geometry_msgs/PoseStamped`, latched | the child in `map` (orientation: facing it from the robot); in `drone_takeoff` while the launch pose is unknown |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | once per child report (`~send_goal`): `~standoff` m short of the child on the line from the robot's current tf pose, facing it |
| `/drone/fix` | `geometry_msgs/PoseArray`, latched | once per child report, same frame as `/drone/child_pose`: `poses[0]` = the drone when it took the fix, straight over the child with the camera at -90 (`z` = its height above the floor, orientation = its yaw), `poses[1]` = the child (`z` 0); `header.stamp` = the fix's frame time on the robot's clock |
| `/drone/pose_map` | `geometry_msgs/PoseStamped` | the drone in `map` once aligned |
| `/drone/odom` | `nav_msgs/Odometry` | the drone in `drone_takeoff` (as before) |
| tf | | `map → drone_takeoff` (once aligned) and `drone_takeoff → drone_base`: rviz shows the drone on the map |
| `/drone/mission/start` (in) | `std_msgs/String` | JSON body of `POST /mission/start` |
| `/drone/mission/abort`, `/drone/stop` (in) | `std_msgs/Empty` | back home and land / STOP (hover) |
| `/drone/launch_pose` (in) | `geometry_msgs/PoseStamped` | the robot's map pose at takeoff, only if tf `map → base_footprint` is not available |
| `/map` (in, `~map_topic`) | `nav_msgs/OccupancyGrid`, latched | sent once to the Mac for the page's map view (`POST /map/grid`), again if the Mac restarted |
| (tf) | | the robot's pose `map → base_footprint` is posted to the Mac at `~map_pose_rate` Hz (2) for the map view (`POST /map/robot_pose`) |

**Frames.** The drone's odometry frame starts at the takeoff point and is not aligned with
anything (no compass on the X1). Two things align it: (a) the robot's pose in `map` at launch:
the node reads tf `map → base_footprint` while the mission is launching/climbing/aligning and
posts it (`POST /mission/launch_pose`), so **the robot must not move until the child is
reported**; (b) the aligning leg: right after the climb the drone flies 1.5 m straight ahead,
**so launch it facing the robot's forward, with 2 m clear in front of the robot**. The takeoff
point is taken as `base_footprint` (set `launch_height_m` to the height of the robot's top).
Without (a) there is no map position or goal (the child is still published in
`drone_takeoff`); without (b) no `launch`/`map` coordinates.

**Demo sequence** (Mac: `drone/fly 60 sd --bind 0.0.0.0`; robot: navigation up and localised,
the node running; the drone on the robot's top, camera forward):
1. Mac: `curl -XPOST <mac>:8077/manual` (or MANUAL CONTROL in the page).
2. Start the mission, e.g. a patrol 1.5-4 m ahead of the robot (frame `launch`: x ahead, y left):
   ```bash
   rostopic pub -1 /drone/mission/start std_msgs/String "data: '{\"waypoints\": [{\"x\": 2, \"y\": -1.5}, {\"x\": 4, \"y\": -1.5}, {\"x\": 4, \"y\": 1.5}, {\"x\": 2, \"y\": 1.5}], \"launch_height_m\": 0.9}'"
   # or: curl -XPOST <mac>:8077/mission/start -d '{"waypoints": [...], "launch_height_m": 0.9}'
   ```
3. Press the drone's button. It climbs to 2 m, sets the camera to -40° (up to ~30 s while it
   anchors the tilt estimate), flies the 1.5 m leg, patrols.
4. The detector on the Mac (drone/identify/feed.py, started before the flight:
   `uv run --offline --with opencv-contrib-python-headless python -m drone.identify.feed <name>`)
   recognises the child by face, then tracks the body and posts it (`POST /mission/target`,
   anchor `"feet"`; a ROS detector can use `/drone/detection`). The page draws its boxes on the
   video. The drone stops the patrol and closes in with the camera following the child, then
   with the camera straight down slides until the body is centred in the image and takes the
   fix (4 centred frames).
5. `reporting`: `/drone/child_pose`, `/drone/fix` (the drone and the child) and
   `/move_base_simple/goal` are published once: the robot drives to the child, stopping 0.8 m
   short, facing it.
6. The drone circles the child twice (r 1.5 m) at its altitude (`orbit_alt_m` to change it),
   flies back to the takeoff point and lands there (on the floor: the robot has left).
Any time: `rostopic pub -1 /drone/stop std_msgs/Empty` (hover, mission ends) or
`/drone/mission/abort` (back home and land); STOP/LAND in the page work too.

**Test without the drone** (the simulator has a fake child, 3.5 m ahead and 1.5 m left of the
launch point, and a fake detector that reports its body box when it is in the camera view):
```bash
uv run python -m drone.sdk sim --http-port 8090 --bind 0.0.0.0 --no-browser       # on the Mac
curl -XPOST <mac>:8090/sim/child -d '{"fwd": 3, "left": -2}'   # move the child (or "detect": false)
python3 ros_node.py _url:=http://<mac ip>:8090                     # on the robot
# no tf map->base_footprint in your test? publish the launch pose by hand:
rostopic pub -1 /drone/launch_pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 2.0}, orientation: {w: 1.0}}}'
curl -XPOST <mac>:8090/manual; rostopic pub -1 /drone/mission/start std_msgs/String "data: '{\"waypoints\": [{\"x\": 1.5, \"y\": 0}, {\"x\": 1.5, \"y\": 2.5}], \"orbit_turns\": 1}'"
curl -XPOST <mac>:8090/sim/button                                   # the takeoff button
rostopic echo /move_base_simple/goal                                # ~1-2 min later
curl <mac>:8090/sim/truth                                           # child.map: the true position
```
Measured in the sim (frame 20° off the heading, approach "track"): overhead and centred within
15-31 px, child within 0.12-0.24 m of the truth, the drone's fix within 0.02 m of the child;
goal 0.80 m short; whole mission ~2.5 min with one orbit. The sim's video is a test pattern, so
a real detector sees nothing there: use the fake one, or replay recorded drone frames as the
camera and run the real feed on them (it shows the page's overlay; the mission then chases
where those people would be, so abort it):
```bash
uv run python -m drone.sdk sim --http-port 8096 --video recordings/frames/<session>
curl -XPOST localhost:8096/sim/child -d '{"detect": false}'      # the fake detector off
uv run --offline --with opencv-contrib-python-headless python -m drone.identify.feed <name> --url http://127.0.0.1:8096
```

**Limits.** Untested on the real X1. The overhead centring needs the camera tilt estimate to
reach -90 (the gimbal held down) and a body box from straight above (YOLOX finds people from
above on our recordings, less surely than from the side). The aligning leg assumes no yaw during takeoff and the
camera facing the robot's forward (error ~atan(odometry noise / leg) ≈ 4°, i.e. 0.25 m at 4 m).
The orbit's heading comes from the estimator (yaw stick × the calibrated 24.2 °/s, or the video
yaw), so the camera may drift off the child over two turns. Odometry drift adds to the child's
position error. The patrol restarts from the nearest waypoint after a lost approach.

## Map view
Press **M** (or MAP in the top bar) in the web page: the video shrinks to a corner and a map
takes its place, with the sticks and STOP still below. Closed by default; it polls only while open.
It shows, in the robot's `map` once the mission has aligned the drone (launch pose + aligning
leg; before that in the drone's takeoff frame, labelled at the top of the panel):
the drone (heading arrow, ±1 sd circle, altitude), its path, the patrol waypoints (or the
mission's planned ones), the geofence and the takeoff point; the robot live (from `ros_node.py`,
2 Hz) and its launch pose; the child (red) and the robot's goal once reported; and two
backgrounds: the robot's occupancy grid and a top-down render of a Gaussian splat of the hall.
Drag to pan, wheel or +/− to zoom, FIT to follow everything again, click for the `map`
coordinates of a point (to type waypoints). Each layer has a toggle.

```bash
# the robot's map from a file (map_server .yaml + .pgm), and/or from ros_node.py (/map, latched)
drone/fly 60 sd --bind 0.0.0.0 --map-yaml ~/maps/hall.yaml
# a Gaussian splat of the hall (3DGS .ply: x y z, f_dc_0..2, opacity); needs numpy (drone/fly has it)
drone/fly 60 sd --splat ~/scans/hall.ply --splat-up=-y        # COLMAP scenes are often -y up
uv run --with numpy python -m drone.sdk sim --http-port 8090 --splat hall.ply --map-yaml hall.yaml
```
The splat is rendered once at start: Gaussians above `--splat-z-max` (default: halfway between
the floor and the ceiling, 2nd/98th height percentiles) are dropped so the ceiling does not hide
the floor; each pixel is the opacity-weighted colour of the Gaussians above it. To place it on
the map: open ALIGN SPLAT in the panel, drag the splat or type scale (map metres per splat unit),
rotation (degrees, counter-clockwise) and x, y (where the splat's origin lands), then SAVE: it
goes to `<splat>.registration.json` and is loaded at the next start (`--splat-scale`,
`--splat-rotation`, `--splat-offset X Y` override it). Without numpy or with an unreadable file
the layer is off and the panel says why. Robot pose from anything else than ROS:
`curl -XPOST <mac>:8077/map/robot_pose -d '{"x": 2, "y": 3, "yaw": 0.5}'` (API.md "Map view").

## Known limits
- The SDK has only flown in the simulator: the real-drone path reuses hoverctl's tested link,
  sticks and video, but goto/patrol gains, m/s per stick %, and the yaw rate are untested on the X1.
- Takeoff is by the drone's button only (the X1 refuses takeoff requests).
- Heading is an estimate (see above); waypoints far apart accumulate drift.
- `/move` and patrols need the drone in MANUAL and the laptop on its Wi-Fi.
- The API has no authentication: bind to 127.0.0.1 unless the robot needs it.
- Drone-side photo/record and the HD preview are untested.
