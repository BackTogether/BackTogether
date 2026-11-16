# bringup

One launch file that starts every block of the demo. Add one include / node per new block.

## Files
| File | What it does |
|---|---|
| `launch/bringup.launch` | includes `slam_nav.launch` and `face_rec/launch/face_rec_topic.launch` (head camera + face recognition), then starts `face_check` and `splash` |
| `launch/slam_nav.launch` | SLAM + navigation block: robot model, chassis, lidars, gmapping, move_base, rviz |
| `scripts/face_check.py` | on arrival at the child's goal, looks for `person` around it: a turn on the spot, then wider and wider circles (up to `search_radius`) driven facing out of them, then back to the map origin if nobody is there. Publishes the name on `/face_check/found` and follows them ([details](#finding-the-child-around-the-goal-face_check)) |
| `scripts/child_goal_server.py` | HTTP on :8001 (`child_port`): the drone POSTs the child's `{x, y}` on `/child_found`, in the map frame or (`"frame": "robot"`, what drone/report_child.py sends) ahead / left of the robot; it becomes a `move_base` goal `standoff` (0.8 m) short of the child, facing them. Robot pose from tf (no amcl in this launch). Publishes `/child_goal/reached` on arrival |
| `scripts/care_trigger.py` | with the child (`/face_check/found`, or `/child_goal/reached` with `start_care_on:=arrived`): hides the image, frees the head camera for the call and POSTs `/care/start` to `bobac/robot/care_server.py` |
| `launch/state_demo.launch` | the state driven demo: splash, `system_state` (`home`), SLAM + navigation, `arrival_watch`, face recognition and `face_check`, then `kid_care` for the `care` state. The goal is sent by hand |
| `scripts/system_state.py` | owns the state of the machine: publishes it as a latched `std_msgs/String` on `/system_state` and changes it when a new state arrives on `/system_state/set`. The states it accepts are the `states` param. Also holds `StateClient`, what the other nodes use to read the state and ask for the next one |
| `scripts/arrival_watch.py` | in `home`, watches `/move_base/status` and asks for `arrived` once move_base reached the goal sent by hand |
| `scripts/splash.py` | covers the whole screen (terminals, rviz) with the face gifs of `assets/`, played full screen and centred, one after another in random order |
| `assets/robot_*.gif` | the robot face animations `splash` plays, one after another in random order |
| `src/bringup/state.py` | `StateClient`: how a node reads the state and asks for the next one. A module, not a script: `catkin_install_python` gives every installed script a devel space wrapper that exports nothing, so scripts cannot import each other |
| `systemd/ngrok.service`, `systemd/web-inbox.service` | user services kept up from boot: the drone laptop's ngrok link to `child_goal_server`, and `scripts/web_inbox.py` saving the parents' web photos in `~/.reinovo/face_data/<name>/` for face_rec (install steps in each file) |
| `package.xml`, `CMakeLists.txt`, `setup.py` | catkin package definition; `setup.py` plus `catkin_python_setup()` is what makes `src/bringup/` importable from `scripts/` |

## Run
```bash
# Build only this package: the vendor packages are already built in ~/bobac3_ws, and several
# of them only build with catkin_make, not with catkin build.
cd ~/leukerbard-robotics-AI-hackathon-2026/bobac/vendor/bobac3_ws
chmod +x src/bringup/scripts/*.py                         # once
catkin build bringup --no-deps && source devel/setup.bash
roslaunch bringup bringup.launch                          # open_rviz:=false to skip rviz
```

Needs `FACE_DIRECTORY` set in the shell, and photos of the person in `$FACE_DIRECTORY/jorge/`.

## Full demo from "the drone found the child" (`demo.launch`)
The drone is replaced by one HTTP request with the child's position. Chain:
`POST :8001/child_found` → `move_base` drives there → `face_check` turns until it recognises
the child and follows them → `care_trigger` frees the head camera and starts the care flow (the
voice agent talks with the child, the parents get the call link, the call opens on the screen) →
the parent hangs up → idle. The screen shows one robot face, the care flow's page
(`bobac/robot/ui/`) full screen: no `splash` image and no rviz in this launch.

```bash
export DISPLAY=:0 GEMINI_API_KEY='<key>' NTFY_TOPIC='<topic>'   # keys only in the shell
roslaunch bringup demo.launch            # person:=jorge (replaces the boot /rei_base)
# no photos of the child: roslaunch bringup demo.launch start_care_on:=arrived
```
Then the "drone", from the robot or a laptop on its network: the child 2.5 m ahead of the
robot (frame `robot`: x ahead, y to the left, metres; the robot stops 0.8 m short of them).
```bash
curl -X POST http://<robot ip>:8001/child_found -H 'Content-Type: application/json' \
  -d '{"mission_id": "demo-1", "x": 2.5, "y": 0.0, "frame": "robot"}'
curl http://<robot ip>:8001/status                  # navigating -> arrived
```
A repeat of the same `mission_id` is ignored: use `demo-2` and so on. `POST :8001/cancel` stops
the drive, `POST :8090/stop` (body `{}`) the care flow. The head camera node is stopped for the
call, so restart the launch before the next run.

## State of the machine (`system_state`)
One node keeps where the demo is, so no other node has to guess. It publishes the state on
`/system_state` (latched: a node that starts late gets the current one straight away) and is
the only node that writes it. Any node changes it by publishing the new state on
`/system_state/set`.

```bash
rostopic echo /system_state                                      # follow the demo
rostopic pub -1 /system_state/set std_msgs/String "data: care"   # change it by hand
```
A state outside the `states` param is refused and logged, the node stays where it was. Params:
`initial_state` (default `home`), `states` (default `home,arrived,searching,care,done`, empty
accepts any state).

Each node works in one state: it starts its task when the machine enters that state and asks
for the next one when it is done (`StateClient` in `src/bringup/state.py`). Nodes that publish a
state are a no-op in the launches that start no `system_state`, so `bringup.launch` and
`demo.launch` are unchanged.

| State | Who leaves it | On what |
|---|---|---|
| `home` | `arrival_watch` | move_base reached the goal sent by hand → `arrived` |
| `arrived` | `face_check` | starts looking for the child → `searching` |
| `searching` | `face_check` | recognised the child → `care`, then it follows them. Nobody around the goal: it drives home and stays in `searching` |

In `care` the `kid_care` package takes over (`kid_care/launch/kid_care.launch`, included by
`state_demo.launch`): its `kid_care_start` node hides the splash, frees the head camera and
starts the care flow — the voice agent talks with the child, the parents get the call link and
the video call opens full screen on the robot's screen. Freeing the camera also ends
`face_check`'s following, which needs it too: `care:=false` leaves the demo stopping at `care`,
`free_camera:=false` keeps the following and leaves the call without a camera.

## The state driven demo (`state_demo.launch`)
SLAM, the state machine and the face search, without the drone and without the care flow: the
goal is sent by hand and the robot takes over from there.

```bash
roslaunch bringup state_demo.launch                          # person:=jorge
roslaunch bringup state_demo.launch open_rviz:=true splash:=false   # to click the goal in rviz
```
`splash` covers the screen, so send the goal from rviz on another machine, with
`open_rviz:=true splash:=false`, or by hand:
```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped   '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}'
rostopic echo /system_state      # home -> arrived -> searching -> care
```
The chain: `home` (waiting) → the goal drives the robot → `arrived` → `face_check` searches
around the goal (`searching`) → it recognises `jorge` → `care`, and `kid_care` starts the voice
agent and the video call with the parents. If the child is nowhere around the goal the robot
drives back to the map origin and the machine stays in `searching`: send the goal again to
retry.

```bash
export DISPLAY=:0                                  # the call window needs the robot's screen
roslaunch bringup state_demo.launch zone:="park"   # care:=false to stop before the care flow
```
The keys for the care flow (Gemini, ntfy) are in `kid_care/.env`, never in the repo.

## Finding the child around the goal (`face_check`)
The drone's odometry is not exact, so the goal it sends is only roughly where the child is:
often enough the child is not in the head camera when the robot gets there. Turning on the spot
until the timeout then finds nothing, so the search widens by itself, watching for the face
through all of it (only `person` counts, as before):

1. **a full turn on the spot**, at the point the robot arrived at, at `turn_speed` (0.3 rad/s,
   slow enough for the recogniser), measured on tf rather than on the clock;
2. **wider and wider circles** around that point — `ring_step` (0.3 m) apart, out to
   `search_radius` (0.6 m). The robot steps straight out onto each circle and then slides
   round it sideways on the omni base, turning as it goes so it **always faces out of the
   circle**: every lap looks at the same ground from a new place, which is what brings out a
   child standing behind something, and the camera never turns back into the circle it has
   already searched;
3. **back home** (`home_x`, `home_y`, the map origin: the pose the robot started the run in) by
   `move_base` when the widest circle finds nobody either, still watching for the face on the
   way — a sighting cancels the drive and wins.

The lidar (`scan_topic`, `/fuse_scan`: both lidars, all around the robot) keeps it off anything
in the way: a blocked way out is tried `90°` further round, a blocked lap goes round the other
way, and a leg that stays blocked is given up instead of forced. Every leg has its own time
budget, `timeout` (default ~2 min, computed from the circles) caps the whole search, and a new
`move_base` goal stops it at once. The robot is always left stopped.

A face seen mid search stops the robot where it is, turns it until the face is in the middle of
the image, and goes to `care`; if the face is lost again the search carries on from there.

```bash
# the robot stays on the spot, as it did before (and 0.2 m circles instead of 0.3 m)
roslaunch bringup bringup.launch search_radius:=0
roslaunch bringup bringup.launch ring_step:=0.2 return_home:=false
rosrun bringup face_check.py _search_on_start:=true _person:=jorge   # the search alone
```

Params, all with defaults in `scripts/face_check.py`: `search_radius`, `ring_step`,
`search_speed` (0.12 m/s), `clearance` (0.4 m of lidar range that counts as in the way),
`return_home`, `home_x`, `home_y`, `home_timeout`, `odom_frame` (`odom`: the search measures
itself there, because unlike `map` it does not jump when gmapping corrects itself),
`robot_frame`, `scan_topic`, plus `person`, `turn_speed`, `timeout`, `target_area` and
`max_linear` as before.

## Full screen face gifs (`splash`)
A few seconds after the launch starts, `splash` opens fullscreen and always on top and plays
the gifs of `assets/` centred, so the demo shows the robot's face instead of the terminals and
rviz. Each gif is played once (`loops`) and the next one is picked at random: every gif comes up
once before any of them repeats, and never twice in a row. It keeps raising itself, because rviz
opens after it.

```bash
roslaunch bringup bringup.launch splash:=false        # do not cover the screen
roslaunch bringup bringup.launch splash_delay:=10     # wait longer (rviz is slow to open)
rostopic pub -1 /splash/show std_msgs/Bool "data: false"   # hide it, "data: true" brings it back
```
With the window focused: `Esc`/`q` closes it (only this node), `h` hides it, `t` toggles
always-on-top, `n` skips to the next gif. Other params (set in `bringup.launch`): `images`
(comma separated gifs or a directory, default every gif in `assets/`), `loops`, `fit`
(`contain` whole frame on a black background, `cover` fill and crop, `stretch`, `none`),
`background`, `topmost`, `raise_period`, `hide_cursor`.

Needs a graphical session on the machine running the launch (over ssh: `export DISPLAY=:0`)
and `python3-tk`; `python3-pil.imagetk` gives smooth scaling and is recommended:
```bash
sudo apt install python3-tk python3-pil.imagetk
```

## See the current pose
Position and orientation of the robot in the map frame, printed once per second:
```bash
rosrun tf tf_echo map base_footprint
```

## Send the robot to a goal pose
`x`, `y` in metres in the map frame; `w: 1.0` means facing along the map's x axis.
```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```
move_base turns this message into a goal of its `/move_base` action (`move_base_msgs/MoveBaseAction`).
Follow it with `rostopic echo /move_base/status` and cancel it with:
```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}'
```
