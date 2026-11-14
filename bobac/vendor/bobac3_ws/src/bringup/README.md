# bringup

One launch file that starts every block of the demo. Add one include / node per new block.

## Files
| File | What it does |
|---|---|
| `launch/bringup.launch` | includes `slam_nav.launch` and `face_rec/launch/face_rec_topic.launch` (head camera + face recognition), then starts `face_check` and `splash` |
| `launch/slam_nav.launch` | SLAM + navigation block: robot model, chassis, lidars, gmapping, move_base, rviz |
| `scripts/face_check.py` | subscribes to `/face_results` and logs `ok` (once per second) while a face named `jorge` is in view. The name is hardcoded in `EXPECTED_PERSON` |
| `scripts/splash.py` | covers the whole screen (terminals, rviz) with `assets/base_image.png` |
| `assets/base_image.png` | the image `splash` shows |
| `package.xml`, `CMakeLists.txt` | catkin package definition |

## Run
```bash
# Build only this package: the vendor packages are already built in ~/bobac3_ws, and several
# of them only build with catkin_make, not with catkin build.
cd ~/leukerbard-robotics-AI-hackathon-2026/bobac/vendor/bobac3_ws
chmod +x src/bringup/scripts/*.py                         # once
catkin build bringup --no-deps && source devel/setup.bash
rosnode kill /rei_base                                    # the nav launch starts its own chassis driver
roslaunch bringup bringup.launch                          # open_rviz:=false to skip rviz
```

Needs `FACE_DIRECTORY` set in the shell, and photos of the person in `$FACE_DIRECTORY/jorge/`.

## Full screen image (`splash`)
A few seconds after the launch starts, `splash` opens `assets/base_image.png` fullscreen and
always on top, so the demo shows one picture instead of the terminals and rviz. It keeps raising
itself, because rviz opens after it.

```bash
roslaunch bringup bringup.launch splash:=false        # do not cover the screen
roslaunch bringup bringup.launch splash_delay:=10     # wait longer (rviz is slow to open)
rostopic pub -1 /splash/show std_msgs/Bool "data: false"   # hide it, "data: true" brings it back
```
With the image focused: `Esc`/`q` closes it (only this node), `h` hides it, `t` toggles
always-on-top. Other params (set in `bringup.launch`): `image`, `fit`
(`contain` whole image on a black background, `cover` fill and crop, `stretch`, `none`),
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
