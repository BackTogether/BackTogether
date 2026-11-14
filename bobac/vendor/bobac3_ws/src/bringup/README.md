# bringup

One launch file that starts every block of the demo. Add one include / node per new block.

## Files
| File | What it does |
|---|---|
| `launch/bringup.launch` | includes `bobac3_navigation/launch/bobac3_slam_nav.launch` (base, lidars, gmapping, move_base) and `face_rec/launch/face_rec_topic.launch` (head camera + face recognition), then starts `face_check` |
| `scripts/face_check.py` | subscribes to `/face_results` and logs `ok` (once per second) while a face named `jorge` is in view. The name is hardcoded in `EXPECTED_PERSON` |
| `package.xml`, `CMakeLists.txt` | catkin package definition |

## Run
```bash
chmod +x ~/bobac3_ws/src/bringup/scripts/face_check.py   # once, after copying the package
cd ~/bobac3_ws && catkin_make && source devel/setup.bash
rosnode kill /rei_base                                    # the nav launch starts its own chassis driver
roslaunch bringup bringup.launch                          # open_rviz:=false to skip rviz
```

Needs `FACE_DIRECTORY` set in the shell, and photos of the person in `$FACE_DIRECTORY/jorge/`.

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
