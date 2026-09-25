# SLAM & Nav2 command reference

Quick reference for mapping a new area with `slam_toolbox` and driving the
robot autonomously with Nav2, on top of `learnbot`'s `launch_robot.launch.py`
bringup. All commands assume the workspace is sourced:

```bash
cd /home/panchpc/LearnBot_v1
source install/setup.bash
```

None of the launch files below (`rplidar`, `online_async` SLAM,
`localization`, `navigation`, `joystick`) are pulled in by
`launch_robot.launch.py` automatically — they're started separately, on
whichever machine makes sense (lidar/base on the Pi, SLAM/Nav2/RViz wherever
you're driving from, same DDS network).

## 1. Base bringup (always first)

**On the Pi:**
```bash
ros2 launch learnbot launch_robot.launch.py
```
Brings up `robot_state_publisher`, the camera, `twist_mux`, and
`ros2_control` (`diff_cont` + `joint_broad`). It does **not** start the
lidar or a teleop source — start those next.

**Lidar** (needed for both SLAM and Nav2 localization):
```bash
ros2 launch learnbot rplidar.launch.py
```

**Teleop** (to drive around while mapping — pick one):
```bash
ros2 launch learnbot joystick.launch.py       # joystick/gamepad
# or use the web dashboard's WASD/D-pad teleop, or `teleop_twist_keyboard`
```

## 2. Mapping a new area (SLAM)

Start `slam_toolbox` in online-async mode:
```bash
ros2 launch learnbot online_async_launch.py use_sim_time:=false
```
Uses `config/mapper_params_online_async.yaml`. Drive the robot slowly around
the whole area, overlapping loops where possible so `do_loop_closing` can
correct drift.

Watch the map build live in RViz:
```bash
rviz2 -d src/learnbot/config/create_map.rviz
```

**Save the finished map** (creates `<name>.pgm` + `<name>.yaml`):
```bash
ros2 run nav2_map_server map_saver_cli -f src/learnbot/maps/<name>
```
Save it directly into `src/learnbot/maps/` so it's picked up by the
`map:=` argument below (and gets committed to git, like the existing maps).

## 3. Localization + navigation with a saved map (Nav2)

Two launch files, both required, in this order:
```bash
ros2 launch learnbot localization_launch.py use_sim_time:=false \
  map:=/home/panchpc/LearnBot_v1/src/learnbot/maps/<name>.yaml

ros2 launch learnbot navigation_launch.py use_sim_time:=false \
  map_subscribe_transient_local:=true
```
`localization_launch.py` starts `map_server` + `amcl`. `navigation_launch.py`
starts the controller/planner/behavior/bt_navigator/waypoint_follower stack,
using `config/nav2_params.yaml`.

**`map_subscribe_transient_local:=true` is mandatory.** `map_server`
publishes `/map` once (latched). If the costmaps subscribe with a volatile
QoS (the launch file's own default, which silently overrides whatever
`nav2_params.yaml` says), they start after that one-shot publish and never
get the map — you'll see repeated `"Robot is out of bounds of the costmap!"`
warnings. Leaving off that argument is the #1 cause of Nav2 "not working."

**Set the initial pose** (AMCL always starts with no idea where the robot
is — this is expected, not a bug):
```bash
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}}'
```
Easier in practice: use RViz's **2D Pose Estimate** tool, or the web
dashboard's **📍 Set pose** button (see [`web/README.md`](../../web/README.md)).

View map + costmaps + send goals in RViz:
```bash
rviz2 -d src/learnbot/config/map.rviz
```
Use RViz's **2D Goal Pose** tool (or **Nav2 Goal**) to send the robot
somewhere autonomously.

## Existing saved maps (`maps/`)

| File | Notes |
|---|---|
| `casa_new.yaml` | Whole-house map |
| `COMEDOR.yaml` | Dining room, has `.posegraph` (was saved as a serialized SLAM session too — can be reloaded into slam_toolbox for further mapping, not just Nav2) |
| `LIVING_PIEZA2.yaml` | Living room, also has a `.posegraph` |
| `my_map.yaml`, `pieza_new.yaml` | Earlier room-scale test maps |

To keep extending a previously-mapped area instead of starting from scratch,
point `slam_toolbox` at the `.posegraph` session file via the
`map_file_name` param in `config/mapper_params_online_async.yaml` (currently
commented out) instead of starting `online_async_launch.py` fresh.

## Key config files

- `config/mapper_params_online_async.yaml` — SLAM Toolbox params
- `config/nav2_params.yaml` — controller/planner/costmap/AMCL params
- `config/create_map.rviz`, `config/map.rviz`, `config/drive_bot.rviz` — RViz layouts
