# LearnBot Dashboard

Custom web dashboard for the real LearnBot robot: live camera, 3D robot model driven
by real joint/odometry data, lidar overlay, teleop, and Nav2 map + click-to-navigate.

Talks to the robot over `rosbridge_suite` (`ws://<host>:9090`) and `web_video_server`
(`http://<host>:8080`) — both launched on the **desktop**, never the Pi, same reasoning
as `foxglove_bridge` in the main `learnbot` package (see the comment in
`src/learnbot/launch/launch_robot.launch.py`).

## Every-session startup

**1. On the Pi** — the robot itself:
```bash
ros2 launch learnbot launch_robot.launch.py
```

**2. On the desktop** — the bridge (camera + rosbridge websocket):
```bash
ros2 launch /home/panchpc/LearnBot_v1/web/ros/rosbridge.launch.py
```

**3. On the desktop** — the dashboard dev server:
```bash
cd /home/panchpc/LearnBot_v1/web
npm run dev
```
Open the printed `http://localhost:5173` (or `http://192.168.1.67:5173` from another
device on the LAN, e.g. your phone).

That's enough for camera + 3D view + lidar + teleop. SLAM and Nav2 no longer
need separate terminal commands — see "SLAM / Nav2 panel" below, both are
buttons in the dashboard itself now.

## Using the dashboard

- **Drive**: arrow keys / WASD, or the on-screen D-pad.
- **3D view toolbar** (top-right of the 3D panel):
  - **⟲ Orbit** — default, drag to rotate the camera.
  - **✛ Move** — drag to pan instead (useful on a phone, no right-click there).
  - **📍 Set pose** — click this, then click on the map roughly where the robot
    actually is. Publishes `/initialpose` so AMCL can start localizing. Do this
    once per Nav2 session (see below).
  - **🏁 Send goal** — click this, then click anywhere on the map to send the
    robot there autonomously via Nav2.

### SLAM / Nav2 panel

Backed by `web/ros/process_manager.py`, launched automatically as part of
step 2 above (the bridge) — see that file for how it starts/stops processes
and enforces SLAM and Nav2 not running at the same time.

- **Start SLAM / Stop SLAM** — runs/stops `slam_toolbox`
  (`online_async_launch.py`) exactly as described in
  [`SLAM_NAV2.md`](../src/learnbot/SLAM_NAV2.md). The map grows live in the
  3D view above as you drive around (subscribed directly to the `/map`
  topic — see `mapService.ts`'s `subscribeMapUpdates`).
- **Map name + Save Map** — only enabled while SLAM is running. Calls
  slam_toolbox's own `/slam_toolbox/save_map` service (the same one its
  RViz panel uses) and writes `<name>.pgm` / `<name>.yaml` straight into
  `src/learnbot/maps/`, matching the convention in `SLAM_NAV2.md`.
- **Map dropdown + ⟳** — lists the `.yaml` files currently in
  `src/learnbot/maps/`. Refreshes automatically right after a successful
  save, or manually via ⟳.
- **Start Nav2 / Stop Nav2** — runs both required launch files together
  against whichever map is selected in the dropdown:
  ```bash
  ros2 launch learnbot localization_launch.py use_sim_time:=false \
    map:=src/learnbot/maps/<selected>.yaml
  ros2 launch learnbot navigation_launch.py use_sim_time:=false \
    map_subscribe_transient_local:=true
  ```
  That second argument is always included — see "Known quirks" below for why
  leaving it off silently breaks the costmaps. Stopping cleanly shuts down
  both.

### Setting the initial pose (step by step)

Nav2's AMCL always starts with no idea where the robot is — this is normal,
same as RViz's "2D Pose Estimate" tool, not a bug to fix.

1. Click **📍** in the 3D view toolbar.
2. Look at the map plane and the robot's current 3D position, and click on the
   map at the spot that matches where the robot is actually sitting in the room.
3. That's it — no need to be pixel-perfect. AMCL's particle filter will refine
   the estimate as the robot moves.

Known limitation right now: heading is always published as 0° (map-frame
"east"), since click-to-set-heading (drag after the click, like RViz's arrow)
isn't implemented yet. AMCL converges the heading from subsequent motion
regardless, so it's rarely necessary — but if localization seems oddly
confident-but-wrong right after setting the pose, drive forward a little and
it'll correct.

## Known quirks

- **`navigation_launch.py` silently overrides `nav2_params.yaml`'s
  `map_subscribe_transient_local: true`.** Root-caused: the launch file
  declares its own `map_subscribe_transient_local` launch argument
  (`src/learnbot/launch/navigation_launch.py:93-95`), defaulting to
  `'false'`, and force-rewrites that value into *every* costmap's
  `static_layer` config via `RewrittenYaml` — regardless of what the yaml
  file itself says. Confirmed directly: `ros2 param get
  /global_costmap/global_costmap static_layer.map_subscribe_transient_local`
  read back `False` even with the yaml set to `True`, and the actual
  rewritten params file nav2 was running from (`/tmp/tmp*`, path from `ps aux`
  on the `planner_server`/`controller_server` process) had
  `map_subscribe_transient_local: false` baked in for both costmaps. Since
  `map_server` only publishes `/map` once (latched), a volatile subscriber
  that starts after that one-shot publish — which is always, in normal
  startup order — gets nothing, and the costmap silently falls back to a tiny
  default window (repeated `"Robot is out of bounds of the costmap!"`
  warnings). **Fix**: always pass `map_subscribe_transient_local:=true` when
  launching `navigation_launch.py` (already in the startup steps above). If
  you ever see the warning again, that argument was probably left off.
- **`/map_server` params still say `yaml_filename: "turtlebot3_world.yaml"`**
  in `nav2_params.yaml` — harmless, it's always overridden by the `map:=`
  launch argument, but confusing to read.
- **Dev-server security note**: `npm audit` flags a known esbuild/Vite
  dev-server CORS issue (any site open in a browser on the same LAN could
  query the dev server while `npm run dev` is running). Low real-world risk
  for home use; fixing it means a breaking Vite major-version bump not yet
  done here.
