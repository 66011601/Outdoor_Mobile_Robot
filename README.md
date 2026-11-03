# DCVG Autonomous Rover

ROS 2 Humble stack for an outdoor rover that drives GPS routes and takes DCVG (Direct Current Voltage Gradient) pipeline measurements at stop points. It runs on a Jetson Orin Nano (Ubuntu 22.04).

## Hardware

| Part | Role | Device |
|---|---|---|
| simpleRTK2B Heading, position board (u-blox F9P) | RTK position, gets NTRIP corrections | `/dev/ttyGPS_movingbase` |
| simpleRTK2B Heading, heading board | Moving-baseline heading (NAV-RELPOSNED) | `/dev/ttyGPS_rover` (UART 460800) |
| Teensy + 2× Pololu G2 | Motor PWM + probe actuator | `/dev/ttyTeensy` |
| RealSense D455/D457 | Visual odometry, obstacle cloud, IMU | USB |
| USB webcam on the DCVG meter | OCR of the meter display | `/dev/v4l/by-id/usb-Pico_Instruments_...` |

The `/dev/tty*` names come from udev rules in `/etc/udev/rules.d/99-usb-serial.rules`. The robot has **no wheel encoders and no LiDAR**.

## Workspace

```
src/
├── dcvg_bringup/                 Runnable hardware stack
│   ├── launch/
│   │   ├── bringup.launch.py     Entry point, starts everything below with staggered delays
│   │   ├── sensors.launch.py     RealSense + imu_filter_madgwick → /imu/data, imu_throttle, imu_tilt (use_imu_tilt only)
│   │   ├── ublox_gps.launch.py   Both F9P boards + gps_heading_to_imu
│   │   ├── ntrip.launch.py       NTRIP client + ntrip_bridge (RTK corrections)
│   │   ├── localization.launch.py  Dual EKF + navsat_transform
│   │   ├── rtabmap.launch.py     Visual odometry (/vo), decimated depth cloud + ground-filtered obstacles + terrain_filter
│   │   └── replay.launch.py      Re-runs VO / localization / obstacles + costmaps on a recorded bag
│   ├── config/                   nav2_params.yaml, localization.yaml, ublox_f9p*.yaml, bag_qos.yaml
│   ├── behavior_trees/           Nav2 BTs (recovery = clear costmaps + wait, never moves)
│   │                               Loaded through the empty default_nav_*_bt_xml keys in nav2_params.yaml, which
│   │                               the launch files fill in. Humble's RewrittenYaml only replaces keys that exist,
│   │                               so without them Nav2 silently runs its stock BT (Spin 90° + BackUp recoveries).
│   │                               Plan → smooth must be a plain Sequence: a ReactiveSequence re-ticks the planner
│   │                               while SmoothPath runs, so FollowPath never starts and the robot does not move.
│   ├── ocr/                      Digit templates for the meter OCR
│   └── dcvg_bringup/
│       ├── low_level_command.py  /cmd_vel → Teensy serial, probe cycle, /teensy/feedback
│       ├── gps_heading_to_imu.py NAV-RELPOSNED → yaw-only Imu for the EKFs
│       ├── imu_throttle.py       /imu/data (~200 Hz) → /imu/data_throttled (50 Hz)
│       ├── imu_tilt.py           Camera IMU roll/pitch → /joint_states (base_footprint → base_link tilt, off by default)
│       ├── terrain_filter.py     Obstacle/ground clouds → obstacles judged by height above the local ground
│       ├── record_bag.py         ros2 bag record with field profiles and a free-space guard
│       ├── ntrip_bridge.py       /rtcm → F9P serial, /navpvt → GGA on /nmea
│       ├── teensy_feedback_server.py  Stamps DCVG + GPS on Teensy frames → TCP :5005
│       └── dcvg_ocr.py           Reads ON/OFF values off the meter LCD
├── dcvg_robot/                   Robot model + operator tools
│   ├── urdf/dcvg_robot.urdf.xacro  TF frames (base_footprint, base_link, camera_link, gps_link, gps_heading_link; base_pitch_link with use_imu_tilt)
│   ├── config/                   waypoints.yaml, stops.yaml (written by the web app)
│   ├── web/index.html            Web UI (Leaflet map, nav controls, teleop, camera view)
│   └── dcvg_robot/
│       ├── osm_route_app.py      Flask server for the web UI (port 5010)
│       ├── osm_route_generator.py  Straight-line or OpenStreetMap routes between stops
│       ├── route_waypoint_follower.py  Drives waypoints.yaml through Nav2, waits at stops
│       └── common.py             /tmp handshake files shared by the app and follower
└── system/dcvg-auto.service  systemd unit that starts the web app on boot
```

## Build and run

```bash
source /opt/ros/humble/setup.bash
cd ~/dcvg_robot && colcon build
source install/setup.bash

ros2 run dcvg_robot osm_route_app          # open http://<jetson-ip>:5010
```

The web app is the normal entry point. It first asks for a mode. **Manual** stops any running bringup and navigation, then starts `dcvg.service` (the legacy manual-control program in `~/Desktop/DCVG_Robot`), and the page offers only **Shutdown** and **Switch to Auto**. **Auto** stops `dcvg.service` and shows the bringup screen. The two modes are mutually exclusive because both open `/dev/ttyTeensy` and TCP 5005. Switching modes runs `sudo -n systemctl start|stop dcvg.service`, so it needs a NOPASSWD sudoers rule for exactly those two commands. Set the `DCVG_PORT` environment variable to change the port (default 5010). **Launch Bringup** runs `ros2 launch dcvg_bringup bringup.launch.py`.

While bringup starts, the setup screen shows a checklist instead of the raw launch output, which is still available under "Show technical log". The checks are defined in `BRINGUP_CHECKS` in `osm_route_app.py`. Each check combines two sources:

- Launch log lines `process started` and `process has died/finished` for the check's executables. Any exit marks the check as an error.
- For most checks, data arriving on a topic within the last 3 s. If a process has run for 20 s without data, the check shows a warning.

| Check | Executables | Ready when |
|---|---|---|
| Robot model | robot_state_publisher | process started |
| Camera | realsense2_camera_node | `/camera/color/camera_info` |
| Camera IMU | imu_filter_madgwick_node, imu_throttle | `/imu/data_throttled` |
| Motor controller | low_level_command | `/teensy/feedback` |
| Visual odometry | stereo/rgbd_odometry, obstacles_detection, terrain_filter | `/vo` |
| Localization | ekf_node, navsat_transform_node | `/odometry/local` |
| GPS position | ublox_gps_node | `/gps/fix` with a fix (status 2 is shown as RTK) |
| GPS heading | gps_heading_to_imu | `/gps/heading/imu` |
| NTRIP corrections | ntrip_ros.py, ntrip_bridge | `/rtcm` |
| Navigation | Nav2 servers, lifecycle_manager | log line `Managed nodes are active` |
| Telemetry server | teensy_feedback_server | process started |

After the grace period, the UI opens automatically if every check is OK. Otherwise it offers **Continue anyway**. When you add a node to the launch files, add it to `BRINGUP_CHECKS` too.

Then you plan a route, add stops, and press **Start**, which runs `route_waypoint_follower`.

To run the stack without the UI:

```bash
ros2 launch dcvg_bringup bringup.launch.py
ros2 launch dcvg_bringup bringup.launch.py use_stereo:=false use_rviz:=True enable_ocr:=False
```

Launch arguments: `use_stereo` (True = D457 IR stereo VO, False = RGBD VO), `use_rviz`, `use_telemetry`, `enable_ocr`, `heading_offset_deg`, `use_imu_tilt` (default False, see Slope handling), `max_ground_angle`, `max_ground_height`, `use_terrain_filter` (default True), `grass_height` (0.12), `step_height` (0.20) (see Terrain filter), `teensy_port`, `gps_device`, `gps_heading_device`.

To autostart the web app on boot:

```bash
sudo cp src/system/dcvg-auto.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now dcvg-auto
```

## How it works

```
RealSense ─ rgbd/stereo_odometry ─ /vo ───────────────────────────┐
          ─ /camera/depth/image_rect_raw ─ point_cloud_xyz ─ obstacles_detection ─ terrain_filter ─ /camera/depth/obstacles ─ Nav2 costmaps
          ─ imu_filter_madgwick ─ /imu/data ─ imu_throttle ─┤      │
                                   imu_tilt ─ /joint_states ┘ (use_imu_tilt only: base_footprint → base_link)
GPS heading board ─ gps_heading_to_imu ─ /gps/heading/imu ──┤      │
                                                            ├─ ekf_filter_node_odom → odom→base_footprint, /odometry/local
GPS position board ─ /gps/fix ─ navsat_transform ─ /odometry/gps ─ ekf_filter_node_map → map→odom, /odometry/global
NTRIP caster ⇄ ntrip_client ⇄ ntrip_bridge ⇄ position board (RTCM in, GGA out)

web app ─ route_waypoint_follower ─ Nav2 (RPP controller) ─ /cmd_vel ─ low_level_command ─ Teensy
```

- **Localization.** `ekf_filter_node_odom` fuses VO velocity, gyro yaw rate and GPS heading, and uses `/cmd_vel` as a control input (`use_control`, vx and yaw rate, limits equal to `velocity_smoother` `max_accel`/`max_decel`). There are no wheel encoders, so VO is the only velocity source: when it loses tracking on a bump, the prediction follows the commanded motion instead of coasting on the last estimate. `control_timeout` (0.5 s) equals the `low_level_command` `cmd_timeout`, so when `/cmd_vel` goes quiet the motors stop and the EKF predicts standstill. If the wheels slip or the robot is blocked, VO pulls the estimate back once it tracks again. `ekf_filter_node_map` fuses GPS position, gyro yaw rate and GPS heading. Neither EKF fuses the camera IMU's absolute yaw: it has no magnetometer, so its zero point changes on every boot.
- **Slope handling.** The TF tree is `map → odom → base_footprint → base_link → camera_link → camera optical frames`. Both EKFs run in `two_d_mode`, so `odom → base_footprint` carries only x, y and yaw. By default `base_footprint → base_link` is fixed, so `base_footprint` lies in the chassis plane and tilts with the robot. `obstacles_detection` then judges ground relative to the surface the wheels are on, which suits rough routes with short, changing slopes.
- **Gravity-levelled mode (`use_imu_tilt:=True`).** Builds the URDF with `base_pitch_joint`/`base_roll_joint` (via `base_pitch_link`) instead of the fixed `base_joint`, and starts `imu_tilt`. `imu_tilt` takes the Madgwick orientation of the camera IMU, rotates it into `base_link` through the static URDF/RealSense TFs, drops yaw (there is no magnetometer), and publishes roll and pitch as the joint positions. `robot_state_publisher` turns them into a moving `base_footprint → base_link`, so `base_footprint` stays level with gravity. This helps on long, even slopes. On rough ground it is off by default: every bump and slope change swings the levelled obstacle cloud, and the Madgwick estimate is disturbed by the jolts, so the costmaps flicker and navigation fails more often than it gains. Only switch it on for smooth, steady grades, and then keep `max_ground_height:=0` (with a levelled frame, a 0.3 m limit turns any uphill steeper than ~6° inside the 3 m range into a wall).
- **Why the EKFs stay 2D.** Both costmaps filter obstacle heights in `odom`/`map` z. A 3D `odom` frame would climb with every hill and push ground into, and real obstacles out of, that height band.
- **Ground vs obstacle.** `obstacles_detection` works in `base_footprint` with normals segmentation. `Grid/MaxGroundAngle` (`max_ground_angle`, default 50°) is the surface slope, relative to `base_footprint`, above which a surface is an obstacle. `Grid/MaxGroundHeight` (`max_ground_height`, default 0 = disabled) would make everything higher than that above `base_footprint` an obstacle. Keep it at 0: on rough routes the chassis pitches into dips, and the ground ahead then rises above any fixed limit within the 3 m range and turns into a wall. With it disabled, flat tops of objects count as ground (their sides are still obstacles), and `Grid/FlatObstacleDetected` has no effect. The costmaps' `min_obstacle_height` is −1.0 m (STVL `origin_z` −1.0, 60 voxels) so obstacles below the robot plane are kept and low clutter is not hidden. If short grass shows up as obstacles, raise `grass_height` (see Terrain filter) rather than raising the costmap floor or tightening `Grid/MinClusterSize`/`Grid/NoiseFiltering*`, which also removes thin real obstacles. `Grid/CellSize` (0.05 m) sets the voxel size the cluster and noise filters count in, so retune them together: at 0.1 m a point has at most ~6 neighbours within the 0.1 m `Grid/NoiseFilteringRadius`, the 8-neighbour filter then removes almost every obstacle, and `Grid/MinClusterSize` 20 needs four times the surface. RTAB-Map silently ignores misspelled `Grid/*` keys (an earlier `Grid/Cellsize` left the default in effect), so check spelling against `rtabmap/core/Parameters.h`.
- **Terrain filter.** `obstacles_detection` publishes `/camera/depth/obstacles_raw` and `/camera/depth/ground`; `terrain_filter` turns them into `/camera/depth/obstacles`. `obstacles_detection` segments in `base_footprint` but publishes both clouds in the input cloud's frame (`camera_depth_optical_frame`, z = depth, not height), so `terrain_filter` transforms them into `base_footprint` through TF, filters there, and publishes back in the input frame. Without that transform, depth is read as height and the far half of every ground patch turns into phantom obstacles. It measures height above the ground in front of the robot instead of above `base_footprint`, so it works like V1's `max_ground_height:=0.1` on slopes and while the chassis pitches.
  - Local ground: the ground cloud is binned into 0.2 m cells (x −1…4 m, y ±3 m in `base_footprint`), and each cell with ≥ 3 ground points gets the median height. Cells that rise above their neighbours faster than `max_slope_deg` (35°, the steepest drivable slope) are pulled down to that slope, so box tops and rounded rocks stand out. Cells without ground (hidden behind an object) take the lowest seen cell within 0.6 m, which keeps more points, not fewer. Heights are interpolated between cell centres, so a 30° slope does not step 12 cm inside a cell.
  - Obstacle points less than `grass_height` (0.12 m) above the local ground are dropped: short grass and small bumps. Ground points more than `step_height` (0.20 m) above it are added as obstacles: flat tops and rounded objects the normals call ground. Ground steeper than 35° relative to the chassis becomes an obstacle the same way.
  - `grass_height` is also the lowest obstacle the robot sees. Grass taller than that counts as an obstacle on purpose (a depth camera cannot tell tall grass from a bush). On concrete routes with curbs, lower it (0.08) or use `use_terrain_filter:=false`.
  - Where there is no ground reference (no ground cloud yet, a ground cloud more than 0.5 s from the obstacle stamp, a frame mismatch, no TF to `base_footprint`, or points outside the grid), obstacles pass through unchanged, which is exactly the behaviour with `use_terrain_filter:=false`. It logs kept/promoted counts every 10 s.
- **Local costmap latency.** The obstacle cloud is built by `point_cloud_xyz` (node `depth_cloud`) straight from the raw depth image, decimated 4× and cut at 3 m, on `/camera/depth/points_decimated`. The RealSense textured cloud (`pointcloud__neon_.enable`) is off: at full resolution it was megabytes per frame, which slowed both the camera driver and `obstacles_detection`, and the 0.05 m `Grid/CellSize` voxelization threw that density away anyway. `obstacles_detection` has no `map_frame_id`, because waiting up to 0.2 s for `map → base_footprint` at every cloud stamp only added delay and dropped clouds. The STVL source clears with a depth-camera frustum: `sensor_frame: camera_depth_optical_frame` (the obstacle cloud is already in that frame, see Terrain filter, so this only makes it explicit), FOV 1.45 × 0.95 rad (D457 depth is 87° × 58°), 0.5–3.0 m, `decay_acceleration: 5.0`. An obstacle that leaves the camera's view clears in about 0.5 s; voxels outside the view (the blind zone in front of the bumper, behind the robot) keep the full `voxel_decay` of 4 s. Without `decay_acceleration` (STVL default 0) `clearing: true` does nothing and every ghost lasts 4 s. `publish_frequency` is 10 Hz so RViz follows the costmap the controller uses (20 Hz `update_frequency`).
- **VO frame.** `rgbd_odometry`/`stereo_odometry` use `frame_id: base_link`, which is rigid to the camera, as RTAB-Map assumes. robot_localization rotates the `/vo` twist into `base_footprint`. `gps_heading_to_imu` stamps its yaw in `base_footprint`, and navsat_transform uses `base_link_frame_id: base_footprint`. With `use_imu_tilt:=True` this makes the GPS heading the yaw of the level frame and lets the 1.15 m antenna mast offset follow the real tilt (about 20 cm per 10° otherwise).
- **navsat_transform** takes its odometry input from `/odometry/global`, not `/odometry/local`. That way `/odometry/gps` is published in the `map` frame. With the `odom` frame it created a feedback loop that sent the map EKF position off to 10⁷ m.
- **Route following.** The route is split into segments at every stop. Each segment is sent as one `NavigateThroughPoses` goal after straight runs are collapsed (`simplify_*` parameters). When a segment fails, the rover holds and waits for the operator (**Run** = retry, **Stop** = end). `-p on_segment_failure:=skip` skips the failed segment instead.
- **Stops.** The follower publishes `/dcvg/stop_reached`. `low_level_command` then waits 2 s to settle, lowers the probe for 2.5 s (`probe_down_delay`, counted from the down command, so it includes the actuator travel), raises it for 3 s, and publishes `/dcvg/stop_confirm`, which releases the follower. The **Confirm** button in the web UI does the same. If the Teensy link was down at any point of the cycle, the probe never moved, so `low_level_command` does not confirm and the stop waits for the operator.
- **Web app ↔ follower** talk through files in `/tmp`: `dcvg_nav_cmd` (run/pause/stop), `dcvg_stop_status`, `dcvg_stop_confirm`, `dcvg_nav_target`.
- **Teensy wire format.** ROS sends `$<left_rad_s>,<right_rad_s>,<probe_up>,<probe_down>#\n`. Lines from the Teensy that start with `$` are republished on `/teensy/feedback`. `teensy_feedback_server` appends `dcvg_on,dcvg_off,lat,lon` to them (17 fields in total) and serves them on TCP 5005.

## Calibrated values (do not change without re-measuring)

- **Sensor positions** (measured from `base_link`, the body centre; xacro z = height above ground − 0.08 m `base_height`). Camera 0.62 m forward, 0.90 m above ground: `camera_link` at `0.62 0 0.82`. Heading antenna 0.62 m forward, 1.23 m above ground: `gps_heading_link` at `0.62 0 1.15`.
- **GPS lever arm.** The position antenna is 0.62 m **behind** `base_link` and 1.23 m above the ground: `gps_link` at `-0.62 0 1.15` in the xacro. If this is wrong, every stop overshoots and every in-place pivot shows up as fake position jumps.
- **GPS heading.** The heading antenna is at the front, on the centreline, so `heading_offset_deg` is 0.0. To recalibrate, point the rover at a known bearing and set `heading_offset_deg = true_bearing − measured_yaw`.
- **Drive limits** (`nav2_params.yaml`): 0.2 m/s linear and 0.3 rad/s angular. The angular limit is set by friction, not speed: turning in place skids both wheels, and on grass anything much below 0.3 rad/s (about 0.12 m/s at each wheel, close to the 0.15 m/s cruise speed) can stall. Keep `velocity_smoother` `max_velocity`/`min_velocity` angular, `rotate_to_heading_angular_vel` and the behavior server `max_rotational_vel` equal, or the smoother silently clips the lower-level value (and with `scale_velocities` it slows the linear speed along with it). `min_rotational_vel` (0.15) is the slowest a Spin recovery will command. `velocity_smoother.max_accel` is `[0.4, 0, 0.8]` and must keep its angular value equal to `FollowPath.max_angular_accel`. `rotate_to_heading_angular_vel` must stay set (RPP's default is 1.8 rad/s). Keep `yaw_goal_tolerance` (0.35) below `rotate_to_heading_min_angle` (0.43), and `lookahead_dist` (0.9) above `xy_goal_tolerance` (0.30).
- **Footprint** (`nav2_params.yaml`, both costmaps): rectangle ±0.65 m long, ±0.46 m wide (outside of the wheels), relative to `base_link`. Inscribed radius 0.46 m (the cyan zone around obstacles), circumscribed 0.80 m. Keep every `inflation_radius` ≥ 0.80 so the cost gradient covers the corners when turning, keep `cost_scaling_dist` (0.6) ≤ the local `inflation_radius`, and keep `FollowPath.inflation_cost_scaling_factor` equal to the local `cost_scaling_factor`. NavFn plans for the centre point only, so it can route through gaps that fit the robot's width but not a turn; RPP checks the real rectangle and stops instead of clipping.
- **Camera mount.** `camera_link` is `rpy="0 0 0"` under `base_link`. With `use_imu_tilt:=True` and the rover at rest on level ground, `ros2 run tf2_ros tf2_echo base_footprint base_link` must show roll and pitch near 0°. A steady offset means the camera is not mounted level: put the measured angle into the `camera_link` origin, otherwise it shows up as a false `base_link` tilt. The obstacle cloud stays level either way, because the IMU is inside the camera.
- **Kinematics** (`low_level_command` parameters): `wheel_separation` 0.801 m, `wheel_radius` 0.2 m.
- **Startup delays** are the `STARTUP_DELAYS` table in `bringup.launch.py`. Starting everything at once makes the serial drivers and RTAB-Map race each other.

## Debugging

| Symptom | Check |
|---|---|
| Robot doesn't move | `low_level_command` log: it stays locked until `/cmd_vel` is zero or quiet after startup. Check `ros2 topic echo /cmd_vel`. |
| Motors stop mid-route | Web app stall alert (no motion for 7 s auto-pauses). Watchdog: no `/cmd_vel` for 0.5 s = STOP. |
| Robot keeps driving, ignores stops (UI shows them reached) | The Teensy USB link dropped: `low_level_command` logs `Teensy link lost`, and `journalctl -k` shows `usb 1-2.2: USB disconnect` / `error -71` right when the motors start (the meter webcam on `1-2.1.1` drops with it). The ROS watchdog cannot reach the motors then, and the Teensy keeps its last command unless its firmware has its own timeout. `low_level_command` reopens `/dev/ttyTeensy` every 0.5 s, sends STOP and re-applies the startup lock; the web app auto-pauses navigation when `/teensy/feedback` (10 Hz) is silent for 1 s. The cause is electrical (motor noise / ground bounce on the USB cables): ferrite chokes, shielded short cables, a powered hub or a USB isolator on the Teensy. |
| No RTK FIXED | `ntrip_bridge` logs carrier transitions and RTCM counts every 10 s. The VRS sends nothing until it gets a GGA. |
| Heading wrong | `ros2 topic echo /gps/heading/imu`. `gps_heading_to_imu` warns when the heading flags are invalid. |
| Position jumps | Make sure navsat_transform did **not** log "Will assume navsat device is mounted at robot's origin" (that means the `gps_link` TF is missing). |
| Phantom obstacles | `/camera/depth/points_decimated`, `/camera/depth/obstacles_raw`, `/camera/depth/obstacles` and `/camera/depth/ground` in RViz (fixed frame `base_footprint`). Points in `obstacles_raw` but not `obstacles` were dropped as grass, points in `obstacles` but not `obstacles_raw` were promoted from ground. Grass height via `grass_height`, ground removal via the `Grid/*` parameters in `rtabmap.launch.py`, slope limits via `max_ground_angle`. |
| Slopes/tilt look wrong (`use_imu_tilt:=True`) | `ros2 run tf2_ros tf2_echo base_footprint base_link` must follow the chassis (lift a caster and watch pitch change), and `tf2_echo odom base_footprint` must stay at roll = pitch = 0. If `base_footprint → base_link` is missing, `imu_tilt` is waiting for `/imu/data_throttled` or the camera TFs (it logs which). |
| No DCVG values (0,0) | `teensy_feedback_server` log: the meter camera is missing or the easyocr load failed. Use `enable_ocr:=False` to rule it out. |
| Survey frames dropped | The counters line logged every 10 s: frames in / stamped / sent / dropped. |

Useful topics: `/odometry/local`, `/odometry/global`, `/gps/fix`, `/gps/heading/imu`, `/vo`, `/teensy/feedback`, `/dcvg/telemetry`, `/dcvg/reading`.

## Recording and replay (offline tuning)

Record during a field run, in a second terminal next to bringup (it can start before or after bringup):

```bash
ros2 run dcvg_bringup record_bag                      # profile "obstacles" (default)
ros2 run dcvg_bringup record_bag full -n rough_route  # name goes into the folder name
ros2 run dcvg_bringup record_bag nav -o /media/ssd/bags
```

| Profile | Adds | Can re-run offline | Approx. rate (zstd) |
|---|---|---|---|
| `nav` | — | localization (from recorded `/vo`), diagnosis of Nav2 runs | ~1 MB/s |
| `obstacles` | `/camera/depth/image_rect_raw` | + `depth_cloud` → `obstacles_detection` → costmaps | ~6 MB/s |
| `full` | `/camera/depth/image_rect_raw` + `/camera/infra1,2/image_rect_raw` | + stereo visual odometry | ~22 MB/s |

Every profile records the raw inputs (camera IMU, camera_info, both GPS boards incl. `NAV-RELPOSNED`, `/cmd_vel`, `/tf_static`) and the outputs to compare against (`/vo`, `/odometry/*`, `/tf`, obstacles/ground clouds, both costmaps and footprints, plans, `/behavior_tree_log`, `/rosout`, `/diagnostics`, `/teensy/feedback`, `/dcvg/*`). Bags go to `~/bags/<profile>[_name]_<time>/`, split every 1 GB, zstd per message. The recorder prints size, rate, free space and minutes left every 10 s, and stops itself when free space drops below `--min-free-gb` (5). The root NVMe is the only disk and is nearly full, so use `-o` on an external SSD for long `full` runs.

- **Compression queue.** Humble's default `--compression-queue-size` is 1; with it, about 70 messages/s were dropped (images, IMU, camera_info). `record_bag` uses queue 500 and 4 threads: 0 drops at 30 Hz stereo + 200 Hz IMU, about 60 % of one core.
- **RGBD VO** (`use_stereo:=False`) needs colour + aligned depth (about 135 MB/s raw, colour barely compresses) and is not covered by any profile.

Replay runs on its own `ROS_DOMAIN_ID` (77), so it cannot reach the live robot (played `/cmd_vel` never gets to `low_level_command`). It uses simulated time, re-runs the selected stages with the current launch files and configs, and plays everything else from the bag:

```bash
ros2 launch dcvg_bringup replay.launch.py bag:=$HOME/bags/obstacles_20261021_101500
ros2 launch dcvg_bringup replay.launch.py bag:=... run_vo:=True rate:=0.5
ros2 launch dcvg_bringup replay.launch.py bag:=... run_localization:=False max_ground_angle:=35.0
ros2 launch dcvg_bringup replay.launch.py bag:=... run_localization:=False grass_height:=0.15
ROS_DOMAIN_ID=77 rviz2
```

| Argument | Default | Re-runs | Recorded copy moved to |
|---|---|---|---|
| `run_localization` | True | both EKFs, navsat_transform, `gps_heading_to_imu` (`heading_offset_deg`) | `/rec/tf`, `/rec/odometry/*`, `/rec/gps/filtered`, `/rec/gps/heading/imu` |
| `run_obstacles` | True | `depth_cloud`, `obstacles_detection` (`max_ground_angle`, `max_ground_height`), `terrain_filter` (`use_terrain_filter`, `grass_height`, `step_height`), Nav2 with both costmaps (no goal is sent) | `/rec/camera/depth/obstacles`, `/rec/camera/depth/obstacles_raw`, `/rec/camera/depth/ground`, `/rec/*_costmap/*` |
| `run_vo` | False | stereo/RGBD odometry (`use_stereo`) | `/rec/vo` |

Stages that are off take their inputs from the bag: with `run_localization:=False` the recorded `/tf` drives the costmaps, with `run_vo:=False` the EKFs use the recorded `/vo`. Edit `localization.yaml`, `nav2_params.yaml` or `rtabmap.launch.py`, `colcon build`, replay again, and compare the new topic with its `/rec/...` copy (`rqt_plot`, RViz). Replay starts its own `robot_state_publisher` from the current URDF (`use_imu_tilt:=false`), so bags recorded before the URDF frames were in `/tf_static` still work, but bags recorded with `use_imu_tilt:=True` are not supported. `bag_qos.yaml` replays `/tf_static` as transient-local, so late subscribers still get the camera frames. `start_delay` (10 s) lets Nav2 activate before playback starts.

## Safety

There is no flight-controller failsafe. Keep an independent hardware or RC kill switch on the motor power during every outdoor run.
