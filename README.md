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
│   │   ├── sensors.launch.py     RealSense + imu_filter_madgwick → /imu/data
│   │   ├── ublox_gps.launch.py   Both F9P boards + gps_heading_to_imu
│   │   ├── ntrip.launch.py       NTRIP client + ntrip_bridge (RTK corrections)
│   │   ├── localization.launch.py  Dual EKF + navsat_transform
│   │   └── rtabmap.launch.py     Visual odometry (/vo) + ground-filtered obstacles
│   ├── config/                   nav2_params.yaml, localization.yaml, ublox_f9p*.yaml
│   ├── behavior_trees/           Nav2 BTs (recovery = clear costmaps + wait, never moves)
│   ├── ocr/                      Digit templates for the meter OCR
│   └── dcvg_bringup/
│       ├── low_level_command.py  /cmd_vel → Teensy serial, probe cycle, /teensy/feedback
│       ├── gps_heading_to_imu.py NAV-RELPOSNED → yaw-only Imu for the EKFs
│       ├── imu_throttle.py       /imu/data (~200 Hz) → /imu/data_throttled (50 Hz)
│       ├── ntrip_bridge.py       /rtcm → F9P serial, /navpvt → GGA on /nmea
│       ├── teensy_feedback_server.py  Stamps DCVG + GPS on Teensy frames → TCP :5005
│       └── dcvg_ocr.py           Reads ON/OFF values off the meter LCD
├── dcvg_robot/                   Robot model + operator tools
│   ├── urdf/dcvg_robot.urdf.xacro  TF frames (base_footprint, base_link, camera_link, gps_link, gps_heading_link)
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
| Visual odometry | stereo/rgbd_odometry, obstacles_detection | `/vo` |
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

Launch arguments: `use_stereo` (True = D457 IR stereo VO, False = RGBD VO), `use_rviz`, `use_telemetry`, `enable_ocr`, `heading_offset_deg`, `teensy_port`, `gps_device`, `gps_heading_device`.

To autostart the web app on boot:

```bash
sudo cp src/system/dcvg-auto.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now dcvg-auto
```

## How it works

```
RealSense ─ rgbd/stereo_odometry ─ /vo ───────────────────────────┐
          ─ obstacles_detection ─ /camera/depth/obstacles ─ Nav2 costmaps
          ─ imu_filter_madgwick ─ /imu/data ─ imu_throttle ─┐      │
GPS heading board ─ gps_heading_to_imu ─ /gps/heading/imu ──┤      │
                                                            ├─ ekf_filter_node_odom → odom→base_footprint, /odometry/local
GPS position board ─ /gps/fix ─ navsat_transform ─ /odometry/gps ─ ekf_filter_node_map → map→odom, /odometry/global
NTRIP caster ⇄ ntrip_client ⇄ ntrip_bridge ⇄ position board (RTCM in, GGA out)

web app ─ route_waypoint_follower ─ Nav2 (RPP controller) ─ /cmd_vel ─ low_level_command ─ Teensy
```

- **Localization.** `ekf_filter_node_odom` fuses VO velocity, gyro yaw rate and GPS heading. `ekf_filter_node_map` fuses GPS position, gyro yaw rate and GPS heading. Neither EKF fuses the camera IMU's absolute yaw: it has no magnetometer, so its zero point changes on every boot.
- **navsat_transform** takes its odometry input from `/odometry/global`, not `/odometry/local`. That way `/odometry/gps` is published in the `map` frame. With the `odom` frame it created a feedback loop that sent the map EKF position off to 10⁷ m.
- **Route following.** The route is split into segments at every stop. Each segment is sent as one `NavigateThroughPoses` goal after straight runs are collapsed (`simplify_*` parameters). When a segment fails, the rover holds and waits for the operator (**Run** = retry, **Stop** = end). `-p on_segment_failure:=skip` skips the failed segment instead.
- **Stops.** The follower publishes `/dcvg/stop_reached`. `low_level_command` then waits 2 s to settle, lowers the probe for 8 s, raises it for 3 s, and publishes `/dcvg/stop_confirm`, which releases the follower. The **Confirm** button in the web UI does the same.
- **Web app ↔ follower** talk through files in `/tmp`: `dcvg_nav_cmd` (run/pause/stop), `dcvg_stop_status`, `dcvg_stop_confirm`, `dcvg_nav_target`.
- **Teensy wire format.** ROS sends `$<left_rad_s>,<right_rad_s>,<probe_up>,<probe_down>#\n`. Lines from the Teensy that start with `$` are republished on `/teensy/feedback`. `teensy_feedback_server` appends `dcvg_on,dcvg_off,lat,lon` to them (17 fields in total) and serves them on TCP 5005.

## Calibrated values (do not change without re-measuring)

- **GPS lever arm.** The position antenna is 0.60 m **behind** `base_link` and 1.00 m above the ground: `gps_link` at `-0.60 0 0.92` in the xacro. If this is wrong, every stop overshoots and every in-place pivot shows up as fake position jumps.
- **GPS heading.** The heading antenna is at the front, on the centreline, so `heading_offset_deg` is 0.0. To recalibrate, point the rover at a known bearing and set `heading_offset_deg = true_bearing − measured_yaw`.
- **Drive limits** (`nav2_params.yaml`): 0.2 m/s linear and 0.1 rad/s angular. `velocity_smoother.max_accel` is `[0.4, 0, 0.8]` and must keep its angular value equal to `FollowPath.max_angular_accel`. `rotate_to_heading_angular_vel: 0.15` must stay set (RPP's default is 1.8 rad/s). Keep `yaw_goal_tolerance` (0.35) below `rotate_to_heading_min_angle` (0.43), and `lookahead_dist` (0.9) above `xy_goal_tolerance` (0.30).
- **Kinematics** (`low_level_command` parameters): `wheel_separation` 0.801 m, `wheel_radius` 0.2 m.
- **Startup delays** are the `STARTUP_DELAYS` table in `bringup.launch.py`. Starting everything at once makes the serial drivers and RTAB-Map race each other.

## Debugging

| Symptom | Check |
|---|---|
| Robot doesn't move | `low_level_command` log: it stays locked until `/cmd_vel` is zero or quiet after startup. Check `ros2 topic echo /cmd_vel`. |
| Motors stop mid-route | Web app stall alert (no motion for 7 s auto-pauses). Watchdog: no `/cmd_vel` for 0.5 s = STOP. |
| No RTK FIXED | `ntrip_bridge` logs carrier transitions and RTCM counts every 10 s. The VRS sends nothing until it gets a GGA. |
| Heading wrong | `ros2 topic echo /gps/heading/imu`. `gps_heading_to_imu` warns when the heading flags are invalid. |
| Position jumps | Make sure navsat_transform did **not** log "Will assume navsat device is mounted at robot's origin" (that means the `gps_link` TF is missing). |
| Phantom obstacles | `/camera/depth/obstacles` in RViz. Ground removal is the `Grid/*` parameters in `rtabmap.launch.py`. |
| No DCVG values (0,0) | `teensy_feedback_server` log: the meter camera is missing or the easyocr load failed. Use `enable_ocr:=False` to rule it out. |
| Survey frames dropped | The counters line logged every 10 s: frames in / stamped / sent / dropped. |

Useful topics: `/odometry/local`, `/odometry/global`, `/gps/fix`, `/gps/heading/imu`, `/vo`, `/teensy/feedback`, `/dcvg/telemetry`, `/dcvg/reading`.

## Safety

There is no flight-controller failsafe. Keep an independent hardware or RC kill switch on the motor power during every outdoor run.
