# DCVG Autonomous Rover Project

## Project Overview
Converting a Chinese grass-cutting robot into a large autonomous outdoor rover for DCVG (Direct Current Voltage Gradient) pipeline inspection. The robot navigates outdoors using GPS waypoints, following OSM (OpenStreetMap) routes, with obstacle avoidance from a depth camera (real hardware has no physical LiDAR — see Known Issues). Localization is **GPS + Visual-Inertial Odometry (VIO)** — the real robot has no wheel encoders. Built on ROS2 Humble.

## Package Layout & Intent (read this first)
`dcvg_bringup` is the **only runnable navigation stack in this workspace** —
it targets real hardware directly. A Gazebo Classic simulation used to live in
`dcvg_robot` and was used to design and validate this architecture (the
`ekf_local`/`ekf_global` split, VIO-vs-GPS reasoning, etc. below all trace back
to that work), but the Gazebo world/model/launch files and the sim-only debug
tools have since been removed from the repo — there is no local simulation to
fall back on anymore. First real-world testing IS the next validation step.

- **`dcvg_bringup` = REAL-HARDWARE bringup (scaffolded).** Brings up the *real* sensor drivers (RealSense, u-blox GPS/RTK — no physical LiDAR in this build), the Teensy motor bridge, and the full RTAB-Map + EKF + Nav2 stack. Self-contained: it carries its own copies of the EKF/Nav2 configs + behavior trees (with `use_sim_time=False`).
- **`dcvg_robot` = shared robot description + operator tooling.** No longer a simulation package. What remains: the robot xacro/mesh (`bringup.launch.py` reuses it directly for `robot_state_publisher`), and the stack-agnostic control/planning tools — `osm_route_app.py` (Flask web UI), `osm_route_generator.py` (OSM route planning), `route_waypoint_follower.py` (GPS waypoint following via Nav2) — which `dcvg_bringup` depends on and runs directly (it has no equivalents of its own).

> **No Pixhawk/ArduPilot/MAVROS in this build.** Motor commands go straight from Nav2's `/cmd_vel` to a Teensy over serial (`low_level_command`, differential-drive kinematics on the ROS side, PWM to the Pololu G2 drivers on the Teensy side), and GPS+RTK comes directly from the simpleRTK2B Heading (u-blox ZED-F9P) via `ublox_gps` + `ntrip_client`. There is no flight controller in the loop.

## Current Status — What's In Place
- GPS waypoint navigation with Nav2 (no static map, rolling costmap)
- Obstacle avoidance from the depth-camera obstacle cloud in the Nav2 costmap (no LiDAR — see Known Issues)
- **VIO via RTAB-Map** publishing `/vo`, consumed by `robot_localization` (no wheel encoders on the real robot)
- Dual EKF + navsat_transform sensor fusion (robot_localization), including GPS moving-baseline heading fused into `ekf_global`
- Depth-based obstacle detection with **ground-plane removal** (a depth camera alone would otherwise see tall grass/weeds as obstacles)
- OSM route planning with web-based control app (Flask on port 5000)
- OSM route following (`route_waypoint_follower.py`)
- Custom recovery behavior trees (BackUp before Spin)
- Custom robot model with STL mesh (shared xacro, reused directly by `dcvg_bringup`)

### Migrated to VIO (done)
- Forward/inverse kinematics, wheel-odometry, and DCVG controller nodes **removed** (no longer in `setup.py`)
- EKF fuses `/vo` (local) + GPS (global) + IMU — see `dual_ekf_navsat_params.yaml`
- Nav2 costmap consumes the ground-filtered depth obstacle cloud (no LiDAR source)

## Next Phase — Real-Hardware Bringup
`dcvg_bringup` is **scaffolded and builds** (launch + config + behavior trees in place).
The launch graph runs the real sensor drivers + GPS/RTK + Teensy motor bridge, then the
same RTAB-Map VIO + EKF + Nav2 stack proven in sim. Remaining work before first hardware run:
1. **Confirm real topic names / device ports** — GPS + Teensy ports are now fixed (see
   "Serial Ports" below); RealSense topic names in `sensors.launch.py` are still TBD
   (no physical LiDAR in this build — RPLIDAR was dropped, obstacle avoidance is
   depth-camera-only).
2. **IMU source** — camera IMU via `imu_filter_madgwick` (wired in `sensors.launch.py`),
   fusing raw accel+gyro into an oriented quaternion on `/imu/data`. No magnetometer, so
   this yaw is gyro integration from an arbitrary boot-time reference — `ekf_local` uses
   it (fine there, only needs to be smooth). `ekf_global` yaw instead comes solely from
   the GPS moving-baseline heading (`gps_heading_to_imu` → `/gps/heading/imu`) — confirmed
   on hardware that also fusing the camera-gyro yaw into `ekf_global` degrades it (see
   Known Issues) — needs `heading_offset_deg` calibrated against the real antenna
   mounting (a `bringup.launch.py` launch arg) before trusting it.
3. **Teensy firmware** — confirm the Teensy sketch parses `low_level_command`'s wire
   format (`$<left>,<right>,<probe_up>,<probe_down>#\n`) and drives the Pololu G2s +
   probe actuator accordingly; measure `wheel_separation`/`wheel_radius` on the real robot.
4. **RTK corrections** — confirm the F9P has `CFG-USBINPROT-RTCM3X` enabled (factory
   default) so `ntrip_bridge`'s RTCM writes actually reach the receiver.
5. **HW robot description** — optionally split a Gazebo-free xacro out of `dcvg_robot`
   (currently `bringup.launch.py` reuses the sim xacro; `<gazebo>` tags are ignored by RSP).
6. **Independent kill-switch** — since there's no ArduPilot failsafe chain anymore, an
   independent RC/hardware kill-switch on the Teensy or motor power is mandatory before
   any autonomous motion outdoors.

## Serial Ports (confirmed)
The simpleRTK2B Heading kit is two separate u-blox F9P boards, each its own USB
serial device; the Teensy is a third. Fixed via udev enumeration (no by-id
symlinks set up yet — replug order matters until they are):

| Device | Port | Purpose |
|---|---|---|
| GPS position board (big board) | `/dev/ttyACM0` | Absolute fix; gets NTRIP RTK corrections |
| GPS heading board (small board) | `/dev/ttyUSB0` | NAV-RELPOSNED (relative heading vector) |
| Teensy | `/dev/ttyACM1` | `low_level_command` motor bridge |

Overridable via `gps_device` / `gps_heading_device` / `teensy_port` launch args on
`bringup.launch.py` if enumeration ever shifts — prefer `/dev/serial/by-id/...` then.

## Sensor Topics — TO CONFIRM (no hardware yet)
`sensors.launch.py` / `dcvg_bringup/launch/rtabmap.launch.py` have settled on the topic
names below (RealSense's own naming, `namespace=''`/`name='camera'` — see the big comment
in `sensors.launch.py` explaining why), but **none of it has been verified against a
physically connected camera yet** — confirm here once hardware is connected.

| Sensor | Topic | Status |
|---|---|---|
| RGB image | `/camera/color/image_raw` | unconfirmed on hardware |
| Depth image | `/camera/aligned_depth_to_color/image_raw` | unconfirmed on hardware |
| Camera info | `/camera/color/camera_info` | unconfirmed on hardware |
| Depth point cloud | `/camera/depth/color/points` | unconfirmed on hardware |
| Stereo IR pair | `/camera/infra1\|2/image_rect_raw` (+ `camera_info`) | unconfirmed on hardware |
| Camera IMU (raw) | `/camera/imu` → filtered to `/imu/data` (imu_filter_madgwick) | unconfirmed on hardware |
| 2D LiDAR scan | — | removed — no physical LiDAR in this build |
| GPS fix | `/gps/fix` (ublox_gps, position board `/dev/ttyACM0`) | fixed |
| GPS velocity | `/gps/vel` (ublox_gps, position board) | fixed |
| GPS heading (relposned) | `/gps/heading/navrelposned` (ublox_gps, heading board `/dev/ttyUSB0`) → `/gps/heading/imu` (`gps_heading_to_imu`) | fixed, fused into `ekf_global` |
| VIO out | `/vo` | fixed |
| Depth obstacles | `/camera/depth/obstacles` | fixed |

## Architecture (current, VIO-based)
```
Depth camera (RGB+Depth+IMU) → RTAB-Map rgbd/stereo_odometry → /vo → ekf_local
                              → obstacles_detection (ground filtered) → /camera/depth/obstacles → Nav2 costmap
GPS (fix) → navsat_transform → /odometry/gps ─────────────────────→ ekf_global
IMU (data) → imu_throttle → /imu/data_throttled ───────────────────→ BOTH EKFs, yaw RATE only
GPS heading (navrelposned) → gps_heading_to_imu → /gps/heading/imu ──→ BOTH EKFs (absolute yaw)

robot_localization:
  ekf_local  (world=odom): /vo vel + gyro yaw RATE + GPS heading yaw → odom → base_footprint (smooth, may drift)
  ekf_global (world=map):  GPS position + gyro yaw RATE + GPS heading yaw → map → odom (GPS-anchored, zero drift)
  Neither EKF fuses the camera's ABSOLUTE yaw — only its rate (see Known Issues).

Nav2 → /cmd_vel → low_level_command → Teensy → Pololu G2s → robot moves

RTK corrections: NTRIP caster → ntrip_client → /rtcm → ntrip_bridge → F9P serial
                 F9P NAV-PVT → ntrip_bridge → GGA on /nmea → ntrip_client → caster (VRS)
```
Rationale: VIO drifts, GPS does not. VIO lives in `ekf_local` only (smooth interpolation between fixes); the global frame stays GPS-anchored to avoid contaminating it with drift.

## Workspace Structure
```
~/dcvg_robot/
├── src/
│   ├── dcvg_robot/                          # Shared robot description + control tools (Python/ament_python)
│   │   ├── package.xml / setup.py / setup.cfg
│   │   ├── App.bash                         # Quick-start (sources + launches web UI)
│   │   ├── resource/dcvg_robot
│   │   │
│   │   ├── urdf/
│   │   │   ├── dcvg_robot.urdf.xacro        # Robot description (xacro source) — reused directly
│   │   │   │                                #   by dcvg_bringup's robot_state_publisher; still has
│   │   │   │                                #   inert <gazebo> tags left over from the removed sim
│   │   │   └── dcvg_robot.urdf              # Stale pre-expanded URDF, not referenced by any launch file
│   │   ├── meshes/DCVG_Robot.stl            # Custom 3D mesh, referenced by the xacro's <visual> tags
│   │   │
│   │   ├── config/
│   │   │   ├── waypoints.yaml / waypoints_ref.yaml / stops.yaml   # GPS route data (read/written by osm_route_app)
│   │   │   ├── vio_test_routes.yaml         # Test routes for VIO evaluation
│   │   │   ├── osm_route_config.yaml
│   │   │   ├── osm_route_comparison.html    # Leftover generated artifact, unused by any live code path
│   │   │   └── mapviz.mvc / gps_wpf_demo.mvc  # Orphaned — mapviz.launch.py (sim-only) was removed
│   │   │
│   │   ├── rviz/dcvg_env.rviz
│   │   │
│   │   └── dcvg_robot/                          # Python nodes — stack-agnostic, used by dcvg_bringup too
│   │       ├── __init__.py
│   │       ├── osm_route_app.py                 # Flask web app for route control
│   │       ├── osm_route_generator.py           # OSM route planning
│   │       └── route_waypoint_follower.py       # Follow OSM routes
│   │
│   └── dcvg_bringup/                            # REAL-HARDWARE bringup (Python/ament_python)
│       ├── package.xml / setup.py / setup.cfg
│       ├── launch/
│       │   ├── bringup.launch.py                # Top-level real-HW entry (chains all below + Nav2)
│       │   ├── sensors.launch.py                # RealSense + imu_filter_madgwick (topics/ports TBD; no LiDAR)
│       │   ├── ublox_gps.launch.py              # simpleRTK2B/F9P GPS driver → /gps/fix, /gps/vel
│       │   ├── ntrip.launch.py                  # NTRIP RTK corrections → the F9P
│       │   ├── localization.launch.py           # dual EKF + navsat (use_sim_time=False)
│       │   └── rtabmap.launch.py                # VIO + obstacles (use_sim_time=False; camera topics as args)
│       ├── dcvg_bringup/
│       │   ├── low_level_command.py             # /cmd_vel → diff-drive kinematics → serial → Teensy
│       │   ├── ntrip_bridge.py                  # /rtcm → F9P serial (write-only); /navpvt → GGA on /nmea
│       │   └── gps_heading_to_imu.py             # NAV-RELPOSNED9 → yaw-only IMU msg for ekf_global
│       ├── config/                              # own copies of nav2 + EKF params (sim-time flipped to False) + ublox_f9p.yaml
│       └── behavior_trees/                      # own copies of the recovery BTs
├── build/ install/ log/
```

## How to Run (real hardware)

### Full stack (one command)
```bash
source ~/.bashrc
source ~/dcvg_robot/install/setup.bash
ros2 launch dcvg_bringup bringup.launch.py
```

### Control via Web App (separate terminal)
```bash
ros2 run dcvg_robot osm_route_app          # http://127.0.0.1:5000
```

### Options
```bash
ros2 launch dcvg_bringup bringup.launch.py use_rviz:=True
ros2 launch dcvg_bringup bringup.launch.py use_stereo:=false   # D455 RGBD instead of D457 IR stereo (default True)
```

## Launch Flow (real hardware)
`bringup.launch.py` is the single entry point. See its own docstring for the full chain
(robot_state_publisher → sensors → GPS/RTK → low_level_command → localization → RTAB-Map
→ Nav2); `osm_route_app` and `route_waypoint_follower` (both from `dcvg_robot`) are run
separately, on demand, from the web UI.

## Key Technical Decisions
- **`dcvg_robot` supplies the shared robot description + control tools; `dcvg_bringup` is the actual runnable stack** — keep that boundary clean.
- **nav2_bringup** — standard `navigation_launch.py` (not individual Nav2 nodes).
- **RTAB-Map for VIO** — odometry-only mode (NOT full SLAM); replaces missing wheel encoders.
- **RTAB-Map for obstacle detection** — depth-based with ground-plane removal (a depth camera alone sees tall grass/weeds as obstacles otherwise).
- **RGBD vs stereo VIO** — `rtabmap.launch.py use_stereo:=` switches D455 RGBD (≤6 m) vs D457 IR stereo (passive, sunlight-immune, ~15 m). Pick the real-HW default deliberately.
- **No Pixhawk/ArduPilot/MAVROS** — motor control and GPS/RTK are handled directly (Teensy serial bridge + u-blox driver); there is no flight controller in this build.
- **Teensy + Pololu G2 for motor control** — `low_level_command` does diff-drive inverse kinematics on the ROS side and streams an ASCII wheel-speed command over serial; the Teensy does PWM to the Pololu G2 drivers.
- **ublox_gps (UBX-native) for GPS/RTK, NOT nmea_navsat_driver** — the F9P streams binary UBX on native USB, not ASCII NMEA.
- **No static map** — outdoor GPS navigation with rolling costmaps only.
- **YOLOv8-seg traversability** — future phase: classify grass vs real obstacles (ICRA 2024, arxiv 2407.18535).

## Hardware (real deployment)
- **Intel RealSense D455/D457** — depth + IMU for VIO. D455: 95 mm baseline, depth 0.6–6 m, BMI270 IMU. D457: IR stereo pair, passive (sunlight-immune), longer range.
- **Jetson Orin Nano** — companion computer (ROS2 Humble native, no Docker).
- **simpleRTK2B Heading (u-blox ZED-F9P)** — RTK GPS module, native USB UBX; RTK corrections via NTRIP (`ublox_gps` + `ntrip_client` + `ntrip_bridge`).
  - **Antenna mounting (measured 2026-09-14).** Both antennas on the robot's centreline:
    **HEADING antenna at the FRONT, POSITION antenna at the REAR.** The position antenna
    (the one producing `/gps/fix`) sits **0.60 m behind `base_link`** and **1.00 m above the
    ground** — modelled as `gps_link` at `xyz="-0.60 0 0.92"` in the xacro (0.92 = 1.00 −
    `base_height` 0.08). That 0.60 m lever arm is **not cosmetic**: with it unmodelled, every
    stop overshot by 0.60 m and every in-place pivot injected an apparent position jump of
    `2 × 0.60 × sin(θ/2)` — 0.21 m at 20°, 0.85 m at 90°, i.e. bigger than the whole 0.5 m
    route corridor. That was the root cause of "drives straight fine, loses the route
    whenever it rotates".
  - Because the baseline runs **rear (moving base) → front (rover)**, which is `base_link`'s
    **+X**, `heading_offset_deg` should be **0.0** — the `bringup.launch.py` default is
    already correct. Confirm once against a known bearing, but expect no 90°/180° offset.
- **Realsense D457 Depth Camera** — Stereo Depth Camera for visual odometry
- **Teensy** — reads `/cmd_vel`-derived wheel speeds over serial from `low_level_command` and drives PWM to the Pololu G2s.
- **Pololu G2 Motor Drivers ×2** — driven by the Teensy.
- **Not using:** Arduino, TFmini Plus (D455 replaces it), Pixhawk/ArduPilot/MAVROS (Teensy + direct GPS replace it), RPLIDAR S2 (dropped — obstacle avoidance is depth-camera-only on real hardware; sim still simulates a 2D LiDAR).

## Software Stack
- **OS:** Ubuntu 22.04 — **ROS2:** Humble
- **Navigation:** Nav2 (via nav2_bringup), no-map rolling costmap
- **Localization:** robot_localization (dual EKF + navsat_transform)
- **VIO + depth obstacles:** RTAB-Map (`sudo apt install ros-humble-rtabmap-ros`)
- **Camera driver:** realsense2_camera (`sudo apt install ros-humble-realsense2-camera`)
- **IMU orientation filter:** imu_filter_madgwick (`sudo apt install ros-humble-imu-filter-madgwick`)
- **GPS/RTK driver:** ublox_gps + ublox_msgs (`sudo apt install ros-humble-ublox-gps ros-humble-ublox-msgs`)
- **NTRIP client:** ntrip_client, nmea_msgs, rtcm_msgs (`sudo apt install ros-humble-ntrip-client ros-humble-nmea-msgs ros-humble-rtcm-msgs`)
- **Motor bridge:** Teensy over serial (`low_level_command`, `python3-serial`) — no flight controller
- **Visualization:** RViz2 — **Control:** Flask web app + OSM route integration

## Research References
- **Nav2 GPS waypoint demo:** https://github.com/ros-navigation/navigation2_tutorials/tree/humble/nav2_gps_waypoint_follower_demo
- **RTAB-Map Stereo Outdoor Navigation:** https://wiki.ros.org/rtabmap_ros/Tutorials/StereoOutdoorNavigation
- **YOLOv8 Nav2 traversability (ICRA 2024):** arxiv 2407.18535
- **RAMP-VO:** REJECTED — needs event camera hardware
- **TERP DRL Planning:** DEFERRED — ROS1 only, needs 3D LiDAR
- **VLM-GroNav:** DEFERRED — no public code, VLM too heavy for edge
- **NVIDIA Isaac Perceptor:** BACKUP — GPU-accelerated perception on Orin

## Known Issues & Gotchas
- conda and ROS2 conflict — never mix them.
- Real robot has no wheel encoders — VIO from depth-camera + RTAB-Map is the replacement.
- A depth camera alone sees tall grass/weeds as obstacles — ground-plane removal in `obstacles_detection` is why that filtering exists.
- GPS provides ~1–3 m accuracy — Nav2 goal tolerance set to ~1 m to match.
- No LiDAR in this build (RPLIDAR dropped) — the depth camera's ~6 m outdoor range (RGBD)
  is the *only* obstacle-avoidance range on real hardware, with no long-range backstop.
  Account for this in waypoint spacing/cruise speed near obstacles.
- There is no local Gazebo simulation anymore — `dcvg_robot`'s Gazebo world/model/launch files
  and sim-only debug tools were removed once real-hardware bringup work took over; don't go
  looking for a `gps_waypoint_follower.launch.py` or similar, it no longer exists.
- `dcvg_robot`'s xacro (`urdf/dcvg_robot.urdf.xacro`) still has `<gazebo>` plugin tags embedded
  from when it was also used for simulation — these are inert on real hardware
  (`robot_state_publisher` ignores them), harmless to leave, not worth stripping out.
- `dcvg_robot` is ament_python (setup.py), NOT ament_cmake.
- GPS (position + heading) and Teensy serial ports are confirmed — see "Serial Ports" above.
  RealSense topic names are still TBD (no LiDAR in this build) — see the
  "Sensor Topics — TO CONFIRM" table.
- The GPS heading board's NAV-RELPOSNED9 is converted to a yaw-only IMU message by
  `gps_heading_to_imu` (`/gps/heading/imu`) and fused into `ekf_global` as `imu1` — its
  **sole** yaw source. It's an absolute, driftless yaw source but its zero-point depends
  on physical antenna mounting — calibrate `heading_offset_deg` (a `bringup.launch.py`
  launch arg) against a known heading before trusting it: e.g. point the robot at a known
  compass bearing, read the resulting yaw from `/gps/heading/imu` with the offset at 0,
  and set `heading_offset_deg` to `(true_bearing − measured_yaw)`, normalized to [-180,180].
- `ekf_global` fuses the camera IMU's yaw **RATE only**, never its absolute yaw.
  `dual_ekf_navsat_params.yaml`'s `ekf_filter_node_map` does have an `imu0` key — but its
  config enables index 11 (`vyaw`) and nothing else, which is what satisfies
  robot_localization's requirement that yaw or its velocity be observed. Confirmed on
  hardware: fusing the camera's *absolute* yaw alongside GPS heading made `ekf_global`'s
  yaw *worse*, not better.
  Reason: `imu_filter_madgwick` runs with `use_mag:=false` (no magnetometer on the
  D455/D457), so its yaw is gyro integration from an arbitrary boot-time reference, fused
  as if absolute — it disagrees with the correctly-referenced GPS heading, and there's no
  covariance value that fixes "this sensor's absolute reference changes every boot."
  `ekf_local` still uses its own separate gyro-only `imu0` — fine there, since `ekf_local`
  only needs to be smooth, not absolutely accurate.
- The F9P's VRS mountpoint sends nothing until it receives a GGA sentence with our position — a few seconds of silent `/rtcm` at startup is normal, not a fault.

## Safety Requirements (real hardware phase)
- No ArduPilot/Pixhawk failsafe chain in this build — an independent RC or hardware kill-switch on the Teensy/motor power is mandatory during outdoor testing.
- Test motor arm/disarm (or equivalent Teensy enable) before any autonomous movement.
- Confirm the `low_level_command` watchdog (STOP on missed `/cmd_vel`) actually halts the motors before relying on it.
