# DCVG Autonomous Rover Project

## Project Overview
Converting a Chinese grass-cutting robot into a large autonomous outdoor rover for DCVG (Direct Current Voltage Gradient) pipeline inspection. The robot navigates outdoors using GPS waypoints, following OSM (OpenStreetMap) routes, with obstacle avoidance from a 2D LiDAR + a depth camera. Localization is **GPS + Visual-Inertial Odometry (VIO)** — the real robot has no wheel encoders. Built on ROS2 Humble.

## Package Layout & Intent (read this first)
This workspace separates **simulation** from **real hardware**:

- **`dcvg_robot` = SIMULATION ONLY.** Gazebo Classic world, robot model/URDF, Gazebo sensor plugins, and the sim entry-point launch. Sensor data here comes from Gazebo plugins. Use this to develop and validate the navigation/localization stack without hardware.
- **`dcvg_bringup` = REAL-HARDWARE bringup (scaffolded).** Brings up the *real* sensor drivers (RealSense, RPLIDAR, MAVROS, GPS) and reuses the same downstream stack. Self-contained: it carries its own copies of the EKF/Nav2 configs + behavior trees (with `use_sim_time=False`) so it can diverge from sim freely. Everything outside `dcvg_robot` targets real hardware.

The **shared middle layer is reused across both** — only the sensor *sources* differ:

```
            SIM (dcvg_robot)                 REAL HW (bringup pkg, planned)
            Gazebo plugins                   realsense2_camera / rplidar_ros /
                  │                          MAVROS / GPS driver
                  ▼                                   │
   ┌──────────────────────────── SHARED ─────────────▼──────────────┐
   │  RTAB-Map (rgbd/stereo VIO  →  /vo)  +  obstacles_detection      │
   │  robot_localization (dual EKF + navsat_transform)                │
   │  Nav2 (nav2_bringup, no-map rolling costmap)                     │
   └──────────────────────────────────────────────────────────────── ┘
```
When building the real-hardware bringup, **reuse** `dual_ekf_navsat_params.yaml`, `nav2_no_map_params.yaml`, the RTAB-Map node config, and the behavior trees. **Replace** only the Gazebo-specific launch (`gazebo_gps_world.launch.py`) and the Gazebo `diff_drive` plugin (→ MAVROS `cmd_vel`).

## Current Status — What Works (in simulation, `dcvg_robot`)
- GPS waypoint navigation with Nav2 (no static map, rolling costmap)
- Obstacle avoidance fusing **2D LiDAR + depth-camera obstacle cloud** in the Nav2 costmap
- **VIO via RTAB-Map** publishing `/vo`, consumed by `robot_localization` (wheel odometry removed)
- Dual EKF + navsat_transform sensor fusion (robot_localization)
- Depth-based obstacle detection with **ground-plane removal** (solves "LiDAR sees grass as walls")
- OSM route planning with web-based control app (Flask on port 5000)
- Route recording / playback and OSM route following
- Custom recovery behavior trees (BackUp before Spin)
- Mapviz and RViz2 visualization; custom robot model with STL mesh
- Simulation on Gazebo Classic (gazebo11), sonoma_raceway worlds

### Already migrated to VIO (done — was the previous "next phase")
- Forward/inverse kinematics, wheel-odometry, and DCVG controller nodes **removed** (no longer in `setup.py`)
- Gazebo `diff_drive` plugin kept *only* to move the robot from `/cmd_vel`; it no longer publishes odom or odom→base_footprint TF (`publish_odom=false`, `publish_odom_tf=false`)
- EKF now fuses `/vo` (local) + GPS (global) + IMU — see `dual_ekf_navsat_params.yaml`
- Nav2 costmap consumes both LiDAR scan and the ground-filtered depth obstacle cloud

## Next Phase — Real-Hardware Bringup
`dcvg_bringup` is **scaffolded and builds** (launch + config + behavior trees in place).
The launch graph runs the real sensor drivers + MAVROS, then the same RTAB-Map VIO +
EKF + Nav2 stack proven in sim. Remaining work before first hardware run:
1. **Confirm real topic names / device ports** — everything marked TBD in the launch
   files and the "Sensor Topics" table (RealSense, RPLIDAR serial, MAVROS FCU URL).
2. **IMU source** — decide camera IMU (+imu_filter_madgwick for orientation) vs Pixhawk
   IMU from MAVROS; ensure it lands on `/imu/data`.
3. **cmd_vel → MAVROS bridge** — relay `/cmd_vel` to `/mavros/setpoint_velocity/cmd_vel_unstamped`
   (commented stub in `mavros.launch.py`).
4. **HW robot description** — optionally split a Gazebo-free xacro out of `dcvg_robot`
   (currently `bringup.launch.py` reuses the sim xacro; `<gazebo>` tags are ignored by RSP).
5. **ArduPilot safety** — RC failsafe, geofence, arm/disarm tested before autonomy.

## Sensor Topics — TO CONFIRM (no hardware yet)
Real-hardware topic names are **not yet finalized** — confirm here once drivers are connected.
The sim topic names are what `dcvg_robot` / RTAB-Map / Nav2 currently use.

| Sensor | Sim topic (`dcvg_robot`) | Real-HW topic | Status |
|---|---|---|---|
| RGB image | `/d455/image_raw` | TBD | confirm |
| Depth image | `/d455/depth/image_raw` | TBD | confirm |
| Camera info | `/d455/camera_info` | TBD | confirm |
| Depth point cloud | `/d455/points` | TBD | confirm |
| Stereo IR pair | `/d457/left|right/image_raw` (+info) | TBD | confirm |
| IMU | `/imu/data` | TBD | confirm |
| 2D LiDAR scan | `/lidar/scan` | TBD | confirm |
| GPS fix | `/gps/fix` | TBD | confirm |
| VIO out (shared) | `/vo` | `/vo` | fixed |
| Depth obstacles (shared) | `/camera/depth/obstacles` | `/camera/depth/obstacles` | fixed |

> When wiring real drivers, prefer **remapping driver outputs onto the sim topic names above** so the shared RTAB-Map/Nav2 config needs no edits.

## Architecture (current, VIO-based)
```
Depth camera (RGB+Depth+IMU) → RTAB-Map rgbd/stereo_odometry → /vo → ekf_local
                              → obstacles_detection (ground filtered) → /camera/depth/obstacles → Nav2 costmap
2D LiDAR (scan) ───────────────────────────────────────────────────→ Nav2 costmap
GPS (fix) → navsat_transform → /odometry/gps ─────────────────────→ ekf_global
IMU (data) ───────────────────────────────────────────────────────→ ekf_local + ekf_global

robot_localization:
  ekf_local  (world=odom): /vo velocity + IMU yaw  → odom → base_footprint (smooth, may drift)
  ekf_global (world=map):  GPS position + IMU yaw   → map → odom (GPS-anchored, zero drift)

Nav2 → /cmd_vel → [SIM: diff_drive plugin] / [REAL: MAVROS] → robot moves
```
Rationale: VIO drifts, GPS does not. VIO lives in `ekf_local` only (smooth interpolation between fixes); the global frame stays GPS-anchored to avoid contaminating it with drift.

## Workspace Structure
```
~/internship_project/diff_drive_ws/
├── src/
│   ├── dcvg_robot/                          # SIMULATION package (Python/ament_python)
│   │   ├── package.xml / setup.py / setup.cfg
│   │   ├── App.bash                         # Quick-start (sources + launches web UI)
│   │   ├── resource/dcvg_robot
│   │   │
│   │   ├── urdf/
│   │   │   ├── dcvg_robot.urdf.xacro        # Robot description (xacro source; sensors + diff_drive plugin)
│   │   │   ├── dcvg_robot.urdf              # Generated URDF
│   │   │   └── dcvg_robot.sdf               # SDF for Gazebo
│   │   ├── meshes/DCVG_Robot.stl            # Custom 3D mesh
│   │   ├── models/dcvg_robot/               # Gazebo model (model.config + dcvg_robot.sdf)
│   │   ├── worlds/
│   │   │   ├── sonoma_raceway.world
│   │   │   └── sonoma_raceway_with_elements.world   # adds obstacles for depth-detection testing
│   │   │
│   │   ├── launch/
│   │   │   ├── gps_waypoint_follower.launch.py  # Main sim entry (chains everything)
│   │   │   ├── gazebo_gps_world.launch.py       # Gazebo + robot spawn + robot_state_publisher (SIM-ONLY)
│   │   │   ├── dual_ekf_navsat.launch.py        # robot_localization EKFs (SHARED config)
│   │   │   ├── rtabmap.launch.py                # RTAB-Map VIO (rgbd/stereo) + obstacles_detection (SHARED config)
│   │   │   └── mapviz.launch.py                 # Mapviz visualization
│   │   │
│   │   ├── config/
│   │   │   ├── nav2_no_map_params.yaml          # Nav2 (no map, rolling costmap; LiDAR + depth obstacle sources)
│   │   │   ├── dual_ekf_navsat_params.yaml      # EKF + navsat (odom0=/vo local, GPS global)
│   │   │   ├── waypoints.yaml / waypoints_ref.yaml / stops.yaml
│   │   │   ├── vio_test_routes.yaml             # Test routes for VIO evaluation
│   │   │   ├── osm_route_config.yaml / osm_route_comparison.html
│   │   │   └── mapviz.mvc / gps_wpf_demo.mvc
│   │   │
│   │   ├── behavior_trees/                      # Custom Nav2 recovery BTs (BackUp before Spin)
│   │   │   ├── navigate_to_pose_w_replanning_and_recovery.xml
│   │   │   └── navigate_through_poses_w_replanning_and_recovery.xml
│   │   │
│   │   ├── rviz/dcvg_env.rviz
│   │   │
│   │   └── dcvg_robot/                          # Python nodes
│   │       ├── __init__.py
│   │       ├── osm_route_app.py                 # Flask web app for route control
│   │       ├── osm_route_generator.py           # OSM route planning
│   │       ├── route_waypoint_follower.py       # Follow OSM routes
│   │       ├── route_recorder.py                # Record routes
│   │       ├── recorded_route_runner.py         # Replay recorded routes
│   │       ├── odom_accuracy_plot.py            # VIO vs ground-truth accuracy plotting
│   │       ├── feature_overlay.py               # RGBD keypoint overlay (debug)
│   │       └── stereo_feature_overlay.py        # Stereo keypoint + epipolar overlay (debug)
│   │
│   └── dcvg_bringup/                            # REAL-HARDWARE bringup (Python/ament_python)
│       ├── package.xml / setup.py / setup.cfg
│       ├── launch/
│       │   ├── bringup.launch.py                # Top-level real-HW entry (chains all below + Nav2)
│       │   ├── sensors.launch.py                # RealSense + RPLIDAR drivers (topics/ports TBD)
│       │   ├── mavros.launch.py                 # Pixhawk bridge: GPS in, cmd_vel out (URL TBD)
│       │   ├── localization.launch.py           # dual EKF + navsat (use_sim_time=False)
│       │   └── rtabmap.launch.py                # VIO + obstacles (use_sim_time=False; camera topics as args)
│       ├── config/                              # own copies of nav2 + EKF params (sim-time flipped to False)
│       └── behavior_trees/                      # own copies of the recovery BTs
├── build/ install/ log/
```

## How to Run (simulation)

### Simulation + Navigation (one command)
```bash
source ~/.bashrc
source ~/internship_project/diff_drive_ws/install/setup.bash
ros2 launch dcvg_robot gps_waypoint_follower.launch.py
```

### Control via Web App (separate terminal)
```bash
./App.bash
# Or manually:
source ~/internship_project/diff_drive_ws/install/setup.bash
ros2 run dcvg_robot osm_route_app          # http://127.0.0.1:5000
```

### With visualization / stereo VIO
```bash
ros2 launch dcvg_robot gps_waypoint_follower.launch.py use_rviz:=True
ros2 launch dcvg_robot gps_waypoint_follower.launch.py use_mapviz:=True
ros2 launch dcvg_robot rtabmap.launch.py use_stereo:=true   # D457 IR stereo instead of D455 RGBD
```

## Launch Flow (sim)
`gps_waypoint_follower.launch.py` is the single sim entry point. It chains:
1. `gazebo_gps_world.launch.py` → Gazebo Classic + robot spawn + robot_state_publisher  *(SIM-ONLY; replaced on real HW)*
2. `dual_ekf_navsat.launch.py` → ekf_local + ekf_global + navsat_transform  *(SHARED)*
3. `rtabmap.launch.py` → RTAB-Map VIO (`/vo`) + obstacles_detection (`/camera/depth/obstacles`)  *(SHARED config)*
4. `nav2_bringup/navigation_launch.py` → full Nav2 stack, pointed at the custom behavior trees  *(SHARED)*
5. Optionally: RViz2, Mapviz

## Key Technical Decisions
- **`dcvg_robot` is sim-only; real HW bringup is a separate package** — keep the boundary clean.
- **Gazebo Classic (gazebo11)** — Classic plugins (`libgazebo_ros_*`), NOT Gazebo Sim/Harmonic.
- **nav2_bringup** — standard `navigation_launch.py` (not individual Nav2 nodes).
- **RTAB-Map for VIO** — odometry-only mode (NOT full SLAM); replaces missing wheel encoders.
- **RTAB-Map for obstacle detection** — depth-based with ground-plane removal (height-segmentation in sim, since Gazebo depth has no normals).
- **RGBD vs stereo VIO** — `rtabmap.launch.py use_stereo:=` switches D455 RGBD (≤6 m) vs D457 IR stereo (passive, sunlight-immune, ~15 m). Pick the real-HW default deliberately.
- **ArduPilot Rover, NOT PX4** — Pixhawk 2.4.8 is FMUv3; PX4 dropped support.
- **MAVROS for real hardware** — ROS2 ↔ Pixhawk bridge; replaces the Gazebo diff_drive plugin.
- **No static map** — outdoor GPS navigation with rolling costmaps only.
- **RPLIDAR S2 for 2D LiDAR** — dToF, 30 m, IP65, sunlight-resistant.
- **YOLOv8-seg traversability** — future phase: classify grass vs real obstacles (ICRA 2024, arxiv 2407.18535).

## Hardware (real deployment)
- **Intel RealSense D455/D457** — depth + IMU for VIO. D455: 95 mm baseline, depth 0.6–6 m, BMI270 IMU. D457: IR stereo pair, passive (sunlight-immune), longer range.
- **RPLIDAR S2** — 2D LiDAR, 360°, 30 m, IP65, sunlight-resistant.
- **Jetson Orin Nano** — companion computer (ROS2 Humble native, no Docker).
- **simpleRTK2B Heading** — RTK GPS module handle all data about GPS
- **Realsense D457 Depth Camera** — Stereo Depth Camera for visual odometry
- **Pololu G2 Motor Drivers ×2** — driven by Pixhawk PWM.
- **Not using:** Arduino, TFmini Plus (D455 replaces it).

## Software Stack
- **OS:** Ubuntu 22.04 — **ROS2:** Humble
- **Simulation:** Gazebo Classic (gazebo11) — NOT Gazebo Sim/Harmonic
- **Navigation:** Nav2 (via nav2_bringup), no-map rolling costmap
- **Localization:** robot_localization (dual EKF + navsat_transform)
- **VIO + depth obstacles:** RTAB-Map (`sudo apt install ros-humble-rtabmap-ros`)
- **2D LiDAR driver (real HW):** rplidar_ros (`sudo apt install ros-humble-rplidar-ros`)
- **Camera driver (real HW):** realsense2_camera (`sudo apt install ros-humble-realsense2-camera`)
- **Pixhawk bridge (real HW):** MAVROS
- **Visualization:** RViz2, Mapviz — **Control:** Flask web app + OSM route integration

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
- Outdoor LiDAR sees grass/weeds as obstacles — depth + ground filtering solves this.
- GPS provides ~1–3 m accuracy — Nav2 goal tolerance set to ~1 m to match.
- Depth range maxes ~6 m outdoors (RGBD) — RPLIDAR S2 (30 m) covers long range.
- `dcvg_robot` uses Gazebo Classic — do NOT use Gazebo Sim/Harmonic plugins.
- On real HW: swap the Gazebo diff_drive plugin for a MAVROS `cmd_vel` bridge.
- `dcvg_robot` is ament_python (setup.py), NOT ament_cmake.
- Real-hardware sensor topic names are still TBD — see the "Sensor Topics — TO CONFIRM" table.

## Safety Requirements (real hardware phase)
- RC failsafe configured before first outdoor autonomous test
- Geofence configured in ArduPilot
- Always have RC kill-switch accessible during outdoor testing
- Test motor arm/disarm before any autonomous movement
