# DCVG Autonomous Rover Project

## Project Overview
Converting a Chinese grass-cutting robot into a large autonomous outdoor rover for DCVG (Direct Current Voltage Gradient) pipeline inspection. The robot navigates outdoors using GPS waypoints with 2D LiDAR obstacle avoidance, following OSM (OpenStreetMap) routes. Built on ROS2 Humble with Gazebo Classic simulation.

## Current Status — What Works
- GPS waypoint navigation with Nav2 (fully functional in simulation)
- 2D LiDAR obstacle avoidance via Nav2 costmap
- Dual EKF + navsat_transform sensor fusion (robot_localization)
- OSM route planning with web-based control app (Flask on port 5000)
- GPS waypoint logging and interactive waypoint following
- Mapviz and RViz2 visualization
- Custom robot model with STL mesh
- Simulation runs on Gazebo Classic (gazebo11) with sonoma_raceway world

## Next Phase — What We're Adding
Integrating Intel RealSense D455 depth camera with RTAB-Map for:
1. **Visual-Inertial Odometry (VIO)** — replaces missing wheel encoders on real hardware
2. **Enhanced obstacle detection** — RTAB-Map's depth-based obstacle detection with ground plane removal solves the "LiDAR sees grass as walls" problem
3. **RTAB-Map handles both VIO and obstacles** (Option A — simpler, fewer nodes)

## Architecture

### Current (working)
```
2D LiDAR (/scan) ──────────────────→ Nav2 costmap (obstacle layer)
GPS (/gps/fix) → navsat_transform → /odometry/gps → ekf_global
Wheel odom (/odom) ─────────────────→ ekf_local + ekf_global
IMU (/imu/data) ────────────────────→ ekf_local + ekf_global

robot_localization:
  ekf_local  → odom → base_footprint (smooth local)
  ekf_global → map → odom (GPS-corrected)

Nav2 → cmd_vel → diff_drive_plugin → robot moves
```

### Target (after D455 + RTAB-Map integration)
```
D455 (RGB + Depth + IMU) → RTAB-Map (odometry mode)
  ├── /vo (visual odometry) → robot_localization EKF
  └── obstacle grid (ground filtered) → Nav2 costmap

2D LiDAR (/scan) → Nav2 costmap (long-range 360° backup)
GPS → navsat_transform → ekf_global
D455 IMU or Pixhawk IMU → ekf_local + ekf_global

Nav2 costmap sources: LiDAR + RTAB-Map depth obstacles
```

## Workspace Structure
```
~/internship_project/diff_drive_ws/
├── src/
│   └── dcvg_robot/                          # Main package (Python/ament_python)
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── App.bash                         # Quick-start script (sources + launches web UI)
│       ├── resource/dcvg_robot
│       │
│       ├── urdf/
│       │   ├── dcvg_robot.urdf.xacro        # Robot description (xacro source)
│       │   ├── dcvg_robot.urdf              # Generated URDF
│       │   └── dcvg_robot.sdf               # SDF for Gazebo
│       │
│       ├── meshes/
│       │   └── DCVG_Robot.stl               # Custom 3D mesh
│       │
│       ├── models/dcvg_robot/
│       │   ├── model.config
│       │   └── dcvg_robot.sdf               # Gazebo model SDF
│       │
│       ├── worlds/
│       │   └── sonoma_raceway.world         # Gazebo Classic world
│       │
│       ├── launch/
│       │   ├── gps_waypoint_follower.launch.py  # Main entry (starts everything)
│       │   ├── gazebo_gps_world.launch.py       # Gazebo + robot spawn
│       │   ├── dual_ekf_navsat.launch.py        # robot_localization EKFs
│       │   ├── bringup.launch.py                # Robot state publisher
│       │   └── mapviz.launch.py                 # Mapviz visualization
│       │
│       ├── config/
│       │   ├── nav2_no_map_params.yaml          # Nav2 params (no static map, rolling costmap)
│       │   ├── dual_ekf_navsat_params.yaml      # EKF + navsat config
│       │   ├── waypoints.yaml                   # Stored GPS waypoints
│       │   ├── waypoints_ref.yaml               # Reference waypoints
│       │   ├── stops.yaml                       # Stop points
│       │   ├── osm_route_config.yaml            # OSM route parameters
│       │   ├── osm_route_comparison.html        # Route comparison view
│       │   ├── mapviz.mvc                       # Mapviz config
│       │   └── gps_wpf_demo.mvc                 # Mapviz GPS demo config
│       │
│       ├── rviz/
│       │   └── dcvg_env.rviz                    # RViz2 config
│       │
│       └── dcvg_robot/                          # Python nodes
│           ├── __init__.py
│           ├── osm_route_app.py                 # Flask web app for route control
│           ├── osm_route_generator.py           # OSM route planning
│           ├── route_waypoint_follower.py        # Follow OSM routes
│           ├── logged_waypoint_follower.py       # Follow logged waypoints
│           ├── interactive_waypoint_follower.py  # Interactive waypoint mode
│           ├── gps_waypoint_logger.py           # Log GPS positions
│           ├── dcvg_controller.py               # DCVG-specific control
│           ├── forward_kinematics.py            # FK calculations
│           ├── inverse_kinematics.py            # IK calculations
│           └── odom.py                          # Odometry processing
├── build/
├── install/
└── log/
```

## How to Run

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
ros2 run dcvg_robot osm_route_app
# Opens http://127.0.0.1:5000
```

### With visualization
```bash
ros2 launch dcvg_robot gps_waypoint_follower.launch.py use_rviz:=True
ros2 launch dcvg_robot gps_waypoint_follower.launch.py use_mapviz:=True
```

## Launch Flow
`gps_waypoint_follower.launch.py` is the single entry point. It chains:
1. `gazebo_gps_world.launch.py` → Gazebo Classic + robot spawn + robot_state_publisher
2. `dual_ekf_navsat.launch.py` → ekf_local + ekf_global + navsat_transform
3. `nav2_bringup/navigation_launch.py` → full Nav2 stack (uses nav2_bringup, NOT individual nodes)
4. Optionally: RViz2, Mapviz

## Key Technical Decisions
- **Gazebo Classic (gazebo11)** — based on nav2_gps_waypoint_follower_demo tutorial. Uses Classic plugins (libgazebo_ros_*), NOT Gazebo Sim/Harmonic.
- **nav2_bringup** — uses the standard nav2_bringup navigation_launch.py (not individual Nav2 nodes)
- **ArduPilot Rover, NOT PX4** — Pixhawk 2.4.8 is FMUv3, PX4 dropped support
- **MAVROS for real hardware** — ROS2 ↔ Pixhawk bridge (not needed in simulation)
- **No static map** — outdoor GPS navigation with rolling costmaps only
- **RTAB-Map for VIO** — replaces wheel encoders (real robot has none). Run in odometry-only mode, NOT full SLAM
- **RTAB-Map for obstacle detection** — depth-based with ground plane removal, solves grass-as-obstacle problem
- **RPLIDAR S2 for 2D LiDAR** — dToF, 30m range, IP65, 80K lux sunlight, ~$200-350 (better AND cheaper than A3)
- **YOLOv8-seg traversability** — future phase: classify grass vs real obstacles in Nav2 costmap (based on ICRA 2024 paper arxiv 2407.18535)

## Hardware (for real deployment, not simulation focus now)
- **Intel RealSense D455** — depth camera + built-in IMU for VIO (~$300)
  - 95mm baseline (accurate depth to 6m), global shutter (no motion blur), BMI270 IMU
- **RPLIDAR S2** — 2D LiDAR, 360°, 30m, 32K samples/sec, IP65, sunlight-resistant
- **Pixhawk 2.4.8** — ArduRover firmware, motor control + RC failsafe + GPS/IMU
- **Jetson Orin Nano** — companion computer (runs ROS2 Humble natively, no Docker needed)
- **u-blox NEO-M8N GPS ×2** — connected to Pixhawk
- **FS-iA10B RC receiver** — safety kill-switch via Pixhawk
- **Pololu G2 Motor Drivers ×2** — driven by Pixhawk PWM
- **Not using:** Arduino (not needed), TFmini Plus (D455 replaces it)

## Software Stack
- **OS:** Ubuntu 22.04
- **ROS2:** Humble
- **Simulation:** Gazebo Classic (gazebo11) — NOT Gazebo Sim/Harmonic
- **Navigation:** Nav2 (via nav2_bringup) with no-map rolling costmap
- **Localization:** robot_localization (dual EKF + navsat_transform)
- **VIO (adding next):** RTAB-Map (`sudo apt install ros-humble-rtabmap-ros`)
- **Depth obstacles (adding next):** RTAB-Map obstacle detection with ground filtering
- **2D LiDAR driver (real hardware):** rplidar_ros (`sudo apt install ros-humble-rplidar-ros`)
- **Camera driver (real hardware):** realsense2_camera (`sudo apt install ros-humble-realsense2-camera`)
- **Visualization:** RViz2, Mapviz
- **Control:** Flask web app + OSM route integration

## Research References
- **Nav2 GPS waypoint demo:** https://github.com/ros-navigation/navigation2_tutorials/tree/humble/nav2_gps_waypoint_follower_demo (basis for current implementation)
- **RTAB-Map Stereo Outdoor Navigation:** https://wiki.ros.org/rtabmap_ros/Tutorials/StereoOutdoorNavigation
- **YOLOv8 Nav2 traversability (ICRA 2024):** arxiv 2407.18535 — "Improving the ROS 2 Navigation Stack with Real-Time Local Costmap Updates for Agricultural Applications"
- **RAMP-VO:** REJECTED — needs event camera hardware we don't have
- **TERP DRL Planning:** DEFERRED — ROS1 only, needs 3D LiDAR, requires porting
- **VLM-GroNav:** DEFERRED — no public code, needs VLM compute too heavy for edge devices
- **NVIDIA Isaac Perceptor:** BACKUP — available if we need GPU-accelerated perception on Orin

## Known Issues & Gotchas
- conda and ROS2 conflict — never mix them
- Real robot has no wheel encoders — VIO from D455+RTAB-Map is the replacement
- Outdoor LiDAR sees grass/weeds as obstacles — D455+RTAB-Map ground filtering solves this
- GPS provides ~1-3m accuracy — Nav2 goal tolerance set to 1m to match
- D455 depth range maxes out at ~6m outdoors — RPLIDAR S2 (30m) covers long range
- This workspace uses Gazebo Classic (gazebo11) — do NOT use Gazebo Sim/Harmonic plugins
- Gazebo Classic uses plugins like libgazebo_ros_diff_drive.so, libgazebo_ros_ray_sensor.so, etc.
- When deploying to Jetson Orin Nano: swap diff-drive for MAVROS cmd_vel bridge
- The project uses ament_python build type (setup.py), NOT ament_cmake (CMakeLists.txt)

## Safety Requirements (for real hardware phase)
- RC failsafe configured before first outdoor autonomous test
- Geofence configured in ArduPilot
- Always have RC kill-switch accessible during outdoor testing
- Test motor arm/disarm before any autonomous movement