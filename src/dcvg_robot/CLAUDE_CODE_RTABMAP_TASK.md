# Task: Integrate RTAB-Map + Simulated D455 into DCVG Rover

## Context — Read CLAUDE.md First
Read CLAUDE.md in the workspace root for full project context. This file describes the specific integration task.

## What Already Works
GPS waypoint navigation with 2D LiDAR obstacle avoidance is fully functional. The simulation uses **Gazebo Classic (gazebo11)** with Classic plugins (libgazebo_ros_*). The robot model uses URDF/xacro with an STL mesh. Nav2 is launched via `nav2_bringup/navigation_launch.py`. All launch files chain from a single entry point: `gps_waypoint_follower.launch.py`.

## What We're Adding
Intel RealSense D455 simulation + RTAB-Map for:
1. Visual-Inertial Odometry (VIO) — outputs `/vo` to replace wheel encoders
2. Obstacle detection with ground filtering — outputs obstacle data for Nav2 costmap
3. RTAB-Map handles BOTH (Option A — fewer nodes, simpler architecture)

## Steps

### Step 1: Add simulated D455 to the robot URDF
Edit `urdf/dcvg_robot.urdf.xacro` to add:
- A `camera_link` mounted on the front of the robot, slightly elevated, facing forward
- Inside it, a Gazebo Classic RGBD camera plugin (`libgazebo_ros_camera.so` or `libgazebo_ros_openni_kinect.so`)
- D455 specs to simulate: RGB 1280×800, Depth 1280×720, FOV ~87°, depth range 0.6-6m
- Published topics: `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/color/camera_info`, `/camera/depth/camera_info`, `/camera/depth/points`
- The D455 has a built-in IMU — can use the existing `/imu/data` topic or add a separate camera IMU
- Add the camera_link to the TF tree (camera_link → camera_optical_frame with the standard ROS optical rotation)

**CRITICAL: Use Gazebo Classic plugins, NOT Gazebo Sim plugins.** This is gazebo11.

### Step 2: Install and configure RTAB-Map
```bash
sudo apt install ros-humble-rtabmap-ros
```
- Use `rtabmap_odom` node for visual odometry (RGBD mode, NOT stereo — D455 provides aligned depth)
- Run in **odometry-only mode** — we do NOT need full SLAM or loop closure (GPS handles global positioning)
- Input topics: `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/color/camera_info`
- Output: `/vo` (nav_msgs/Odometry)
- Key params: `frame_id=base_footprint`, `subscribe_depth=true`, `approx_sync=true`

### Step 3: Configure RTAB-Map obstacle detection
- Use `rtabmap_util/point_cloud_xyzrgb` or RTAB-Map's built-in obstacle detection
- RTAB-Map separates ground plane from obstacles using depth data
- It can publish an occupancy grid or filtered point cloud for Nav2
- Key advantage: ground plane removal means flat grass won't appear as obstacles
- Alternative: use `rtabmap_util/obstacles_detection` node to get ground vs obstacle point clouds, then feed obstacle cloud into Nav2 costmap as a VoxelLayer or ObstacleLayer source

### Step 4: Update robot_localization EKF
Edit `config/dual_ekf_navsat_params.yaml`:
- Add VIO as a new odometry source in both ekf_local and ekf_global:
```yaml
odom1: /vo
odom1_config: [true, true, false,
               false, false, true,
               false, false, false,
               false, false, false,
               false, false, false]
odom1_differential: false
odom1_relative: false
```
- Keep existing odom0 (wheel odom from Gazebo — removed on real hardware later)
- Keep existing imu0

### Step 5: Update Nav2 costmap
Edit `config/nav2_no_map_params.yaml`:
- Add RTAB-Map's obstacle output as a second observation source alongside existing LiDAR /scan
- Both local_costmap and global_costmap should have two sources: LiDAR (360° long-range) + depth camera (ground-filtered, close-range)

### Step 6: Create or update launch file
Either create a new launch file or modify `gps_waypoint_follower.launch.py` to include RTAB-Map nodes. The RTAB-Map nodes need:
- `use_sim_time: true`
- Correct topic remappings for camera and odometry
- Launch after Gazebo (camera needs to be publishing) but before Nav2 (Nav2 needs the obstacle data)

### Step 7: Test
1. `colcon build --packages-select dcvg_robot && source install/setup.bash`
2. Launch simulation: `ros2 launch dcvg_robot gps_waypoint_follower.launch.py`
3. Verify: `ros2 topic echo /vo --once` (VIO publishing?)
4. Verify: camera topics in `ros2 topic list | grep camera`
5. Verify in RViz2: add Camera display for `/camera/color/image_raw`, add PointCloud2 for depth
6. Run navigation via `ros2 run dcvg_robot osm_route_app` and verify rover navigates

## File Locations
```
~/internship_project/diff_drive_ws/src/dcvg_robot/
├── urdf/dcvg_robot.urdf.xacro          ← ADD D455 camera here
├── config/dual_ekf_navsat_params.yaml  ← ADD VIO source here
├── config/nav2_no_map_params.yaml      ← ADD depth obstacle source here
├── launch/gps_waypoint_follower.launch.py  ← ADD RTAB-Map launch here (or create new)
└── launch/dual_ekf_navsat.launch.py    ← May need RTAB-Map nodes here instead
```

## What NOT to Do
- Do NOT use Gazebo Sim/Harmonic plugins — this is Gazebo Classic (gazebo11)
- Do NOT run RTAB-Map in full SLAM mode — odometry-only (no map building, no loop closure)
- Do NOT remove existing LiDAR obstacle source from Nav2 — depth camera supplements it, doesn't replace it
- Do NOT change the launch flow structure — keep gps_waypoint_follower.launch.py as the single entry point
- Do NOT switch from nav2_bringup to individual Nav2 nodes — the current setup works with nav2_bringup