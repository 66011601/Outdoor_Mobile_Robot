# DCVG Rover Simulation (รุ่นเก่า)

Branch `main` คือ simulation ของหุ่นยนต์ DCVG บน Gazebo Classic ที่ใช้พัฒนา navigation system ด้วย GPS ก่อนจะย้ายไปทำบนหุ่นจริง

> **นี่ไม่ใช่ code ที่ใช้งานบนหุ่นยนต์** code ปัจจุบันอยู่ที่ [branch `jetson`](https://github.com/66011601/Outdoor_Mobile_Robot/tree/jetson) พร้อมคู่มือฉบับเต็ม
>
> Branch นี้ไม่ได้ใช้งานมาราว 5 เดือน (commit ล่าสุด กรกฎาคม 2026) เก็บไว้เพื่ออ้างอิงและทดลองแนวคิด ค่าส่วนใหญ่เป็นรุ่นเก่า และยังไม่ได้ยืนยันว่ายัง run ได้ตามขั้นตอนด้านล่าง

## ความแตกต่างจากหุ่นจริง

| หัวข้อ | `main` (simulation) | `jetson` (หุ่นจริง) |
|---|---|---|
| แหล่งข้อมูล | Gazebo plugin | Sensor จริง |
| Obstacle | Depth camera (Gazebo มี LiDAR จำลองบน `/lidar/scan` แต่ค่า `scan` ใน costmap ถูก comment ไว้) | Depth camera อย่างเดียว ไม่มี LiDAR |
| Ground filter | `obstacles_detection` | `obstacles_detection` + `terrain_filter` |
| ทิศทาง | IMU จำลอง (`imu/data`) | GPS moving baseline + gyro จากกล้อง |
| Controller ของ Nav2 | DWB | Regulated Pure Pursuit |
| ความเร็วสูงสุด (`velocity_smoother`) | 1.5 m/s, 1.0 rad/s | 0.2 m/s, 0.3 rad/s |
| Behavior tree | มี BackUp และ Spin | ล้าง costmap และรอเท่านั้น |
| Web app port | 5000 | 5010 |
| Motor | `diff_drive` ของ Gazebo | Teensy ผ่าน `low_level_command` |
| Probe, OCR, telemetry | ไม่มี | มี |

ค่าที่ตั้งบน branch นี้จึงนำไปใช้กับหุ่นจริงโดยตรงไม่ได้

## Requirement

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11 และ `gazebo_ros_pkgs`
- `navigation2`, `nav2_bringup`, `robot_localization`, `spatio_temporal_voxel_layer`
- `rtabmap_odom`, `rtabmap_util`
- `mapviz`, `mapviz_plugins`, `tile_map`
- Python libraries: `flask`, `pyyaml`, `osmnx`, `networkx`

## การ run

Build จากรากของ repo

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

เปิด Gazebo พร้อม localization, RTAB-Map และ Nav2

```bash
ros2 launch dcvg_robot gps_waypoint_follower.launch.py use_rviz:=True
```

เปิด web app ในอีก terminal แล้วเข้า `http://localhost:5000`

```bash
ros2 run dcvg_robot osm_route_app
```

ใส่ `use_mapviz:=True` เพื่อเปิด Mapviz ส่วน `rtabmap.launch.py` ตั้ง `debug` เป็น true ไว้เป็น default

## โครงสร้าง

| ตำแหน่ง | บทบาท |
|---|---|
| `src/dcvg_robot/` | Package หลักของ simulation: world, model หุ่น, launch, config, web app, เครื่องมือบันทึกและวิเคราะห์เส้นทาง |
| `src/dcvg_robot/launch/` | `gps_waypoint_follower` (entry point), `gazebo_gps_world`, `dual_ekf_navsat`, `rtabmap`, `mapviz` |
| `src/dcvg_robot/config/` | `nav2_no_map_params.yaml`, `dual_ekf_navsat_params.yaml`, เส้นทาง และ `osm_route_config.yaml` |
| `src/dcvg_robot/worlds/` | `sonoma_raceway` และ `sonoma_raceway_with_elements` (default) |
| `src/dcvg_robot/urdf/` | Model หุ่นและ Gazebo sensor plugin |
| `src/dcvg_bringup/` | โครงเริ่มแรกของระบบบน hardware ซึ่งยังใช้ MAVROS **เลิกใช้แล้ว** รุ่นจริงอยู่บน `jetson` |
| `src/dcvg_robot_description/` | Package model หุ่นแยกต่างหาก |
| `diff_drive_ws/` | File เหลือจาก workspace เดิม |

## หมายเหตุ

- `CLAUDE.md` บน branch นี้ เป็น note สมัยวางแผนย้ายไปหุ่นจริง จึงพูดถึง RPLIDAR, MAVROS และ Pixhawk ซึ่งหุ่นปัจจุบันไม่ได้ใช้
- `src/README.md` เป็น file ที่ติดมาจาก `navigation2_tutorials`
- `src/dcvg_robot/App.bash` กำหนด path ของเครื่องผู้พัฒนาเดิมไว้ ต้องแก้ก่อนใช้
- entry point ของ model ใน simulation คือ x=10, y=-10, z=0.5
- ถ้าจะนำความคิดจากที่นี่ไปใช้บนหุ่นจริง ควรอ่าน [คู่มือบน branch `jetson`](https://github.com/66011601/Outdoor_Mobile_Robot/blob/jetson/docs/GUIDE.md) ก่อน
