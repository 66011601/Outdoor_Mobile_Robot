# ตาราง tune ค่า

รวม variable ที่ใช้ tune ระบบบน branch `jetson` ค่าในตารางคือค่าปัจจุบันใน code ส่วนคำอธิบายผลเมื่อเพิ่มหรือลดเป็นแนวโน้มทั่วไป ต้องยืนยันบนหุ่นจริงหรือบน bag ทุกครั้ง

## สารบัญ

1. [วิธี tune อย่างปลอดภัย](#1-วิธี-tune-อย่างปลอดภัย)
2. [กลุ่มค่าที่ต้องตั้งให้ตรงกัน](#2-กลุ่มค่าที่ต้องตั้งให้ตรงกัน)
3. [Launch arguments](#3-launch-arguments)
4. [ความเร็วและ acceleration](#4-ความเร็วและ-acceleration)
5. [Controller (Regulated Pure Pursuit)](#5-controller-regulated-pure-pursuit)
6. [การถึง goal และการติดตามความคืบหน้า](#6-การถึง-goal-และการติดตามความคืบหน้า)
7. [Costmap](#7-costmap)
8. [Planner, smoother และ behavior tree](#8-planner-smoother-และ-behavior-tree)
9. [Localization](#9-localization)
10. [Visual odometry และ obstacle](#10-visual-odometry-และ-obstacle)
11. [Node ฝั่ง hardware](#11-node-ฝั่ง-hardware)
12. [การวิ่งตามเส้นทางและ web app](#12-การวิ่งตามเส้นทางและ-web-app)
13. [Firmware ของ Teensy](#13-firmware-ของ-teensy)
14. [ค่าที่วัดจากตัวหุ่นยนต์](#14-ค่าที่วัดจากตัวหุ่นยนต์)

---

## 1. วิธี tune อย่างปลอดภัย

1. เปลี่ยนทีละค่า และจดค่าเดิมไว้
2. check ตาราง[กลุ่มค่าที่ต้องตรงกัน](#2-กลุ่มค่าที่ต้องตั้งให้ตรงกัน)
3. `colcon build` เพราะ config file ถูกคัดลอกไป `install/` ตอน build
4. ถ้าเป็น localization หรือ obstacle ทดสอบกับ bag ด้วย `replay.launch.py` ก่อน และเทียบกับ topic `/rec/...` (ดู [COMMANDS ข้อ 6](COMMANDS.md#6-บันทึกและเล่น-bag-ซ้ำ))
5. ถ้าเป็นการขับเคลื่อน ทดสอบบนหุ่นจริงในพื้นที่โล่งโดยมีคนถือ kill switch

ตำแหน่ง file ที่ใช้ย่อในเอกสารนี้

| ชื่อย่อ | File |
|---|---|
| `nav2` | `src/dcvg_bringup/config/nav2_params.yaml` |
| `loc` | `src/dcvg_bringup/config/localization.yaml` |
| `rtab` | `src/dcvg_bringup/launch/rtabmap.launch.py` |
| `bringup` | `src/dcvg_bringup/launch/bringup.launch.py` |

---

## 2. กลุ่มค่าที่ต้องตั้งให้ตรงกัน

แก้ค่าหนึ่งในกลุ่มแล้วต้องแก้ค่าที่เหลือด้วย ส่วนใหญ่ไม่มี error เตือน แต่หุ่นจะทำงานผิดปกติ

| เงื่อนไข | ค่าที่เกี่ยวข้อง | ค่าปัจจุบัน | ถ้าไม่ตรง |
|---|---|---|---|
| Angular velocity สูงสุดเท่ากัน | `velocity_smoother.max_velocity[2]` และ `min_velocity[2]`, `FollowPath.rotate_to_heading_angular_vel`, `behavior_server.max_rotational_vel` | 0.3 | Smoother ตัดค่าเงียบ ๆ และเมื่อใช้ `scale_velocities` จะลด linear velocity ไปด้วย |
| Acceleration เท่ากัน | `velocity_smoother.max_accel` กับ `acceleration_limits` ของ EKF | Linear 0.4, angular 0.8 | EKF predict ไม่ตรงกับการเคลื่อนที่ |
| Angular acceleration เท่ากัน | `velocity_smoother.max_accel[2]` กับ `FollowPath.max_angular_accel` | 0.8 | การหมุนไม่สม่ำเสมอ |
| Deceleration เท่ากัน | `velocity_smoother.max_decel` กับ `deceleration_limits` ของ EKF | Linear 3.0, angular 1.2 | EKF predict การหยุดไม่ตรง |
| Timeout เท่ากัน | `control_timeout` ของ EKF กับ `cmd_timeout` ของ `low_level_command` | 0.5 s | EKF เชื่อว่ายังเคลื่อนที่ขณะ motor หยุดแล้ว |
| `yaw_goal_tolerance` < `rotate_to_heading_min_angle` | ใน `nav2` | 0.35 < 0.43 | หุ่นอาจหมุนแก้ทิศไม่รู้จบตอนถึง goal |
| `lookahead_dist` > `xy_goal_tolerance` | ใน `nav2` | 0.9 > 0.30 | การเข้า goal ไม่เสถียร |
| `inflation_radius` ≥ circumscribed radius footprint | ทั้งสอง costmap | 1.0 และ 0.9 ≥ 0.80 | Gradient ของ cost ไม่ครอบมุมหุ่นตอนเลี้ยว |
| `cost_scaling_dist` ≤ `inflation_radius` ของ local | ใน `nav2` | 0.6 ≤ 1.0 | การลดความเร็วตาม cost ใช้งานได้ไม่เต็มช่วง |
| `inflation_cost_scaling_factor` = `cost_scaling_factor` ของ local | ใน `nav2` | 4.0 | RPP คำนวณระยะจาก cost ผิด |
| `footprint` เท่ากันทั้งสอง costmap | ใน `nav2` | ±0.65 × ±0.46 | การตรวจการชนสองชั้นไม่ตรงกัน |
| ความสูงของ STVL ครอบคลุมช่วงที่ต้องการ | `origin_z` + `z_voxels` × `z_resolution` = `max_obstacle_height` | -1.0 + 60 × 0.05 = 2.0 | จุดนอกช่วงถูกทิ้ง |
| ระยะ depth สอดคล้องกัน | `depth_cloud.max_depth`, `Grid/RangeMax`, STVL `max_z`, `obstacle_range` | 3.0 m | Obstacle ถูกตัดก่อนถึงชั้นถัดไป |
| Filter ของ RTAB-Map ปรับพร้อมกัน | `Grid/CellSize`, `Grid/MinClusterSize`, `Grid/NoiseFilteringRadius`, `Grid/NoiseFilteringMinNeighbors` | 0.05, 20, 0.1, 8 | Filter ลบ obstacle จริงเกือบหมด |
| ระยะล้อตรง URDF | `wheel_separation` กับ `left_y - right_y` | 0.8009617 | Model ไม่ตรงกับคำสั่งล้อ |
| รัศมีล้อตรง URDF | `wheel_radius` กับรัศมีใน macro `wheel` | 0.2 | Model ไม่ตรงกับคำสั่งล้อ |
| Default ซ้ำหลาย file | `max_ground_angle`, `max_ground_height`, `grass_height`, `step_height`, `use_terrain_filter` ใน `bringup`, `rtab`, `replay` | ตามข้อ 3 | Run เดี่ยวกับ run ผ่าน bringup ได้ผลไม่เท่ากัน |

---

## 3. Launch arguments

### `bringup.launch.py`

| Argument | Default | ความหมาย |
|---|---|---|
| `use_stereo` | `True` | `True` ใช้ VO จากกล้อง IR คู่, `False` ใช้ RGBD |
| `use_rviz` | `False` | เปิด RViz |
| `use_telemetry` | `True` | เปิด `teensy_feedback_server` |
| `enable_ocr` | `True` | อ่านจอ meter ด้วย OCR |
| `heading_offset_deg` | `0.0` | ชดเชยทิศจาก GPS |
| `use_imu_tilt` | `False` | ให้ `base_footprint` ขนาน gravity |
| `max_ground_angle` | `50.0` | Slope ที่เกินแล้วเป็น obstacle (องศา) |
| `max_ground_height` | `0.0` | ความสูงที่เกินแล้วเป็น obstacle 0 = ปิด |
| `use_terrain_filter` | `True` | เปิด `terrain_filter` |
| `grass_height` | `0.12` | ความสูงที่ตัดทิ้งเป็นหญ้า (เมตร) |
| `step_height` | `0.20` | ความสูงที่เปลี่ยนพื้นเป็น obstacle (เมตร) |
| `teensy_port` | `/dev/ttyTeensy` | Serial ของ Teensy |
| `gps_device` | `/dev/ttyGPS_movingbase` | Serial position board |
| `gps_heading_device` | `/dev/ttyGPS_rover` | Serial heading board |

### `replay.launch.py`

มี argument ของ obstacle และ VO ชุดเดียวกับข้างบน และเพิ่มดังนี้

| Argument | Default | ความหมาย |
|---|---|---|
| `bag` | จำเป็นต้องระบุ | Path ของ bag |
| `run_vo` | `False` | คำนวณ VO ใหม่ |
| `run_localization` | `True` | Run EKF, `navsat_transform`, `gps_heading_to_imu` ใหม่ |
| `run_obstacles` | `True` | Run การตรวจจับ obstacle และ costmap ใหม่ |
| `rate` | `1.0` | ความเร็วการเล่น |
| `start_delay` | `10.0` | รอให้ Nav2 พร้อมก่อนเล่น (วินาที) |
| `domain_id` | `77` | `ROS_DOMAIN_ID` เพื่อแยกจากหุ่นจริง |

### เวลาเปิด (`STARTUP_DELAYS` ใน `bringup`)

| ส่วน | วินาที |
|---|---|
| `sensors` | 2 |
| `motors`, `localization` | 12 |
| `rtabmap` | 16 |
| `gps`, `ntrip` | 20 |
| `nav2` | 21 |
| `rviz` | 22 |
| `telemetry` | 24 |

ถ้าเพิ่มเวลาเหล่านี้จนเกิน 30 วินาที ควรปรับ `BRINGUP_GRACE_S` ใน web app ด้วย

---

## 4. ความเร็วและ acceleration

ก่อนปรับกลุ่มนี้ให้อ่านเรื่อง[หน่วยคำสั่งล้อ](GUIDE.md#12-ปัญหาที่ทราบแล้ว) เพราะความเร็วจริงอาจไม่เท่ากับค่าที่ตั้ง

| Parameter | File | ค่า | ความหมาย | เพิ่ม | ลด |
|---|---|---|---|---|---|
| `desired_linear_vel` | `nav2` FollowPath | 0.15 | ความเร็วเดินทาง (m/s) | ถึงเร็วขึ้น หยุดเลยจุดมากขึ้น | แม่นยำขึ้น แต่บนหญ้าอาจไม่มีแรงพอ |
| `max_velocity` | `nav2` velocity_smoother | `[0.2, 0, 0.3]` | limit คำสั่ง [x, y, หมุน] | เปิดให้เร็วขึ้น | จำกัดความเร็ว |
| `min_velocity` | `nav2` velocity_smoother | `[-0.2, 0, -0.3]` | limit ด้านลบ | - | - |
| `max_accel` | `nav2` velocity_smoother | `[0.4, 0, 0.8]` | Acceleration | ออกตัวไวขึ้น แต่สะบัดและ VO หลุดง่ายขึ้น | นุ่มขึ้น แต่ตอบสนองช้าลง |
| `max_decel` | `nav2` velocity_smoother | `[-3.0, 0, -1.2]` | Deceleration | หยุดไวขึ้น | ใช้ระยะ brake มากขึ้น |
| `velocity_timeout` | `nav2` velocity_smoother | 0.25 | เวลาเงียบก่อนเริ่มสั่งลดความเร็ว (s) | หยุดช้าลง | หยุดเร็วขึ้น |
| `smoothing_frequency` | `nav2` velocity_smoother | 20.0 | Frequency ของ smoother (Hz) | - | - |
| `scale_velocities` | `nav2` velocity_smoother | True | ลดทุกแกนพร้อมกันเพื่อรักษาเส้นโค้ง | - | - |
| `rotate_to_heading_angular_vel` | `nav2` FollowPath | 0.3 | ความเร็วหมุนอยู่กับที่ (rad/s) ต้องตั้งเสมอ เพราะ default ของ RPP คือ 1.8 | หมุนเร็ว แต่ overshoot จาก delay ของ GPS มากขึ้น | ล้อลื่นหรือหยุดบนหญ้า |
| `max_angular_accel` | `nav2` FollowPath | 0.8 | Angular acceleration ของ RPP | - | - |
| `min_approach_linear_velocity` | `nav2` FollowPath | 0.10 | ความเร็วต่ำสุดตอนเข้า goal | หยุดเลยมากขึ้น | อาจหยุดก่อนถึงเมื่อแรงไม่พอ |
| `approach_velocity_scaling_dist` | `nav2` FollowPath | 0.4 | ระยะเริ่มชะลอก่อนถึง goal (m) | ชะลอไกลขึ้น | Brake กระชั้นขึ้น |
| `regulated_linear_scaling_min_radius` | `nav2` FollowPath | 1.6 | รัศมีโค้งที่เริ่มชะลอ (m) | ชะลอในโค้งกว้างด้วย | เลี้ยวเร็วขึ้น |
| `regulated_linear_scaling_min_speed` | `nav2` FollowPath | 0.10 | ความเร็วต่ำสุดเมื่อถูกชะลอ | - | เสี่ยงหยุดนิ่ง |
| `max_rotational_vel` | `nav2` behavior_server | 0.3 | ความเร็วหมุนสูงสุดของ Spin | - | - |
| `min_rotational_vel` | `nav2` behavior_server | 0.15 | ความเร็วหมุนต่ำสุดของ Spin | - | - |
| `rotational_acc_lim` | `nav2` behavior_server | 0.2 | Acceleration การหมุนของ behavior | - | - |

**เหตุผลของขีดจำกัด angular 0.3 rad/s:** การหมุนอยู่กับที่ทำให้ล้อทั้งสองไถล บนหญ้าที่ต่ำกว่า 0.3 rad/s มาก (ราว 0.12 m/s ต่อล้อ ใกล้ความเร็วเดินทาง 0.15 m/s) หุ่นอาจหยุดนิ่ง ขีดจำกัดนี้จึงมาจาก friction ไม่ใช่ความเร็ว

---

## 5. Controller (Regulated Pure Pursuit)

ทั้งหมดอยู่ใต้ `controller_server.FollowPath` ใน `nav2`

| Parameter | ค่า | ความหมาย | เพิ่ม | ลด |
|---|---|---|---|---|
| `lookahead_dist` | 0.9 | ระยะมองล่วงหน้าเมื่อไม่ได้ใช้การปรับตามความเร็ว | เรียบขึ้น ตัดโค้ง | ตามเส้นแม่นขึ้น แต่ส่าย |
| `min_lookahead_dist` | 0.9 | ระยะต่ำสุด **และเป็นค่าที่ใช้จริงในปัจจุบัน** | เหมือนข้างบน | เหมือนข้างบน |
| `max_lookahead_dist` | 1.2 | ระยะสูงสุด | - | - |
| `lookahead_time` | 1.5 | เวลาที่คูณความเร็วเพื่อหาระยะ | จะมีผลเมื่อ `v × time` > 0.9 | - |
| `use_velocity_scaled_lookahead_dist` | true | ปรับระยะตามความเร็ว | - | - |
| `transform_tolerance` | 0.2 | ยอมให้ TF เก่าได้ (s) | ทนต่อ delay ขึ้น | เข้มงวดขึ้น |
| `use_collision_detection` | true | ตรวจการชนตามเส้นทางที่ predict | - | - |
| `max_allowed_time_to_collision_up_to_carrot` | 2.0 | เวลาล่วงหน้าที่ตรวจการชน (s) | หยุดเร็วขึ้น อาจหยุดพร่ำเพรื่อ | เข้าใกล้ obstacle มากขึ้น |
| `use_regulated_linear_velocity_scaling` | true | ชะลอในโค้ง | - | - |
| `use_cost_regulated_linear_velocity_scaling` | true | ชะลอเมื่อเข้าใกล้ obstacle | - | - |
| `inflation_cost_scaling_factor` | 4.0 | ใช้แปลง cost เป็นระยะ | - | - |
| `cost_scaling_dist` | 0.6 | ระยะที่เริ่มชะลอตาม cost (m) | ชะลอไกลขึ้น | เข้าใกล้ได้เร็วขึ้น |
| `cost_scaling_gain` | 1.0 | ความแรงของการชะลอตาม cost | - | - |
| `use_rotate_to_heading` | true | หมุนอยู่กับที่เมื่อทิศต่างมาก | - | - |
| `rotate_to_heading_min_angle` | 0.43 | มุมที่เริ่มหมุนอยู่กับที่ (rad) | เลี้ยวขณะเดินหน้ามากขึ้น | หมุนอยู่กับที่บ่อยขึ้น |
| `max_robot_pose_search_dist` | 5.0 | ระยะค้นหาตำแหน่งบนเส้นทาง (m) | - | - |
| `use_interpolation` | true | ประมาณจุดระหว่างจุดบนเส้นทาง | - | - |

`controller_frequency` คือ 20 Hz, `min_x_velocity_threshold` คือ 0.001, `min_theta_velocity_threshold` คือ 0.001, และ `failure_tolerance` คือ 0.3 วินาที

---

## 6. การถึง goal และการติดตามความคืบหน้า

| Parameter | File | ค่า | ความหมาย | เพิ่ม | ลด |
|---|---|---|---|---|---|
| `xy_goal_tolerance` | `nav2` | 0.30 | ระยะที่นับว่าถึง goal (m) | ง่ายขึ้น แม่นน้อยลง | แม่นขึ้น แต่อาจวนหาเป้า |
| `yaw_goal_tolerance` | `nav2` | 0.35 | มุมที่นับว่าถึง goal (rad) | ไม่ต้องจัดทิศมาก | เสียเวลาหมุน |
| `stateful` | `nav2` | True | เมื่อผ่านเงื่อนไขระยะแล้วไม่กลับไป check อีก | - | - |
| `required_movement_radius` | `nav2` | 0.5 | ต้องขยับเท่านี้ใน `movement_time_allowance` (m) | ล้มเหลวเร็วขึ้น | ทนต่อการหยุดรอ |
| `movement_time_allowance` | `nav2` | 45.0 | เวลาที่ยอมให้ไม่คืบหน้า (s) | รอนานขึ้น | ล้มเหลวเร็วขึ้น |

---

## 7. Costmap

### Footprint

| Parameter | ค่า | หมายเหตุ |
|---|---|---|
| `footprint` | `[[0.65, 0.46], [0.65, -0.46], [-0.65, -0.46], [-0.65, 0.46]]` | อ้างอิง `base_link` วัดถึงขอบนอกล้อ inscribed radius 0.46 m, circumscribed radius 0.80 m |

### ทั่วไป

| Parameter | Local | Global | ความหมาย |
|---|---|---|---|
| `update_frequency` | 20.0 | 3.0 | Frequency คำนวณ (Hz) |
| `publish_frequency` | 10.0 | 1.0 | Frequency publish (Hz) |
| `global_frame` | `odom` | `map` | Frame ของ costmap |
| `rolling_window` | true | True | เลื่อนตามหุ่น |
| `width` × `height` | 10 × 10 | 40 × 40 | ขนาด (m) ค่ามากกินทรัพยากรมาก |
| `resolution` | 0.05 | 0.05 | ขนาดช่อง (m) |
| `cost_scaling_factor` | 4.0 | 4.0 | ค่ามาก cost ลดเร็ว และหุ่นชิด obstacle ได้มากขึ้น |
| `inflation_radius` | 1.0 | 0.9 | รัศมีขยาย cost (m) |

### STVL (local costmap)

| Parameter | ค่า | ความหมาย | เพิ่ม | ลด |
|---|---|---|---|---|
| `voxel_decay` | 4.0 | อายุ voxel นอกมุมกล้อง (s) | จำ obstacle นานขึ้น แต่ ghost นานขึ้น | clear เร็วขึ้น แต่ลืม blind spot เร็ว |
| `decay_model` | 0 | รูปแบบการลดอายุ (linear) | - | - |
| `voxel_size`/`z_resolution` | 0.05 | ขนาด voxel (m) | - | - |
| `mark_threshold` | 0 | จำนวน voxel ใน column ก่อนนับว่ามี obstacle | ลด false positive | ไวขึ้น |
| `origin_z`/`z_voxels` | -1.0 / 60 | ช่วงความสูงใน column | - | - |
| `min_obstacle_height`/`max_obstacle_height` | -1.0 / 2.0 | ช่วงความสูงที่เก็บ (m) | - | - |
| `horizontal_fov_angle` | 1.45 | มุมแนวนอนของการ clear (rad) | - | - |
| `vertical_fov_angle` | 0.95 | มุมแนวตั้งของการ clear (rad) | - | - |
| `min_z`/`max_z` | 0.5 / 3.0 | ช่วง depth ของ frustum (m) | - | - |
| `decay_acceleration` | 5.0 | เร่งการลบ voxel ที่อยู่ในมุมกล้องแต่ไม่พบแล้ว | ghost หายเร็วขึ้น | ghost นานขึ้น 0 = `clearing` ไม่มีผล |
| `obstacle_range` | 3.0 | ระยะที่บันทึก obstacle (m) | - | - |
| `raytrace_range` | 4.0 | ระยะ clear (m) | - | - |
| `minimal_group_size` (denoise) | 4 | จำนวนช่องต่ำสุดของกลุ่ม | ลด noise แต่ตัดของบาง | เหลือจุด noise |
| `group_connectivity_type` (denoise) | 8 | การนับเพื่อนบ้าน | - | - |

### Obstacle layer (global costmap)

| Parameter | ค่า | ความหมาย |
|---|---|---|
| `obstacle_max_range` | 5.5 | ระยะไกลสุดที่บันทึก (m) |
| `obstacle_min_range` | 0.6 | ระยะใกล้สุดที่บันทึก (m) |
| `raytrace_max_range` | 6.0 | ระยะ clear ไกลสุด (m) |
| `raytrace_min_range` | 0.0 | ระยะ clear ใกล้สุด (m) |
| `min_obstacle_height`/`max_obstacle_height` | -1.0 / 2.0 | ช่วงความสูง (m) |
| `track_unknown_space` | false | ไม่แยกพื้นที่ที่ยังไม่รู้จัก |

---

## 8. Planner, smoother และ behavior tree

| Parameter | File | ค่า | ความหมาย |
|---|---|---|---|
| `expected_planner_frequency` | `nav2` | 2.0 | Frequency ที่คาดไว้ของ planner (Hz) |
| `GridBased.tolerance` | `nav2` | 0.15 | ระยะที่ยอมให้แผนจบห่าง goal (m) |
| `GridBased.use_astar` | `nav2` | false | false = Dijkstra |
| `GridBased.allow_unknown` | `nav2` | true | plan ผ่านพื้นที่ที่ไม่รู้จัก จำเป็นเพราะไม่มีแผนที่ |
| `simple_smoother.max_its` | `nav2` | 1000 | จำนวนรอบสูงสุด |
| `simple_smoother.tolerance` | `nav2` | 1.0e-10 | Threshold หยุด |
| `simple_smoother.do_refinement` | `nav2` | True | ทำซ้ำอีกรอบเพื่อให้เรียบขึ้น |
| `bt_loop_duration` | `nav2` | 10 | รอบของ BT (ms) |
| `default_server_timeout` | `nav2` | 20 | Timeout ของ server (ms) |

### ใน `behavior_trees/navigate_through_poses.xml`

ค่าเหล่านี้เขียนใน file XML โดยตรง

| ค่า | ค่าปัจจุบัน | ความหมาย |
|---|---|---|
| `RateController hz` | 1.5 | Frequency replan (`navigate_to_pose.xml` ใช้ 2.0) |
| `RemovePassedGoals radius` | 0.8 | ระยะที่นับว่าผ่าน waypoint กลางทาง (m) ควรมากกว่า error ของ GPS |
| `NavigateRecovery number_of_retries` | 2 | จำนวนการลองใหม่ก่อนล้มเหลว |
| `Wait wait_duration` | 5 | เวลารอในรอบ recovery (s) |

---

## 9. Localization

### ลำดับค่าใน `*_config`

```
[ x,  y,  z,  roll, pitch, yaw,
 vx, vy, vz, vroll, vpitch, vyaw,
 ax, ay, az ]
```

### ค่าทั่วไปของ EKF (`loc`)

| Parameter | ค่า | ความหมาย |
|---|---|---|
| `frequency` | 30.0 | Frequency filter (Hz) |
| `two_d_mode` | true | ต้องคงไว้ เพราะ costmap filter ความสูงในแกน z (ดู [GUIDE ข้อ 7](GUIDE.md#7-localization-และ-tf)) |
| `smooth_lagged_data` | true | Process ข้อมูลที่มาช้าโดยย้อนสถานะ |
| `history_length` | 1.0 | ระยะเวลาย้อนสถานะ (s) ค่ามา filter รับ delay ได้มาก แต่กินทรัพยากร |
| `print_diagnostics` | true | publish `/diagnostics` |

### ค่าเฉพาะตัว

| Parameter | `ekf_filter_node_odom` | `ekf_filter_node_map` |
|---|---|---|
| `world_frame` | `odom` | `map` |
| `odom0` | `vo`: vx, vy, vz, vyaw | `odometry/gps`: x, y |
| `imu0` | `imu/data_throttled`: vyaw | เหมือนกัน |
| `imu1` | `gps/heading/imu`: yaw | เหมือนกัน |
| `use_control` | true | false |
| `process_noise_covariance` x, y | 0.2 | 0.01 |
| `process_noise_covariance` yaw | 0.01 | 0.01 |
| `process_noise_covariance` vx, vy | 0.5 | 0.01 |

**การปรับ process noise:** เพิ่มค่าเมื่ออยากให้เชื่อ sensor มากขึ้นและตามทันการเปลี่ยนแปลง แต่ค่าจะสั่นตาม noise ถ้าลดค่า ผลจะเรียบขึ้นแต่ตอบสนองช้าลง

### Control input (`ekf_filter_node_odom`)

| Parameter | ค่า | ความหมาย |
|---|---|---|
| `stamped_control` | false | `/cmd_vel` เป็น `Twist` ธรรมดา |
| `control_timeout` | 0.5 | เวลาหลัง `/cmd_vel` เงียบ จึง predict ว่าอยู่นิ่ง (s) |
| `control_config` | `[true, false, false, false, false, true]` | ใช้ vx กับ vyaw |
| `acceleration_limits` | `[0.4, 0, 0, 0, 0, 0.8]` | ต้องตรงกับ `max_accel` |
| `deceleration_limits` | `[3.0, 0, 0, 0, 0, 1.2]` | ต้องตรงกับ `max_decel` |

### `navsat_transform` (`loc`)

| Parameter | ค่า | ความหมาย |
|---|---|---|
| `base_link_frame_id` | `base_footprint` | ใช้คำนวณ lever arm ของ `gps_link` |
| `delay` | 3.0 | รอก่อนคำนวณ transform (s) |
| `magnetic_declination_radians` | 0.0 | ไม่ใช้ เพราะทิศมาจาก GPS |
| `yaw_offset` | 0.0 | ไม่ใช้ ให้ปรับที่ `heading_offset_deg` แทน |
| `zero_altitude` | true | ตัดความสูง |
| `use_odometry_yaw` | true | ใช้ทิศจาก `/odometry/global` |
| `wait_for_datum` | false | ใช้ตำแหน่งแรกเป็นจุดอ้างอิง |
| `publish_filtered_gps` | true | publish `/gps/filtered` |
| `broadcast_cartesian_transform` | true | publish transform ของระบบพิกัด |

---

## 10. Visual odometry และ obstacle

### Visual odometry (`rtab`)

ค่ากลางอยู่ใน `ODOM_COMMON`

| Parameter | ค่า | ความหมาย |
|---|---|---|
| `frame_id` | `base_link` | Frame ที่ยึดแน่นกับกล้อง |
| `publish_tf` | False | EKF เป็นเจ้าของ TF |
| `wait_imu_to_init` | True | รอ IMU ก่อนเริ่ม ถ้า `/imu/data` ไม่มา VO จะไม่ทำงาน |
| `Odom/Strategy` | 1 | Frame-to-frame |
| `Vis/EstimationType` | 1 | 3D-to-2D (PnP) |
| `Vis/RoiRatios` | `0.0 0.0 0.0 0.10` | ตัดขอบล่างของภาพ 10% |
| `GFTT/QualityLevel` | 0.001 | ค่าน้อยได้ feature มากขึ้น รวมถึงบนผิวเรียบ |
| `GFTT/MinDistance` | 10 | ระยะห่างระหว่าง feature (pixel) |
| `Vis/CorFlowMaxLevel` | 5 | ชั้น pyramid ของ optical flow มีผลเมื่อ `Vis/CorType` เป็น 1 ซึ่ง file นี้ไม่ได้ตั้งไว้ ควรตรวจก่อน tune |
| `OdomF2M/BundleAdjustment` | 0 | ใช้กับ `Odom/Strategy` 0 (F2M) จึงไม่น่ามีผลขณะใช้ F2F |
| `OdomF2M/MaxSize` | 1000 | ใช้กับ `Odom/Strategy` 0 (F2M) จึงไม่น่ามีผลขณะใช้ F2F |

| Parameter | Stereo (ใช้งานปกติ) | RGBD |
|---|---|---|
| `Vis/MaxDepth` | 15.0 | 6.0 |
| `Vis/MinDepth` | 0.5 | 0.6 |
| `Odom/ResetCountdown` | 10 | 15 |
| `approx_sync_max_interval` | 0.05 | ไม่ได้ตั้ง |

`Odom/ResetCountdown` คือจำนวน frame ที่ล้มเหลวก่อน reset ค่าน้อยจะกลับมาติดตามเร็วแต่ reset บ่อย

### Depth cloud (`rtab`)

| Parameter | ค่า | เพิ่ม | ลด |
|---|---|---|---|
| `decimation` | 4 | CPU ลดลง แต่ของเล็กหาย | ละเอียดขึ้น แต่ delay สูงขึ้น |
| `max_depth` | 3.0 | เห็นไกลขึ้น แต่ noise และภาระสูงขึ้น | เหลือเวลาหลบน้อยลง |

### `obstacles_detection` (`rtab`)

| Parameter | ค่า | ความหมาย | เพิ่ม | ลด |
|---|---|---|---|---|
| `Grid/MaxGroundAngle` | 50 (`max_ground_angle`) | Slope ที่เกินแล้วเป็น obstacle (องศา) | ผิวเอียงเป็นพื้นมากขึ้น | เนินกลายเป็น obstacle |
| `Grid/MaxGroundHeight` | 0 (`max_ground_height`) | 0 = ปิด ควรคงไว้บนทางขรุขระ | พื้นด้านหน้าอาจกลายเป็นกำแพง | - |
| `Grid/MaxObstacleHeight` | 2.0 | ความสูงสูงสุดที่เก็บ (m) | - | - |
| `Grid/NormalK` | 40 | จำนวนจุดคำนวณ normal | เรียบขึ้น แต่ช้าลง | ไวต่อ noise |
| `Grid/ClusterRadius` | 0.1 | ระยะรวมกลุ่ม (m) | - | - |
| `Grid/MinClusterSize` | 20 | ขนาดกลุ่มต่ำสุด | ตัด noise แต่ตัดของบาง | เหลือ noise |
| `Grid/NoiseFilteringRadius` | 0.1 | รัศมี filter (m) | - | - |
| `Grid/NoiseFilteringMinNeighbors` | 8 | จำนวนเพื่อนบ้านต่ำสุด | ตัด noise มากขึ้น | เหลือ noise |
| `Grid/CellSize` | 0.05 | ขนาด voxel (m) ต้องตรงกับ filter | เร็วขึ้น แต่หยาบขึ้น | ภาระสูงขึ้น |
| `Grid/RangeMax` | 3.0 | ระยะไกลสุด (m) | - | - |
| `Grid/FlatObstacleDetected` | false | ไม่มีผลเมื่อ `Grid/MaxGroundHeight` เป็น 0 | - | - |
| `wait_for_transform` | 0.2 | เวลารอ TF (s) | - | - |

> ถ้าหญ้าถูกนับเป็น obstacle ให้เพิ่ม `grass_height` ก่อน ไม่ควรเพิ่ม costmap floor หรือเข้มงวด `Grid/MinClusterSize` เพราะจะตัด obstacle บางจริงไปด้วย

### `terrain_filter`

ค่า default อยู่ใน `PARAMETERS` ของ `terrain_filter.py` สามตัวแรกส่งผ่าน launch ได้

| Parameter | ค่า | ความหมาย | เพิ่ม | ลด |
|---|---|---|---|---|
| `enabled` | True | เปิดการ filter | - | - |
| `grass_height` | 0.12 | ต่ำกว่านี้เหนือพื้นถูกตัดทิ้ง (m) | ทนหญ้าสูง แต่มองไม่เห็นของเตี้ย | เห็นขอบทาง แต่หญ้าเป็น obstacle |
| `step_height` | 0.20 | จุดพื้นที่สูงกว่านี้เป็น obstacle (m) | ของเรียบเตี้ยถูกมองข้าม | พื้นขรุขระเป็น obstacle |
| `cell_size` | 0.2 | ขนาดช่องพื้น (m) | ทนรูโหว่ แต่หยาบขึ้น | ละเอียดขึ้น |
| `min_x`, `max_x` | -1.0, 4.0 | ช่วงตาราง (m) | - | - |
| `half_width` | 3.0 | ครึ่งความกว้าง (m) | - | - |
| `min_ground_points` | 3 | จุดขั้นต่ำต่อช่อง | ข้อมูลน่าเชื่อถือขึ้น | ได้ช่องมากขึ้น |
| `ground_percentile` | 0.5 | ตำแหน่งที่ใช้เป็นความสูงพื้น 0.5 = median | พื้นสูงขึ้น ตัดจุดมากขึ้น | พื้นต่ำลง |
| `max_slope_deg` | 35.0 | Slope สูงสุดที่ขับได้ | เนินชันผ่าน แต่ของมนกลืนกับพื้น | เนินกลายเป็น obstacle |
| `relax_iterations` | 5 | จำนวนรอบกดความสูง | - | - |
| `fill_cells` | 3 | รอบเติมช่องว่าง (3 × 0.2 = 0.6 m) | - | - |
| `max_ground_age` | 0.5 | อายุ cloud พื้นสูงสุดก่อนไม่ filter (s) | - | - |

---

## 11. Node ฝั่ง hardware

### `low_level_command`

| Parameter | ค่า | ความหมาย | ผลเมื่อเปลี่ยน |
|---|---|---|---|
| `port` | `/dev/ttyTeensy` | Serial | - |
| `baud` | 115200 | ต้องตรงกับ firmware | ไม่ตรงจะเห็นคำเตือน `baud mismatch?` |
| `wheel_separation` | 0.8009616629101749 | ระยะห่างล้อ (m) | มีผลกับคำสั่งเลี้ยว |
| `wheel_radius` | 0.2 | รัศมีล้อ (m) | ค่าน้อยคำสั่งล้อมากขึ้น |
| `max_wheel_speed` | 15.0 | limit คำสั่งล้อ | ค่าปัจจุบันสูงกว่าที่ใช้งานจริงมาก จึงแทบไม่มีผล |
| `cmd_timeout` | 0.5 | ไม่มี `/cmd_vel` แล้วหยุด (s) | ต้องตรงกับ `control_timeout` |
| `startup_hold` | 2.0 | เวลา lock หลังเริ่ม (s) | - |
| `probe_settle_delay` | 2.0 | รอให้นิ่งก่อนลง probe (s) | - |
| `probe_down_delay` | 2.5 | เวลาคงการลง รวมเวลาที่ actuator เคลื่อนที่ (s) | ควรมากกว่าเวลาลงจริงและเวลาที่ meter ต้องใช้ |
| `probe_raise_delay` | 3.0 | เวลารอให้ขึ้นก่อนยืนยัน (s) | ถ้าน้อยกว่าเวลาขึ้นจริง หุ่นจะออกตัวขณะ probe ยังไม่ขึ้นสุด |
| `auto_confirm` | True | ยืนยันจุดหยุดเอง | False = รอ operator |

ค่าคงที่ใน file: `READ_PERIOD_S` 0.05, `RECONNECT_PERIOD_S` 0.5, `MAX_RX_LINE_BYTES` 1024

### `gps_heading_to_imu`

| Parameter | ค่า | ความหมาย |
|---|---|---|
| `heading_offset_deg` | 0.0 | ชดเชยทิศ วิธี calibrate: หันหุ่นตามทิศที่ทราบแล้วตั้ง `true_bearing - measured_yaw` |
| `min_heading_std_deg` | 2.0 | ค่าต่ำสุดของ uncertainty ที่แจ้ง EKF ค่าน้อยทำให้ EKF เชื่อ GPS มากขึ้น |

### อื่น ๆ

| Node | Parameter | ค่า | ความหมาย |
|---|---|---|---|
| `imu_throttle` | `max_rate_hz` | 50.0 | อัตรา IMU ที่ส่งให้ EKF |
| `imu_tilt` | `base_frame`, `pitch_joint`, `roll_joint` | `base_link`, `base_pitch_joint`, `base_roll_joint` | ต้องตรงกับ URDF |
| `ntrip_bridge` | `device` | `/dev/ttyGPS_movingbase` | Serial ที่เขียน RTCM |
| `ntrip_bridge` | `gga_period` | 1.0 | period การส่ง GGA (s) |
| `ntrip_client` (`ntrip.launch.py`) | `host`, `port`, `mountpoint`, `username`, `password` | ดูใน file | ข้อมูล provider |
| `ntrip_client` | `reconnect_attempt_max`, `reconnect_attempt_wait_seconds`, `rtcm_timeout_seconds` | 10, 5, 15 | การเชื่อมต่อใหม่ |
| `teensy_feedback_server` | `tcp_port` | 5005 | Port telemetry |
| `teensy_feedback_server` | `enable_ocr` | True | เปิด OCR |
| `teensy_feedback_server` | `camera_device` | ตาม `CAMERA_DEVICE` | Meter camera |

### ค่าคงที่ใน `teensy_feedback_server.py`

| ชื่อ | ค่า | ความหมาย |
|---|---|---|
| `CAMERA_SIZE` | (320, 240) | ขนาดภาพ ค่าตำแหน่งใน `dcvg_ocr.py` อิงจากขนาดนี้ |
| `OCR_PERIOD_S` | 0.1 | period การอ่าน OCR (s) |
| `EXPECTED_FIELDS` | 17 | จำนวน field ต่อ frame |
| `QUEUE_SIZE` | 200 | จำนวน frame ใน buffer |
| `CLIENT_TIMEOUT_S` | 5.0 | Timeout การส่ง (s) |
| `DCVG_STALE_S`, `GPS_STALE_S` | 5.0, 3.0 | อายุข้อมูลก่อนเตือน (s) |

### `dcvg_ocr.py`

ค่า pixel ทั้งหมดอิงจากตำแหน่งกล้องปัจจุบัน

| ชื่อ | ค่า (y0, y1, x0, x1) | ความหมาย |
|---|---|---|
| `TOP_PASTE`/`BOT_PASTE` | (105,145,230,255) / (140,180,230,255) | ตำแหน่งวางรูปตัวเลข |
| `UNIT_BLANK_DST`/`UNIT_BLANK_SRC` | (105,145,250,300) / (140,180,250,300) | ปิดส่วนหน่วยบนจอ |
| `SIGN_TOP`/`SIGN_BOT` | (120,130,125,145) / (155,165,125,145) | พื้นที่อ่านเครื่องหมาย |
| `SIGN_THRESHOLD`, `SIGN_LEVEL` | 140, 250.0 | Threshold ตัดสินว่าเป็นค่าบวก |
| `DIGIT_X_MIN`, `DIGIT_X_MAX` | 150, 250 | ช่วงแกน x ของตัวเลข |
| `ROW_SPLIT_Y` | 120 | เส้นแบ่งแถวบนกับล่าง |

### GPS driver (`ublox_f9p*.yaml`)

| Parameter | ค่า | ความหมาย |
|---|---|---|
| `config_on_startup` | false | ไม่เขียนค่าลง board ค่าต้องตั้งใน board ไว้แล้ว |
| `dynamic_model` | portable | รูปแบบการเคลื่อนที่ |
| `dgnss_mode` | 3 | RTK แบบ fixed |
| `publish.nav.pvt` | true (position), false (heading) | publish `/navpvt` |
| `publish.nav.relposned` | true | publish `navrelposned` |
| `uart1.baudrate` | 460800 (heading) | Baud ของ heading board |

---

## 12. การวิ่งตามเส้นทางและ web app

### `route_waypoint_follower`

Web app เรียกโดยไม่ส่ง parameter ถ้าจะเปลี่ยนค่าเมื่อใช้ web ต้องแก้ default ใน code หรือ run ด้วยมือตาม [COMMANDS ข้อ 4](COMMANDS.md#4-วิ่งตามเส้นทางด้วยมือ)

| Parameter | ค่า | ความหมาย | เพิ่ม | ลด |
|---|---|---|---|---|
| `min_start_distance` | 1.0 | ข้าม waypoint ต้นทางที่ใกล้หุ่นเกินนี้ (m) | ข้ามจุดมากขึ้น รวมถึงจุดหยุด | หุ่นอาจหมุนกลับหาจุดใกล้ตัว |
| `min_wp_distance` | 0.3 | ตัดจุดที่ใกล้จุดก่อนหน้าเกินนี้ (m) | เส้นทางเบาขึ้น แต่จุดหยุดถี่อาจหาย | - |
| `simplify_angle_deg` | 8.0 | ยุบจุดเมื่อทิศเปลี่ยนน้อยกว่านี้ 0 = ปิด | จุดน้อยลง | ตามเส้นทางเดิมมากขึ้น |
| `simplify_max_run_m` | 12.0 | ความยาวสูงสุดของเส้นที่ยุบ (m) | ส่ง goal น้อยลง | ส่ง goal ถี่ขึ้น |
| `simplify_max_deviation_m` | 0.15 | ความเบี่ยงเบนสูงสุดจากเส้นเดิม (m) | ตัดโค้งมากขึ้น | รักษารูปเส้นทางดีขึ้น |
| `on_segment_failure` | `hold` | `hold` รอ operator, `skip` ข้ามช่วง | - | - |

### การสร้างเส้นทาง (ค่าใน web page)

| ค่า | Default ใน web | ความหมาย |
|---|---|---|
| `network_type` | `none` | `none` = เส้นตรงระหว่างจุด, นอกนั้นตาม network OSM (`walk`, `bike`, `drive`, `drive_service`, `all`, `all_private`) |
| `max_gap_m` | 25 | ระยะห่างสูงสุดระหว่าง waypoint (m) |
| `lateral_offset_m` | 0 | เลื่อนเส้นทางออกด้านข้าง (m) |
| `offset_side` | `right` | ด้านที่เลื่อนเทียบทิศเดินทาง |
| `buffer_m` | 500 | ขอบเขต download OSM รอบจุด (m) |
| `custom_filter` | ว่าง | Filter Overpass ถ้าใส่จะใช้แทน `network_type` |

### ค่าคงที่ใน `osm_route_app.py`

| ชื่อ | ค่า | ความหมาย |
|---|---|---|
| `PORT` | 5010 | เปลี่ยนด้วย environment `DCVG_PORT` |
| `MANUAL_SERVICE` | `dcvg.service` | Service Manual mode |
| `DEFAULT_CAMERA_TOPIC` | `/camera/color/image_raw` | เปลี่ยนด้วย parameter `camera_topic` |
| `BRINGUP_GRACE_S` | 30.0 | เวลารอก่อนสถานะ `running` (s) |
| `BRINGUP_STOP_TIMEOUT_S` | 20.0 | เวลารอปิด bringup ก่อนบังคับ (s) |
| `TELEOP_DEADMAN_S` | 0.5 | ไม่มีคำสั่ง teleop แล้วหยุด (s) |
| `STALL_LIN`, `STALL_ANG` | 0.03, 0.05 | ความเร็วที่ต่ำกว่านี้ถือว่าไม่ขยับ |
| `STALL_TIMEOUT_S` | 7.0 | ไม่ขยับแล้ว pause (s) |
| `STALL_GRACE_S` | 6.0 | เวลายกเว้นหลังกดเริ่ม (s) |
| `MOTOR_LINK_TIMEOUT_S` | 1.0 | ไม่มี `/teensy/feedback` แล้ว pause (s) |
| `WAYPOINT_SANITY_RADIUS_M` | 2000.0 | จุดเริ่มไกลกว่านี้จะถามยืนยัน (m) |
| `CHECK_FRESH_S` | 3.0 | อายุข้อมูลที่ถือว่าสดในรายการตรวจ (s) |
| `CHECK_NO_DATA_S` | 20.0 | รอข้อมูลก่อนแสดงคำเตือน (s) |

---

## 13. Firmware ของ Teensy

ค่าอยู่ใน `teensy/DCVG_Robot2_R1_Teensy.ino` หลังแก้ต้อง flash ใหม่ และควรทดสอบแบบยกล้อก่อนเสมอ

### PID และ feedforward

| ค่า | ล้อซ้าย | ล้อขวา | ความหมาย | เพิ่ม | ลด |
|---|---|---|---|---|---|
| `Kp` | 0.2 | 0.3 | ตอบสนองต่อ error | เข้าหา goal เร็วขึ้น แต่สั่น | ช้าลง |
| `Ki` | 0.0 | 0.0 | แก้ error สะสม (ปิดอยู่) | ลด error คงค้าง แต่เสี่ยง windup | - |
| `Kd` | 0.02 | 0.02 | ลดการแกว่ง | หน่วงขึ้น แต่ไวต่อ noise ของ Hall | แกว่งขึ้น |
| `Kff` | 0.60 | 0.60 | Feedforward ตาม setpoint | แรงส่งมากขึ้น | อาศัย PID มากขึ้น |
| `KsFF` | 0.085 | 0.077 | แรงเริ่มเพื่อชนะ friction | ออกตัวง่าย แต่กระตุก | หยุดนิ่งที่ความเร็วต่ำ |
| `integralLimit` | 0.75 | 0.75 | จำกัด integral | - | - |
| `reverse` | false | true | ทิศของ ESC | ห้ามเปลี่ยนถ้าไม่ได้เปลี่ยนการต่อ | - |

### Mapping สู่ PWM

| ค่า | ค่าปัจจุบัน | ความหมาย |
|---|---|---|
| `RATIO` | 0.004812 | ระยะต่อ pulse (m) ใช้คำนวณ feedback |
| `CENTER_PWM_L`/`CENTER_PWM_R` | 1600 / 1485 | จุดหยุดของ ESC (µs) ตรวจสอบเมื่อล้อขยับเองตอนคำสั่ง 0 |
| `DEADBAND_HALF` | 25 | ครึ่งของช่วงที่ ESC ไม่ตอบสนอง (µs) |
| `PWM_RANGE` | 400 | ช่วง PWM จาก `DEADBAND` ถึงเต็ม (µs) |
| `MAX_SPEED` | 0.85 | จำกัด output ของ PID ไม่ได้จำกัด setpoint |
| `SPEED_THRESH` | 0.01 | ต่ำกว่านี้ถือว่าหยุด |
| `ALPHA` | 0.4 | EMA ของ feedback ค่ามากตอบสนองเร็วแต่ noisy |
| `pidInterval` | 100 | period การควบคุม (ms) |

### Threshold อื่น

| ค่า | ค่าปัจจุบัน | ความหมาย |
|---|---|---|
| Threshold `Mode` | < 1300 µs = 1, > 1700 µs = 2 | Mode จาก RC |
| `isRCValid` | 100 ms | เวลาสูงสุดระหว่าง pulse RC |
| Hall debounce | 1000 µs | ช่วงเวลาตัด pulse ซ้ำ |
| `Serial.setTimeout` | 50 ms | เวลารอ frame |
| `delay` ท้าย `loop()` | 100 ms | กำหนดอัตรา feedback ราว 10 Hz |
| `MAX_FIELDS` | 8 | จำนวน field คำสั่งสูงสุด |

---

## 14. ค่าที่วัดจากตัวหุ่นยนต์

ห้ามเปลี่ยนค่าเหล่านี้โดยไม่วัดใหม่ อ้างอิงจาก `base_link` (กลางตัวหุ่น) โดยค่า z ใน xacro = ความสูงจากพื้น - 0.08 m (`base_height`)

| ค่า | ตำแหน่งใน `dcvg_robot.urdf.xacro` | จริง | ถ้าผิด |
|---|---|---|---|
| `camera_link` | `0.62 0 0.82` | หน้า 0.62 m, สูงจากพื้น 0.90 m | Obstacle อยู่ผิดตำแหน่ง |
| `gps_link` (position) | `-0.62 0 1.15` | **หลัง** 0.62 m, สูงจากพื้น 1.23 m | หยุดเลยทุกจุด และการหมุนกลายเป็นตำแหน่งกระโดด |
| `gps_heading_link` | `0.62 0 1.15` | หน้า 0.62 m, สูงจากพื้น 1.23 m, แนวกลางตัว | ทิศเพี้ยน |
| `base_height` | 0.08 | ระยะ `base_footprint` ถึง `base_link` | - |
| `heading_offset_deg` | 0.0 | antenna heading อยู่แนวกลางตัวด้านหน้า | หุ่นวิ่งเฉียงทิศ |
| `camera_link rpy` | `0 0 0` | ใช้ `tf2_echo base_footprint base_link` ตรวจเมื่อเปิด `use_imu_tilt` บนพื้นราบ ค่า roll/pitch ควรใกล้ 0° | ตัวหุ่นดูเอียงทั้งที่จริงไม่เอียง |
| `wheel_x`, `wheel_z` | -0.127, 0.103 | ตำแหน่งล้อขับ | ไม่มีผลกับการคำนวณ แต่กระทบภาพ model |
| `left_y`, `right_y` | 0.4, -0.40096 | ระยะห่างล้อ | ต้องตรงกับ `wheel_separation` |
