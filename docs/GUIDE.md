# คู่มือสำหรับผู้พัฒนาต่อ

คู่มือนี้อธิบาย branch `jetson` ซึ่งเป็น code ที่ run บนหุ่นยนต์จริง เขียนจากการอ่าน code โดยคำสั่งและขั้นตอนยังไม่ได้ทดลอง run ซ้ำบนหุ่นยนต์ระหว่างเขียนเอกสาร หากเจอจุดไม่ตรงให้ถือว่า code ถูกต้องกว่า

เอกสารที่เกี่ยวข้อง: [PARAMETERS.md](PARAMETERS.md) สำหรับตาราง tune ค่า และ [COMMANDS.md](COMMANDS.md) สำหรับคำสั่ง run ด้วยมือ

## สารบัญ

1. [ควรเริ่มอ่านจากตรงไหน](#1-ควรเริ่มอ่านจากตรงไหน)
2. [Requirement](#2-requirement)
3. [การติดตั้งเครื่องใหม่](#3-การติดตั้งเครื่องใหม่)
4. [บทบาทของแต่ละ folder](#4-บทบาทของแต่ละ-folder)
5. [Node และลำดับการเปิด](#5-node-และลำดับการเปิด)
6. [การทำงานตั้งแต่กด Start จนจบเส้นทาง](#6-การทำงานตั้งแต่กด-start-จนจบเส้นทาง)
7. [Localization และ TF](#7-localization-และ-tf)
8. [การตรวจจับ obstacle](#8-การตรวจจับ-obstacle)
9. [Firmware ของ Teensy](#9-firmware-ของ-teensy)
10. [Telemetry และการอ่าน meter](#10-telemetry-และการอ่าน-meter)
11. [วิธีแก้ project อย่างปลอดภัย](#11-วิธีแก้-project-อย่างปลอดภัย)
12. [ปัญหาที่ทราบแล้ว](#12-ปัญหาที่ทราบแล้ว)
13. [การแก้ปัญหาตามอาการ](#13-การแก้ปัญหาตามอาการ)
14. [งานที่ยังค้างในการส่งมอบ](#14-งานที่ยังค้างในการส่งมอบ)

---

## 1. ควรเริ่มอ่านจากตรงไหน

แนะนำให้อ่าน code ตามลำดับนี้

| ลำดับ | File | สิ่งที่จะได้ |
|---|---|---|
| 1 | `src/dcvg_bringup/launch/bringup.launch.py` | เห็นว่าระบบเปิดอะไรบ้าง ตามลำดับใด |
| 2 | `src/dcvg_robot/dcvg_robot/route_waypoint_follower.py` | Logic การวิ่งตามเส้นทางและการหยุด |
| 3 | `src/dcvg_bringup/dcvg_bringup/low_level_command.py` | ทางออกจาก ROS ไปยัง motor และ probe |
| 4 | `src/dcvg_bringup/config/nav2_params.yaml` และ `localization.yaml` | ค่าที่มีผลต่อการวิ่งมากที่สุด |
| 5 | `src/dcvg_robot/dcvg_robot/osm_route_app.py` | Web app และการตรวจสถานะระบบ |
| 6 | Firmware `teensy/DCVG_Robot2_R1_Teensy.ino` | สิ่งที่เกิดจริงหลังจากคำสั่งออกจาก ROS |

**ข้อตกลงของ project**

- Code ตั้งใจไม่ใส่ comment คำอธิบายให้อยู่ในเอกสารชุดนี้
- Package ทั้งสองเป็น `ament_python`
- ใช้ `colcon build` จากรากของ workspace ห้ามใช้ `--merge-install` (ดู [ข้อ 12](#12-ปัญหาที่ทราบแล้ว))
- ห้ามใช้ conda ร่วมกับ ROS 2

---

## 2. Requirement

### ระบบหลัก

| รายการ | ค่าที่ใช้ |
|---|---|
| Computer | Jetson Orin Nano |
| OS | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| ผู้ใช้และ path ที่ service คาดไว้ | ผู้ใช้ `jetson`, workspace ที่ `/home/jetson/dcvg_robot` |
| Network | internet สำหรับ NTRIP และแผนที่บน web, Tailscale สำหรับส่ง telemetry ไป monitoring laptop |

### ROS packages

รายการนี้มาจาก `package.xml` และการเรียกใช้ใน launch file

| หมวด | Package |
|---|---|
| Navigation | `navigation2`, `nav2_bringup`, `nav2_common`, `nav2_simple_commander`, `spatio_temporal_voxel_layer` |
| Localization | `robot_localization`, `imu_filter_madgwick` |
| Vision | `realsense2_camera`, `rtabmap_odom`, `rtabmap_util`, `cv_bridge` |
| GPS | `ublox_gps`, `ublox_msgs`, `ntrip_client`, `nmea_msgs`, `rtcm_msgs` |
| Model และ TF | `robot_state_publisher`, `xacro`, `tf2_ros` |

### Python libraries

| Library | ใช้ใน | ระบุใน `package.xml` |
|---|---|---|
| `flask` | Web app | มี |
| `pyyaml` | Web app, follower | มี |
| `pyserial`, `numpy` | Teensy, `terrain_filter` | มี |
| `osmnx` (2.x), `networkx` | สร้างเส้นทาง | **ไม่มี ต้องติดตั้งเอง** |
| `opencv-python` หรือ system package | กล้องบน web, OCR | **ไม่มี ต้องมีในระบบ** |
| `easyocr` (ใช้ GPU ผ่าน PyTorch) | อ่านจอ meter | **ไม่มี ต้องติดตั้งเอง** |

ควรใช้ `osmnx` รุ่น 2 ขึ้นไป เพราะ code เรียก `graph_from_bbox(bbox, ...)` ตามรูปแบบของรุ่นนั้น

> `osmnx` จำเป็นแม้สร้างเส้นทางแบบเส้นตรง (`network_type: none`) เพราะ `osm_route_generator.py` import ไว้ที่ต้น file ถ้าไม่มี package นี้ ปุ่มสร้างเส้นทางจะตอบ error 500

### สิ่งที่อยู่นอก repo แต่ระบบต้องใช้

| รายการ | ตำแหน่ง | ใช้ทำอะไร |
|---|---|---|
| Manual mode program | `~/Desktop/DCVG_Robot` บน Jetson ผ่าน `dcvg.service` | Manual mode ใน web app |
| Udev rules | `/etc/udev/rules.d/99-usb-serial.rules` | สร้างชื่อ `/dev/ttyTeensy`, `/dev/ttyGPS_*` |
| Sudoers rule | `/etc/sudoers.d/` | สลับ mode และปิดเครื่องจาก web |
| Telemetry receiver program | Monitoring laptop | เชื่อมผ่าน Tailscale มาที่ TCP 5005 |

---

## 3. การติดตั้งเครื่องใหม่

1. Clone branch `jetson` ไปที่ `~/dcvg_robot`
2. ติดตั้ง dependency และ build ตาม [COMMANDS ข้อ 1](COMMANDS.md#1-ติดตั้งและ-build)
3. เพิ่มผู้ใช้ในกลุ่ม `dialout` เพื่อเปิด serial port ได้
4. สร้าง udev rules ให้ได้ชื่ออุปกรณ์ตามตารางใน [README](../README.md) โดยดูค่าจาก `udevadm info`

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="<vid>", ATTRS{idProduct}=="<pid>", ATTRS{serial}=="<serial>", SYMLINK+="ttyTeensy"
```

Udev rules จริงบนหุ่นยนต์ไม่ได้อยู่ใน repo ควรคัดลอกจากเครื่องเดิม ตัวอย่างข้างบนเป็นรูปแบบเท่านั้น

5. สร้าง sudoers rule ด้วย `sudo visudo -f /etc/sudoers.d/dcvg` โดย web app เรียกคำสั่งเหล่านี้แบบ `sudo -n`

```
jetson ALL=(root) NOPASSWD: /usr/bin/systemctl start dcvg.service, /usr/bin/systemctl stop dcvg.service, /usr/sbin/poweroff
```

6. ติดตั้ง web app service ตาม [COMMANDS ข้อ 2](COMMANDS.md#2-เปิดระบบผ่าน-web-app)
7. ตั้งค่า NTRIP ใน `src/dcvg_bringup/launch/ntrip.launch.py` และเข้าร่วม Tailscale
8. วัดและยืนยันตำแหน่ง sensor ตาม [PARAMETERS ข้อ 14](PARAMETERS.md#14-ค่าที่วัดจากตัวหุ่นยนต์)

> ⚠️ **NTRIP login ถูกฝังเป็นค่า default ใน `ntrip.launch.py` และ repo นี้เป็น public** ควรเปลี่ยน password กับ provider เพราะการลบออกจาก file อย่างเดียวไม่ได้ลบออกจาก git history เมื่อเปลี่ยน password แล้วต้องแก้ค่า default ใน file และ build ใหม่ เนื่องจาก web app เรียก bringup โดยไม่ส่ง argument ใด ๆ

**เรื่องที่ลืมบ่อย:** service ตั้ง `ROS_DOMAIN_ID=30` terminal ที่ไม่ได้ `export ROS_DOMAIN_ID=30` จะไม่เห็น topic ของหุ่นยนต์เลย

---

## 4. บทบาทของแต่ละ folder

```
repo root = รากของ colcon workspace (~/dcvg_robot)
├── README.md, CLAUDE.md
├── docs/                      เอกสารชุดนี้
├── cache/                     cache OpenStreetMap ที่ osmnx สร้าง (ไม่ใช่ source code)
├── teensy/                    Firmware ของ Teensy
└── src/
    ├── dcvg_bringup/          ระบบที่ run บน hardware จริง
    ├── dcvg_robot/            model หุ่นยนต์และเครื่องมือของ operator
    └── system/                systemd file
```

### `src/dcvg_bringup/`

| ตำแหน่ง | บทบาท |
|---|---|
| `launch/bringup.launch.py` | main entry point เปิดทุกอย่างแบบหน่วงเวลาตาม `STARTUP_DELAYS` |
| `launch/sensors.launch.py` | RealSense, `imu_filter_madgwick`, `imu_throttle`, และ `imu_tilt` เมื่อเปิด `use_imu_tilt` |
| `launch/ublox_gps.launch.py` | F9P board ทั้งสองและ `gps_heading_to_imu` |
| `launch/ntrip.launch.py` | `ntrip_client` และ `ntrip_bridge` |
| `launch/localization.launch.py` | EKF ทั้งสองตัวและ `navsat_transform` |
| `launch/rtabmap.launch.py` | Visual odometry, `depth_cloud`, `obstacles_detection`, `terrain_filter` |
| `launch/replay.launch.py` | Run บางส่วนของระบบซ้ำบน bag ที่บันทึกไว้ |
| `config/nav2_params.yaml` | ค่าทั้งหมดของ Nav2 |
| `config/localization.yaml` | ค่า EKF และ `navsat_transform` |
| `config/ublox_f9p.yaml`, `ublox_f9p_heading.yaml` | ค่า driver ของ GPS board |
| `config/bag_qos.yaml` | ให้เล่น `/tf_static` แบบ transient-local |
| `config/note` | Note งานค้าง: ลดขนาด costmap และปรับกล้องให้ได้ระดับ |
| `behavior_trees/` | Behavior tree ของ Nav2 ที่ recovery ไม่ขยับตัวหุ่น |
| `ocr/` | รูปตัวเลขสำหรับช่วย OCR |
| `dcvg_bringup/` | Python code ของ node (ดูข้อ 5) |

### `src/dcvg_robot/`

| ตำแหน่ง | บทบาท |
|---|---|
| `urdf/dcvg_robot.urdf.xacro` | ตำแหน่งของ frame ทั้งหมด |
| `config/waypoints.yaml` | เส้นทางปัจจุบัน เขียนโดย web app |
| `config/stops.yaml` | จุดหยุดปัจจุบัน เขียนโดย web app |
| `web/index.html` | Web page: แผนที่ Leaflet, ปุ่มควบคุม, teleop, ภาพกล้อง |
| `dcvg_robot/osm_route_app.py` | Flask server ของ web app (port 5010) |
| `dcvg_robot/osm_route_generator.py` | สร้างเส้นตรงหรือเส้นทางจาก OpenStreetMap |
| `dcvg_robot/route_waypoint_follower.py` | ส่งเส้นทางให้ Nav2 และรอที่จุดหยุด |
| `dcvg_robot/common.py` | File สื่อสารใน `/tmp` และ function ระยะทาง |

### รูปแบบ route file

```yaml
# waypoints.yaml
waypoints:
- latitude: 12.71889586
  longitude: 101.15256607
  yaw: -0.068326    # follower ไม่ใช้ค่านี้ แต่คำนวณทิศใหม่จากจุดถัดไป

# stops.yaml
stops:
- index: 1          # ลำดับของจุดใน waypoints.yaml เริ่มจาก 0
  latitude: 12.71889297
  longitude: 101.15260925
```

Web app บันทึก file ทั้งใน `src/dcvg_robot/config/` และใน `install/` จึงเห็นผลทันทีและไม่หายเมื่อ build ใหม่

---

## 5. Node และลำดับการเปิด

`bringup.launch.py` หน่วงเวลาเพราะถ้าเปิดพร้อมกัน serial driver กับ RTAB-Map จะแย่งทรัพยากรกัน

| วินาที | ส่วนที่เปิด |
|---|---|
| 0 | `robot_state_publisher` |
| 2 | sensors |
| 12 | `low_level_command`, localization |
| 16 | rtabmap |
| 20 | GPS, NTRIP |
| 21 | Nav2 |
| 22 | RViz (เมื่อเปิด `use_rviz`) |
| 24 | `teensy_feedback_server` |

### Node ที่เขียนเอง

| Node | รับ | ส่ง | หน้าที่ |
|---|---|---|---|
| `low_level_command` | `/cmd_vel`, `/dcvg/stop_reached`, serial | `/dcvg/stop_confirm`, `/teensy/feedback`, serial | แปลงความเร็วเป็นคำสั่งล้อ, watchdog, รอบการลง probe |
| `gps_heading_to_imu` | `/gps/heading/navrelposned` | `/gps/heading/imu` | แปลงทิศจาก GPS เป็น `Imu` ที่มีเฉพาะ yaw ให้ EKF |
| `imu_throttle` | `imu/data` | `imu/data_throttled` | ลดอัตรา IMU เหลือ 50 Hz |
| `imu_tilt` | `imu/data_throttled` | `joint_states` | แปลง roll/pitch เป็นตำแหน่ง joint (ปิดเป็น default) |
| `terrain_filter` | `obstacles_raw`, `ground` | `obstacles` | คัด obstacle จากความสูงเหนือพื้นบริเวณนั้น |
| `ntrip_bridge` | `/rtcm`, `/navpvt` | `/nmea`, serial | ส่ง RTCM เข้า GPS board และส่ง GGA กลับให้ caster |
| `teensy_feedback_server` | `/teensy/feedback`, `/gps/fix`, meter camera | `/dcvg/telemetry`, `/dcvg/reading`, TCP 5005 | รวมข้อมูล meter และ GPS ส่งออก |
| `osm_route_monitor` | สถานะหลาย topic | `/cmd_vel` (teleop) | ส่วน ROS ภายใน web app |
| `route_waypoint_follower` | `/odometry/global`, `/dcvg/stop_confirm` | `/dcvg/stop_reached`, action ของ Nav2 | วิ่งตามเส้นทาง |

`record_bag` เป็น script บันทึก bag ไม่ใช่ node และ `dcvg_ocr.py` เป็น class ที่ `teensy_feedback_server` เรียกใช้

### การตรวจสถานะใน web page

ตอน bringup web app แสดงรายการจาก `BRINGUP_CHECKS` ใน `osm_route_app.py` โดยแต่ละข้อดูจาก

- ข้อความ `process started` และ `process has died/finished` ใน log ของ launch ถ้า process หยุดจะขึ้น error
- ข้อมูลบน topic ภายใน 3 วินาทีล่าสุด ถ้า process run เกิน 20 วินาทีแล้วไม่มีข้อมูลจะขึ้น warning

| รายการ | Executable | พร้อมเมื่อ |
|---|---|---|
| Robot model | `robot_state_publisher` | Process เริ่ม |
| Camera | `realsense2_camera_node` | `/camera/color/camera_info` |
| Camera IMU | `imu_filter_madgwick_node`, `imu_throttle` | `/imu/data_throttled` |
| Motor controller | `low_level_command` | `/teensy/feedback` |
| Visual odometry | `stereo_odometry`/`rgbd_odometry`, `obstacles_detection`, `terrain_filter` | `/vo` |
| Localization | `ekf_node`, `navsat_transform_node` | `/odometry/local` |
| GPS position | `ublox_gps_node` | `/gps/fix` มี fix (status 2 ขึ้นไปแสดงเป็น RTK) |
| GPS heading | `gps_heading_to_imu` | `/gps/heading/imu` |
| NTRIP corrections | `ntrip_ros.py`, `ntrip_bridge` | `/rtcm` |
| Navigation | Server ของ Nav2, `lifecycle_manager` | log `Managed nodes are active` |
| Telemetry server | `teensy_feedback_server` | Process เริ่ม |

**เมื่อเพิ่ม node ใน launch ต้องเพิ่มใน `BRINGUP_CHECKS` ด้วย**

### Manual mode และ Auto mode ของ web app

Mode นี้เป็นคนละเรื่องกับ switch Mode บน RC (ดูข้อ 9)

| Mode ใน web | สิ่งที่เกิด |
|---|---|
| **Manual** | หยุด navigation และ bringup แล้ว `systemctl start dcvg.service` web page เหลือเฉพาะ Shutdown กับ Switch to Auto |
| **Auto** | `systemctl stop dcvg.service` แล้วแสดงหน้า bringup |

สอง mode ใช้พร้อมกันไม่ได้เพราะเปิด `/dev/ttyTeensy` และ TCP 5005 เหมือนกัน

---

## 6. การทำงานตั้งแต่กด Start จนจบเส้นทาง

### ขั้นตอนการวิ่ง

1. Web app เรียก `ros2 run dcvg_robot route_waypoint_follower`
2. Follower load `waypoints.yaml` และ `stops.yaml` จาก folder เดียวกัน
3. แปลงพิกัดเป็น frame `map` ผ่าน service `/fromLL`
4. ตัดจุดที่ห่างจากจุดก่อนหน้าน้อยกว่า `min_wp_distance` และจุดต้นทางที่อยู่ใกล้หุ่นยนต์เกิน `min_start_distance`
5. คำนวณทิศของแต่ละจุดจากจุดถัดไป
6. แบ่งเส้นทางเป็นช่วงตามจุดหยุด
7. ยุบจุดในแนวตรงตามค่า `simplify_*` แล้วส่งแต่ละช่วงเป็น `NavigateThroughPoses` หนึ่ง goal

### เมื่อถึงจุดหยุด

```
route_waypoint_follower ── /dcvg/stop_reached ──> low_level_command
                                                      รอให้นิ่ง  2.0 s (probe_settle_delay)
                                                      probe ลง   2.5 s (probe_down_delay)
                                                      probe ขึ้น  3.0 s (probe_raise_delay)
route_waypoint_follower <── /dcvg/stop_confirm ──────┘
```

ปุ่ม **Confirm** บน web ปลดการรอได้เช่นกัน ถ้า Teensy link หลุดระหว่างรอบ `low_level_command` จะไม่ยืนยันให้ และจุดหยุดจะรอ operator

> รอบ probe ใช้ timer ล้วน software ฝั่ง ROS ไม่ได้อ่านสถานะ probe กลับมายืนยัน ดู[ปัญหาที่ทราบแล้ว](#12-ปัญหาที่ทราบแล้ว)

### เมื่อวิ่งไม่สำเร็จ

| เหตุการณ์ | ผลลัพธ์ |
|---|---|
| Nav2 ทำช่วงนั้นไม่สำเร็จ | หุ่นหยุดรอ กด **Run** เพื่อลองใหม่ หรือ **Stop** เพื่อจบ หรือตั้ง `on_segment_failure:=skip` เพื่อข้ามช่วง |
| ไม่ขยับเกิน 7 วินาที | Web app สั่ง pause อัตโนมัติและแจ้งเตือน |
| ไม่มี `/teensy/feedback` เกิน 1 วินาที | Web app สั่ง pause อัตโนมัติและแจ้งให้ใช้ kill switch |
| ไม่มี `/cmd_vel` เกิน 0.5 วินาที | `low_level_command` ส่งคำสั่งหยุด |

### Behavior tree

File ใน `behavior_trees/` ตั้งใจให้ recovery ทำแค่ล้าง costmap และรอ โดยไม่หมุนหรือถอยหลัง

- ถูก load ผ่าน key ว่าง `default_nav_*_bt_xml` ใน `nav2_params.yaml` ซึ่ง launch เติม path ให้ เพราะ `RewrittenYaml` ของ Humble แทนค่าเฉพาะ key ที่มีอยู่แล้ว
- ถ้าไม่มีการเติมค่า Nav2 จะใช้ BT มาตรฐานที่มี Spin 90° และ BackUp โดยไม่แจ้งเตือน
- ขั้น plan แล้ว smooth ต้องเป็น `Sequence` ธรรมดา ถ้าใช้ `ReactiveSequence` planner จะถูกเรียกซ้ำระหว่าง `SmoothPath` ทำให้ `FollowPath` ไม่ได้เริ่มและหุ่นไม่ขยับ

### การสื่อสารระหว่าง web app กับ follower

ทั้งสองคุยกันผ่าน file ใน `/tmp` ไม่ใช่ topic

| File | ค่า | เขียนโดย |
|---|---|---|
| `dcvg_nav_cmd` | `run`, `pause`, `stop`, `stopped` | Web app, follower |
| `dcvg_stop_status` | ข้อความสถานะการรอ | follower |
| `dcvg_stop_confirm` | `confirm` | Web app |
| `dcvg_nav_target` | JSON ของ goal ปัจจุบัน | follower |

---

## 7. Localization และ TF

### Frame

```
map ─(ekf_filter_node_map)─> odom ─(ekf_filter_node_odom)─> base_footprint ─> base_link ─┬─> camera_link
                                                                                         ├─> gps_link
                                                                                         └─> gps_heading_link
```

### ข้อมูลที่ EKF แต่ละตัวใช้

| ข้อมูล | `ekf_filter_node_odom` | `ekf_filter_node_map` |
|---|---|---|
| `/vo` | vx, vy, vz, vyaw | ไม่ใช้ |
| `/odometry/gps` | ไม่ใช้ | x, y |
| `/imu/data_throttled` | vyaw | vyaw |
| `/gps/heading/imu` | yaw | yaw |
| `/cmd_vel` (`use_control`) | vx, vyaw | ไม่ใช้ |

### เหตุผลของการออกแบบ

- **ไม่ใช้ yaw สัมบูรณ์จาก IMU ของกล้อง** เพราะไม่มี magnetometer จุดศูนย์จึงเปลี่ยนทุกครั้งที่ boot
- **ใช้ `/cmd_vel` เป็น control input** ROS ไม่มี wheel odometry และ VO เป็นแหล่งความเร็วเดียว เมื่อ VO หลุดจากการกระแทก การ predict จะตามคำสั่งแทนการไหลต่อจากค่าล่าสุด เมื่อล้อลื่นหรือถูกขวาง VO จะดึงค่ากลับหลังติดตามได้อีกครั้ง
- **`control_timeout` เท่ากับ `cmd_timeout` ที่ 0.5 วินาที** เมื่อ `/cmd_vel` เงียบ motor หยุดและ EKF predict ว่าอยู่นิ่งพร้อมกัน
- **EKF เป็น 2D (`two_d_mode`)** costmap filter ความสูงในแกน z ของ `odom`/`map` ถ้า `odom` เป็น 3D จะสูงขึ้นตามเนิน ทำให้พื้นเข้าไปอยู่ในช่วง obstacle และ obstacle จริงหลุดออก
- **`navsat_transform` รับ odometry จาก `/odometry/global`** ทำให้ `/odometry/gps` อยู่ใน frame `map` ถ้าใช้ `odom` จะเกิด feedback loop ที่เคยทำให้ตำแหน่งของ map EKF เพี้ยนถึงระดับ 10⁷ เมตร
- **VO ใช้ `frame_id: base_link`** ซึ่งยึดแน่นกับกล้องตามที่ RTAB-Map คาดไว้ แล้วให้ `robot_localization` หมุน twist เข้า `base_footprint`

### พื้นเอียง

- **Default:** `base_footprint → base_link` คงที่ `base_footprint` จึงเอียงตามตัวหุ่น การแยกพื้นอิงผิวที่ล้อสัมผัส เหมาะกับเส้นทางขรุขระที่เนินสั้นและเปลี่ยนบ่อย
- **`use_imu_tilt:=True`:** URDF เปลี่ยนเป็น `base_pitch_joint`/`base_roll_joint` และ `imu_tilt` publish roll/pitch เพื่อให้ `base_footprint` ขนานกับ gravity เหมาะกับเนินยาวและเรียบ บนพื้นขรุขระทำให้ cloud แกว่งและ costmap flicker จึงปิดไว้ ถ้าเปิดต้องคง `max_ground_height:=0`

### GPS

- Position board อยู่ด้านหลัง และเป็นตำแหน่งที่ EKF ใช้ผ่าน `gps_link`
- Heading board ส่ง NAV-RELPOSNED เข้า `gps_heading_to_imu`
- VRS caster จะไม่ส่ง correction จนกว่าจะได้รับ GGA จาก `ntrip_bridge`
- ถ้า `gps_link` TF หาย `navsat_transform` จะ log ว่า `Will assume navsat device is mounted at robot's origin` และตำแหน่งจะกระโดดเมื่อหมุนตัว

---

## 8. การตรวจจับ obstacle

```
/camera/depth/image_rect_raw
  └─ depth_cloud (decimate 4 เท่า, ตัดที่ 3 m) ─ /camera/depth/points_decimated
       └─ obstacles_detection ─┬─ /camera/depth/obstacles_raw ─┐
                               └─ /camera/depth/ground ────────┴─ terrain_filter ─ /camera/depth/obstacles
                                                                                     ├─ local costmap (STVL)
                                                                                     └─ global costmap
```

### `obstacles_detection`

- แยกพื้นด้วย normals ใน frame `base_footprint`
- `Grid/MaxGroundAngle` คือ slope ที่เกินแล้วนับเป็น obstacle
- ควรคง `Grid/MaxGroundHeight` ที่ 0 บนเส้นทางขรุขระ ตัวหุ่นก้มลงหลุมทำให้พื้นด้านหน้าสูงเกินค่าคงที่ใด ๆ และกลายเป็นกำแพง
- RTAB-Map ไม่เตือนเมื่อสะกด key `Grid/*` ผิด เคยมี `Grid/Cellsize` ที่ทำให้ใช้ค่า default โดยไม่รู้ตัว ให้ตรวจการสะกดกับ `rtabmap/core/Parameters.h`
- `Grid/CellSize` กำหนดขนาด voxel ที่ cluster และ noise filter ใช้นับจุด จึงต้องปรับพร้อมกัน เช่น ที่ 0.1 m จุดหนึ่งมีเพื่อนบ้านไม่เกินราว 6 จุดภายใน `Grid/NoiseFilteringRadius` 0.1 m ทำให้ filter 8 เพื่อนบ้านลบ obstacle เกือบหมด

### `terrain_filter`

Node นี้วัดความสูงเทียบกับพื้นด้านหน้าหุ่น ไม่ใช่เทียบกับ `base_footprint` จึงทนต่อเนินและการก้มเงยได้ดีกว่า

1. แปลง cloud เข้า `base_footprint` เพราะ `obstacles_detection` ส่งออกใน frame ของกล้อง ซึ่งแกน z คือ depth
2. แบ่งพื้นเป็นช่องขนาด 0.2 m (x จาก -1 ถึง 4 m, y ±3 m) ช่องที่มีจุดอย่างน้อย 3 จุดใช้ median เป็นความสูงพื้น
3. ช่องที่สูงกว่าข้างเคียงเกิน slope `max_slope_deg` ถูกกดลง เพื่อให้กล่องหรือหินโดดเด่นขึ้น
4. ช่องที่ไม่มีข้อมูลใช้ค่าต่ำสุดของช่องข้างเคียงในระยะราว 0.6 m
5. จุด obstacle ที่สูงจากพื้นน้อยกว่า `grass_height` ถูกตัดทิ้ง
6. จุดพื้นที่สูงเกิน `step_height` ถูกเพิ่มเป็น obstacle
7. แปลงกลับเป็น frame เดิมแล้วส่งออก

ถ้าไม่มีข้อมูลพื้น (cloud พื้นยังไม่มา, ห่างจาก stamp เกิน 0.5 s, frame ไม่ตรง, หรือไม่มี TF) จุดจะผ่านไปโดยไม่ filter และ log จำนวนจุดทุก 10 วินาที

> `grass_height` คือ obstacle ที่เตี้ยที่สุดที่หุ่นมองเห็นด้วย หญ้าที่สูงกว่านี้จะนับเป็น obstacle เพราะ depth camera แยกหญ้าสูงจากพุ่มไม้ไม่ได้ บนเส้นทางคอนกรีตที่มีขอบทางควรลดเป็น 0.08 หรือปิดด้วย `use_terrain_filter:=false`

### Costmap

- `min_obstacle_height` เป็น -1.0 m (STVL `origin_z` -1.0, 60 voxels) เพื่อเก็บ obstacle ที่ต่ำกว่าระนาบหุ่น
- STVL clear ด้วย frustum ของกล้อง สิ่งที่ออกจากมุมกล้องหายในราว 0.5 วินาที voxel นอกมุมมอง (เช่น blind spot หน้า bumper) อยู่ครบ 4 วินาทีตาม `voxel_decay`
- ถ้าไม่ตั้ง `decay_acceleration` (default ของ STVL คือ 0) `clearing: true` จะไม่มีผล
- ปิด point cloud ของ RealSense (`pointcloud__neon_.enable`) เพราะขนาดระดับ MB ต่อ frame ทำให้ driver ช้า และการแบ่ง voxel 0.05 m ทิ้งความละเอียดนั้นอยู่ดี
- `obstacles_detection` ไม่ตั้ง `map_frame_id` เพราะการรอ TF `map → base_footprint` เพิ่ม delay และทำให้ cloud หลุด
- NavFn plan สำหรับจุดกึ่งกลางหุ่นเท่านั้น จึงอาจเลือกช่องที่พอดีความกว้างแต่เลี้ยวไม่ได้ ส่วน RPP ตรวจรูปสี่เหลี่ยมจริงแล้วหยุดแทนการชน

---

## 9. Firmware ของ Teensy

> Source code อยู่ที่ `teensy/DCVG_Robot2_R1_Teensy.ino` ควรยืนยันว่าตรงกับที่ flash อยู่บนหุ่นจริงก่อนอ้างอิง
>
> Arduino IDE ต้องการให้ชื่อ folder ตรงกับชื่อ file `.ino` ก่อน compile ให้คัดลอก file ไปไว้ใน folder ชื่อ `DCVG_Robot2_R1_Teensy/` หรือให้ IDE สร้าง folder นี้ให้

### Serial protocol (115200 baud)

**Jetson ไป Teensy**

```
$<left>,<right>,<probe_up>,<probe_down>#\n
```

| Field | ความหมาย |
|---|---|
| `left`, `right` | ค่าที่ `low_level_command` คำนวณเป็น rad/s และ firmware ใช้เป็น setpoint ของ PID (ดูเรื่องหน่วยใน[ข้อ 12](#12-ปัญหาที่ทราบแล้ว)) |
| `probe_up` | 1 = สั่งขึ้น |
| `probe_down` | 1 = สั่งลง |

**Teensy ไป Jetson** ส่งราว 10 ครั้งต่อวินาที มี 13 field

| ลำดับ | ชื่อ | ความหมาย |
|---|---|---|
| 1 | `batt_ADC` | ค่า ADC 12 bit ของ battery (ขา A1) ค่าดิบที่ยังไม่แปลงเป็น volt |
| 2 | `level_ADC` | ค่า ADC ดิบจากขา `Gas_pin` (A0) |
| 3 | `encoder_LEFT` | จำนวน pulse ล้อซ้ายในรอบควบคุมล่าสุด มีเครื่องหมาย และไม่ใช่ค่าสะสม |
| 4 | `encoder_RIGHT` | จำนวน pulse ล้อขวาในรอบควบคุมล่าสุด |
| 5 | `bumpF_status` | Bump sensor หน้า ปิดใช้งานใน code ค่าเป็น 0 เสมอ |
| 6 | `bumpR_status` | Bump sensor หลัง ปิดใช้งานใน code ค่าเป็น 0 เสมอ |
| 7 | `Status_Prob1` | สถานะ probe หน้า (ดูตารางด้านล่าง) |
| 8 | `Status_Prob2` | สถานะ probe หลัง |
| 9 | `Forward_Backward` | ค่า RC ช่องหน้า-หลัง แปลงเป็น 0-200 (100 = กลาง) |
| 10 | `MR_RL` | ค่า RC ช่องซ้าย-ขวา แปลงเป็น 0-200 (100 = กลาง) |
| 11 | `ProbeF_command` | คำสั่งที่ใช้กับ probe หน้า: 0 หยุด, 1 ขึ้น, 2 ลง |
| 12 | `ProbeB_command` | คำสั่งที่ใช้กับ probe หลัง: 0 หยุด, 1 ขึ้น, 2 ลง |
| 13 | `Mode` | Mode จาก RC: 0 ไม่มี mode, 1 Manual, 2 Auto |

**ค่า `Status_Prob`**

| ค่า | เงื่อนไข | ขา (หน้า/หลัง) |
|---|---|---|
| 1 | Sensor บนทำงาน (probe อยู่บนสุด) | `prox1_T`/`prox2_T` |
| 3 | Sensor ล่างทำงาน (probe อยู่ล่างสุด) | `prox1_B`/`prox2_B` |
| 2 | Sensor `Prox` ทำงาน ซึ่งสั่งหยุดการลงด้วย | `Prox1`/`Prox2` |
| 0 | ไม่มี sensor ใดทำงาน | - |

### Mode จาก RC

> **หุ่นจะไม่ขยับตามคำสั่งจาก ROS ถ้า switch Mode บน RC ไม่ได้อยู่ที่ Auto** ให้ตรวจ field ที่ 13 ของ `/teensy/feedback` ก่อนเมื่อหุ่นไม่ขยับ

| ความกว้าง pulse ช่อง Mode | `Mode` | ล้อ | Probe | Relay A/M |
|---|---|---|---|---|
| ต่ำกว่า 1300 µs | 1 Manual | Teensy ไม่สั่ง PID | ตาม RC (ช่อง `ProbeF_R`) | LOW |
| สูงกว่า 1700 µs | 2 Auto | PID ตามคำสั่ง serial | ตามคำสั่ง serial | HIGH |
| ระหว่างนั้น | 0 | **ไม่ update คำสั่ง ESC** | หยุด | คงค่าเดิม |

RC ถือว่าสัญญาณใช้ได้เมื่อทุกช่อง update ภายใน 100 ms (`isRCValid`) ถ้าไม่ผ่าน probe จะหยุด **แต่ล้อไม่หยุด** (ดูข้อ 12)

### การควบคุมล้อ

ทำงานที่ 10 Hz โดยคำนวณเฉพาะใน Mode 2

```
feedback = pulse × RATIO / dt  ─ zero-holdoff ─ median 3 ค่า ─ EMA (ALPHA)
output   = feedforward + Kp·e + Ki·∫e + Kd·de/dt   (จำกัดที่ ±MAX_SPEED)
PWM      = CENTER_PWM ± DEADBAND_HALF ± (output / MAX_SPEED) × PWM_RANGE   (จำกัดที่ 1000-2000 µs)
```

ค่าที่ tune ได้อยู่ใน [PARAMETERS ข้อ 13](PARAMETERS.md#13-firmware-ของ-teensy)

**เรื่องชื่อที่สับสน**

- ขา 11/12 ชื่อ `MR_Hall*` แต่ต่อกับล้อ**ซ้าย** และขา 9/10 ชื่อ `ML_Hall*` แต่ต่อกับล้อ**ขวา**
- `pidLeft.setpoint` กลับเครื่องหมายด้วย `-1 * speed_l` และ `pidRight` ใช้ `reverse = true`
- `PULSES_PER_REV`, `WHEEL_CIRCUM` และ `interval` ประกาศไว้แต่ไม่ได้ใช้
- ค่ากลางตอน `setup()` ของล้อขวาคือ 1500 แต่ `CENTER_PWM_R` คือ 1485

### pin mapping ที่สำคัญ

| ขา | ชื่อ | หน้าที่ |
|---|---|---|
| 22, 21 | `PWM1`, `Dir1` | probe motor หน้า |
| 23, 20 | `PWM2`, `Dir2` | probe motor หลัง |
| 8, 7 | `Auto_L_R_pin`, `Auto_F_B_pin` | สัญญาณ ESC ล้อซ้าย, ล้อขวา |
| 5, 4, 2, 3 | `ML_R`, `MR_R`, `ProbeF_R`, `Mode_R` | RC input |
| 16 | `Relay_A_M_pin` | Relay สลับ Auto/Manual |
| 17 | `Relay_Safety_pin` | HIGH เมื่อ probe อันใดอันหนึ่งไม่อยู่บนสุด |
| 15 | `Auto_Manual` | LOW ใน Mode 1, HIGH ใน Mode 2 |
| A1, A0 | `Batt_pin`, `Gas_pin` | analog input |

ผลของ relay ต่อ circuit ดูจาก code ไม่ได้ ต้องตรวจจาก circuit จริง

### Hall sensor กับคำว่า "ไม่มี encoder"

Teensy อ่าน Hall sensor ที่ล้อเพื่อทำ PID และส่งจำนวน pulse ใน telemetry แต่ฝั่ง ROS ไม่ได้นำมาทำ odometry EKF จึงใช้ VO เป็นแหล่งความเร็วเพียงอย่างเดียว

---

## 10. Telemetry และการอ่าน meter

`teensy_feedback_server` เปิด TCP server ที่ port 5005 บนทุก interface แล้ว monitoring laptop เชื่อมเข้ามาผ่าน Tailscale IP ของ Jetson ฝั่ง Jetson เป็น server และรับได้ทีละหนึ่ง client

```
$<13 field จาก Teensy>,dcvg_on,dcvg_off,lat,lon#
```

| ลำดับ | ชื่อ | ความหมาย |
|---|---|---|
| 1-13 | Field จาก Teensy | ตามตารางในข้อ 9 |
| 14 | `dcvg_on` | ค่า ON จากจอ meter (จำนวนเต็ม มีเครื่องหมาย) |
| 15 | `dcvg_off` | ค่า OFF จากจอ meter (จำนวนเต็ม มีเครื่องหมาย) |
| 16 | `lat` | latitude จาก `/gps/fix` ทศนิยม 8 ตำแหน่ง |
| 17 | `lon` | longitude จาก `/gps/fix` ทศนิยม 8 ตำแหน่ง |

- Frame ที่จำนวน field ไม่ใช่ 17 จะถูกทิ้ง ถ้าเพิ่ม field ใน firmware ต้องแก้ `EXPECTED_FIELDS` ด้วย
- Buffer เก็บ 200 frame เกินแล้วทิ้ง frame เก่า และล้าง buffer เมื่อมี client ใหม่
- ถ้าข้อมูลเก่าเกินกำหนด ระบบยังส่ง frame ต่อและ log คำเตือน (DCVG เกิน 5 วินาที, GPS เกิน 3 วินาที)
- ถ้า OCR ไม่พร้อม ค่า DCVG จะเป็น `0,0`
- log ตัวนับ frames in / stamped / sent / dropped ทุก 10 วินาที

### OCR

`dcvg_ocr.py` อ่านภาพ 320x240 ด้วย easyocr โดยอาศัยตำแหน่ง pixel คงที่

1. ปรับ contrast ด้วย CLAHE
2. วางรูป `ocr/append_7.jpg` บนแถวบน และ `append_8.jpg` บนแถวล่าง ต่อท้ายตัวเลขบนจอ
3. อ่านเครื่องหมายบวกหรือลบจากความสว่างของพื้นที่ `SIGN_TOP`/`SIGN_BOT`
4. อ่านตัวเลข แล้วใช้ตัวสุดท้าย (7 หรือ 8) ยืนยันว่าเป็นแถวใด และตัดตัวนั้นทิ้งก่อนแปลงเป็นจำนวนเต็ม

> ถ้ากล้องขยับหรือเปลี่ยน meter ต้องปรับค่าตำแหน่งใน `dcvg_ocr.py` ใหม่

---

## 11. วิธีแก้ project อย่างปลอดภัย

**รอบการทำงานมาตรฐาน**

1. บันทึก bag จากภาคสนามก่อนถ้าเป็นไปได้
2. แก้ file ใน `src/`
3. `colcon build` และเปิด terminal ใหม่หรือ source อีกครั้ง
4. ทดสอบบน bag ด้วย `replay.launch.py` โดยเทียบกับ topic `/rec/...`
5. ทดสอบบนหุ่นจริงในพื้นที่โล่ง โดยมีคนถือ kill switch

File ใน `config/`, `launch/` และ `web/` ถูกคัดลอกไป `install/` ตอน build จึงต้อง build ใหม่หลังแก้ file ถ้าแก้ `index.html` ต้องเริ่ม `osm_route_app` ใหม่ด้วย

| ต้องการทำ | ต้องแก้ |
|---|---|
| เพิ่ม node ใหม่ | Python file, `entry_points` ใน `setup.py`, launch ที่เกี่ยวข้อง, `BRINGUP_CHECKS`, `CORE` ใน `record_bag.py` ถ้าอยากบันทึก topic, dependency ใน `package.xml` |
| เปลี่ยนความเร็ว | ดู[กลุ่มค่าที่ต้องตรงกัน](PARAMETERS.md#2-กลุ่มค่าที่ต้องตั้งให้ตรงกัน) และเรื่องหน่วยใน[ข้อ 12](#12-ปัญหาที่ทราบแล้ว) |
| ย้าย sensor | `dcvg_robot.urdf.xacro` และ[ค่าที่วัดจากตัวหุ่น](PARAMETERS.md#14-ค่าที่วัดจากตัวหุ่นยนต์) |
| เปลี่ยนขนาดหุ่น | `footprint` ทั้งสอง costmap, `inflation_radius`, `wheel_separation`, `wheel_radius`, URDF |
| เปลี่ยนรูปแบบ telemetry | Firmware `Send_Data()`, `EXPECTED_FIELDS`, program บน laptop |
| เปลี่ยนคำสั่ง serial | Firmware ส่วนรับคำสั่ง (`MAX_FIELDS`), `send()` ใน `low_level_command.py` |
| เปลี่ยนรอบ probe | Parameter `probe_*_delay` ของ `low_level_command` |
| เปลี่ยน meter camera | `CAMERA_DEVICE` และค่าตำแหน่งใน `dcvg_ocr.py` |
| เพิ่ม topic ใน bag | `CORE`, `DEPTH` หรือ `STEREO` ใน `record_bag.py` |

---

## 12. ปัญหาที่ทราบแล้ว

### ความปลอดภัย (ควรแก้ก่อน)

| # | ปัญหา | ผลกระทบ | แนวทาง |
|---|---|---|---|
| S1 | Firmware ไม่มี watchdog ของคำสั่ง serial `speed_l`/`speed_r` คงค่าเดิมจนกว่าจะมี frame ใหม่ | ถ้าสาย USB หลุด หุ่นวิ่งต่อด้วยคำสั่งล่าสุด ข้อความ log ใน `low_level_command` ที่พูดถึง watchdog ของ Teensy ไม่ตรงกับ firmware ปัจจุบัน | เพิ่ม timeout ใน firmware ให้ตั้ง setpoint เป็น 0 |
| S2 | ใน Mode 2 การ check `isRCValid()` ครอบเฉพาะ probe | RC ขาดสัญญาณแล้ว `Mode` ยังเป็น 2 และล้อยังวิ่ง | ให้สั่งหยุดล้อเมื่อสัญญาณ RC หาย |
| S3 | Mode 0 ไม่เขียน ESC เป็นค่ากลาง | เมื่อสับ switch จาก Auto ไปกลาง ESC อาจได้ pulse ล่าสุดต่อ ขึ้นกับการต่อ relay | ใส่คำสั่งค่ากลางในกรณี Mode 0 และทดสอบแบบยกล้อ |
| S4 | Bump sensor ถูกปิดใน firmware | ไม่มี bumper ช่วยตรวจการชน | ต่อและเปิดใช้งาน หรือแจ้งผู้ใช้ให้ชัดเจน |
| S5 | ไม่มี interlock ใน software ระหว่าง probe ลง | `/cmd_vel` เข้ามาระหว่างรอบ probe motor ก็รับคำสั่ง ต้องอาศัย safety relay ซึ่งยังไม่ได้ยืนยันการต่อ | ตรวจ circuit และเพิ่มการ lock ใน `low_level_command` |
| S6 | Web app เปิดบน `0.0.0.0` และไม่มี login | ทุกคนใน network เดียวกันสั่งหุ่นหรือปิดเครื่องได้ | ใช้ใน network ที่ควบคุมได้เท่านั้น |
| S7 | NTRIP login อยู่ใน repo สาธารณะ | บุคคลอื่นนำ account ไปใช้ได้ | เปลี่ยน password และย้ายออกจาก code |

### หน่วยคำสั่งล้อไม่ตรงกัน (ต้องวัดจริง)

`low_level_command` คำนวณค่าเป็น rad/s

```
left = (v - w × wheel_separation / 2) / wheel_radius
```

แต่ firmware นำค่านั้นไปเทียบกับ feedback ที่คำนวณจาก `pulse × RATIO / dt` โดย comment ระบุ `RATIO` เป็นระยะทางต่อ pulse ในหน่วยเมตร ซึ่งเท่ากับหน่วย m/s

| คำสั่ง Nav2 | ค่าที่ส่งถึง firmware |
|---|---|
| `v` = 0.15 m/s | 0.75 |
| `w` = 0.3 rad/s (หมุนอยู่กับที่) | ±0.60 |

ถ้า feedback เป็น m/s จริง target wheel speed จะสูงกว่าที่ Nav2 ตั้งใจราว 5 เท่า (`1 / wheel_radius`) แต่ `RATIO` ถูกปรับให้เข้ากับ gain เดิม จึงยังสรุปจาก code อย่างเดียวไม่ได้ ควรทดสอบตาม [COMMANDS ข้อ 5](COMMANDS.md#5-ทดสอบ-motor-และ-probe) ก่อนปรับค่าความเร็ว เรื่องนี้กระทบ EKF ด้วย เพราะ `use_control` predict จากคำสั่งโดยตรง

### พฤติกรรมที่ต้องรู้

| # | พฤติกรรม | รายละเอียด |
|---|---|---|
| B1 | รอบ probe ไม่ตรวจสอบผลจริง | `low_level_command` ยืนยันตาม timer โดยไม่อ่าน `Status_Prob` ถ้า RC ปิดหรือ `Mode` ไม่ใช่ 2 probe จะไม่ขยับ แต่ระบบยังยืนยันจุดหยุดและวิ่งต่อ |
| B2 | ต้องเปิด RC เพื่อให้ probe ทำงานใน Auto | `Prob_Auto_Main` check `isRCValid()` ก่อน |
| B3 | คำสั่ง probe ส่งครั้งเดียวตอนเปลี่ยนสถานะ | ถ้า frame หาย ระบบจะไม่ส่งซ้ำ เว้นแต่มี `/cmd_vel` เข้ามา |
| B4 | จุดหยุดอาจหายไปเงียบ ๆ | จุดหยุดที่ใกล้จุดก่อนหน้าเกิน `min_wp_distance` หรือใกล้หุ่นเกิน `min_start_distance` ตอนเริ่มจะถูกตัด และ probe จะไม่ทำงานที่จุดนั้น |
| B5 | GPS node ตัวใดตัวหนึ่งหยุด ทั้ง bringup จะปิด | `ublox_gps.launch.py` สั่ง `Shutdown` เมื่อ process จบ |
| B6 | Teensy ไม่ต่อตอนเริ่ม `low_level_command` จะ crash | การเชื่อมต่อใหม่ทำงานหลังเปิดสำเร็จครั้งแรกเท่านั้น |
| B7 | `Startup lock` | `low_level_command` ไม่รับคำสั่งจนผ่าน `startup_hold` และได้รับ `/cmd_vel` ที่เป็นศูนย์ หรือ `/cmd_vel` เงียบเกิน `cmd_timeout` ระบบจะ lock ซ้ำเมื่อต่อ Teensy ใหม่ |
| B8 | ใช้ `--merge-install` ไม่ได้ | Web app คำนวณ path กลับไป `src/` จากโครงสร้าง `install/dcvg_robot/share/dcvg_robot` |
| B9 | `rtabmap.launch.py` มี default ต่างจาก bringup | Run เดี่ยวได้ `use_stereo=false` ส่วน bringup ใช้ `True` |
| B10 | เปิด Nav2 ตรง ๆ ได้ BT มาตรฐาน | ถ้าใช้ `nav2_params.yaml` โดยไม่เติม path BT หุ่นอาจหมุนหรือถอยตอน recovery ใช้คำสั่งใน [COMMANDS ข้อ 3](COMMANDS.md#3-เปิดทีละส่วนด้วยมือ) |
| B11 | การ override parameter ของ follower | Argument แรกถูกอ่านเป็น path จึงต้องระบุ path ก่อน `--ros-args` |
| B12 | Lookahead ของ RPP คงที่ในทางปฏิบัติ | `0.15 × 1.5 = 0.225 m` ต่ำกว่า `min_lookahead_dist` 0.9 m |
| B13 | `waypoint_follower` ของ Nav2 ไม่ได้ถูกใช้ | Follower เรียก `NavigateThroughPoses` โดยตรง ค่า `waypoint_pause_duration` จึงไม่มีผล |
| B14 | แผนที่บน web ต้องใช้ internet | Leaflet, tile และการค้นหาสถานที่ load จากบริการภายนอก ถ้าเลือก `network_type` อื่นนอกจาก `none` ก็ต้องดึงข้อมูลจาก OSM |
| B15 | พื้นที่ disk | NVMe ที่เป็น disk เดียวของเครื่องเกือบเต็ม ควรบันทึก bag ยาวลง SSD ภายนอก |

---

## 13. การแก้ปัญหาตามอาการ

| อาการ | ตรวจอะไร |
|---|---|
| ไม่เห็น topic | `echo $ROS_DOMAIN_ID` ต้องเป็น 30 เมื่อเปิดระบบผ่าน service |
| หุ่นไม่ขยับ | Field `Mode` ใน `/teensy/feedback` ต้องเป็น 2, log `Startup lock` ของ `low_level_command`, `ros2 topic echo /cmd_vel` |
| Motor หยุดกลางทาง | การแจ้งเตือน stall บน web (ไม่ขยับ 7 วินาที) หรือ watchdog (`/cmd_vel` เงียบ 0.5 วินาที) |
| หุ่นวิ่งต่อไม่หยุดตามจุด แต่ web แสดงว่าถึงแล้ว | link USB ของ Teensy หลุด: ดู log `Teensy link lost` และ `journalctl -k` ว่ามี `usb 1-2.2: USB disconnect`/`error -71` ตอน motor เริ่มทำงานหรือไม่ สาเหตุคือสัญญาณรบกวนจาก motor แก้ด้วย ferrite choke, สาย shield สั้น, powered hub หรือ USB isolator |
| Probe ไม่ขยับ | RC ต้องเปิดอยู่, `Mode` ต้องเป็น 2, ดู `ProbeF_command` และ `Status_Prob` |
| ไม่ได้ RTK FIXED | log ของ `ntrip_bridge` ทุก 10 วินาที, ตรวจว่า `/rtcm` มีข้อมูล และ GGA ถูกส่ง |
| ทิศผิด | `ros2 topic echo /gps/heading/imu` และคำเตือน `GPS heading invalid` |
| ตำแหน่งกระโดด | `navsat_transform` ต้องไม่ log `Will assume navsat device is mounted at robot's origin` |
| Phantom obstacle | ดู `/camera/depth/points_decimated`, `obstacles_raw`, `obstacles`, `ground` ใน RViz โดยตั้ง fixed frame เป็น `base_footprint` จุดที่อยู่ใน `obstacles_raw` แต่ไม่อยู่ใน `obstacles` คือจุดที่ถูกตัดเป็นหญ้า จุดที่อยู่แค่ใน `obstacles` คือจุดพื้นที่ถูกเพิ่ม |
| พื้นเอียงผิด (`use_imu_tilt:=True`) | `tf2_echo base_footprint base_link` ต้องเปลี่ยนตามตัวหุ่น และ `tf2_echo odom base_footprint` ต้องมี roll และ pitch เป็น 0 |
| ค่า DCVG เป็น 0,0 | log ของ `teensy_feedback_server` เรื่องกล้องหรือการ load easyocr ทดลอง `enable_ocr:=False` เพื่อแยกปัญหา |
| Telemetry ไม่ถึง laptop | `tailscale status`, ทดสอบ `nc <jetson-tailscale-ip> 5005` และ log `Laptop connected from` หรือ `Cannot listen on port` |
| Telemetry ถูกทิ้ง | ตัวนับใน log ทุก 10 วินาที, `malformed` แปลว่าจำนวน field ไม่ตรง 17 |
| ปุ่มสร้างเส้นทางตอบ error 500 | ติดตั้ง `osmnx`, `networkx` แล้วหรือยัง |
| สลับ mode หรือปิดเครื่องจาก web ไม่ได้ | Sudoers rule ของ `systemctl` และ `poweroff` |
| Follower crash ทันทีเมื่อใส่ `-p` | ต้องระบุ path ของ waypoints ก่อน `--ros-args` |
| Bringup ปิดเองทั้งชุด | สาย GPS หลุดหรือ `ublox_gps_node` หยุดทำงาน |

Topic ที่ควรดูเป็นอันดับแรก: `/odometry/local`, `/odometry/global`, `/gps/fix`, `/gps/heading/imu`, `/vo`, `/teensy/feedback`, `/dcvg/telemetry`, `/dcvg/reading`

---

## 14. งานที่ยังค้างในการส่งมอบ

| รายการ | สถานะ | สิ่งที่ต้องทำ |
|---|---|---|
| Firmware ที่ flash อยู่บนหุ่น | ยังไม่ได้ยืนยัน | ยืนยันว่า `teensy/DCVG_Robot2_R1_Teensy.ino` คือ version ที่ flash อยู่ และบันทึกรุ่นของ Teensy |
| Manual mode program (`~/Desktop/DCVG_Robot`, `dcvg.service`) | อยู่บน Jetson เท่านั้น | สำรอง file และเก็บเข้า version control |
| Udev rules และ sudoers | อยู่บน Jetson เท่านั้น | สำรอง file ไว้เป็นตัวอย่าง |
| วิธีติดตั้ง package บน Jetson | ยังไม่ได้บันทึก | บันทึกว่า package ใดมาจาก apt หรือ build เอง โดยเฉพาะ RealSense, u-blox, ntrip_client |
| Telemetry receiver program บน laptop | ยังไม่ทราบตำแหน่ง | ขอ program และ account Tailscale จากผู้พัฒนาเดิม |
| account NTRIP | ฝังใน code | เปลี่ยน password และตกลงว่าใครเป็นเจ้าของ account |
| หน่วยคำสั่งล้อ | ยังไม่ได้วัด | ทดสอบตามหัวข้อด้านบน |
| ความหมายของ `level_ADC` และขา `Prox1`/`Prox2` | ยังไม่ได้ยืนยัน | ตรวจจาก hardware |
| Circuit ของ safety relay และ A/M relay | ยังไม่มีเอกสาร | เขียน circuit diagram ไว้ |
| Driver ของล้อ | ยังไม่ได้ยืนยัน | Firmware สั่งล้อด้วย pulse แบบ servo (1000-2000 µs) และสั่ง probe ด้วย PWM+DIR ควรตรวจว่า Pololu G2 ต่อกับส่วนใด |
| Folder `cache/` | cache OSM ติดเข้า repo | พิจารณาลบและเพิ่ม `.gitignore` เพราะ branch นี้ยังไม่มี file ดังกล่าว |
| คำอธิบาย repo บน GitHub | ยังพูดถึง LiDAR | ปรับให้ตรงกับหุ่นปัจจุบัน |
| `config/note` | งานค้างเดิม | ลดขนาด costmap และปรับกล้องให้ได้ระดับ |
