# DCVG Autonomous Rover

ระบบ ROS 2 Humble สำหรับหุ่นยนต์ภาคสนามที่วิ่งตามเส้นทาง GPS และวัดค่า DCVG (Direct Current Voltage Gradient) ของท่อตามจุดหยุดที่กำหนด โดยดัดแปลงจากรถตัดหญ้า และ run บน Jetson Orin Nano (Ubuntu 22.04)

> **Branch นี้ (`jetson`) คือ code ที่ใช้งานจริงบนหุ่นยนต์** ส่วน `main` เป็น project simulation รุ่นเก่าที่ทำไว้ก่อนมี branch นี้

## เอกสาร

| File | ใช้เมื่อ |
|---|---|
| [docs/GUIDE.md](docs/GUIDE.md) | ต้องการเข้าใจระบบหรือจะแก้ project: requirement, บทบาทของแต่ละ folder และ node, การไหลของข้อมูล, firmware, การแก้ปัญหา |
| [docs/PARAMETERS.md](docs/PARAMETERS.md) | ต้องการ tune ค่า: ตาราง variable ค่าปัจจุบัน ความหมาย ผลเมื่อเพิ่มหรือลด และค่าที่ต้องตั้งให้ตรงกัน |
| [docs/COMMANDS.md](docs/COMMANDS.md) | ต้องการ run ด้วยมือ: build, เปิดทีละส่วน, ทดสอบ motor และ probe, บันทึกและเล่นซ้ำ bag, คำสั่งตรวจสอบ |

ผู้รับช่วงต่อควรอ่าน [ปัญหาที่ทราบแล้ว](docs/GUIDE.md#12-ปัญหาที่ทราบแล้ว) และ [งานที่ยังค้างในการส่งมอบ](docs/GUIDE.md#14-งานที่ยังค้างในการส่งมอบ) ก่อนนำหุ่นยนต์ออกวิ่ง

## Hardware

| อุปกรณ์ | หน้าที่ | Device |
|---|---|---|
| Jetson Orin Nano | Run ROS 2 ทั้งหมดและ web app | - |
| simpleRTK2B Heading, position board (u-blox F9P) | ตำแหน่ง RTK รับ correction ผ่าน NTRIP | `/dev/ttyGPS_movingbase` |
| simpleRTK2B Heading, heading board | ทิศทางแบบ moving baseline (NAV-RELPOSNED) | `/dev/ttyGPS_rover` (UART 460800) |
| Teensy + Pololu G2 x2 | ควบคุมความเร็วล้อด้วย PID, สั่ง probe, อ่าน RC | `/dev/ttyTeensy` |
| RealSense D455/D457 | Visual odometry, point cloud obstacle, IMU | USB |
| USB webcam ที่ส่องจอ DCVG meter | อ่านค่าจากจอด้วย OCR | `/dev/v4l/by-id/usb-Pico_Instruments_...` |

หุ่นยนต์ไม่มี LiDAR และ ROS ไม่ได้ใช้ wheel odometry แม้ Teensy จะอ่าน Hall sensor ของล้อเพื่อทำ PID ก็ตาม

## ภาพรวมระบบ

```
RealSense ─┬─ stereo/rgbd_odometry ─ /vo ─────────────────────┐
           ├─ imu_filter_madgwick ─ imu_throttle ─ /imu/data_throttled ──┐
           └─ depth ─ point_cloud_xyz ─ obstacles_detection ─ terrain_filter
                                                   └─ /camera/depth/obstacles ─ Nav2 costmaps

GPS heading ─ gps_heading_to_imu ─ /gps/heading/imu ─┬─ ekf_filter_node_odom ─ odom→base_footprint, /odometry/local
                                                     └─ ekf_filter_node_map  ─ map→odom, /odometry/global
GPS position ─ /gps/fix ─ navsat_transform ─ /odometry/gps ─ ekf_filter_node_map
NTRIP caster ⇄ ntrip_client ⇄ ntrip_bridge ⇄ position board (RTCM เข้า, GGA ออก)

web app ─ route_waypoint_follower ─ Nav2 (RPP) ─ /cmd_vel ─ low_level_command ─ Teensy ─ motor, probe
                                                               Teensy ─ /teensy/feedback ─ teensy_feedback_server
                                                                  └─ TCP 5005 ─ Tailscale ─ monitoring laptop
```

## เริ่มใช้งานอย่างเร็ว

ตำแหน่ง workspace บนหุ่นยนต์คือ `~/dcvg_robot` และราก repo นี้คือรากของ workspace

```bash
source /opt/ros/humble/setup.bash
cd ~/dcvg_robot && colcon build
source install/setup.bash
export ROS_DOMAIN_ID=30
ros2 run dcvg_robot osm_route_app
```

จากนั้นเปิด `http://<jetson-ip>:5010` แล้วทำตามขั้นตอน

1. เลือก mode **Auto** (ส่วน **Manual** ใช้ legacy control program นอก repo นี้)
2. กด **Launch Bringup** และรอให้รายการตรวจสอบผ่าน
3. สร้างเส้นทางและกำหนดจุดหยุดบนแผนที่
4. ตั้ง switch Mode บน RC เป็น Auto แล้วกด **Start**

ถ้าติดตั้ง service `dcvg-auto` แล้ว web app จะเปิดเองตอน boot วิธีติดตั้งเครื่องใหม่อยู่ใน [GUIDE](docs/GUIDE.md#3-การติดตั้งเครื่องใหม่) และวิธี run โดยไม่ใช้ web อยู่ใน [COMMANDS](docs/COMMANDS.md)

## ความปลอดภัย

> ⚠️ **ต้องมี kill switch แยกอิสระที่ตัดไฟ motor ได้ทุกครั้งที่วิ่งกลางแจ้ง** firmware ปัจจุบันไม่มี watchdog ของคำสั่ง ถ้าสาย USB ของ Teensy หลุดหรือสัญญาณ RC ขาด หุ่นยนต์อาจวิ่งต่อด้วยคำสั่งล่าสุด ดูรายละเอียดใน [ปัญหาที่ทราบแล้ว](docs/GUIDE.md#12-ปัญหาที่ทราบแล้ว)

## ข้อจำกัด

- ทดสอบได้บนหุ่นยนต์จริงเท่านั้น หรือ tune แบบ offline ด้วยการเล่น bag ซ้ำ เพราะ branch นี้ไม่มี simulation
- ไม่ควรใช้ conda ร่วมกับ ROS 2 ใน workspace นี้
- Code ตั้งใจไม่ใส่ comment ให้เขียนคำอธิบายไว้ในเอกสารชุดนี้แทน
