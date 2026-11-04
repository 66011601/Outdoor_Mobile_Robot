# คำสั่งสำหรับ run ด้วยมือ

คำสั่งทั้งหมด run บน Jetson เว้นแต่ระบุว่าเป็น laptop คำสั่งเหล่านี้เขียนจากการอ่าน code และยังไม่ได้ทดลอง run บนหุ่นระหว่างเขียนเอกสาร

> ⚠️ ทุกครั้งที่คำสั่งอาจทำให้หุ่นเคลื่อนที่ ต้องมีคนถือ kill switch และควรยกล้อขึ้นจากพื้นเมื่อทดสอบครั้งแรก

## ตั้งค่า terminal

ทุก terminal ต้อง run ชุดนี้ก่อน ถ้าไม่ตั้ง `ROS_DOMAIN_ID` จะไม่เห็น topic ของระบบที่เปิดผ่าน service

```bash
source /opt/ros/humble/setup.bash
source ~/dcvg_robot/install/setup.bash
export ROS_DOMAIN_ID=30
```

---

## 1. ติดตั้งและ build

### Download code

```bash
git clone -b jetson https://github.com/66011601/Outdoor_Mobile_Robot.git ~/dcvg_robot
```

### ติดตั้ง dependency

```bash
cd ~/dcvg_robot
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Python libraries ที่ไม่ได้ระบุใน `package.xml`

```bash
pip3 install "osmnx>=2" networkx easyocr
```

บน Jetson ควรใช้ PyTorch รุ่นที่รองรับ GPU ของ Jetson สำหรับ easyocr ถ้าไม่ใช้ OCR ให้ข้าม `easyocr` และเปิดระบบด้วย `enable_ocr:=False`

### สิทธิ์ของ serial port

```bash
sudo usermod -aG dialout $USER
```

ต้องออกจากระบบแล้วเข้าใหม่จึงจะมีผล

### ตรวจข้อมูลสำหรับเขียน udev rule

```bash
udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct|serial' | head
```

```bash
ls -l /dev/ttyTeensy /dev/ttyGPS_*
```

### Build

```bash
cd ~/dcvg_robot
colcon build
source install/setup.bash
```

Build เฉพาะ package

```bash
colcon build --packages-select dcvg_bringup
```

```bash
colcon build --packages-select dcvg_robot
```

ห้ามใช้ `--merge-install` เพราะ web app จะหา folder `src/` ไม่เจอ

---

## 2. เปิดระบบผ่าน web app

### Run ชั่วคราว

```bash
ros2 run dcvg_robot osm_route_app
```

เปิด `http://<jetson-ip>:5010` หรือเปลี่ยน port ด้วย

```bash
DCVG_PORT=5020 ros2 run dcvg_robot osm_route_app
```

### ติดตั้งให้เปิดเองตอน boot

```bash
sudo cp ~/dcvg_robot/src/system/dcvg-auto.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now dcvg-auto
```

### จัดการ service

```bash
systemctl status dcvg-auto
```

```bash
journalctl -u dcvg-auto -f
```

```bash
sudo systemctl restart dcvg-auto
```

```bash
sudo systemctl stop dcvg-auto
```

### Manual mode เดิม

ใช้พร้อมกับ bringup ไม่ได้ เพราะเปิด `/dev/ttyTeensy` และ TCP 5005 เหมือนกัน

```bash
systemctl status dcvg.service
```

```bash
sudo systemctl stop dcvg.service
```

---

## 3. เปิดทีละส่วนด้วยมือ

### เปิดทั้งระบบโดยไม่ใช้ web

ก่อนเปิดให้ตรวจว่า web app ไม่ได้เปิด bringup อยู่ และ `dcvg.service` ถูกหยุดแล้ว

```bash
ros2 launch dcvg_bringup bringup.launch.py
```

ตัวอย่างการปรับค่า

```bash
ros2 launch dcvg_bringup bringup.launch.py use_rviz:=True enable_ocr:=False
```

```bash
ros2 launch dcvg_bringup bringup.launch.py use_stereo:=False
```

```bash
ros2 launch dcvg_bringup bringup.launch.py grass_height:=0.08 max_ground_angle:=35.0
```

ดูรายการ argument

```bash
ros2 launch dcvg_bringup bringup.launch.py --show-args
```

### เปิดทีละ terminal

ใช้เมื่อต้องการแยกหาปัญหา เปิดตามลำดับนี้ และรอให้แต่ละส่วนพร้อมก่อนเปิดส่วนถัดไป

**1. Robot model**

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro "$(ros2 pkg prefix dcvg_robot)/share/dcvg_robot/urdf/dcvg_robot.urdf.xacro" use_imu_tilt:=false)"
```

**2. กล้องและ IMU**

```bash
ros2 launch dcvg_bringup sensors.launch.py
```

**3. Motor command** (Teensy ต้องต่ออยู่ ไม่อย่างนั้น node จะ crash)

```bash
ros2 run dcvg_bringup low_level_command --ros-args -p port:=/dev/ttyTeensy
```

**4. Localization**

```bash
ros2 launch dcvg_bringup localization.launch.py
```

**5. Visual odometry และ obstacle** (ต้องใส่ `use_stereo:=True` เอง เพราะ default ของ file นี้ต่างจาก bringup)

```bash
ros2 launch dcvg_bringup rtabmap.launch.py use_stereo:=True
```

**6. GPS** (ถ้า GPS node ตัวใดหยุด launch นี้จะปิดทั้งชุด)

```bash
ros2 launch dcvg_bringup ublox_gps.launch.py
```

**7. NTRIP** (ใช้ login ที่เป็น default ใน file หรือส่งเองตามหัวข้อด้านล่าง)

```bash
ros2 launch dcvg_bringup ntrip.launch.py
```

**8. Nav2** ต้องเติม path ของ behavior tree ก่อน เพราะ `nav2_params.yaml` เว้น key นี้ไว้ ถ้าไม่เติม Nav2 จะใช้ BT มาตรฐานที่มี Spin และ BackUp โดยไม่เตือน

```bash
PKG="$(ros2 pkg prefix dcvg_bringup)/share/dcvg_bringup"
sed -e "s|default_nav_to_pose_bt_xml: \"\"|default_nav_to_pose_bt_xml: \"$PKG/behavior_trees/navigate_to_pose.xml\"|" \
    -e "s|default_nav_through_poses_bt_xml: \"\"|default_nav_through_poses_bt_xml: \"$PKG/behavior_trees/navigate_through_poses.xml\"|" \
    "$PKG/config/nav2_params.yaml" > /tmp/dcvg_nav2_params.yaml
```

```bash
ros2 launch nav2_bringup navigation_launch.py params_file:=/tmp/dcvg_nav2_params.yaml autostart:=True
```

**9. RViz** (ไม่บังคับ)

```bash
ros2 launch nav2_bringup rviz_launch.py
```

**10. Telemetry**

```bash
ros2 run dcvg_bringup teensy_feedback_server --ros-args -p enable_ocr:=true
```

### ส่ง NTRIP login เอง

เก็บค่าไว้ใน environment เพื่อไม่ให้ค้างในประวัติคำสั่ง อ่าน password ด้วย `read -s`

```bash
read -p "NTRIP user: " NTRIP_USER; read -s -p "NTRIP password: " NTRIP_PASS; echo
```

```bash
ros2 launch dcvg_bringup ntrip.launch.py username:="$NTRIP_USER" password:="$NTRIP_PASS"
```

---

## 4. วิ่งตามเส้นทางด้วยมือ

ใช้เส้นทางปัจจุบันจาก `install/`

```bash
ros2 run dcvg_robot route_waypoint_follower
```

ระบุ waypoint file (`stops.yaml` จะถูกอ่านจาก folder เดียวกัน ถ้าไม่มี file นี้จะวิ่งโดยไม่มีจุดหยุด)

```bash
ros2 run dcvg_robot route_waypoint_follower ~/dcvg_robot/src/dcvg_robot/config/waypoints.yaml
```

เปลี่ยน parameter: **ต้องระบุ path ก่อน `--ros-args`** เพราะ argument แรกจะถูกอ่านเป็น path

```bash
ros2 run dcvg_robot route_waypoint_follower ~/dcvg_robot/src/dcvg_robot/config/waypoints.yaml --ros-args -p on_segment_failure:=skip
```

### ควบคุมขณะวิ่ง

```bash
echo pause > /tmp/dcvg_nav_cmd
```

```bash
echo run > /tmp/dcvg_nav_cmd
```

```bash
echo stop > /tmp/dcvg_nav_cmd
```

### ยืนยันจุดหยุด

```bash
echo confirm > /tmp/dcvg_stop_confirm
```

หรือ

```bash
ros2 topic pub --once /dcvg/stop_confirm std_msgs/msg/String "{data: manual}"
```

### ดูสถานะ

```bash
cat /tmp/dcvg_nav_cmd /tmp/dcvg_stop_status /tmp/dcvg_nav_target
```

---

## 5. ทดสอบ motor และ probe

ก่อนทดสอบ: เปิด `low_level_command` และตั้ง switch Mode บน RC เป็น Auto ตรวจว่า field สุดท้ายของ frame เป็น `2`

```bash
ros2 topic echo /teensy/feedback --field data
```

ลำดับ field ดูได้ที่ [GUIDE ข้อ 9](GUIDE.md#9-firmware-ของ-teensy)

### สั่งล้อ

เดินหน้าช้า ๆ (กด Ctrl-C เพื่อหยุด motor หยุดภายใน 0.5 วินาทีจาก watchdog หาก link ปกติ)

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

หมุนอยู่กับที่

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.3}}"
```

สั่งหยุด

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

ขับด้วย keyboard (ถ้าติดตั้ง `teleop_twist_keyboard` แล้ว)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### วัดความเร็วจริงเพื่อตรวจเรื่องหน่วย

คำสั่ง 0.05 m/s ถูกส่งถึง firmware เป็น 0.25 ถ้าระยะทางจริงไม่ตรงกับที่คำนวณ ให้บันทึกอัตราส่วนไว้ เรื่องนี้อธิบายใน [GUIDE ข้อ 12](GUIDE.md#12-ปัญหาที่ทราบแล้ว)

1. ทำเครื่องหมายระยะ 5 m บนพื้นเรียบ ให้มีระยะเผื่อ brake
2. Run คำสั่งเดินหน้า 0.05 m/s พร้อมจับเวลา ถ้าความเร็วตรงจะใช้เวลาราว 100 วินาที ถ้าสูงกว่า 5 เท่าจะใช้เวลาราว 20 วินาที
3. ดูความเร็วที่ VO วัดได้ประกอบ

```bash
ros2 topic echo /vo --field twist.twist.linear.x
```

### ทดสอบ probe

คำสั่งนี้เริ่มรอบ นิ่ง → ลง → ขึ้น → ยืนยัน ต้องตรวจว่าใต้หุ่นปลอดภัย และ RC เปิดอยู่ มิฉะนั้น probe จะไม่ขยับ แต่รอบยังจบตามเวลา

```bash
ros2 topic pub --once /dcvg/stop_reached std_msgs/msg/String "{data: test}"
```

ดูการยืนยันใน terminal อื่น

```bash
ros2 topic echo /dcvg/stop_confirm
```

ปิดการยืนยันอัตโนมัติ

```bash
ros2 run dcvg_bringup low_level_command --ros-args -p auto_confirm:=false
```

### อ่าน serial ตรง ๆ

ต้องปิด `low_level_command` ก่อน คำสั่งนี้อ่านอย่างเดียว

```bash
stty -F /dev/ttyTeensy 115200 raw && cat /dev/ttyTeensy
```

---

## 6. บันทึกและเล่น bag ซ้ำ

### บันทึก

Run คู่กับ bringup ในอีก terminal เริ่มก่อนหรือหลัง bringup ก็ได้

```bash
ros2 run dcvg_bringup record_bag
```

```bash
ros2 run dcvg_bringup record_bag full -n rough_route
```

```bash
ros2 run dcvg_bringup record_bag nav -o /media/ssd/bags
```

| Profile | เพิ่มจากชุดหลัก | Run ซ้ำ offline ได้ | อัตราโดยประมาณ (zstd) |
|---|---|---|---|
| `nav` | - | Localization (จาก `/vo` ที่บันทึกไว้), วิเคราะห์การวิ่งของ Nav2 | ~1 MB/s |
| `obstacles` (default) | `/camera/depth/image_rect_raw` | เพิ่ม `depth_cloud` → `obstacles_detection` → costmap | ~6 MB/s |
| `full` | Depth + `/camera/infra1,2/image_rect_raw` | เพิ่ม stereo VO | ~22 MB/s |

| Argument | Default | ความหมาย |
|---|---|---|
| `profile` | `obstacles` | `nav`, `obstacles`, `full` |
| `-o`, `--out-dir` | `~/bags` | Folder เก็บ |
| `-n`, `--name` | ว่าง | ชื่อที่ใส่ในชื่อ folder |
| `--min-free-gb` | 5.0 | หยุดเองเมื่อพื้นที่ต่ำกว่านี้ |

- File ถูกแบ่งทุก 1 GB ด้วยการบีบอัด zstd รายข้อความ
- ใช้ queue การบีบอัด 500 และ 4 thread เพราะ default ของ Humble (queue 1) ทำให้ข้อความหาย
- RGBD VO (`use_stereo:=False`) ต้องใช้ภาพสีและ aligned depth ราว 135 MB/s และไม่มี profile ใดเก็บให้

### เล่นซ้ำ

Run บน `ROS_DOMAIN_ID` 77 จึงไม่ไปถึงหุ่นจริง ใช้เวลาจำลอง และ run ส่วนที่เลือกใหม่ด้วย file ปัจจุบัน

```bash
ros2 launch dcvg_bringup replay.launch.py bag:=$HOME/bags/obstacles_20261021_101500
```

```bash
ros2 launch dcvg_bringup replay.launch.py bag:=$HOME/bags/<ชื่อ bag> run_vo:=True rate:=0.5
```

```bash
ros2 launch dcvg_bringup replay.launch.py bag:=$HOME/bags/<ชื่อ bag> run_localization:=False max_ground_angle:=35.0
```

```bash
ros2 launch dcvg_bringup replay.launch.py bag:=$HOME/bags/<ชื่อ bag> run_localization:=False grass_height:=0.15
```

ดูผล

```bash
ROS_DOMAIN_ID=77 rviz2
```

| Argument | Default | Run ใหม่ | ค่าที่บันทึกไว้ย้ายไปที่ |
|---|---|---|---|
| `run_localization` | True | EKF ทั้งสอง, `navsat_transform`, `gps_heading_to_imu` | `/rec/tf`, `/rec/odometry/*`, `/rec/gps/filtered`, `/rec/gps/heading/imu` |
| `run_obstacles` | True | `depth_cloud`, `obstacles_detection`, `terrain_filter`, Nav2 พร้อม costmap (ไม่ส่ง goal) | `/rec/camera/depth/*`, `/rec/*_costmap/*` |
| `run_vo` | False | Stereo/RGBD odometry | `/rec/vo` |

ส่วนที่ปิดจะรับข้อมูลจาก bag เช่น `run_localization:=False` ใช้ `/tf` ที่บันทึกไว้ และ `run_vo:=False` ใช้ `/vo` ที่บันทึกไว้

ข้อจำกัด: ไม่รองรับ bag ที่บันทึกด้วย `use_imu_tilt:=True` และ replay เริ่ม `robot_state_publisher` จาก URDF ปัจจุบัน

---

## 7. คำสั่งตรวจสอบ

### ภาพรวม

```bash
ros2 node list
```

```bash
ros2 topic list
```

```bash
ros2 action list
```

```bash
ros2 topic hz /vo
```

```bash
ros2 topic hz /odometry/local
```

```bash
ros2 topic hz /camera/depth/obstacles
```

### GPS

```bash
ros2 topic echo /gps/fix --field status
```

```bash
ros2 topic echo /gps/heading/imu --once
```

```bash
ros2 topic hz /rtcm
```

### Localization และ TF

```bash
ros2 run tf2_ros tf2_echo map base_link
```

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link
```

```bash
ros2 run tf2_tools view_frames
```

```bash
ros2 topic echo /odometry/global --once
```

### Nav2

```bash
ros2 lifecycle get /bt_navigator
```

```bash
ros2 lifecycle get /controller_server
```

ดูค่า parameter ที่ทำงานจริง

```bash
ros2 param get /controller_server FollowPath.desired_linear_vel
```

```bash
ros2 param get /bt_navigator default_nav_through_poses_bt_xml
```

ล้าง costmap

```bash
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
```

### คำสั่งล้อและ telemetry

```bash
ros2 topic echo /cmd_vel
```

```bash
ros2 topic echo /dcvg/telemetry --field data
```

```bash
ros2 topic echo /dcvg/reading
```

### ระบบ

USB หลุดระหว่างวิ่ง

```bash
journalctl -k -f | grep -iE 'usb|error -71'
```

พื้นที่ disk

```bash
df -h
```

Meter camera

```bash
ls -l /dev/v4l/by-id/
```

ตรวจ RealSense

```bash
rs-enumerate-devices -s
```

---

## 8. Telemetry ผ่าน Tailscale

ดู Tailscale IP บน Jetson

```bash
tailscale ip -4
```

ดูเครื่องใน network

```bash
tailscale status
```

ทดสอบจาก monitoring laptop ถ้าเชื่อมสำเร็จจะเห็น frame `$...#` ไหลเข้ามา

```bash
nc <jetson-tailscale-ip> 5005
```

Server รับได้ทีละหนึ่ง client ต้องปิด `nc` ก่อนเปิด program มอนิเตอร์จริง

---

## 9. เรียก web app ด้วย curl

ใช้แทนการกดปุ่มบน web page เมื่อต้องการทดสอบ

| หน้าที่ | คำสั่ง |
|---|---|
| ดูสถานะ bringup | `curl -s http://localhost:5010/api/bringup/status` |
| เริ่ม bringup | `curl -X POST http://localhost:5010/api/bringup/start` |
| หยุด bringup | `curl -X POST http://localhost:5010/api/bringup/stop` |
| ดูเส้นทาง | `curl -s http://localhost:5010/api/waypoints` |
| เริ่ม/ทำต่อ | `curl -X POST http://localhost:5010/api/nav/start` |
| Pause | `curl -X POST http://localhost:5010/api/nav/pause` |
| Stop | `curl -X POST http://localhost:5010/api/nav/stop` |
| ดูสถานะการวิ่ง | `curl -s http://localhost:5010/api/nav/status` |
| ดูสถานะจุดหยุด | `curl -s http://localhost:5010/api/stop_status` |
| ยืนยันจุดหยุด | `curl -X POST http://localhost:5010/api/stop_confirm` |
| ดูตำแหน่ง | `curl -s http://localhost:5010/api/robot_pose` |
| ดู mode | `curl -s http://localhost:5010/api/mode` |
| สลับเป็น Auto | `curl -X POST -H 'Content-Type: application/json' -d '{"mode":"auto"}' http://localhost:5010/api/mode` |
| สลับเป็น Manual | `curl -X POST -H 'Content-Type: application/json' -d '{"mode":"manual"}' http://localhost:5010/api/mode` |

`/api/system/shutdown` จะปิดเครื่อง Jetson จริง

---

## 10. ปิดระบบ

เมื่อเปิดด้วยมือ กด Ctrl-C ทีละ terminal โดยเริ่มจาก follower แล้วปิด Nav2 และส่วนอื่นย้อนลำดับ ตรวจว่าไม่มี process ค้าง

```bash
ros2 node list
```

ถ้ารายชื่อค้างทั้งที่ไม่มี process แล้ว ให้ reset daemon

```bash
ros2 daemon stop
```

ปิดเครื่อง

```bash
sudo poweroff
```
