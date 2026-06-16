"""
sensors.launch.py  (REAL HARDWARE)

Brings up the perception sensor drivers on the Jetson:
  - realsense2_camera  → RGB + depth + (camera) IMU  for VIO and obstacle cloud
  - rplidar_ros        → 2D LiDAR scan, remapped to /lidar/scan (Nav2 costmap source)

GPS is NOT here — it arrives through MAVROS from the Pixhawk (see mavros.launch.py).

IMU NOTE (TBD): RTAB-Map wants an oriented IMU (with a quaternion). The RealSense
publishes only raw accel+gyro. Either:
  (a) run imu_filter_madgwick to fuse them into /imu/data (orientation), or
  (b) use the Pixhawk IMU from MAVROS (/mavros/imu/data) instead.
Pick one and make sure the chosen source ends up on /imu/data.

ALL TOPIC/PORT VALUES ARE TBD — confirm against the connected hardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    lidar_baudrate    = LaunchConfiguration('lidar_baudrate')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0',
            description='RPLIDAR S2 serial device (TBD — confirm on hardware)'),
        DeclareLaunchArgument('lidar_baudrate', default_value='1000000',
            description='RPLIDAR S2 baudrate (1 Mbaud)'),

        # ── Intel RealSense D455 / D457 ───────────────────────────────────
        # enable_gyro/accel + unite_imu_method produces a combined IMU stream.
        # pointcloud.enable gives the depth cloud consumed by obstacles_detection.
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'enable_color':      True,
                'enable_depth':      True,
                'enable_infra1':     True,    # left IR  (stereo VIO)
                'enable_infra2':     True,    # right IR (stereo VIO)
                'enable_gyro':       True,
                'enable_accel':      True,
                'unite_imu_method':  2,       # 2 = linear interpolation → single /imu topic
                'pointcloud.enable': True,
                'align_depth.enable': True,
            }],
        ),

        # ── RPLIDAR S2 (2D LiDAR) ─────────────────────────────────────────
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port':  lidar_serial_port,
                'serial_baudrate': lidar_baudrate,
                'frame_id':     'laser',
                'scan_mode':    'DenseBoost',
            }],
            remappings=[('scan', '/lidar/scan')],
        ),
    ])
