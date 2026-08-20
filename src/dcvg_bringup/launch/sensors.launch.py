"""
sensors.launch.py  (REAL HARDWARE)

Brings up the perception sensor drivers on the Jetson:
  - realsense2_camera    → RGB + depth + (camera) IMU  for VIO and obstacle cloud
  - imu_filter_madgwick  → fuses the camera's raw accel+gyro into an oriented
                           quaternion IMU on /imu/data (there is no Pixhawk in this
                           build to source IMU/GPS from — see ublox_gps.launch.py
                           for GPS)

No physical LiDAR in this build (RPLIDAR was dropped) — obstacle avoidance on
real hardware is depth-camera-only via rtabmap.launch.py's obstacles_detection
(/camera/depth/obstacles), capped at the RGBD camera's ~6 m outdoor range. See
nav2_no_map_params.yaml (no `scan` observation source) and CLAUDE.md's Known
Issues.

ALL TOPIC/PORT VALUES ARE TBD — confirm against the connected hardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # ── Intel RealSense D455 / D457 ───────────────────────────────────
        # enable_gyro/accel + unite_imu_method produces a combined IMU stream.
        # pointcloud.enable gives the depth cloud consumed by obstacles_detection.
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='',
            output='screen',
            parameters=[{
                'enable_color':      True,
                'enable_depth':      True,
                'enable_infra1':     True,    # left IR  (stereo VIO)
                'enable_infra2':     True,    # right IR (stereo VIO)
                'enable_gyro':       True,
                'enable_accel':      True,
                'unite_imu_method':  2,       # 2 = linear interpolation → single /imu topic
                'pointcloud__neon_.enable': True,
                'align_depth.enable': True,
            }],
        ),

        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
            }],
            remappings=[
                ('imu/data_raw', '/camera/imu'),
                ('imu/data', '/imu/data'),
            ],
        ),
    ])
