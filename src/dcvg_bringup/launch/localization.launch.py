"""
localization.launch.py  (REAL HARDWARE)

Dual EKF + navsat_transform from robot_localization — identical structure to the
simulation (dcvg_robot) but with use_sim_time=False and params from this package.

  ekf_local  (world=odom): /vo velocity + IMU yaw  → odom → base_footprint
  ekf_global (world=map):  GPS position + IMU yaw   → map → odom

Topic contract (kept identical to sim so the config is reusable):
  in : /vo  /imu/data  /gps/fix
  out: /odometry/local  /odometry/global  /odometry/gps
Remap the real driver outputs onto these names in sensors.launch.py / mavros.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('dcvg_bringup')
    rl_params = os.path.join(pkg, 'config', 'dual_ekf_navsat_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[rl_params, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/local')],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[rl_params, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/global')],
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[rl_params, {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu/data', 'imu/data'),
                ('gps/fix', 'gps/fix'),
                ('gps/filtered', 'gps/filtered'),
                ('odometry/gps', 'odometry/gps'),
                ('odometry/filtered', 'odometry/global'),
            ],
        ),
    ])
