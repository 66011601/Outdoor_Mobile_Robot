import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('dcvg_bringup'), 'config',
                          'localization.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[params],
            remappings=[('odometry/filtered', 'odometry/local')],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[params],
            remappings=[('odometry/filtered', 'odometry/global')],
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn'],
            parameters=[params],
            remappings=[('odometry/filtered', 'odometry/global')],
        ),
    ])
