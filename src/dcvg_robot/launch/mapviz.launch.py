import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    pkg = get_package_share_directory('dcvg_robot')
    mapviz_config_file = os.path.join(pkg, 'config', 'mapviz.mvc')

    return LaunchDescription([
        launch_ros.actions.Node(
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            parameters=[{'config': mapviz_config_file, 'use_sim_time': True}],
        ),

        launch_ros.actions.Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            remappings=[('fix', 'gps/fix')],
            parameters=[{'use_sim_time': True}],
        ),

        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='swri_transform',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'origin'],
            parameters=[{'use_sim_time': True}],
        ),
    ])
