import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('dcvg_bringup'), 'config')

    position_board = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='both',
        parameters=[os.path.join(config, 'ublox_f9p.yaml'),
                    {'device': LaunchConfiguration('device')}],
        remappings=[('fix', '/gps/fix'), ('fix_velocity', '/gps/vel')],
    )

    heading_board = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node_heading',
        output='both',
        parameters=[os.path.join(config, 'ublox_f9p_heading.yaml'),
                    {'device': LaunchConfiguration('heading_device')}],
        remappings=[('navrelposned', '/gps/heading/navrelposned')],
    )

    def shutdown_on_exit(node):
        return RegisterEventHandler(OnProcessExit(target_action=node,
                                                  on_exit=[EmitEvent(event=Shutdown())]))

    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/ttyGPS_movingbase'),
        DeclareLaunchArgument('heading_device', default_value='/dev/ttyGPS_rover'),
        DeclareLaunchArgument('heading_offset_deg', default_value='0.0'),

        position_board,
        heading_board,
        Node(
            package='dcvg_bringup',
            executable='gps_heading_to_imu',
            output='screen',
            parameters=[{'heading_offset_deg': LaunchConfiguration('heading_offset_deg')}],
        ),

        shutdown_on_exit(position_board),
        shutdown_on_exit(heading_board),
    ])
