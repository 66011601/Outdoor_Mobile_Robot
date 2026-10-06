from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg = LaunchConfiguration

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='118.174.12.3'),
        DeclareLaunchArgument('port', default_value='2106'),
        DeclareLaunchArgument('mountpoint', default_value='VRS_MSM5_UTM47_ITRF2014'),
        DeclareLaunchArgument('username', default_value='Thanapoom322'),
        DeclareLaunchArgument('password', default_value='Mammam0322'),
        DeclareLaunchArgument('device', default_value='/dev/ttyGPS_movingbase'),

        Node(
            package='ntrip_client',
            executable='ntrip_ros.py',
            name='ntrip_client',
            output='both',
            parameters=[{
                'host': arg('host'),
                'port': arg('port'),
                'mountpoint': arg('mountpoint'),
                'ntrip_version': 'None',
                'authenticate': True,
                'username': arg('username'),
                'password': arg('password'),
                'ssl': False,
                'rtcm_message_package': 'rtcm_msgs',
                'rtcm_frame_id': 'gps_link',
                'reconnect_attempt_max': 10,
                'reconnect_attempt_wait_seconds': 5,
                'rtcm_timeout_seconds': 15,
            }],
            remappings=[('fix', 'ntrip_client/fix_unused')],
        ),
        Node(
            package='dcvg_bringup',
            executable='ntrip_bridge',
            name='ntrip_bridge',
            output='both',
            parameters=[{'device': arg('device')}],
        ),
    ])
