import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

STARTUP_DELAYS = {
    'sensors': 2.0,
    'motors': 12.0,
    'localization': 12.0,
    'rtabmap': 16.0,
    'gps': 20.0,
    'ntrip': 20.0,
    'nav2': 21.0,
    'rviz': 22.0,
    'telemetry': 24.0,
}


def generate_launch_description():
    pkg = get_package_share_directory('dcvg_bringup')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(pkg, 'launch')
    bt_dir = os.path.join(pkg, 'behavior_trees')
    xacro_file = os.path.join(get_package_share_directory('dcvg_robot'), 'urdf',
                              'dcvg_robot.urdf.xacro')
    arg = LaunchConfiguration

    nav2_params = RewrittenYaml(
        source_file=os.path.join(pkg, 'config', 'nav2_params.yaml'),
        root_key='',
        convert_types=True,
        param_rewrites={
            'default_nav_to_pose_bt_xml': os.path.join(bt_dir, 'navigate_to_pose.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(bt_dir, 'navigate_through_poses.xml'),
        },
    )

    def include(file, launch_args=None, condition=None, directory=launch_dir):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(directory, file)),
            launch_arguments=(launch_args or {}).items(),
            condition=condition,
        )

    def after(stage, action):
        return TimerAction(period=STARTUP_DELAYS[stage], actions=[action])

    return LaunchDescription([
        DeclareLaunchArgument('use_stereo', default_value='True'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('use_telemetry', default_value='True'),
        DeclareLaunchArgument('enable_ocr', default_value='True'),
        DeclareLaunchArgument('heading_offset_deg', default_value='0.0'),
        DeclareLaunchArgument('teensy_port', default_value='/dev/ttyTeensy'),
        DeclareLaunchArgument('gps_device', default_value='/dev/ttyGPS_movingbase'),
        DeclareLaunchArgument('gps_heading_device', default_value='/dev/ttyGPS_rover'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro_file]),
                                                             value_type=str)}],
        ),

        after('sensors', include('sensors.launch.py')),

        after('motors', Node(
            package='dcvg_bringup',
            executable='low_level_command',
            output='screen',
            parameters=[{'port': arg('teensy_port')}],
        )),

        after('localization', include('localization.launch.py')),

        after('rtabmap', include('rtabmap.launch.py', {'use_stereo': arg('use_stereo')})),

        after('gps', include('ublox_gps.launch.py', {
            'device': arg('gps_device'),
            'heading_device': arg('gps_heading_device'),
            'heading_offset_deg': arg('heading_offset_deg'),
        })),

        after('ntrip', include('ntrip.launch.py', {'device': arg('gps_device')})),

        after('nav2', include('navigation_launch.py', {
            'params_file': nav2_params,
            'autostart': 'True',
        }, directory=os.path.join(nav2_bringup, 'launch'))),

        after('rviz', include('rviz_launch.py', condition=IfCondition(arg('use_rviz')),
                              directory=os.path.join(nav2_bringup, 'launch'))),

        after('telemetry', Node(
            package='dcvg_bringup',
            executable='teensy_feedback_server',
            output='screen',
            parameters=[{'enable_ocr': ParameterValue(arg('enable_ocr'), value_type=bool)}],
            condition=IfCondition(arg('use_telemetry')),
        )),
    ])
