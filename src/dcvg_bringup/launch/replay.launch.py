import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

VO_OUTPUTS = ['/vo']
LOCALIZATION_OUTPUTS = ['/tf', '/odometry/local', '/odometry/global', '/odometry/gps',
                        '/gps/filtered', '/gps/heading/imu']
OBSTACLE_OUTPUTS = ['/camera/depth/obstacles', '/camera/depth/obstacles_raw',
                    '/camera/depth/ground', '/local_costmap/costmap',
                    '/local_costmap/published_footprint', '/global_costmap/costmap',
                    '/global_costmap/published_footprint']


def replay(context):
    pkg = get_package_share_directory('dcvg_bringup')
    launch_dir = os.path.join(pkg, 'launch')
    bt_dir = os.path.join(pkg, 'behavior_trees')
    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                               'navigation_launch.py')

    def value(name):
        return LaunchConfiguration(name).perform(context)

    def enabled(name):
        return value(name).lower() in ('true', '1')

    def include(path, launch_args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                        launch_arguments=launch_args.items())

    run_vo = enabled('run_vo')
    run_localization = enabled('run_localization')
    run_obstacles = enabled('run_obstacles')

    xacro_file = os.path.join(get_package_share_directory('dcvg_robot'), 'urdf',
                              'dcvg_robot.urdf.xacro')
    actions = [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' use_imu_tilt:=false']), value_type=str)}],
    )]
    if run_vo or run_obstacles:
        actions.append(include(os.path.join(launch_dir, 'rtabmap.launch.py'), {
            'use_vo': str(run_vo),
            'use_obstacles': str(run_obstacles),
            'use_stereo': value('use_stereo'),
            'max_ground_angle': value('max_ground_angle'),
            'max_ground_height': value('max_ground_height'),
            'use_terrain_filter': value('use_terrain_filter'),
            'grass_height': value('grass_height'),
            'step_height': value('step_height'),
        }))
    if run_localization:
        actions.append(include(os.path.join(launch_dir, 'localization.launch.py'), {}))
        actions.append(Node(
            package='dcvg_bringup',
            executable='gps_heading_to_imu',
            output='screen',
            parameters=[{'heading_offset_deg': float(value('heading_offset_deg'))}],
        ))
    if run_obstacles:
        nav2_params = RewrittenYaml(
            source_file=os.path.join(pkg, 'config', 'nav2_params.yaml'),
            root_key='',
            convert_types=True,
            param_rewrites={
                'default_nav_to_pose_bt_xml': os.path.join(bt_dir, 'navigate_to_pose.xml'),
                'default_nav_through_poses_bt_xml': os.path.join(bt_dir,
                                                                 'navigate_through_poses.xml'),
            },
        )
        actions.append(include(nav2_launch, {
            'params_file': nav2_params,
            'use_sim_time': 'True',
            'autostart': 'True',
        }))

    remapped = ((VO_OUTPUTS if run_vo else [])
                + (LOCALIZATION_OUTPUTS if run_localization else [])
                + (OBSTACLE_OUTPUTS if run_obstacles else []))
    cmd = ['ros2', 'bag', 'play', value('bag'), '--clock', '100', '-r', value('rate'),
           '--qos-profile-overrides-path', os.path.join(pkg, 'config', 'bag_qos.yaml')]
    if remapped:
        cmd += ['--remap', *[f'{topic}:=/rec{topic}' for topic in remapped]]
    actions.append(TimerAction(period=float(value('start_delay')),
                               actions=[ExecuteProcess(cmd=cmd, output='screen')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag'),
        DeclareLaunchArgument('run_vo', default_value='False'),
        DeclareLaunchArgument('run_localization', default_value='True'),
        DeclareLaunchArgument('run_obstacles', default_value='True'),
        DeclareLaunchArgument('use_stereo', default_value='True'),
        DeclareLaunchArgument('heading_offset_deg', default_value='0.0'),
        DeclareLaunchArgument('max_ground_angle', default_value='50.0'),
        DeclareLaunchArgument('max_ground_height', default_value='0.0'),
        DeclareLaunchArgument('use_terrain_filter', default_value='True'),
        DeclareLaunchArgument('grass_height', default_value='0.12'),
        DeclareLaunchArgument('step_height', default_value='0.20'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('start_delay', default_value='10.0'),
        DeclareLaunchArgument('domain_id', default_value='77'),

        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetParameter(name='use_sim_time', value=True),
        OpaqueFunction(function=replay),
    ])
