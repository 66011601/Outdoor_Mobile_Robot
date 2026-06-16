"""
bringup.launch.py  (REAL HARDWARE — top-level entry point)

Real-robot counterpart of dcvg_robot/gps_waypoint_follower.launch.py. Instead of
Gazebo, it starts the real sensor drivers + MAVROS, then the SAME shared stack
(RTAB-Map VIO + dual EKF + Nav2) the simulation already validated.

Chain:
  1. robot_state_publisher  → publishes the robot TF tree (frames for camera/lidar/imu)
  2. sensors.launch.py      → RealSense + RPLIDAR drivers
  3. mavros.launch.py       → Pixhawk bridge (GPS in, cmd_vel out)
  4. localization.launch.py → dual EKF + navsat_transform
  5. rtabmap.launch.py      → /vo + /camera/depth/obstacles
  6. nav2_bringup           → full Nav2 stack with the custom recovery BTs

Everything runs with use_sim_time:=False.

Usage:
  ros2 launch dcvg_bringup bringup.launch.py
  ros2 launch dcvg_bringup bringup.launch.py use_stereo:=true use_rviz:=True
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('dcvg_bringup')
    bringup_dir = get_package_share_directory('nav2_bringup')

    launch_dir = os.path.join(pkg, 'launch')
    nav2_params = os.path.join(pkg, 'config', 'nav2_no_map_params.yaml')
    bt_dir = os.path.join(pkg, 'behavior_trees')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_stereo   = LaunchConfiguration('use_stereo')
    use_rviz     = LaunchConfiguration('use_rviz')
    description_file = LaunchConfiguration('description_file')

    # Point bt_navigator at the custom recovery BTs (BackUp before Spin).
    param_substitutions = {
        'default_nav_to_pose_bt_xml': os.path.join(
            bt_dir, 'navigate_to_pose_w_replanning_and_recovery.xml'),
        'default_nav_through_poses_bt_xml': os.path.join(
            bt_dir, 'navigate_through_poses_w_replanning_and_recovery.xml'),
    }
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key='',
        param_rewrites=param_substitutions, convert_types=True,
    )

    robot_description = ParameterValue(
        Command(['xacro ', description_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('use_stereo',   default_value='false'),
        DeclareLaunchArgument('use_rviz',     default_value='False'),
        # TODO: split a hardware-specific (Gazebo-free) robot description out of the
        # sim package. For now reuse the sim xacro — robot_state_publisher ignores
        # the <gazebo> plugin tags, so the TF tree it publishes is correct.
        DeclareLaunchArgument(
            'description_file',
            default_value=os.path.join(
                get_package_share_directory('dcvg_robot'), 'urdf', 'dcvg_robot.urdf.xacro'),
            description='Robot xacro/urdf for the TF tree'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': use_sim_time}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sensors.launch.py')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mavros.launch.py')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rtabmap.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'use_stereo': use_stereo}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': configured_params,
                'autostart': 'True',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'rviz_launch.py')),
            condition=IfCondition(use_rviz),
        ),
    ])
