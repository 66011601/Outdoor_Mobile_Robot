import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('dcvg_robot')
    bringup_dir = get_package_share_directory('nav2_bringup')

    launch_dir = os.path.join(pkg, 'launch')
    nav2_params = os.path.join(pkg, 'config', 'nav2_no_map_params.yaml')
    bt_dir = os.path.join(pkg, 'behavior_trees')

    # Point bt_navigator at our custom recovery BTs (BackUp before Spin).
    # These paths are install-specific so they are injected here, not in the yaml.
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

    use_rviz = LaunchConfiguration('use_rviz')
    use_mapviz = LaunchConfiguration('use_mapviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz',    default_value='False'),
        DeclareLaunchArgument('use_mapviz',  default_value='False'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'gazebo_gps_world.launch.py')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'dual_ekf_navsat.launch.py')),
        ),

        # RTAB-Map: visual odometry (/vo) + ground-filtered obstacle cloud
        # Must start after Gazebo (camera topics) and before Nav2 (obstacle source)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'rtabmap.launch.py')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'True',
                'params_file': configured_params,
                'autostart': 'True',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'rviz_launch.py')),
            condition=IfCondition(use_rviz),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'mapviz.launch.py')),
            condition=IfCondition(use_mapviz),
        ),
    ])
