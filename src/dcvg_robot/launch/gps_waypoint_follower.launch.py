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
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key='', param_rewrites='', convert_types=True,
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
