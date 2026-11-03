from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_imu_tilt', default_value='False'),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='',
            output='screen',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_gyro': True,
                'enable_accel': True,
                'unite_imu_method': 2,
                'pointcloud__neon_.enable': False,
                'align_depth.enable': True,
            }],
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'enu'}],
            remappings=[('imu/data_raw', '/camera/imu'), ('imu/data', '/imu/data')],
        ),
        Node(
            package='dcvg_bringup',
            executable='imu_throttle',
            output='screen',
        ),
        Node(
            package='dcvg_bringup',
            executable='imu_tilt',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_imu_tilt')),
        ),
    ])
