import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('dcvg_robot')
    xacro_file = os.path.join(pkg, 'urdf', 'dcvg_robot.urdf.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'dcvg_env.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),

        # Publish robot TF tree from xacro-processed URDF + joint states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        # Hardware abstraction / simulated motor controller
        # Subscribes  /wheel_cmd  ->  publishes  /joint_states
        Node(
            package='dcvg_robot',
            executable='dcvg_controller',
        ),

        # /cmd_vel  ->  /wheel_cmd  (diff-drive IK)
        Node(
            package='dcvg_robot',
            executable='inverse_kinematics',
        ),

        # /joint_states  ->  /dcvg/vel  (diff-drive FK)
        Node(
            package='dcvg_robot',
            executable='forward_kinematics',
        ),

        # /joint_states  ->  /odom  +  TF odom->base_footprint
        Node(
            package='dcvg_robot',
            executable='odom',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
