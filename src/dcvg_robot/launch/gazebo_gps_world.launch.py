import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('dcvg_robot')

    xacro_file = os.path.join(pkg, 'urdf', 'dcvg_robot.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'sonoma_raceway_with_elements.world')
    # world_file = os.path.join(pkg, 'worlds', 'sonoma_raceway.world')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Add dcvg_robot's share directory to GAZEBO_MODEL_PATH so the SDF
    # mesh URI  model://meshes/DCVG_Robot.stl  resolves correctly.
    models_dir = os.path.join(pkg, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + os.pathsep + models_dir
    else:
        gazebo_model_path = models_dir

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),

        ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so', world_file],
            output='both',
        ),

        ExecuteProcess(
            cmd=['gzclient'],
            output='both',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}],
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'dcvg_robot',
                '-topic', 'robot_description',
                '-x', '10', '-y', '-10', '-z', '0.5',
            ],
            output='both',
        ),
    ])
