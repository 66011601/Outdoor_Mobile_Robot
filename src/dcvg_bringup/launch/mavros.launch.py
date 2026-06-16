"""
mavros.launch.py  (REAL HARDWARE)

ROS2 ↔ Pixhawk (ArduRover) bridge. Replaces the Gazebo diff_drive plugin used in
simulation. Two jobs:

  1. Motion out: Nav2 publishes /cmd_vel → MAVROS → Pixhawk motor PWM.
     ArduPilot Rover expects /mavros/setpoint_velocity/cmd_vel_unstamped
     (geometry_msgs/Twist). A small relay/bridge from Nav2's /cmd_vel may be
     needed — see the commented relay node below.

  2. Sensors in: the Pixhawk's GPS + IMU come back over MAVLink. Remap them to the
     shared topic contract used by localization.launch.py:
       /mavros/global_position/raw/fix → /gps/fix
       /mavros/imu/data                → /imu/data   (only if NOT using camera IMU)

PORTS / FCU URL ARE TBD — confirm against the Pixhawk connection (USB vs telemetry).

SAFETY: arming, RC failsafe and geofence are configured in ArduPilot, NOT here.
Never autostart autonomous motion without the RC kill-switch in reach.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:115200',
            description='Pixhawk connection URL (TBD — confirm device/baud)'),
        DeclareLaunchArgument('gcs_url', default_value='',
            description='Optional ground-station forwarding URL'),

        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                'target_system_id': 1,
                'target_component_id': 1,
            }],
            remappings=[
                # Pixhawk sensors → shared topic contract (confirm exact MAVROS topic names)
                ('/mavros/global_position/raw/fix', '/gps/fix'),
                # ('/mavros/imu/data', '/imu/data'),   # enable if using Pixhawk IMU instead of camera IMU
            ],
        ),

        # ── cmd_vel → MAVROS velocity setpoint relay (TBD) ────────────────
        # Nav2 publishes geometry_msgs/Twist on /cmd_vel; ArduPilot wants it on
        # /mavros/setpoint_velocity/cmd_vel_unstamped. Use topic_tools relay or a
        # small custom node (add it to setup.py console_scripts).
        # Node(
        #     package='topic_tools',
        #     executable='relay',
        #     name='cmd_vel_to_mavros',
        #     arguments=['/cmd_vel', '/mavros/setpoint_velocity/cmd_vel_unstamped'],
        #     output='screen',
        # ),
    ])
