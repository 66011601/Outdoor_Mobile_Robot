"""
ublox_gps.launch.py  (REAL HARDWARE)

UBX-native GPS drivers for the ArduSimple simpleRTK2B Heading kit — TWO u-blox
ZED-F9P boards, each its own USB serial device:
  - POSITION board (big board):  /dev/ttyACM0 — absolute fix, gets NTRIP RTK
    corrections (see ntrip.launch.py), publishes /gps/fix + /gps/vel.
  - HEADING board (small board): /dev/ttyUSB0 — receives moving-base
    corrections from the position board over the kit's own internal UART
    link (not through this host), publishes NAV-RELPOSNED (relative vector
    to the base antenna). Converted to a yaw-only IMU message by
    gps_heading_to_imu and fused into ekf_global — see dual_ekf_navsat_params.yaml.

Replaces gps.launch.py (nmea_navsat_driver), which failed because the F9P streams
binary UBX on its native USB, not ASCII NMEA — the driver crashed decoding the UBX
sync byte 0xb5. The ublox_gps driver speaks UBX directly.

  driver out (relative)   remapped to      consumed by
  ─────────────────────   ─────────────    ────────────────────────────
  fix (NavSatFix)         /gps/fix         navsat_transform (localization.launch.py)
  fix_velocity (Twist…)   /gps/vel
  navrelposned            /gps/heading/navrelposned   gps_heading_to_imu → /gps/heading/imu
                                                       → ekf_global (localization.launch.py)

Config lives in config/ublox_f9p.yaml (position) and config/ublox_f9p_heading.yaml
(heading). Both run the receiver READ-ONLY (config_on_startup: false) so this
will not clobber the moving-baseline / message-rate setup you flash with
u-center — configure the receivers there, not here.

Notes
- /dev/ttyACM0 is not stable if a Pixhawk (also ACM) is plugged in — not a concern
  in this build (no Pixhawk), but prefer the by-id symlink if port numbering ever
  drifts: ls -l /dev/serial/by-id/
- Each node exits the whole launch on driver death (OnProcessExit → Shutdown) so a
  disconnected receiver fails loudly instead of silently publishing nothing.

Usage:
  ros2 launch dcvg_bringup ublox_gps.launch.py
  ros2 launch dcvg_bringup ublox_gps.launch.py device:=/dev/serial/by-id/usb-u-blox_...-if00
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration('device')
    heading_device = LaunchConfiguration('heading_device')
    heading_offset_deg = LaunchConfiguration('heading_offset_deg')
    min_heading_std_deg = LaunchConfiguration('min_heading_std_deg')
    pkg_config = os.path.join(get_package_share_directory('dcvg_bringup'), 'config')
    params_file = os.path.join(pkg_config, 'ublox_f9p.yaml')
    heading_params_file = os.path.join(pkg_config, 'ublox_f9p_heading.yaml')

    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',          # must match the top-level key in ublox_f9p.yaml
        output='both',
        # yaml first, then override just the device from the launch arg
        parameters=[params_file, {'device': device}],
        remappings=[
            ('fix', '/gps/fix'),        # → shared contract topic for navsat_transform
            ('fix_velocity', '/gps/vel'),
        ],
    )

    ublox_heading_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node_heading',  # must match the top-level key in ublox_f9p_heading.yaml
        output='both',
        parameters=[heading_params_file, {'device': heading_device}],
        remappings=[
            ('navrelposned', '/gps/heading/navrelposned'),
        ],
    )

    gps_heading_to_imu_node = Node(
        package='dcvg_bringup',
        executable='gps_heading_to_imu',
        name='gps_heading_to_imu',
        output='screen',
        parameters=[{'heading_offset_deg': heading_offset_deg,
                     'min_heading_std_deg': min_heading_std_deg}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'device', default_value='/dev/ttyGPS_movingbase',
            description='Position board (big board) F9P serial port '
                        '(prefer /dev/serial/by-id/...)'),
        DeclareLaunchArgument(
            'heading_device', default_value='/dev/ttyGPS_rover',
            description='Heading board (small board) F9P serial port '
                        '(prefer /dev/serial/by-id/...)'),
        DeclareLaunchArgument(
            'heading_offset_deg', default_value='0.0',
            description='GPS heading antenna mounting offset (deg) — calibrate against '
                        'a known heading once on hardware, see gps_heading_to_imu.py'),
        DeclareLaunchArgument(
            'min_heading_std_deg', default_value='2.0',
            description='Floor on GPS-heading yaw std-dev — realistic sensor-accuracy '
                        'floor for both EKF instances; see gps_heading_to_imu.py for '
                        'why this no longer needs to be inflated as a feedback-loop '
                        'damping knob (that loop is now fixed structurally instead).'),

        ublox_node,
        ublox_heading_node,
        gps_heading_to_imu_node,

        # If a driver dies (receiver unplugged / port gone), bring the whole
        # launch down so the failure is obvious rather than silently fixless.
        RegisterEventHandler(
            OnProcessExit(target_action=ublox_node,
                          on_exit=[EmitEvent(event=Shutdown())])),
        RegisterEventHandler(
            OnProcessExit(target_action=ublox_heading_node,
                          on_exit=[EmitEvent(event=Shutdown())])),
    ])
