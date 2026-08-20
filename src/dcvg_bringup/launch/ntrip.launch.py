"""
ntrip.launch.py  (REAL HARDWARE — RTK corrections)

NTRIP → RTK correction chain for the simpleRTK2B Heading (ZED-F9P).
Run NEXT TO ublox_gps.launch.py — this launch does not start the GPS driver.

  caster 118.174.12.3:2106 (VRS_MSM5_UTM47_ITRF2014)
      │ RTCM3 over NTRIP                     ▲ NMEA GGA (VRS needs our position)
      ▼                                      │
  ntrip_client ──/rtcm──▶ ntrip_bridge ──────┤
                              │              └── /nmea ◀── built from /navpvt
                              └─(write-only serial)──▶ F9P

The mountpoint is a VRS: the caster streams nothing until it receives a GGA,
so no /rtcm at startup is NORMAL for the first few seconds — ntrip_bridge
warns if it stays silent. Credentials default to the test account; override
any of them as launch args instead of editing this file.

NOTE: this file carries the caster password as a default value — keep that in
mind before pushing the repo anywhere public.

Usage:
  ros2 launch dcvg_bringup ublox_gps.launch.py            # terminal 1
  ros2 launch dcvg_bringup ntrip.launch.py                # terminal 2

  ros2 launch dcvg_bringup ntrip.launch.py device:=/dev/serial/by-id/usb-u-blox_...-if00

No separate watch tool is needed to see FLOAT/FIXED: ntrip_bridge already logs
every RTK carrier-solution transition (NONE → FLOAT → FIXED) to its own console
output as it happens.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── caster account (defaults = current test credentials) ────────────
        DeclareLaunchArgument('host',          default_value='118.174.12.3'),
        DeclareLaunchArgument('port',          default_value='2106'),
        DeclareLaunchArgument('mountpoint',    default_value='VRS_MSM5_UTM47_ITRF2014'),
        DeclareLaunchArgument('username',      default_value='Thanapoom322'),
        DeclareLaunchArgument('password',      default_value='Mammam0322'),
        DeclareLaunchArgument('ntrip_version', default_value='None'),

        # ── receiver port (position/base board — same device ublox_gps's
        #    position node reads; we only write). RTCM only needs to reach
        #    the base board; it relays moving-base corrections to the
        #    heading board over the kit's own internal UART link. ─────────
        DeclareLaunchArgument(
            'device', default_value='/dev/ttyGPS_movingbase',
            description='Position board F9P serial port RTCM is written to '
                        '(prefer /dev/serial/by-id/...)'),
        DeclareLaunchArgument(
            'gga_period', default_value='1.0',
            description='Seconds between GGA position uploads to the caster'),

        # ntrip_client: connects to the caster, publishes /rtcm, and forwards
        # the GGA sentences it hears on /nmea up to the VRS.
        Node(
            package='ntrip_client',
            executable='ntrip_ros.py',
            name='ntrip_client',
            output='both',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'mountpoint': LaunchConfiguration('mountpoint'),
                'ntrip_version': LaunchConfiguration('ntrip_version'),
                'authenticate': True,
                'username': LaunchConfiguration('username'),
                'password': LaunchConfiguration('password'),
                'ssl': False,
                # rtcm_msgs (not the default mavros_msgs) — matches ntrip_bridge
                'rtcm_message_package': 'rtcm_msgs',
                'rtcm_frame_id': 'gps_link',
                'reconnect_attempt_max': 10,
                'reconnect_attempt_wait_seconds': 5,
                # VRS sends in bursts after each GGA; be generous before declaring dead
                'rtcm_timeout_seconds': 15,
            }],
            # Leave the client's own NavSatFix→GGA fallback disconnected — our
            # bridge publishes a better GGA (real altitude/quality, throttled).
            remappings=[('fix', 'ntrip_client/fix_unused')],
        ),

        # ntrip_bridge: /rtcm → F9P serial (write-only) + /navpvt → /nmea GGA
        Node(
            package='dcvg_bringup',
            executable='ntrip_bridge',
            name='ntrip_bridge',
            output='both',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'gga_period': LaunchConfiguration('gga_period'),
            }],
        ),
    ])
