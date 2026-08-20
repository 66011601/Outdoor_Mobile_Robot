"""
localization.launch.py  (REAL HARDWARE)

Dual EKF + navsat_transform from robot_localization — identical structure to the
simulation (dcvg_robot) but with use_sim_time=False and params from this package.

  ekf_local  (world=odom): /vo velocity + camera-gyro IMU yaw        → odom → base_footprint
  ekf_global (world=map):  GPS position + GPS heading yaw only       → map → odom

navsat_transform_node's own odometry/filtered input is remapped to odometry/global
(NOT odometry/local) deliberately: robot_localization's own odomCallback sets its
world_frame_id_ from whatever it's subscribed to here, and that frame_id is stamped
directly onto the /odometry/gps it publishes. ekf_filter_node_map's odom0 (GPS
position) is fused via preparePose(), which needs a TF lookup from that message's
frame into ekf_filter_node_map's own world_frame ("map") whenever they differ.
With odometry/local (frame "odom") feeding navsat_transform, that lookup needed
map->odom -- the exact transform ekf_filter_node_map itself broadcasts, built from
its own current state. Any yaw error in that state (from any source) then
misinterpreted the next GPS fix, pushing the state further off, corrupting the next
transform more -- a positive-feedback loop, confirmed live: raw /odometry/gps sat at
~4 m while ekf_filter_node_map's output ran away past 10^7 m within under a minute,
diagnostics reporting "functioning properly" throughout (no NaN -- this was the
filter's own math, not a crash). Feeding odometry/global here instead makes
/odometry/gps publish directly in the "map" frame, so preparePose's target frame
equals the message's own frame and takes the identity path -- no lookup, no loop,
structurally impossible for this feedback to recur. ekf_filter_node_map's own yaw
comes from its imu0 (camera-gyro rate), independent of navsat_transform, so it can
publish an early bootstrap message before any GPS fix exists, breaking what would
otherwise be a startup circular wait.

Topic contract (kept identical to sim so the config is reusable):
  in : /vo  /imu/data  /gps/fix  /gps/heading/imu (real HW only, see ublox_gps.launch.py)
  out: /odometry/local  /odometry/global  /odometry/gps
Remap the real driver outputs onto these names in sensors.launch.py / ublox_gps.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('dcvg_bringup')
    rl_params = os.path.join(pkg, 'config', 'dual_ekf_navsat_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('imu_max_rate_hz', default_value='50.0',
            description='Cap on /imu/data_throttled feeding ekf_filter_node_odom (imu0) — '
                        'the raw camera IMU runs at ~200 Hz, far faster than the EKF\'s own '
                        '30 Hz update cycle needs; confirmed on hardware to blow the EKF\'s '
                        'own update-rate budget otherwise, see imu_throttle.py'),

        Node(
            package='dcvg_bringup',
            executable='imu_throttle',
            name='imu_throttle',
            output='screen',
            parameters=[{'max_rate_hz': LaunchConfiguration('imu_max_rate_hz')}],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[rl_params, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/local')],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[rl_params, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/global')],
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            # Silences the routine "Datum UTM/(lat,lon,alt)" INFO lines logged
            # once at startup — WARN and above still show.
            arguments=['--ros-args', '--log-level', 'warn'],
            parameters=[rl_params, {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu/data', 'imu/data'),
                ('gps/fix', 'gps/fix'),
                ('gps/filtered', 'gps/filtered'),
                ('odometry/gps', 'odometry/gps'),
                ('odometry/filtered', 'odometry/global'),
            ],
        ),
    ])
