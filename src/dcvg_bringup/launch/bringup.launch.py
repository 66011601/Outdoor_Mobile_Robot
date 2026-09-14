"""
bringup.launch.py  (REAL HARDWARE — top-level entry point)

Top-level real-robot entry point: starts the real sensor drivers + GPS/RTK + Teensy
motor bridge, then the shared stack (RTAB-Map VIO + dual EKF + Nav2) — the same
architecture dcvg_robot's Gazebo simulation was used to design and validate
against (dcvg_robot no longer carries a runnable Gazebo sim; see CLAUDE.md).

No Pixhawk/MAVROS in this build: motor commands go straight to a Teensy over serial
(low_level_command), and GPS+RTK comes directly from the simpleRTK2B Heading (u-blox
ZED-F9P) via ublox_gps + ntrip_client, not through a flight controller.

Chain:
  1. robot_state_publisher  → publishes the robot TF tree (frames for camera/imu; no LiDAR)
  2. sensors.launch.py      → RealSense driver + IMU orientation filter
  3. ublox_gps.launch.py    → position (/dev/ttyGPS_movingbase) + heading (/dev/ttyGPS_rover)
                              F9P boards → /gps/fix, /gps/vel, /gps/heading/navrelposned
                              → gps_heading_to_imu → /gps/heading/imu (yaw, fused in step 6)
  4. ntrip.launch.py        → NTRIP RTK corrections → the position board (FLOAT/FIXED)
  5. low_level_command      → /cmd_vel → serial (/dev/ttyTeensy) → Teensy → Pololu motor drivers
  6. localization.launch.py → dual EKF + navsat_transform
  7. rtabmap.launch.py      → /vo + /camera/depth/obstacles
  8. nav2_bringup           → full Nav2 stack with the custom recovery BTs

Everything runs with use_sim_time:=False.

Staggered startup: launching everything at once has been observed to throw errors on
this hardware (drivers racing for serial ports before udev settles, RTAB-Map/Nav2
subscribing before their upstream topics exist, etc.). Each stage above is wrapped in
a TimerAction so it starts only after the previous stage has had time to come up
cleanly, rather than all-at-once. Total time to Nav2 coming up is ~20s by default —
slower, but stable. Tune via the *_delay launch args below if a stage still races on
your hardware (bump the delay after it) or if you want a faster start once you've
confirmed it's reliable (shrink the delays).

Serial ports (fixed on this robot via /etc/udev/rules.d/99-usb-serial.rules —
keyed on physical USB port / vendor+serial, NOT raw ttyACM*/ttyUSB* enumeration
order, which drifts across replugs/reboots):
  /dev/ttyGPS_movingbase — GPS position board (big board, simpleRTK2B Heading)
  /dev/ttyGPS_rover      — GPS heading board (small board)
  /dev/ttyTeensy         — Teensy (low_level_command)
Override any of them with the gps_device / gps_heading_device / teensy_port args
if a device ever needs a one-off different path.

Usage:
  ros2 launch dcvg_bringup bringup.launch.py
  ros2 launch dcvg_bringup bringup.launch.py use_stereo:=true use_rviz:=True
  ros2 launch dcvg_bringup bringup.launch.py controller_type:=dwb   # A/B vs RPP (default)
  ros2 launch dcvg_bringup bringup.launch.py max_accel_linear:=0.3  # accel sweep, see below
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Substitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


class AccelOverride(Substitution):
    """Patch the acceleration limits into an already-RewrittenYaml params file.

    Exists because RewrittenYaml cannot write a LIST-valued parameter.
    velocity_smoother's max_accel is a double[3]; RewrittenYaml's convert()
    only ever produces int/float/bool/str, and its indexed-path form
    ('...max_accel.0') throws because updateYamlPathVals indexes the list with
    a string key. So the list has to be patched after the fact, here.

    Sets BOTH ends of the angular ramp together — velocity_smoother's
    max_accel[2] and RPP's FollowPath.max_angular_accel — because
    nav2_no_map_params.yaml requires them to stay equal. If they diverge with
    the smoother tighter, RPP plans a ramp the smoother then clips, and the
    controller is reasoning about a more responsive vehicle than the one it is
    actually commanding.

    max_decel is deliberately NOT exposed: it is the emergency-stop path.
    """

    def __init__(self, base_yaml, linear, angular):
        super().__init__()
        self._base = normalize_to_list_of_substitutions(base_yaml)
        self._linear = normalize_to_list_of_substitutions(linear)
        self._angular = normalize_to_list_of_substitutions(angular)

    def describe(self):
        return 'AccelOverride'

    def perform(self, context):
        path = perform_substitutions(context, self._base)
        linear = float(perform_substitutions(context, self._linear))
        angular = float(perform_substitutions(context, self._angular))

        with open(path, 'r') as handle:
            data = yaml.safe_load(handle)

        smoother = data['velocity_smoother']['ros__parameters']
        smoother['max_accel'] = [linear, 0.0, angular]
        data['controller_server']['ros__parameters']['FollowPath']['max_angular_accel'] = angular

        with open(path, 'w') as handle:
            yaml.dump(data, handle)
        return path


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
    controller_type = LaunchConfiguration('controller_type')
    is_rpp = PythonExpression(["'", controller_type, "' == 'rpp'"])
    is_dwb = PythonExpression(["'", controller_type, "' == 'dwb'"])

    # Point bt_navigator at the custom recovery BTs (BackUp before Spin).
    # Two variants per BT, differing only in which controller_id the FollowPath
    # BT node targets — "FollowPath" (RPP, the active plugin id in
    # nav2_no_map_params.yaml) or "FollowPath_DWB" (the archived DWB config,
    # still loaded alongside it) — selected by controller_type below so DWB vs
    # RPP can be A/B tested on hardware without hand-editing the yaml.
    def bt_param_substitutions(suffix):
        return {
            'default_nav_to_pose_bt_xml': os.path.join(
                bt_dir, f'navigate_to_pose_w_replanning_and_recovery{suffix}.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(
                bt_dir, f'navigate_through_poses_w_replanning_and_recovery{suffix}.xml'),
        }
    # THE ACCELERATION SWEEP LIVES HERE — change it at launch, not in the yaml:
    #   ros2 launch dcvg_bringup bringup.launch.py max_accel_linear:=0.3
    # It can also be walked WITHOUT relaunching, which is what field bisection
    # actually wants (no 21s nav2_delay, no re-acquiring the RTK fix):
    #   ros2 param set /velocity_smoother max_accel "[0.3, 0.0, 0.8]"
    #   ros2 param set /controller_server FollowPath.max_angular_accel 0.8
    # nav2_velocity_smoother registers a dynamicParametersCallback that accepts
    # max_accel live; keep the two angular values equal when doing that.
    accel_linear = LaunchConfiguration('max_accel_linear')
    accel_angular = LaunchConfiguration('max_accel_angular')
    configured_params_rpp = AccelOverride(
        RewrittenYaml(
            source_file=nav2_params, root_key='',
            param_rewrites=bt_param_substitutions(''), convert_types=True,
        ), accel_linear, accel_angular,
    )
    configured_params_dwb = AccelOverride(
        RewrittenYaml(
            source_file=nav2_params, root_key='',
            param_rewrites=bt_param_substitutions('_dwb'), convert_types=True,
        ), accel_linear, accel_angular,
    )

    robot_description = ParameterValue(
        Command(['xacro ', description_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('use_stereo',   default_value='True'),
        DeclareLaunchArgument('use_rviz',     default_value='False'),
        DeclareLaunchArgument('controller_type', default_value='rpp',
            description='Controller for FollowPath A/B testing: "rpp" (default — '
                        'Regulated Pure Pursuit, ported from sim) or "dwb" (original '
                        'DWB, still loaded as FollowPath_DWB in nav2_no_map_params.yaml). '
                        'Selects between BT XML variants that differ only in which '
                        'controller_id the FollowPath action targets.'),
        # Defaults are the CURRENT FIELD-VERIFIED values from
        # nav2_no_map_params.yaml, so passing neither arg changes nothing.
        # Do not read the default as "untested and probably too fast": this
        # already came DOWN from 2.5/3.2 and back UP from 0.2/0.5, which was
        # tried and was worse (stiction — the tracks spend longer below
        # breakaway). Sweep from here in both directions.
        DeclareLaunchArgument('max_accel_linear', default_value='0.4',
            description='velocity_smoother max_accel[0], m/s^2. Ramp rate out of '
                        'a standstill. Lower = gentler start; too low and the '
                        'tracks sit below stiction breakaway for longer.'),
        DeclareLaunchArgument('max_accel_angular', default_value='0.8',
            description='Angular accel, rad/s^2. Applied to BOTH '
                        'velocity_smoother max_accel[2] and RPP '
                        'FollowPath.max_angular_accel — they must stay equal.'),
        DeclareLaunchArgument('teensy_port',  default_value='/dev/ttyTeensy',
            description='Teensy serial device for low_level_command'),
        DeclareLaunchArgument('gps_device',   default_value='/dev/ttyGPS_movingbase',
            description='Position board (big board) F9P serial port'),
        DeclareLaunchArgument('gps_heading_device', default_value='/dev/ttyGPS_rover',
            description='Heading board (small board) F9P serial port'),
        DeclareLaunchArgument('heading_offset_deg', default_value='0.0',
            description='GPS heading antenna mounting offset (deg) — calibrate against '
                        'a known heading once on hardware, see gps_heading_to_imu.py'),
        DeclareLaunchArgument('min_heading_std_deg', default_value='2.0',
            description='Floor on GPS-heading yaw std-dev — realistic sensor-accuracy '
                        'floor; see gps_heading_to_imu.py for why this no longer needs '
                        'to be inflated for feedback-loop damping'),

        # --- Staggered-startup delays (seconds, measured from bringup launch) ---
        # Each stage waits for the one before it to be up before starting, instead of
        # everything racing at t=0. Bump a delay if that stage still errors on startup;
        # shrink it once you've confirmed the stage comes up reliably before it fires.
        DeclareLaunchArgument('sensors_delay', default_value='2.0',
            description='Delay before starting the RealSense driver + IMU filter'),
        DeclareLaunchArgument('gps_delay', default_value='12.0',
            description='Delay before starting the GPS/RTK drivers (after RealSense init)'),
        DeclareLaunchArgument('ntrip_delay', default_value='9.0',
            description='Delay before starting the NTRIP client (needs GPS driver + fix)'),
        DeclareLaunchArgument('motor_delay', default_value='9.0',
            description='Delay before starting low_level_command (independent of GPS/NTRIP)'),
        DeclareLaunchArgument('localization_delay', default_value='12.0',
            description='Delay before starting the dual EKF + navsat_transform'),
        DeclareLaunchArgument('rtabmap_delay', default_value='16.0',
            description='Delay before starting RTAB-Map (needs camera topics flowing)'),
        DeclareLaunchArgument('nav2_delay', default_value='21.0',
            description='Delay before starting Nav2 (needs TF tree + odometry + costmap sources up)'),
        DeclareLaunchArgument('rviz_delay', default_value='22.0',
            description='Delay before starting RViz'),
        # TODO: split a hardware-specific (Gazebo-free) robot description out of the
        # sim package. For now reuse the sim xacro — robot_state_publisher ignores
        # the <gazebo> plugin tags, so the TF tree it publishes is correct.
        DeclareLaunchArgument(
            'description_file',
            default_value=os.path.join(
                get_package_share_directory('dcvg_robot'), 'urdf', 'dcvg_robot.urdf.xacro'),
            description='Robot xacro/urdf for the TF tree'),

        # Starts immediately — everything else keys its delay off this being first.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': use_sim_time}],
        ),

        TimerAction(
            period=LaunchConfiguration('ntrip_delay'),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ntrip.launch.py')),
                launch_arguments={'device': LaunchConfiguration('gps_device')}.items(),
            )],
        ),

        TimerAction(
            period=LaunchConfiguration('gps_delay'),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ublox_gps.launch.py')),
                launch_arguments={
                    'device': LaunchConfiguration('gps_device'),
                    'heading_device': LaunchConfiguration('gps_heading_device'),
                    'heading_offset_deg': LaunchConfiguration('heading_offset_deg'),
                    'min_heading_std_deg': LaunchConfiguration('min_heading_std_deg'),
                }.items(),
            )],
        ),

        TimerAction(
            period=LaunchConfiguration('motor_delay'),
            actions=[Node(
                package='dcvg_bringup',
                executable='low_level_command',
                name='low_level_command',
                output='screen',
                parameters=[{'port': LaunchConfiguration('teensy_port')}],
            )],
        ),

        TimerAction(
            period=LaunchConfiguration('sensors_delay'),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sensors.launch.py')),
            )],
        ),

        TimerAction(
            period=LaunchConfiguration('localization_delay'),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            )],
        ),

        TimerAction(
            period=LaunchConfiguration('rtabmap_delay'),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rtabmap.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time,
                                  'use_stereo': use_stereo}.items(),
            )],
        ),

        TimerAction(
            period=LaunchConfiguration('nav2_delay'),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': configured_params_rpp,
                    'autostart': 'True',
                }.items(),
                condition=IfCondition(is_rpp),
            )],
        ),
        TimerAction(
            period=LaunchConfiguration('nav2_delay'),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': configured_params_dwb,
                    'autostart': 'True',
                }.items(),
                condition=IfCondition(is_dwb),
            )],
        ),

        TimerAction(
            period=LaunchConfiguration('rviz_delay'),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'rviz_launch.py')),
                condition=IfCondition(use_rviz),
            )],
        ),
    ])
