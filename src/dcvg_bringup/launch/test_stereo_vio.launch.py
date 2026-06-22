"""
test_stereo_vio.launch.py  (REAL HARDWARE — standalone bench test)

Minimal, self-contained test of STEREO visual-inertial odometry from a real
RealSense camera. Currently bench-tested with a D455 over USB; the same graph
applies to the D457 once it is on GMSL (just confirm the topic names below).

Brings up everything needed to see /vo with ONE command:

  1. realsense2_camera_node  — passive IR stereo pair (infra1 + infra2) + IMU
       * emitter OFF  — the IR projector dots are scene-locked texture that
         corrupts feature TRACKING for stereo *odometry* (great for depth, bad
         for VO). Passive stereo on natural texture is what we want here.
       * depth/color OFF — only the IR pair + IMU are needed for VIO.
  2. imu_filter_madgwick     — fuses raw gyro+accel into ORIENTED /imu/data.
       The RealSense IMU publishes angular vel + linear accel but no orientation;
       RTAB-Map needs an orientation quaternion (gravity) to initialise.
  3. stereo_odometry (rtabmap_odom) — publishes /vo (nav_msgs/Odometry).

This is a BENCH test, NOT the full robot bringup, so it differs from the
production dcvg_bringup/rtabmap.launch.py on two points:
  - frame_id := camera_link   (the driver's own frame; there is no
    base_footprint without the robot URDF / robot_state_publisher running)
  - publish_tf := True        (so odom→camera_link shows the camera moving in
    RViz; in production publish_tf=False because the EKF owns that transform)

Run:
  ros2 launch dcvg_bringup test_stereo_vio.launch.py

Verify:
  ros2 topic hz /vo                 # should be ~camera frame rate when moving
  ros2 topic list | grep camera     # confirm the infra/imu topic names
  RViz2: Fixed Frame = odom; add TF + Odometry(/vo). Move the camera by hand
         and the camera_link frame should track the motion.

Override topic names if your driver namespaces differ (e.g. D457):
  ros2 launch dcvg_bringup test_stereo_vio.launch.py \
       left_topic:=/camera/camera/infra1/image_rect_raw ...
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    left_topic       = LaunchConfiguration('left_topic')
    right_topic      = LaunchConfiguration('right_topic')
    left_info_topic  = LaunchConfiguration('left_info_topic')
    right_info_topic = LaunchConfiguration('right_info_topic')
    imu_raw_topic    = LaunchConfiguration('imu_raw_topic')
    frame_id         = LaunchConfiguration('frame_id')

    return LaunchDescription([

        # ── Topic / frame args (defaults = realsense2_camera_node run with
        #    namespace 'camera' + name 'camera' → /camera/camera/...) ─────────
        DeclareLaunchArgument('left_topic',     default_value='/camera/camera/infra1/image_rect_raw'),
        DeclareLaunchArgument('right_topic',    default_value='/camera/camera/infra2/image_rect_raw'),
        DeclareLaunchArgument('left_info_topic',default_value='/camera/camera/infra1/camera_info'),
        DeclareLaunchArgument('right_info_topic',default_value='/camera/camera/infra2/camera_info'),
        DeclareLaunchArgument('imu_raw_topic',  default_value='/camera/camera/imu'),
        DeclareLaunchArgument('frame_id', default_value='camera_link',
            description='TF frame for odometry; camera_link for a standalone '
                        'camera, base_footprint when the robot URDF is running'),

        # ── 1. RealSense driver — IR stereo pair + IMU, emitter off ─────────
        # Run the node directly (not rs_launch.py) so we can set
        # depth_module.emitter_enabled, which rs_launch does not expose.
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            output='screen',
            parameters=[{
                # passive IR stereo pair
                'enable_infra1':                True,
                'enable_infra2':                True,
                'depth_module.infra_profile':   '848x480x30',
                'depth_module.emitter_enabled': 0,      # 0=off → no projector dots
                # not needed for VIO — keep bandwidth/CPU down
                'enable_depth':                 False,
                'enable_color':                 False,
                # IMU: combine gyro+accel into one /camera/camera/imu stream
                'enable_gyro':                  True,
                'enable_accel':                 True,
                'unite_imu_method':             2,      # 2 = linear_interpolation
            }],
        ),

        # ── 2. Madgwick filter — raw IMU → oriented /imu/data ───────────────
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag':       False,   # RealSense has no magnetometer
                'world_frame':   'enu',
                'publish_tf':    False,
            }],
            remappings=[
                ('imu/data_raw', imu_raw_topic),
                ('imu/data',     '/imu/data'),
            ],
        ),

        # ── 3. Stereo visual-inertial odometry → /vo ────────────────────────
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',
            parameters=[{
                'use_sim_time':              False,
                'frame_id':                  frame_id,
                'odom_frame_id':             'odom',
                'publish_tf':                True,    # bench: visualise in RViz
                'subscribe_stereo':          True,
                'subscribe_depth':           False,
                'subscribe_imu':             True,
                # cameras ~30 Hz, IMU ~200 Hz → never share an exact stamp;
                # approx sync with a 50 ms window pairs each frame with an IMU msg
                'approx_sync':               True,
                'approx_sync_max_interval':  0.05,
                'wait_imu_to_init':          True,
                'Odom/Strategy':             '1',     # frame-to-frame (outdoor robust)
                'Vis/EstimationType':        '1',     # PnP
                'Vis/MaxDepth':              '15.0',
                'Vis/MinDepth':              '0.5',
                'Vis/RoiRatios':             '0.0 0.0 0.0 0.10',
                'GFTT/QualityLevel':         '0.001',
                'GFTT/MinDistance':          '10',
                'Vis/CorFlowMaxLevel':       '5',
                'OdomF2M/BundleAdjustment':  '0',
                'Odom/ResetCountdown':       '10',
                'OdomF2M/MaxSize':           '1000',
            }],
            remappings=[
                ('left/image_rect',   left_topic),
                ('right/image_rect',  right_topic),
                ('left/camera_info',  left_info_topic),
                ('right/camera_info', right_info_topic),
                ('imu',               '/imu/data'),
                ('odom',              '/vo'),
            ],
        ),

        Node(
            package='dcvg_bringup',
            executable='stereo_feature_overlay',
            name='stereo_feature_overlay',
            output='log',
        ),

        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='view_features_stereo',
            arguments=['/d457/stereo_features'],
        ),
    ])
