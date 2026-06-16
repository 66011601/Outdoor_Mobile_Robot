"""
rtabmap.launch.py

Launches two RTAB-Map nodes in simulation-time mode:

  1. rgbd_odometry  (rtabmap_odom)  — visual odometry from D455 RGB+Depth
       publishes /vo (nav_msgs/Odometry) → consumed by robot_localization EKF
       publish_tf=false so the EKF owns the odom→base_footprint transform

  2. obstacles_detection (rtabmap_util) — separates ground plane from obstacles
       input:  /camera/depth/points  (PointCloud2 from Gazebo depth sensor)
       output: /camera/depth/obstacles (PointCloud2 without ground)
               → consumed by Nav2 costmap as a second observation source

Launch arguments:
  debug:=true   Opens two rqt_image_view windows:
                  /d455/image_features   — RGB + RTAB-Map keypoints + tracking stats
                  /d455/depth/image_raw  — raw depth (near=bright, far=dark)
                NOTE: /d455/image_features only populates AFTER wait_imu_to_init
                      completes — prerequisite: /imu/data must be flowing.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition  # [CHANGED] added UnlessCondition for use_stereo switching
from launch.substitutions import LaunchConfiguration, PythonExpression  # [CHANGED] added PythonExpression for conditional topic selection
from launch_ros.actions import Node


def generate_launch_description():
    debug      = LaunchConfiguration('debug')
    use_stereo = LaunchConfiguration('use_stereo')  # [ADDED] true → stereo_odometry, false → rgbd_odometry

    return LaunchDescription([

        DeclareLaunchArgument(
            'debug', default_value='true',
            description='Open rqt_image_view windows for RGB, depth, and keypoint tracking'),

        # [ADDED] Switch between rgbd_odometry (default) and stereo_odometry.
        # rgbd_odometry uses D455 RGB+Depth → max range 6 m, sensitive to outdoor IR sunlight.
        # stereo_odometry uses D457 IR left+right pair → passive stereo, up to ~15 m, sunlight-immune.
        # Usage: ros2 launch dcvg_robot rtabmap.launch.py use_stereo:=true
        DeclareLaunchArgument(
            'use_stereo', default_value='false',
            description='true = stereo_odometry (D457 IR pair), false = rgbd_odometry (D455 depth)'),

        # ── Visual-Inertial Odometry (RGBD mode — default, use_stereo:=false) ────────
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='log',
            condition=UnlessCondition(use_stereo),  # [ADDED] disabled when use_stereo:=true
            parameters=[{
                'use_sim_time':              True,
                'frame_id':                  'base_footprint',
                'odom_frame_id':             'odom',
                'publish_tf':                False,   # EKF owns odom→base_footprint
                'subscribe_depth':           True,
                'subscribe_imu':             True,
                'approx_sync':               False,
                'wait_imu_to_init':          True,
                # Frame-to-Frame strategy — more robust for fast outdoor motion
                'Odom/Strategy':             '1',
                # PnP estimator — faster than 3D→3D
                'Vis/EstimationType':        '1',
                'Vis/MaxDepth':              '6.0',
                'Vis/MinDepth':              '0.6',
                'Vis/RoiRatios': "0.0 0.0 0.0 0.10",
                # 'Vis/FeatureType': "8",
                # Lower feature threshold for grass/outdoor textures
                'GFTT/QualityLevel':         '0.001',
                'GFTT/MinDistance':          '10',
                # Publish debug image with keypoints drawn on each frame.
                # View in RViz2: add Image display → /odom_last_frame
                'Vis/CorFlowMaxLevel':        '5',   # pyramid levels for optical flow
                'OdomF2M/BundleAdjustment':  '0',   # disable BA — faster, easier to see raw tracking
                # ── Recovery after tracking loss ──────────────────────────────
                # After this many consecutive lost frames, wipe the local keyframe
                # map and reinitialise from the current camera view.
                # At 15 Hz camera: 15 frames ≈ 1 s of lost tracking before reset.
                # Set to 0 to disable (manual /reset_odom service only).
                'Odom/ResetCountdown':       '15',
                # Keep a larger local map so recovery can match against more
                # keyframes before giving up and triggering a full reset.
                'OdomF2M/MaxSize':           '1000',
            }],
            remappings=[
                ('rgb/image',       '/d455/image_raw'),
                ('depth/image',     '/d455/depth/image_raw'),
                ('rgb/camera_info', '/d455/camera_info'),
                ('imu',             '/imu/data'),
                ('odom',            '/vo'),
            ],
        ),

        # [ADDED] ── Visual-Inertial Odometry (Stereo mode — use_stereo:=true) ──────
        # Uses D457 left+right IR cameras instead of the D455 RGBD sensor.
        # Key advantages over rgbd_odometry for outdoor use:
        #   - Passive stereo: no IR projector → immune to direct sunlight washout
        #   - Effective depth range up to ~15 m (vs 6 m RGBD limit)
        # approx_sync:=true with 10 ms tolerance because the two Gazebo camera plugins
        # fire in the same physics step but may timestamp at slightly different wall times.
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='log',
            condition=IfCondition(use_stereo),  # [ADDED] only runs when use_stereo:=true
            parameters=[{
                'use_sim_time':              True,
                'frame_id':                  'base_footprint',
                'odom_frame_id':             'odom',
                'publish_tf':                False,
                'subscribe_stereo':          True,   # [ADDED] stereo pair input instead of RGBD
                'subscribe_depth':           False,  # [ADDED] no depth topic in stereo mode
                'subscribe_imu':             True,
                # approx_sync MUST be True when subscribe_imu=True: cameras fire at 30 Hz,
                # IMU at 100 Hz — they never share an exact timestamp so ExactSync would drop
                # every message. 50 ms slop ensures every 30 Hz camera frame finds an IMU match.
                'approx_sync':               True,   # [FIXED] was False, caused all IMU msgs to be dropped
                'approx_sync_max_interval':  0.05,   # 50 ms — covers the 33 ms camera period with margin
                'wait_imu_to_init':          True,
                'Odom/Strategy':             '1',
                'Vis/EstimationType':        '1',
                'Vis/MaxDepth':              '15.0', # [ADDED] stereo triangulates further than 6 m RGBD
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
                # [ADDED] Gazebo publishes image_raw; in simulation cameras are ideal so
                # no rectification step is needed — remap image_rect directly to image_raw.
                ('left/image_rect',   '/d457/left/image_raw'),
                ('right/image_rect',  '/d457/right/image_raw'),
                ('left/camera_info',  '/d457/left/camera_info'),
                ('right/camera_info', '/d457/right/camera_info'),
                ('imu',               '/imu/data'),
                ('odom',              '/vo'),
            ],
        ),

        # ── Ground-filtered obstacle cloud ────────────────────────────────
        Node(
            package='rtabmap_util',
            executable='obstacles_detection',
            name='obstacles_detection',
            output='log',
            parameters=[{
                'use_sim_time':             True,
                'frame_id':                 'base_footprint',
                'map_frame_id':             'map',
                'wait_for_transform':       0.2,
                # NormalsSegmentation=true requires surface normals in the point
                # cloud.  Gazebo's depth camera does NOT publish normals, so the
                # RANSAC plane-fit misfires and the entire ground becomes
                # "obstacles" (the dense fan of magenta dots in RViz).
                # Height-based segmentation is deterministic and reliable in
                # simulation: in base_footprint frame (z=0 at ground), ground
                # points from the camera land at z≈0; anything < MaxGroundHeight
                # is stripped before publishing the obstacle cloud.
                'Grid/NormalsSegmentation': 'false',
                'Proj/MaxGroundHeight':     '0.1',   # m above base_footprint — ground threshold
                'Proj/MaxGroundAngle':      '45.0',  # deg — sloped surfaces ≤45° still = ground
                'Proj/MinClusterSize':      '20',    # discard clusters < 20 pts (noise / grass blades)
                'Proj/MaxObstacleHeight':   '2.0',   # m — ignore points above robot height
            }],
            remappings=[
                ('cloud',     '/d455/points'),
                ('obstacles', '/camera/depth/obstacles'),
            ],
        ),

        # ── Keypoint overlay — RGBD mode (use_stereo:=false) ─────────────────
        # Single camera: keypoints drawn on /d455/image_raw, published to /d455/image_features
        # Node(
        #     package='dcvg_robot',
        #     executable='feature_overlay',
        #     name='feature_overlay',
        #     output='log',
        #     condition=UnlessCondition(use_stereo),  # RGBD mode only
        #     parameters=[{'use_sim_time': True}],
        # ),

        # ── Keypoint overlay — Stereo mode (use_stereo:=true) ────────────────
        # Side-by-side: left panel keypoints + right panel epipolar guide lines.
        # Epipolar lines must be perfectly horizontal for a correctly rectified pair.
        # Node(
        #     package='dcvg_robot',
        #     executable='stereo_feature_overlay',
        #     name='stereo_feature_overlay',
        #     output='log',
        #     condition=IfCondition(use_stereo),      # Stereo mode only
        #     parameters=[{'use_sim_time': True}],
        # ),

        # ── Debug viewer (debug:=true) ─────────────────────────────────────
        # RGBD mode: single annotated camera image
        # Node(
        #     package='rqt_image_view',
        #     executable='rqt_image_view',
        #     name='view_features_rgbd',
        #     arguments=['/d455/image_features'],
        #     condition=IfCondition(
        #         PythonExpression(["'", debug, "' == 'true' and '", use_stereo, "' == 'false'"])
        #     ),
        # ),
        # Stereo mode: side-by-side left+right with epipolar guide lines
        # Node(
        #     package='rqt_image_view',
        #     executable='rqt_image_view',
        #     name='view_features_stereo',
        #     arguments=['/d457/stereo_features'],
        #     condition=IfCondition(
        #         PythonExpression(["'", debug, "' == 'true' and '", use_stereo, "' == 'true'"])
        #     ),
        # ),
    ])
