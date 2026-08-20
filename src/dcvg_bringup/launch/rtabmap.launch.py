"""
rtabmap.launch.py  (REAL HARDWARE)

RTAB-Map VIO + ground-filtered obstacle cloud — the same nodes proven in sim
(dcvg_robot) but with use_sim_time=False and the camera topics declared as launch
arguments so they can be pointed at the real realsense2_camera outputs.

  rgbd_odometry / stereo_odometry → /vo            (consumed by ekf_local)
  obstacles_detection             → /camera/depth/obstacles  (Nav2 costmap source)

  use_stereo:=true  → stereo_odometry (D457 IR pair, passive, sunlight-immune, ~15 m)
  use_stereo:=false → rgbd_odometry   (D455 RGB+Depth, ≤6 m)            [default]

TOPIC NAMES ARE TBD — the defaults below follow realsense2_camera conventions.
Confirm against the real driver and override at launch, e.g.:
  ros2 launch dcvg_bringup rtabmap.launch.py rgb_topic:=/camera/color/image_raw
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_stereo   = LaunchConfiguration('use_stereo')

    # --- TBD topic names (realsense2_camera defaults; confirm on hardware) ---
    rgb_topic         = LaunchConfiguration('rgb_topic')
    depth_topic       = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    points_topic      = LaunchConfiguration('points_topic')
    imu_topic         = LaunchConfiguration('imu_topic')
    left_topic        = LaunchConfiguration('left_topic')
    right_topic       = LaunchConfiguration('right_topic')
    left_info_topic   = LaunchConfiguration('left_info_topic')
    right_info_topic  = LaunchConfiguration('right_info_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('use_stereo',   default_value='false',
            description='true = stereo_odometry (D457 IR pair), false = rgbd_odometry (D455 depth)'),

        # RGBD-mode topics
        DeclareLaunchArgument('rgb_topic',         default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic',       default_value='/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('points_topic',      default_value='/camera/depth/color/points'),
        DeclareLaunchArgument('imu_topic',         default_value='/imu/data'),
        # Stereo-mode topics
        DeclareLaunchArgument('left_topic',        default_value='/camera/infra1/image_rect_raw'),
        DeclareLaunchArgument('right_topic',       default_value='/camera/infra2/image_rect_raw'),
        DeclareLaunchArgument('left_info_topic',   default_value='/camera/infra1/camera_info'),
        DeclareLaunchArgument('right_info_topic',  default_value='/camera/infra2/camera_info'),

        # ── VIO (RGBD mode — default) ─────────────────────────────────────
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='log',
            condition=UnlessCondition(use_stereo),
            parameters=[{
                'use_sim_time':              use_sim_time,
                'frame_id':                  'base_footprint',
                'odom_frame_id':             'odom',
                'publish_tf':                False,   # EKF owns odom→base_footprint
                'subscribe_depth':           True,
                'subscribe_imu':             True,
                'approx_sync':               True,    # real driver timestamps rarely align exactly
                'wait_imu_to_init':          True,
                'Odom/Strategy':             '1',
                'Vis/EstimationType':        '1',
                'Vis/MaxDepth':              '6.0',
                'Vis/MinDepth':              '0.6',
                'Vis/RoiRatios':             '0.0 0.0 0.0 0.10',
                'GFTT/QualityLevel':         '0.001',
                'GFTT/MinDistance':          '10',
                'Vis/CorFlowMaxLevel':       '5',
                'OdomF2M/BundleAdjustment':  '0',
                'Odom/ResetCountdown':       '15',
                'OdomF2M/MaxSize':           '1000',
            }],
            remappings=[
                ('rgb/image',       rgb_topic),
                ('depth/image',     depth_topic),
                ('rgb/camera_info', camera_info_topic),
                ('imu',             imu_topic),
                ('odom',            '/vo'),
            ],
        ),

        # ── VIO (Stereo mode — use_stereo:=true) ──────────────────────────
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='log',
            condition=IfCondition(use_stereo),
            parameters=[{
                'use_sim_time':              use_sim_time,
                'frame_id':                  'base_footprint',
                'odom_frame_id':             'odom',
                'publish_tf':                False,
                'subscribe_stereo':          True,
                'subscribe_depth':           False,
                'subscribe_imu':             True,
                'approx_sync':               True,
                'approx_sync_max_interval':  0.05,
                'wait_imu_to_init':          True,
                'Odom/Strategy':             '1',
                'Vis/EstimationType':        '1',
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
                ('imu',               imu_topic),
                ('odom',              '/vo'),
            ],
        ),

        # ── Ground-filtered obstacle cloud ────────────────────────────────
        # NOTE: RTAB-Map 0.22 (installed) exposes these under the "Grid/" prefix,
        # NOT "Proj/" — the previous "Proj/*" keys here were unrecognized and
        # silently ignored, which combined with NormalsSegmentation:'false' left
        # Grid/MaxGroundHeight at its disabled default (0.0) i.e. NO ground
        # filtering was actually happening. Fixed to real keys + normals-based
        # segmentation (RTAB-Map's own default, and more robust to the grazing-
        # angle depth noise a level, elevated camera sees on distant ground).
        Node(
            package='rtabmap_util',
            executable='obstacles_detection',
            name='obstacles_detection',
            output='log',
            parameters=[{
                'use_sim_time':                   use_sim_time,
                'frame_id':                       'base_footprint',
                'map_frame_id':                   'map',
                'wait_for_transform':             0.2,
                'Grid/NormalsSegmentation':       'true',
                'Grid/MaxGroundAngle':            '45.0',
                'Grid/NormalK':                   '20',
                'Grid/ClusterRadius':             '0.1',
                'Grid/MinClusterSize':            '20',
                'Grid/MaxObstacleHeight':         '2.0',
                'Grid/MaxGroundHeight':           '0.1',   # fallback only; unused while NormalsSegmentation=true
                'Grid/NoiseFilteringRadius':      '0.1',   # drop sparse speckle points before classification
                'Grid/NoiseFilteringMinNeighbors':'5',
            }],
            remappings=[
                ('cloud',     points_topic),
                ('obstacles', '/camera/depth/obstacles'),
            ],
        ),
    ])
