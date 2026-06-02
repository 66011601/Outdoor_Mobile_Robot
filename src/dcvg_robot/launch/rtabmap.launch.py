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
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── Visual-Inertial Odometry ────────────────────────────────────────
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='log',
            parameters=[{
                'use_sim_time':              True,
                'frame_id':                  'base_footprint',
                'odom_frame_id':             'odom',
                'publish_tf':                False,   # EKF owns odom→base_footprint
                'subscribe_depth':           True,
                'subscribe_imu':             True,
                'approx_sync':               True,
                'approx_sync_max_interval':  0.1,
                'wait_imu_to_init':          True,
                # Frame-to-Frame strategy — more robust for fast outdoor motion
                'Odom/Strategy':             '1',
                # PnP estimator — faster than 3D→3D
                'Vis/EstimationType':        '1',
                'Vis/MaxDepth':              '6.0',
                'Vis/MinDepth':              '0.6',
                # Lower feature threshold for grass/outdoor textures
                'GFTT/QualityLevel':         '0.001',
                'GFTT/MinDistance':          '10',
            }],
            remappings=[
                ('rgb/image',       '/d455/image_raw'),
                ('depth/image',     '/d455/depth/image_raw'),
                ('rgb/camera_info', '/d455/camera_info'),
                ('imu',             '/imu/data'),
                ('odom',            '/vo'),
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
    ])
