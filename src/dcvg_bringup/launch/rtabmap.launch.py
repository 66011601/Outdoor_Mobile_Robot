from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ODOM_COMMON = {
    'frame_id': 'base_link',
    'odom_frame_id': 'odom',
    'publish_tf': False,
    'subscribe_imu': True,
    'approx_sync': True,
    'wait_imu_to_init': True,
    'Odom/Strategy': '1',
    'Vis/EstimationType': '1',
    'Vis/RoiRatios': '0.0 0.0 0.0 0.10',
    'GFTT/QualityLevel': '0.001',
    'GFTT/MinDistance': '10',
    'Vis/CorFlowMaxLevel': '5',
    'OdomF2M/BundleAdjustment': '0',
    'OdomF2M/MaxSize': '1000',
}


def generate_launch_description():
    use_stereo = LaunchConfiguration('use_stereo')
    max_ground_angle = ParameterValue(LaunchConfiguration('max_ground_angle'), value_type=str)
    max_ground_height = ParameterValue(LaunchConfiguration('max_ground_height'), value_type=str)
    use_terrain_filter = ParameterValue(LaunchConfiguration('use_terrain_filter'), value_type=bool)
    grass_height = ParameterValue(LaunchConfiguration('grass_height'), value_type=float)
    step_height = ParameterValue(LaunchConfiguration('step_height'), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('use_stereo', default_value='false'),
        DeclareLaunchArgument('use_vo', default_value='true'),
        DeclareLaunchArgument('use_obstacles', default_value='true'),
        DeclareLaunchArgument('max_ground_angle', default_value='50.0'),
        DeclareLaunchArgument('max_ground_height', default_value='0.0'),
        DeclareLaunchArgument('use_terrain_filter', default_value='true'),
        DeclareLaunchArgument('grass_height', default_value='0.12'),
        DeclareLaunchArgument('step_height', default_value='0.20'),

        GroupAction(
            condition=IfCondition(LaunchConfiguration('use_vo')),
            actions=[
                Node(
                    package='rtabmap_odom',
                    executable='rgbd_odometry',
                    name='rgbd_odometry',
                    output='log',
                    condition=UnlessCondition(use_stereo),
                    parameters=[{
                        **ODOM_COMMON,
                        'subscribe_depth': True,
                        'Vis/MaxDepth': '6.0',
                        'Vis/MinDepth': '0.6',
                        'Odom/ResetCountdown': '15',
                    }],
                    remappings=[
                        ('rgb/image', '/camera/color/image_raw'),
                        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                        ('rgb/camera_info', '/camera/color/camera_info'),
                        ('imu', '/imu/data'),
                        ('odom', '/vo'),
                    ],
                ),

                Node(
                    package='rtabmap_odom',
                    executable='stereo_odometry',
                    name='stereo_odometry',
                    output='log',
                    condition=IfCondition(use_stereo),
                    parameters=[{
                        **ODOM_COMMON,
                        'subscribe_stereo': True,
                        'subscribe_depth': False,
                        'approx_sync_max_interval': 0.05,
                        'Vis/MaxDepth': '15.0',
                        'Vis/MinDepth': '0.5',
                        'Odom/ResetCountdown': '10',
                    }],
                    remappings=[
                        ('left/image_rect', '/camera/infra1/image_rect_raw'),
                        ('right/image_rect', '/camera/infra2/image_rect_raw'),
                        ('left/camera_info', '/camera/infra1/camera_info'),
                        ('right/camera_info', '/camera/infra2/camera_info'),
                        ('imu', '/imu/data'),
                        ('odom', '/vo'),
                    ],
                ),
            ],
        ),

        GroupAction(
            condition=IfCondition(LaunchConfiguration('use_obstacles')),
            actions=[
                Node(
                    package='rtabmap_util',
                    executable='point_cloud_xyz',
                    name='depth_cloud',
                    output='log',
                    parameters=[{
                        'approx_sync': False,
                        'decimation': 4,
                        'max_depth': 3.0,
                    }],
                    remappings=[
                        ('depth/image', '/camera/depth/image_rect_raw'),
                        ('depth/camera_info', '/camera/depth/camera_info'),
                        ('cloud', '/camera/depth/points_decimated'),
                    ],
                ),

                Node(
                    package='rtabmap_util',
                    executable='obstacles_detection',
                    name='obstacles_detection',
                    output='log',
                    parameters=[{
                        'frame_id': 'base_footprint',
                        'wait_for_transform': 0.2,
                        'Grid/NormalsSegmentation': 'true',
                        'Grid/FlatObstacleDetected' : 'false',
                        'Grid/MaxGroundAngle': max_ground_angle,
                        'Grid/NormalK': '40',
                        'Grid/ClusterRadius': '0.1',
                        'Grid/MinClusterSize': '20',
                        'Grid/MaxObstacleHeight': '2.0',
                        'Grid/MaxGroundHeight': max_ground_height,
                        'Grid/NoiseFilteringRadius': '0.1',
                        'Grid/NoiseFilteringMinNeighbors': '8',
                        'Grid/CellSize': '0.05',
                        'Grid/RangeMax': '3.0',
                    }],
                    remappings=[
                        ('cloud', '/camera/depth/points_decimated'),
                        ('obstacles', '/camera/depth/obstacles_raw'),
                        ('ground', '/camera/depth/ground'),
                    ],
                ),

                Node(
                    package='dcvg_bringup',
                    executable='terrain_filter',
                    name='terrain_filter',
                    output='log',
                    parameters=[{
                        'enabled': use_terrain_filter,
                        'grass_height': grass_height,
                        'step_height': step_height,
                    }],
                    remappings=[
                        ('obstacles_raw', '/camera/depth/obstacles_raw'),
                        ('ground', '/camera/depth/ground'),
                        ('obstacles', '/camera/depth/obstacles'),
                    ],
                ),
            ],
        ),
    ])
