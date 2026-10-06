from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ODOM_COMMON = {
    'frame_id': 'base_footprint',
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

    return LaunchDescription([
        DeclareLaunchArgument('use_stereo', default_value='false'),

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

        Node(
            package='rtabmap_util',
            executable='obstacles_detection',
            name='obstacles_detection',
            output='log',
            parameters=[{
                'frame_id': 'base_footprint',
                'map_frame_id': 'map',
                'wait_for_transform': 0.2,
                'Grid/NormalsSegmentation': 'true',
                'Grid/FlatObstacleDetected' : 'false',
                'Grid/MaxGroundAngle': '50.0',
                'Grid/NormalK': '40',
                'Grid/ClusterRadius': '0.1',
                'Grid/MinClusterSize': '20',
                'Grid/MaxObstacleHeight': '2.0',
                'Grid/MaxGroundHeight': '0.3',
                'Grid/NoiseFilteringRadius': '0.1',
                'Grid/NoiseFilteringMinNeighbors': '8',
                'Grid/Cellsize': '0.1',
                'Grid/RangeMax': '3.0',
            }],
            remappings=[
                ('cloud', '/camera/depth/color/points'),
                ('obstacles', '/camera/depth/obstacles'),
            ],
        ),
    ])
