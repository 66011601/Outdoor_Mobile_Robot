"""
replay_vo.launch.py  (OFFLINE VIO TUNING)

Re-runs the VIO stack over a recorded sensor bag so a tune can be evaluated
repeatably, without going back outside. Everything runs on the bag's /clock
(use_sim_time:=True) so RTAB-Map sees the original sensor timestamps.

    robot_state_publisher  → base_footprint → base_link → camera_link
    (the bag's own /tf_static supplies camera_link → *_optical_frame)
    rtabmap.launch.py      → rgbd_odometry | stereo_odometry → /vo
    ros2 bag record        → /vo + /odom_info + the GPS reference
    ros2 bag play          → the recorded sensors; shutdown when it finishes

Run:
  ros2 launch dcvg_bringup replay_vo.launch.py \
      bag:=/home/jetson/vo_gps_20260922_171342 tag:=rgbd_baseline

  ros2 launch dcvg_bringup replay_vo.launch.py \
      bag:=/home/jetson/vo_gps_20260922_171342 use_stereo:=true tag:=stereo_baseline

Then score the result:
  ros2 run dcvg_bringup vo_bag_eval ~/vo_eval/rgbd_baseline

WHY rate DEFAULTS TO 0.5: at 1.0x neither the DDS transport nor the odometry node
keeps up on the Orin, so frames get dropped and the trajectory you score is not
the one the tune would produce — and the drop pattern shifts run to run, which
makes two tunes incomparable. vo_bag_eval reports processed frames and the
effective rate, so you can confirm a run was clean before trusting its numbers.
RGBD moves ~5.6x more bytes per frame than stereo, so it needs the lower rate;
stereo usually survives rate:=1.0.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('dcvg_bringup'), 'launch')

    bag        = LaunchConfiguration('bag')
    rate       = LaunchConfiguration('rate')
    use_stereo = LaunchConfiguration('use_stereo')
    out_dir    = LaunchConfiguration('out_dir')
    tag        = LaunchConfiguration('tag')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('description_file')]), value_type=str)

    # Only stream what the selected mode consumes. This is not an optimisation,
    # it is what makes the replay finish: the bag holds BOTH the RGBD pair
    # (1280x720 rgb8 + 16UC1 = 4.6 MB per frame) and the IR stereo pair
    # (848x480 mono8 = 0.8 MB per frame). Publishing all of it is ~163 MB/s of
    # DDS traffic on a box with ~2 GB free, which thrashes long before the
    # odometry node is the limit. Filtered, stereo needs ~25 MB/s and RGBD
    # ~138 MB/s — hence the different default rates below.
    common_topics = ['/tf_static', '/imu/data', '/gps/fix', '/gps/heading/imu']
    rgbd_topics = ['/camera/color/image_raw',
                   '/camera/aligned_depth_to_color/image_raw',
                   '/camera/color/camera_info']
    stereo_topics = ['/camera/infra1/image_rect_raw', '/camera/infra2/image_rect_raw',
                     '/camera/infra1/camera_info', '/camera/infra2/camera_info']

    def play_proc(topics, condition):
        return ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag,
                 '--clock', '200',
                 '--rate', rate,
                 '--delay', '4',              # let the odometry node subscribe first
                 # The 1000-message default queue is ~4.6 GB of read-ahead for the
                 # RGBD stream. Keep it small enough to stay in RAM.
                 '--read-ahead-queue-size', '30',
                 '--disable-keyboard-controls',
                 '--topics'] + topics + common_topics,
            output='screen',
            condition=condition,
        )

    play_rgbd = play_proc(rgbd_topics, UnlessCondition(use_stereo))
    play_stereo = play_proc(stereo_topics, IfCondition(use_stereo))

    return LaunchDescription([
        DeclareLaunchArgument('bag', description='Path to the recorded sensor bag directory'),
        DeclareLaunchArgument('rate', default_value='0.5',
            description='Playback rate. Keep <1.0 so no camera frames are dropped (see docstring)'),
        DeclareLaunchArgument('use_stereo', default_value='false',
            description='true = stereo_odometry (IR pair), false = rgbd_odometry (depth)'),
        DeclareLaunchArgument('out_dir', default_value=os.path.expanduser('~/vo_eval'),
            description='Where the /vo result bag is written'),
        DeclareLaunchArgument('tag', default_value='run',
            description='Name for this run — becomes <out_dir>/<tag>. Name it after the tune.'),
        DeclareLaunchArgument(
            'description_file',
            default_value=os.path.join(
                get_package_share_directory('dcvg_robot'), 'urdf', 'dcvg_robot.urdf.xacro'),
            description='Robot xacro — supplies base_footprint → camera_link'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='log',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rtabmap.launch.py')),
            launch_arguments={'use_sim_time': 'True',
                              'use_stereo': use_stereo}.items(),
        ),

        # Recording /odom_info is what makes this worth doing offline: it carries
        # the per-frame inliers/features/lost flags that say WHY a tune drifts,
        # which the trajectory alone never tells you. It is only computed by
        # rgbd_odometry when something subscribes, so this recorder is the reason
        # it exists at all.
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 '-o', PathJoinSubstitution([out_dir, tag]),
                 '/vo', '/odom_info', '/gps/fix', '/gps/heading/imu'],
            output='screen',
        ),

        TimerAction(period=2.0, actions=[play_rgbd, play_stereo]),

        RegisterEventHandler(OnProcessExit(
            target_action=play_rgbd,
            on_exit=[TimerAction(period=3.0,          # let the recorder flush
                                 actions=[EmitEvent(event=Shutdown(reason='bag finished'))])],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=play_stereo,
            on_exit=[TimerAction(period=3.0,
                                 actions=[EmitEvent(event=Shutdown(reason='bag finished'))])],
        )),
    ])
