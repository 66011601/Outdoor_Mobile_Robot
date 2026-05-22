"""
route_waypoint_follower.py

Reads GPS waypoints from waypoints.yaml and follows them one by one.
Listens to /tmp/dcvg_nav_cmd for pause / stop commands written by osm_route_app.

Commands written to the file:
  run   — navigate normally
  pause — cancel current goal and wait in place
  stop  — cancel and exit

Usage:
  ros2 run dcvg_robot route_waypoint_follower
  ros2 run dcvg_robot route_waypoint_follower /path/to/waypoints.yaml
"""

import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from robot_localization.srv import FromLL

from nav2_gps_waypoint_follower_demo.utils.gps_utils import quaternion_from_euler

_NAV_CMD_FILE = '/tmp/dcvg_nav_cmd'


def _read_cmd():
    try:
        with open(_NAV_CMD_FILE) as f:
            return f.read().strip()
    except FileNotFoundError:
        return 'run'


def _write_cmd(cmd):
    with open(_NAV_CMD_FILE, 'w') as f:
        f.write(cmd)


class RouteWaypointFollower(Node):

    def __init__(self, wps_file_path):
        super().__init__('route_waypoint_follower')
        self.navigator = BasicNavigator('basic_navigator')
        self.from_ll_client = self.create_client(FromLL, '/fromLL')

        with open(wps_file_path, 'r') as f:
            data = yaml.safe_load(f)
        self.waypoints = data['waypoints']
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {wps_file_path}")

    def _convert_to_posestamped(self, lat, lon, yaw=0.0):
        if not self.from_ll_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/fromLL service not available — is navsat_transform_node running?')
            return None

        req = FromLL.Request()
        req.ll_point.latitude = lat
        req.ll_point.longitude = lon
        req.ll_point.altitude = 0.0

        future = self.from_ll_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None:
            self.get_logger().error(f'fromLL conversion failed for ({lat:.6f}, {lon:.6f})')
            return None

        mp = result.map_point
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = mp.x
        pose.pose.position.y = mp.y
        pose.pose.position.z = mp.z
        pose.pose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
        return pose

    def start(self):
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')

        self.get_logger().info("Converting GPS waypoints to map frame...")
        poses = []
        for i, wp in enumerate(self.waypoints):
            pose = self._convert_to_posestamped(
                wp['latitude'], wp['longitude'], wp.get('yaw', 0.0))
            if pose is None:
                self.get_logger().warn(f"Skipping waypoint {i + 1} — conversion failed")
                continue
            poses.append(pose)
            self.get_logger().info(
                f"  [{i + 1:>3}/{len(self.waypoints)}] "
                f"GPS ({wp['latitude']:.6f}, {wp['longitude']:.6f}) → "
                f"map ({pose.pose.position.x:.1f}, {pose.pose.position.y:.1f})"
            )

        if not poses:
            self.get_logger().error("No valid waypoints — aborting")
            return

        _write_cmd('run')
        self.get_logger().info(f"Starting navigation: {len(poses)} waypoints")

        i = 0
        while i < len(poses):
            # --- Check command before starting each waypoint ---
            while True:
                cmd = _read_cmd()
                if cmd == 'stop':
                    self.get_logger().info("Navigation stopped.")
                    return
                if cmd == 'pause':
                    rclpy.spin_once(self, timeout_sec=0.2)
                    continue
                break  # cmd == 'run'

            self.get_logger().info(f"→ Waypoint {i + 1}/{len(poses)}")
            self.navigator.goToPose(poses[i])

            # --- Monitor until waypoint reached or command received ---
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.05)
                cmd = _read_cmd()

                if cmd == 'stop':
                    self.navigator.cancelTask()
                    self.get_logger().info("Navigation stopped.")
                    return

                if cmd == 'pause':
                    self.navigator.cancelTask()
                    self.get_logger().info(f"Paused at waypoint {i + 1}")
                    # Wait until resumed or stopped
                    cmd = 'pause'
                    while cmd == 'pause':
                        rclpy.spin_once(self, timeout_sec=0.2)
                        cmd = _read_cmd()
                    if cmd == 'stop':
                        self.get_logger().info("Navigation stopped while paused.")
                        return
                    # Resumed — re-issue the same goal
                    self.get_logger().info(f"Resumed — re-navigating to waypoint {i + 1}")
                    self.navigator.goToPose(poses[i])

            result = self.navigator.getResult()
            self.get_logger().info(f"Waypoint {i + 1}/{len(poses)} complete — result: {result}")
            i += 1

        self.get_logger().info("All waypoints reached!")
        _write_cmd('stopped')


def main():
    rclpy.init()

    default_path = os.path.join(
        get_package_share_directory('dcvg_robot'),
        'config', 'waypoints.yaml')

    wps_file = sys.argv[1] if len(sys.argv) > 1 else default_path

    node = RouteWaypointFollower(wps_file)
    node.start()


if __name__ == '__main__':
    main()
