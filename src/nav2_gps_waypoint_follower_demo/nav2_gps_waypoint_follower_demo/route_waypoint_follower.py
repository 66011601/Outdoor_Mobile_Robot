"""
route_waypoint_follower.py

Reads GPS waypoints from a YAML file, automatically inserts intermediate
waypoints wherever the gap between two consecutive points exceeds a distance
threshold, then follows the full densified route as one continuous navigation.

Usage:
  ros2 run nav2_gps_waypoint_follower_demo route_waypoint_follower
  ros2 run nav2_gps_waypoint_follower_demo route_waypoint_follower /path/to/waypoints.yaml
  ros2 run nav2_gps_waypoint_follower_demo route_waypoint_follower /path/to/waypoints.yaml 30.0

Arguments:
  [1] yaml_file   - Path to GPS waypoints YAML (default: config/demo_waypoints.yaml)
  [2] max_gap_m   - Max allowed distance between consecutive waypoints in meters (default: 50.0)
"""

import math
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from robot_localization.srv import FromLL
from sensor_msgs.msg import NavSatFix

from nav2_gps_waypoint_follower_demo.utils.gps_utils import quaternion_from_euler


# ---------------------------------------------------------------------------
# GPS math helpers
# ---------------------------------------------------------------------------

def haversine_distance(lat1, lon1, lat2, lon2):
    """Distance in metres between two GPS coordinates."""
    R = 6_371_000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def interpolate_wp(wp1, wp2, fraction):
    """Linearly interpolate between two waypoint dicts at the given fraction."""
    lat = wp1['latitude'] + fraction * (wp2['latitude'] - wp1['latitude'])
    lon = wp1['longitude'] + fraction * (wp2['longitude'] - wp1['longitude'])
    yaw = wp1.get('yaw', 0.0) + fraction * (wp2.get('yaw', 0.0) - wp1.get('yaw', 0.0))
    return {'latitude': lat, 'longitude': lon, 'yaw': yaw}


def densify_route(waypoints, max_gap_m):
    """
    Insert intermediate waypoints so no consecutive gap exceeds max_gap_m.
    Returns the densified list and a summary dict.
    """
    dense = [waypoints[0]]
    total_dist = 0.0
    inserted = 0

    for i in range(1, len(waypoints)):
        p1, p2 = waypoints[i - 1], waypoints[i]
        dist = haversine_distance(p1['latitude'], p1['longitude'],
                                  p2['latitude'], p2['longitude'])
        total_dist += dist

        if dist > max_gap_m:
            n_segments = math.ceil(dist / max_gap_m)
            for j in range(1, n_segments):
                dense.append(interpolate_wp(p1, p2, j / n_segments))
                inserted += 1

        dense.append(p2)

    return dense, {'total_m': total_dist, 'inserted': inserted}


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class RouteWaypointFollower(Node):

    def __init__(self, wps_file_path, max_gap_m):
        super().__init__('route_waypoint_follower')
        self.max_gap_m = max_gap_m
        self.navigator = BasicNavigator('basic_navigator')
        self.from_ll_client = self.create_client(FromLL, '/fromLL')
        self.current_gps: NavSatFix | None = None
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self._gps_cb, 10)

        with open(wps_file_path, 'r') as f:
            data = yaml.safe_load(f)
        self.raw_waypoints = data['waypoints']

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status >= 0:  # 0=fix, 1=SBAS, 2=GBAS — all valid
            self.current_gps = msg

    def _wait_for_gps(self, timeout_sec=10.0):
        """Spin until a valid GPS fix arrives or timeout."""
        self.get_logger().info("Waiting for GPS fix to determine robot initial position...")
        start = self.get_clock().now()
        while self.current_gps is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                return False
        return True

    def _convert_to_posestamped(self, lat, lon, yaw=0.0):
        """Call /fromLL to convert GPS → map-frame PoseStamped."""
        if not self.from_ll_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/fromLL service not available — is navsat_transform_node running?')
            return None

        req = FromLL.Request()
        req.ll_point.latitude = lat
        req.ll_point.longitude = lon
        req.ll_point.altitude = 0.0

        future = self.from_ll_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(f'fromLL conversion failed for ({lat:.6f}, {lon:.6f})')
            return None

        mp = future.result().map_point
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

        # --- Prepend robot's current GPS position as first waypoint ---
        if self._wait_for_gps(timeout_sec=10.0):
            initial_wp = {
                'latitude': self.current_gps.latitude,
                'longitude': self.current_gps.longitude,
                'yaw': 0.0
            }
            self.raw_waypoints = [initial_wp] + self.raw_waypoints
            self.get_logger().info(
                f"Robot initial position: ({initial_wp['latitude']:.6f}, "
                f"{initial_wp['longitude']:.6f}) — prepended to route")
        else:
            self.get_logger().warn(
                "No GPS fix received within timeout — starting route from first YAML waypoint")

        # --- Analyse and densify the route ---
        dense_wps, summary = densify_route(self.raw_waypoints, self.max_gap_m)

        self.get_logger().info(
            f"Route analysis: {len(self.raw_waypoints)} original waypoints | "
            f"total distance: {summary['total_m']:.0f} m | "
            f"inserted {summary['inserted']} intermediate points (max gap: {self.max_gap_m} m) | "
            f"{len(dense_wps)} waypoints total"
        )

        # --- Convert all GPS points to map frame ---
        self.get_logger().info("Converting GPS waypoints to map frame...")
        poses = []
        for i, wp in enumerate(dense_wps):
            pose = self._convert_to_posestamped(
                wp['latitude'], wp['longitude'], wp.get('yaw', 0.0))
            if pose is None:
                self.get_logger().warn(f"Skipping waypoint {i + 1} — conversion failed")
                continue
            poses.append(pose)
            self.get_logger().info(
                f"  [{i + 1:>3}/{len(dense_wps)}] "
                f"GPS ({wp['latitude']:.6f}, {wp['longitude']:.6f}) → "
                f"map ({pose.pose.position.x:.1f}, {pose.pose.position.y:.1f})"
            )

        if not poses:
            self.get_logger().error("No valid waypoints to follow — aborting")
            return

        # --- Follow the full route ---
        self.get_logger().info(f"Starting navigation: {len(poses)} waypoints")
        self.navigator.followWaypoints(poses)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                current = feedback.current_waypoint + 1
                pct = 100.0 * current / len(poses)
                self.get_logger().info(
                    f"Progress: waypoint {current}/{len(poses)} ({pct:.0f}%)")
            time.sleep(2.0)

        result = self.navigator.getResult()
        self.get_logger().info(f"Navigation finished — result: {result}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()

    default_path = os.path.join(
        get_package_share_directory('nav2_gps_waypoint_follower_demo'),
        'config', 'demo_waypoints.yaml')

    wps_file = sys.argv[1] if len(sys.argv) > 1 else default_path
    max_gap_m = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

    node = RouteWaypointFollower(wps_file, max_gap_m)
    node.start()


if __name__ == '__main__':
    main()
