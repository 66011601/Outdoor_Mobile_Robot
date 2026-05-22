import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from robot_localization.srv import FromLL
from geometry_msgs.msg import PoseStamped
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time

from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose


class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps


class GpsWpCommander(Node):
    """
    Class to use nav2 waypoint follower to follow a set of waypoints logged in a yaml file
    """

    def __init__(self, wps_file_path):
        super().__init__('gps_wp_commander')
        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = YamlWaypointParser(wps_file_path)
        self.from_ll_client = self.create_client(FromLL, '/fromLL')

    def geopose_to_posestamped(self, geopose):
        """Convert a GeoPose (lat/lon) to PoseStamped in map frame using /fromLL service"""
        if not self.from_ll_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/fromLL service not available — is navsat_transform_node running?')
            return None

        request = FromLL.Request()
        request.ll_point.latitude = geopose.position.latitude
        request.ll_point.longitude = geopose.position.longitude
        request.ll_point.altitude = 0.0

        future = self.from_ll_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('Failed to convert GPS coordinates to map frame')
            return None

        map_point = future.result().map_point

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = map_point.x
        pose.pose.position.y = map_point.y
        pose.pose.position.z = map_point.z
        pose.pose.orientation = geopose.orientation
        return pose

    def start_wpf(self):
        """
        Convert all GPS waypoints to map frame then follow them
        """
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')

        geoposes = self.wp_parser.get_wps()
        poses = []
        for geopose in geoposes:
            pose = self.geopose_to_posestamped(geopose)
            if pose is not None:
                poses.append(pose)

        if not poses:
            self.get_logger().error('No valid waypoints to follow — aborting')
            return

        self.get_logger().info(f"Following {len(poses)} waypoints")
        self.navigator.followWaypoints(poses)
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)
        self.get_logger().info("Waypoints completed successfully")


def main():
    rclpy.init()

    default_yaml_file_path = os.path.join(get_package_share_directory(
        "nav2_gps_waypoint_follower_demo"), "config", "demo_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_wpf = GpsWpCommander(yaml_file_path)
    gps_wpf.start_wpf()


if __name__ == "__main__":
    main()
