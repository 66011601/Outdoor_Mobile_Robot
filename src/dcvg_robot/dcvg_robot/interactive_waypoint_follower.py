import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped, PoseStamped
from robot_localization.srv import FromLL


class InteractiveGpsWpCommander(Node):
    """
    ROS2 node to send gps waypoints to nav2 received from mapviz's point click publisher
    """

    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")

        cb_group = ReentrantCallbackGroup()
        self.from_ll_client = self.create_client(FromLL, '/fromLL', callback_group=cb_group)
        self.mapviz_wp_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.mapviz_wp_cb, 1, callback_group=cb_group)

        self.navigator.waitUntilNav2Active(localizer='bt_navigator')
        self.get_logger().info("Nav2 active — waiting for GPS waypoints from Mapviz")

    async def mapviz_wp_cb(self, msg: PointStamped):
        """
        Clicked point callback — converts GPS point to map frame and sends to nav2
        """
        if msg.header.frame_id != "wgs84":
            self.get_logger().warning(
                "Received point is not in wgs84 frame — not a GPS point, ignoring")
            return

        if not self.from_ll_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/fromLL service not available — is navsat_transform_node running?')
            return

        request = FromLL.Request()
        request.ll_point.latitude = msg.point.y
        request.ll_point.longitude = msg.point.x
        request.ll_point.altitude = 0.0

        response = await self.from_ll_client.call_async(request)

        if response is None:
            self.get_logger().error('Failed to convert GPS coordinates to map frame')
            return

        map_point = response.map_point

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = map_point.x
        pose.pose.position.y = map_point.y
        pose.pose.position.z = map_point.z
        pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Navigating to map coords: x={map_point.x:.2f}, y={map_point.y:.2f}")
        self.navigator.followWaypoints([pose])
        if self.navigator.isTaskComplete():
            self.get_logger().info("Waypoint completed successfully")


def main():
    rclpy.init()
    gps_wpf = InteractiveGpsWpCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(gps_wpf)
    executor.spin()


if __name__ == "__main__":
    main()
