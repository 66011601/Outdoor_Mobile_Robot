import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from ublox_msgs.msg import NavRELPOSNED9

HEADING_SCALE_DEG = 1e-5
REQUIRED_FLAGS = NavRELPOSNED9.FLAGS_GNSS_FIX_OK | NavRELPOSNED9.FLAGS_REL_POS_HEAD_VALID
CARRIER_NAMES = {
    NavRELPOSNED9.FLAGS_CARR_SOLN_NONE: 'NONE (no RTK)',
    NavRELPOSNED9.FLAGS_CARR_SOLN_FLOAT: 'RTK FLOAT',
    NavRELPOSNED9.FLAGS_CARR_SOLN_FIXED: 'RTK FIXED',
}


class GpsHeadingToImu(Node):

    def __init__(self):
        super().__init__('gps_heading_to_imu')
        self.declare_parameter('heading_offset_deg', 0.0)
        self.declare_parameter('min_heading_std_deg', 2.0)
        self.offset_rad = math.radians(float(self.get_parameter('heading_offset_deg').value))
        self.min_std_rad = math.radians(float(self.get_parameter('min_heading_std_deg').value))
        self.last_carrier = None
        self.dropped = 0
        self.pub = self.create_publisher(Imu, '/gps/heading/imu', 10)
        self.create_subscription(NavRELPOSNED9, '/gps/heading/navrelposned', self.on_relposned, 10)

    def on_relposned(self, msg):
        carrier = msg.flags & NavRELPOSNED9.FLAGS_CARR_SOLN_MASK
        if carrier != self.last_carrier:
            self.get_logger().info(f"GPS heading carrier: {CARRIER_NAMES.get(self.last_carrier, 'startup')}"
                                   f" → {CARRIER_NAMES.get(carrier, '?')}")
            self.last_carrier = carrier

        if (msg.flags & REQUIRED_FLAGS) != REQUIRED_FLAGS:
            self.dropped += 1
            self.get_logger().warning(f'GPS heading invalid (flags=0x{msg.flags:x}) — '
                                      f'{self.dropped} dropped', throttle_duration_sec=5.0)
            return

        bearing = math.radians(msg.rel_pos_heading * HEADING_SCALE_DEG)
        yaw = math.pi / 2.0 - bearing + self.offset_rad
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        std = max(self.min_std_rad, math.radians(msg.acc_heading * HEADING_SCALE_DEG))

        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_footprint'
        out.orientation.z = math.sin(yaw / 2.0)
        out.orientation.w = math.cos(yaw / 2.0)
        out.orientation_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, std * std]
        out.angular_velocity_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0
        self.pub.publish(out)


def main():
    rclpy.init()
    node = GpsHeadingToImu()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
