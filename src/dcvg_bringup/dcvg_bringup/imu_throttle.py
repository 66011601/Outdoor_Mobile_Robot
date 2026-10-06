import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuThrottle(Node):

    def __init__(self):
        super().__init__('imu_throttle')
        self.declare_parameter('max_rate_hz', 50.0)
        self.min_period_ns = 1e9 / float(self.get_parameter('max_rate_hz').value)
        self.last_pub_time = None
        self.pub = self.create_publisher(Imu, 'imu/data_throttled', 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, 10)

    def on_imu(self, msg):
        now = self.get_clock().now()
        if self.last_pub_time is not None and (now - self.last_pub_time).nanoseconds < self.min_period_ns:
            return
        self.last_pub_time = now
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuThrottle()
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
