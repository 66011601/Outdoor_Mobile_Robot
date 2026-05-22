import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class InverseKinematics(Node):
    WHEEL_RADIUS = 0.2                     # metres
    WHEEL_SEPARATION = 0.8009616629101749  # metres

    def __init__(self):
        super().__init__('inverse_kinematics')
        self._wheel_cmd_pub = self.create_publisher(Float64MultiArray, '/wheel_cmd', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

    def _cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        omega = msg.angular.z

        # diff-drive inverse kinematics
        omega_right = (2.0 * v + omega * self.WHEEL_SEPARATION) / (2.0 * self.WHEEL_RADIUS)
        omega_left = (2.0 * v - omega * self.WHEEL_SEPARATION) / (2.0 * self.WHEEL_RADIUS)

        cmd = Float64MultiArray()
        cmd.data = [omega_left, omega_right]
        self._wheel_cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
