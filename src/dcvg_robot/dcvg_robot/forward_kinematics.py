import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist


class ForwardKinematics(Node):
    WHEEL_RADIUS = 0.2                     # metres
    WHEEL_SEPARATION = 0.8009616629101749  # metres

    def __init__(self):
        super().__init__('forward_kinematics')
        self._robot_vel_pub = self.create_publisher(Twist, '/dcvg/vel', 10)
        # self.create_subscription(Twist, '/dcvg/mootor/speed', self._speed_fb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)

    def _joint_cb(self, msg: JointState):
        if ('base_link_to_right_wheel' not in msg.name or
                'base_link_to_left_wheel' not in msg.name):
            return
        if len(msg.velocity) == 0:
            return

        names: list[str] = list(msg.name)
        r_idx = names.index('base_link_to_right_wheel')
        l_idx = names.index('base_link_to_left_wheel')

        omega_right: float = msg.velocity[r_idx]
        omega_left: float = msg.velocity[l_idx]

        v = self.WHEEL_RADIUS * (omega_right + omega_left) / 2.0
        omega = self.WHEEL_RADIUS * (omega_right - omega_left) / self.WHEEL_SEPARATION

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        self._robot_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
