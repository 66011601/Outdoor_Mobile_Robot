import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class DCVGController(Node):
    """Hardware abstraction layer for the DCVG diff-drive robot.

    In bringup/teleop mode this node acts as a software-simulated motor
    controller: it integrates the wheel velocity commands into joint
    positions and publishes them so robot_state_publisher and odom can
    visualise the robot in RViz.

    When running on real hardware, replace the timer callback body with
    actual motor driver communication (serial, CAN, etc.) and read back
    real encoder positions instead of integrating commands.
    """

    PUBLISH_RATE_HZ = 50.0

    def __init__(self):
        super().__init__('dcvg_controller')

        self._left_pos = 0.0
        self._right_pos = 0.0
        self._left_omega = 0.0
        self._right_omega = 0.0
        self._prev_time = self.get_clock().now()

        self._joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Float64MultiArray, '/wheel_cmd', self._wheel_cmd_cb, 10)
        self.create_timer(1.0 / self.PUBLISH_RATE_HZ, self._publish_joint_states)

    def _wheel_cmd_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            self._left_omega = msg.data[0]
            self._right_omega = msg.data[1]

    def _publish_joint_states(self):
        now = self.get_clock().now()
        dt = (now - self._prev_time).nanoseconds * 1e-9
        self._prev_time = now

        self._left_pos += self._left_omega * dt
        self._right_pos += self._right_omega * dt

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [
            'base_link_to_right_wheel',
            'base_link_to_left_wheel',
            'base_link_to_right_caster_front',
            'base_link_to_left_caster_front',
        ]
        js.position = [self._right_pos, self._left_pos, 0.0, 0.0]
        js.velocity = [self._right_omega, self._left_omega, 0.0, 0.0]
        js.effort = [0.0, 0.0, 0.0, 0.0]
        self._joint_state_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = DCVGController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
