import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomNode(Node):
    WHEEL_RADIUS = 0.2                     # metres
    WHEEL_SEPARATION = 0.8009616629101749  # metres (centre to centre)

    def __init__(self):
        super().__init__('odom_node')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self._prev_left_pos = None
        self._prev_right_pos = None
        self._prev_time = None

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)

    def _joint_cb(self, msg: JointState):
        if ('base_link_to_right_wheel' not in msg.name or
                'base_link_to_left_wheel' not in msg.name):
            return

        names: list[str] = list(msg.name)
        r_idx = names.index('base_link_to_right_wheel')
        l_idx = names.index('base_link_to_left_wheel')

        right_pos: float = msg.position[r_idx]
        left_pos: float = msg.position[l_idx]
        now = self.get_clock().now()

        if self._prev_left_pos is None:
            self._prev_left_pos = left_pos
            self._prev_right_pos = right_pos
            self._prev_time = now
            return

        dt = (now - self._prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        prev_right: float = self._prev_right_pos  # type: ignore[assignment]
        prev_left: float = self._prev_left_pos

        d_right = (right_pos - prev_right) * self.WHEEL_RADIUS
        d_left = (left_pos - prev_left) * self.WHEEL_RADIUS

        d_center = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.WHEEL_SEPARATION

        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        v = d_center / dt
        omega = d_theta / dt

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)

        self._prev_left_pos = left_pos
        self._prev_right_pos = right_pos
        self._prev_time = now


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
