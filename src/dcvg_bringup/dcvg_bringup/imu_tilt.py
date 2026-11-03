import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from tf2_ros import Buffer, TransformException, TransformListener


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def roll_pitch(q):
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


class ImuTilt(Node):

    def __init__(self):
        super().__init__('imu_tilt')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pitch_joint', 'base_pitch_joint')
        self.declare_parameter('roll_joint', 'base_roll_joint')
        self.base_frame = self.get_parameter('base_frame').value
        self.joint_names = [self.get_parameter('pitch_joint').value,
                            self.get_parameter('roll_joint').value]
        self.base_to_imu = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(Imu, 'imu/data_throttled', self.on_imu, 10)

    def lookup_base_to_imu(self, imu_frame):
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, imu_frame, Time())
        except TransformException as exc:
            self.get_logger().warning(f'Waiting for {self.base_frame} → {imu_frame}: {exc}',
                                      throttle_duration_sec=5.0)
            return None
        r = tf.transform.rotation
        self.get_logger().info(f'Tilt source: {imu_frame} (rotation in {self.base_frame} cached)')
        return (r.x, r.y, r.z, r.w)

    def on_imu(self, msg):
        if self.base_to_imu is None:
            self.base_to_imu = self.lookup_base_to_imu(msg.header.frame_id)
            if self.base_to_imu is None:
                return
        o = msg.orientation
        world_to_base = quat_multiply((o.x, o.y, o.z, o.w), quat_conjugate(self.base_to_imu))
        roll, pitch = roll_pitch(world_to_base)
        out = JointState()
        out.header.stamp = msg.header.stamp
        out.name = self.joint_names
        out.position = [pitch, roll]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuTilt()
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
