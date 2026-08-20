"""
imu_throttle.py  (REAL HARDWARE)

The camera IMU (via imu_filter_madgwick) publishes /imu/data at its raw rate
(~200 Hz), but ekf_filter_node_odom only runs its own predict/publish cycle at
30 Hz. Fusing every single 200 Hz message means each 30 Hz cycle has to process
~6-7 queued IMU corrections back to back — confirmed on hardware to blow the
EKF's own update-rate budget ("Failed to meet update rate!") on the Jetson
while RTAB-Map/Nav2/RealSense are also running. There's no accuracy benefit to
feeding a 30 Hz filter faster than a few times its own rate, so this just
drops messages to a fixed max rate before they reach the EKF.

Not using ros-humble-topic-tools' throttle node for this: no working sudo in
the environment this was written in to install it, and this is small enough
to own directly.

Output: /imu/data_throttled (sensor_msgs/Imu), same message content, capped rate.

Run (normally started by localization.launch.py):
  ros2 run dcvg_bringup imu_throttle
  ros2 run dcvg_bringup imu_throttle --ros-args -p max_rate_hz:=30.0
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

DEFAULT_MAX_RATE_HZ = 50.0


class ImuThrottle(Node):

    def __init__(self):
        super().__init__('imu_throttle')

        self.declare_parameter('max_rate_hz', DEFAULT_MAX_RATE_HZ)
        max_rate_hz = float(self.get_parameter('max_rate_hz').value)
        self._min_period = 1.0 / max_rate_hz

        self._last_pub_time = None
        self._pub = self.create_publisher(Imu, 'imu/data_throttled', 10)
        self.create_subscription(Imu, 'imu/data', self._cb, 10)

        self.get_logger().info(
            f"imu_throttle started (max_rate={max_rate_hz:.1f} Hz)")

    def _cb(self, msg: Imu) -> None:
        now = self.get_clock().now()
        if (self._last_pub_time is not None and
                (now - self._last_pub_time).nanoseconds < self._min_period * 1e9):
            return
        self._last_pub_time = now
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
