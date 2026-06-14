#!/usr/bin/env python3
import math

import rclpy
import rclpy.time
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ClearEntireCostmap
from robot_localization.srv import SetPose


class OdomResetNode(Node):
    """
    Re-anchors the local EKF to GPS truth every N metres, then clears the
    local costmap.

    Why this works:
      - Local EKF uses VIO velocity — smooth odom but drifts over distance
      - As VIO drifts, the odom frame shifts → local costmap shifts with it
        → old obstacle marks appear at wrong positions → fake obstacles
      - Fix: every N metres, take GPS position (map frame coordinates) and
        write it DIRECTLY into the local EKF as the new odom-frame position
        (do NOT transform through map→odom — that would give back the same
        drifted position and change nothing)
      - Then clear the local costmap to wipe stale misaligned marks
      - The global EKF + GPS keeps map-frame navigation accurate throughout

    Set one_shot:=true to fire once immediately (for testing).
    """

    def __init__(self):
        super().__init__('odom_reset_node')

        self.declare_parameter('reset_distance', 1000.0)
        self.declare_parameter('one_shot', False)

        self._reset_dist: float = self.get_parameter('reset_distance').value or 1000.0
        self._one_shot: bool = self.get_parameter('one_shot').value or False

        self._dist = 0.0
        self._last_x: float | None = None
        self._last_y: float | None = None
        self._pending = False
        self._global_pose: Odometry | None = None

        self._set_pose_cli = self.create_client(SetPose, '/set_pose')
        self._clear_costmap_cli = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        self.create_subscription(Odometry, 'odometry/global', self._global_cb, 10)

        self._one_shot_timer = None
        if self._one_shot:
            self.get_logger().info('One-shot mode — will reset in 2 s')
            self._one_shot_timer = self.create_timer(2.0, self._one_shot_cb)
        else:
            self.create_subscription(Odometry, 'odometry/local', self._local_cb, 10)
            self.get_logger().info(
                f'Odom reset node ready — resets every {self._reset_dist:.0f} m')

    def _global_cb(self, msg: Odometry) -> None:
        self._global_pose = msg

    def _local_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_x is not None:
            dx = x - self._last_x
            dy = y - self._last_y
            self._dist += math.sqrt(dx * dx + dy * dy)
        self._last_x = x
        self._last_y = y
        if self._dist >= self._reset_dist and not self._pending:
            self._pending = True
            self._trigger_reset()

    def _one_shot_cb(self) -> None:
        if self._one_shot_timer is not None:
            self.destroy_timer(self._one_shot_timer)
            self._one_shot_timer = None
        if not self._pending:
            self._pending = True
            self._trigger_reset()

    def _trigger_reset(self) -> None:
        if self._global_pose is None:
            self.get_logger().warn('No GPS pose yet — skipping reset')
            self._finish()
            return

        if not self._set_pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('set_pose service unavailable — skipping reset')
            self._finish()
            return

        req = SetPose.Request()
        req.pose = PoseWithCovarianceStamped()
        req.pose.header.frame_id = 'odom'
        req.pose.header.stamp = rclpy.time.Time().to_msg()

        # Use GPS coordinates (map frame) DIRECTLY as new odom-frame position.
        # Intentionally NO map→odom transform — transforming through the current
        # map→odom TF would give back the same drifted odom position (the global
        # EKF compensates it), making the reset a no-op.
        # Writing GPS values straight into odom re-anchors the odom frame to
        # GPS truth, and the global EKF's map→odom TF resets toward zero.
        req.pose.pose.pose = self._global_pose.pose.pose

        c = req.pose.pose.covariance
        c[0] = 0.1    # x  (metres²) — GPS-level accuracy
        c[7] = 0.1    # y
        c[14] = 0.1   # z  (ignored in two_d_mode)
        c[21] = 0.01  # roll  (ignored in two_d_mode)
        c[28] = 0.01  # pitch (ignored in two_d_mode)
        c[35] = 0.1   # yaw (rad²)

        future = self._set_pose_cli.call_async(req)
        future.add_done_callback(self._reset_done_cb)

    def _reset_done_cb(self, _) -> None:
        self.get_logger().info('Local EKF re-anchored to GPS position')
        if not self._clear_costmap_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('clear_costmap unavailable — skipping costmap clear')
            self._finish()
            if self._one_shot:
                raise SystemExit
            return
        future = self._clear_costmap_cli.call_async(ClearEntireCostmap.Request())
        future.add_done_callback(self._clear_done_cb)

    def _clear_done_cb(self, _) -> None:
        self.get_logger().info('Local costmap cleared — state is fresh')
        self._finish()
        if self._one_shot:
            self.get_logger().info('One-shot complete — shutting down')
            raise SystemExit

    def _finish(self) -> None:
        self._dist = 0.0
        self._pending = False


def main(args=None):
    rclpy.init(args=args)
    node = OdomResetNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
