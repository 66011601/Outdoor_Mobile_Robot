"""
route_recorder.py

Records the robot's path from /ground_truth/odom while you drive manually
with teleop_twist_keyboard (or any cmd_vel source).

A new waypoint is saved every RECORD_DIST_M metres of actual travel so curves
get dense coverage and long straight sections stay compact.

Press Ctrl+C to stop and save the route to ~/VIO_Compare/recorded_routes/.

Usage:
  Terminal 1 (this node):
    ros2 run dcvg_robot route_recorder

  Terminal 2 (keyboard driving):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Output:
  ~/VIO_Compare/recorded_routes/route_<stamp>.yaml

  Load in recorded_route_runner:
    ros2 run dcvg_robot recorded_route_runner \\
      --ros-args -p route_file:=$HOME/VIO_Compare/recorded_routes/route_<stamp>.yaml
"""

import math
import os
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

RECORD_DIST_M = 0.3   # save a waypoint every this many metres of travel
OUTPUT_DIR    = os.path.expanduser('~/VIO_Compare/recorded_routes')

_BE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST, depth=5)


def _yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class RouteRecorder(Node):

    def __init__(self):
        super().__init__('route_recorder')
        Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)

        self._waypoints: list[dict] = []
        self._last_x = self._last_y = None
        self._total_dist = 0.0

        self.create_subscription(
            Odometry, '/ground_truth/odom', self._odom_cb, _BE_QOS)

        self.get_logger().info(
            f'route_recorder ready  (record every {RECORD_DIST_M} m)\n'
            '  Drive the robot with teleop_twist_keyboard.\n'
            '  Press Ctrl+C to stop and save the route.')

    def _odom_cb(self, msg: Odometry):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = _yaw(msg.pose.pose.orientation)

        if self._last_x is None:
            # First message — save starting waypoint
            self._last_x, self._last_y = x, y
            self._waypoints.append({'x': round(x, 4),
                                    'y': round(y, 4),
                                    'yaw': round(yaw, 4)})
            self.get_logger().info('  Start waypoint recorded — drive now')
            return

        dist = math.hypot(x - self._last_x, y - self._last_y)
        if dist >= RECORD_DIST_M:
            self._total_dist += dist
            self._last_x, self._last_y = x, y
            self._waypoints.append({'x': round(x, 4),
                                    'y': round(y, 4),
                                    'yaw': round(yaw, 4)})
            n = len(self._waypoints)
            if n % 10 == 0:
                self.get_logger().info(
                    f'  {n} waypoints  |  {self._total_dist:.1f} m total')

    def save(self):
        if len(self._waypoints) < 2:
            self.get_logger().warn('Too few waypoints — nothing saved')
            return

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path  = os.path.join(OUTPUT_DIR, f'route_{stamp}.yaml')

        data = {
            'stamp':          stamp,
            'total_distance': round(self._total_dist, 2),
            'waypoint_count': len(self._waypoints),
            'record_dist_m':  RECORD_DIST_M,
            'waypoints':      self._waypoints,
        }
        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(
            f'\nRoute saved:\n'
            f'  File       : {path}\n'
            f'  Waypoints  : {len(self._waypoints)}\n'
            f'  Distance   : {self._total_dist:.1f} m\n\n'
            f'  To replay:\n'
            f'    ros2 run dcvg_robot recorded_route_runner \\\n'
            f'      --ros-args -p route_file:={path}')


def main(args=None):
    rclpy.init(args=args)
    node = RouteRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
