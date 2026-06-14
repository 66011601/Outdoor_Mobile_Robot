"""
recorded_route_runner.py

Loads a route recorded by route_recorder.py and follows it autonomously
using a pure-pursuit controller with /ground_truth/odom feedback.

While replaying, records both /ground_truth/odom and /vo (VIO odometry)
and saves comparison plots + CSV when the route finishes.

Usage:
  ros2 run dcvg_robot recorded_route_runner \\
    --ros-args -p route_file:=$HOME/VIO_Compare/recorded_routes/route_<stamp>.yaml

Parameters:
  route_file      — path to the route YAML saved by route_recorder
  lookahead_dist  — pure-pursuit lookahead distance in metres  (default 1.0)
  linear_speed    — max forward speed in m/s                   (default 0.5)
  goal_tolerance  — distance to final waypoint to declare done (default 0.5)
"""

import math
import os
from datetime import datetime
from enum import Enum, auto
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

OUTPUT_DIR = os.path.expanduser('~/VIO_Compare')
RECORD_HZ  = 10.0

_BE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST, depth=5)


def _yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _angle_diff(a: float, b: float) -> float:
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


class State(Enum):
    WAITING = auto()
    RUNNING = auto()
    DONE    = auto()


class RecordedRouteRunner(Node):

    def __init__(self):
        super().__init__('recorded_route_runner')

        self.declare_parameter('route_file',     '')
        self.declare_parameter('out_file',     OUTPUT_DIR)
        self.declare_parameter('lookahead_dist',    1.0)
        self.declare_parameter('linear_speed',      0.25)
        self.declare_parameter('goal_tolerance',    0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        
        out_file             = str(self.get_parameter('out_file').value or OUTPUT_DIR)
        route_file             = str(self.get_parameter('route_file').value or '')
        self._lookahead        = float(self.get_parameter('lookahead_dist').value    or 1.0)
        self._linear_speed     = float(self.get_parameter('linear_speed').value      or 0.25)
        self._goal_tol         = float(self.get_parameter('goal_tolerance').value    or 0.5)
        self._max_angular      = float(self.get_parameter('max_angular_speed').value or 1.0)

        if not route_file:
            self.get_logger().error(
                'route_file parameter is required.\n'
                '  --ros-args -p route_file:=/path/to/route.yaml')
            raise SystemExit(1)

        self._waypoints = self._load_route(route_file)
        self._route_file = route_file
        self._out_file = out_file
        self._seg_idx    = 0          # lookahead target index

        self._state  = State.WAITING
        self._gt_msg: Odometry | None = None
        self._vo_msg: Odometry | None = None

        # Offsets — align recorded start to current robot position at runtime
        self._gt_off_x = self._gt_off_y = 0.0
        self._vo_off_x = self._vo_off_y = 0.0
        self._wp_off_x = self._wp_off_y = 0.0   # shift recorded waypoints to current pose

        # Recording
        self._t0: float | None = None
        self._records: list[tuple] = []

        Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self._gt_cb, _BE_QOS)
        self.create_subscription(Odometry, '/vo',                self._vo_cb, _BE_QOS)

        self.create_timer(0.05,          self._ctrl_tick)
        self.create_timer(1.0/RECORD_HZ, self._record_tick)

        self.get_logger().info(
            f'recorded_route_runner\n'
            f'  Route file : {route_file}\n'
            f'  Waypoints  : {len(self._waypoints)}\n'
            f'  Lookahead  : {self._lookahead} m\n'
            '  Waiting for /ground_truth/odom and /vo ...')

    # ── route loading ─────────────────────────────────────────────────────────

    def _load_route(self, path: str) -> list[tuple[float, float]]:
        with open(path) as f:
            data = yaml.safe_load(f)
        wps = data['waypoints']
        self.get_logger().info(
            f"  Loaded {len(wps)} waypoints  "
            f"({data.get('total_distance', '?')} m recorded)")
        return [(float(w['x']), float(w['y'])) for w in wps]

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _gt_cb(self, msg: Odometry): self._gt_msg = msg
    def _vo_cb(self, msg: Odometry): self._vo_msg = msg

    # ── control loop ─────────────────────────────────────────────────────────

    def _ctrl_tick(self):
        if self._state == State.DONE:
            return

        if self._state == State.WAITING:
            if self._gt_msg is not None and self._vo_msg is not None:
                self._init_offsets()
                self._state = State.RUNNING
                self.get_logger().info('Starting replay ...')
            return

        gt = self._gt_msg
        rx = gt.pose.pose.position.x
        ry = gt.pose.pose.position.y
        ryaw = _yaw(gt.pose.pose.orientation)

        # Advance seg_idx past waypoints already within lookahead of robot
        while self._seg_idx < len(self._waypoints) - 1:
            wx, wy = self._shifted_wp(self._seg_idx)
            if math.hypot(rx - wx, ry - wy) < self._lookahead:
                self._seg_idx += 1
            else:
                break

        # Check if we've reached the final waypoint
        fx, fy = self._shifted_wp(-1)
        dist_to_end = math.hypot(rx - fx, ry - fy)
        if dist_to_end <= self._goal_tol:
            self._cmd_pub.publish(Twist())
            self.get_logger().info(
                f'Goal reached  (final dist = {dist_to_end:.2f} m)')
            self._finish()
            return

        # Pure pursuit — steer toward lookahead waypoint
        tx, ty = self._shifted_wp(self._seg_idx)
        heading_err = _angle_diff(math.atan2(ty - ry, tx - rx), ryaw)

        # Speed: full speed ahead, slow to 20% when heading error > 90°
        k_linear  = max(0.2, 1.0 - abs(heading_err) / math.pi)
        linear_x  = self._linear_speed * k_linear
        angular_z = 1.5 * heading_err   # proportional steering

        cmd = Twist()
        cmd.linear.x  = linear_x
        cmd.angular.z = max(-self._max_angular, min(self._max_angular, angular_z))
        self._cmd_pub.publish(cmd)

    def _shifted_wp(self, idx: int) -> tuple[float, float]:
        x, y = self._waypoints[idx]
        return x + self._wp_off_x, y + self._wp_off_y

    def _init_offsets(self):
        gt = self._gt_msg
        vo = self._vo_msg

        # Shift recorded waypoints so the first one aligns with current robot pose
        rec_x0, rec_y0 = self._waypoints[0]
        self._wp_off_x = gt.pose.pose.position.x - rec_x0
        self._wp_off_y = gt.pose.pose.position.y - rec_y0

        # Recording offsets — both trajectories start at (0, 0) in the plots
        self._gt_off_x = gt.pose.pose.position.x
        self._gt_off_y = gt.pose.pose.position.y
        self._vo_off_x = vo.pose.pose.position.x
        self._vo_off_y = vo.pose.pose.position.y
        self._t0 = self.get_clock().now().nanoseconds * 1e-9

    def _finish(self):
        self._state = State.DONE
        self.save_and_plot()
        raise SystemExit(0)

    # ── recording ─────────────────────────────────────────────────────────────

    def _record_tick(self):
        if self._state != State.RUNNING:
            return
        if self._gt_msg is None or self._vo_msg is None or self._t0 is None:
            return

        t  = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        gx = self._gt_msg.pose.pose.position.x - self._gt_off_x
        gy = self._gt_msg.pose.pose.position.y - self._gt_off_y
        vx = self._vo_msg.pose.pose.position.x - self._vo_off_x
        vy = self._vo_msg.pose.pose.position.y - self._vo_off_y
        self._records.append((t, gx, gy, vx, vy))

    # ── output ────────────────────────────────────────────────────────────────

    def save_and_plot(self):
        if len(self._records) < 5:
            self.get_logger().warn('Too few samples — nothing to plot')
            return

        arr    = np.array(self._records)
        ts              = arr[:, 0]
        gt_x, gt_y      = arr[:, 1], arr[:, 2]
        vo_x, vo_y      = arr[:, 3], arr[:, 4]
        errors          = np.sqrt((vo_x - gt_x)**2 + (vo_y - gt_y)**2)

        mean_e = float(np.mean(errors))
        rmse   = float(np.sqrt(np.mean(errors**2)))
        max_e  = float(np.max(errors))

        # Recorded reference path (shifted to origin)
        rec_x = [w[0] - self._waypoints[0][0] for w in self._waypoints]
        rec_y = [w[1] - self._waypoints[0][1] for w in self._waypoints]

        label = 'Recorded'
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        self._plot_route(rec_x, rec_y, gt_x, gt_y, vo_x, vo_y, label, stamp)
        self._plot_error(ts, errors, mean_e, rmse, max_e, label, stamp)
        self._save_csv(ts, gt_x, gt_y, vo_x, vo_y, errors, label, stamp)

        self.get_logger().info(
            f'\n  Replay results:\n'
            f'    Mean error : {mean_e:.3f} m\n'
            f'    RMSE       : {rmse:.3f} m\n'
            f'    Max error  : {max_e:.3f} m\n'
            f'    Samples    : {len(errors)}\n'
            f'    Output dir : {self._out_file}/')

    def _plot_route(self, rec_x, rec_y, gt_x, gt_y, vo_x, vo_y, label, stamp):
        fig, ax = plt.subplots(figsize=(10, 8))
        fig.suptitle(f'VIO Accuracy — {label} Route Replay',
                     fontsize=14, fontweight='bold')
        ax.plot(rec_x, rec_y, color='#95a5a6', lw=1.5, ls=':',
                label='Recorded path (reference)', zorder=1)
        ax.plot(gt_x,  gt_y,  color='#2ecc71', lw=2.5,
                label='Ground truth (replay)', zorder=3)
        ax.plot(vo_x,  vo_y,  color='#e74c3c', lw=2.0, ls='--',
                label='VIO (/vo)', zorder=2)
        ax.scatter([0], [0], color='#2c3e50', s=80, zorder=5, label='Start')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title('2D Trajectory: Recorded vs Ground Truth vs VIO', fontsize=12)
        ax.legend(); ax.grid(True, alpha=0.35); ax.set_aspect('equal')
        plt.tight_layout()
        out = f'{self._out_file}/route_{label}_{stamp}.png'
        fig.savefig(out, dpi=150, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f'  Route plot → {out}')

    def _plot_error(self, ts, errors, mean_e, rmse, max_e, label, stamp):
        fig, ax = plt.subplots(figsize=(10, 6))
        fig.suptitle(f'VIO Position Error — {label}',
                     fontsize=14, fontweight='bold')
        ax.plot(ts, errors, color='#3498db', lw=1.5, label='Position error')
        ax.axhline(mean_e, color='#e67e22', ls='--', lw=1.5,
                   label=f'Mean {mean_e:.2f} m')
        ax.axhline(rmse,   color='#c0392b', ls=':',  lw=1.5,
                   label=f'RMSE {rmse:.2f} m')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Error (m)')
        ax.set_title('Position Error over Time', fontsize=12)
        ax.set_ylim(0, 100.0)
        ax.legend(); ax.grid(True, alpha=0.35)
        plt.tight_layout()
        out = f'{self._out_file}/error_{label}_{stamp}.png'
        fig.savefig(out, dpi=150, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f'  Error plot → {out}')

    def _save_csv(self, ts, gt_x, gt_y, vo_x, vo_y, errors, label, stamp):
        out = f'{self._out_file}/route_{label}_{stamp}.csv'
        with open(out, 'w') as f:
            f.write('time,gt_x,gt_y,vo_x,vo_y,error\n')
            for i in range(len(ts)):
                f.write(f'{ts[i]:.3f},{gt_x[i]:.4f},{gt_y[i]:.4f},'
                        f'{vo_x[i]:.4f},{vo_y[i]:.4f},{errors[i]:.4f}\n')
        self.get_logger().info(f'  CSV        → {out}')


def main(args=None):
    rclpy.init(args=args)
    node = RecordedRouteRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node._state != State.DONE:
            node._cmd_pub.publish(Twist())
            node.save_and_plot()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
