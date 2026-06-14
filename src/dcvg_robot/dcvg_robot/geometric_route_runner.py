"""
geometric_route_runner.py

Drives the robot along a predefined geometric route (straight + turn segments)
using /ground_truth/odom feedback for exact distance and angle measurement.
Simultaneously records ground truth vs /vo (VIO odometry) and saves comparison
plots + CSV when the route completes.

Routes defined in config/vio_test_routes.yaml.

Usage:
  ros2 run dcvg_robot geometric_route_runner --ros-args -p route:=A
  ros2 run dcvg_robot geometric_route_runner --ros-args -p route:=B
  ros2 run dcvg_robot geometric_route_runner --ros-args -p route:=C

Output saved to ~/VIO_Compare/:
  route_RouteA_<stamp>.png  — 2D path: ground truth (green) vs VIO (red)
  error_RouteA_<stamp>.png  — position error over time
  route_RouteA_<stamp>.csv  — raw timestamped data
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
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ── tuning ────────────────────────────────────────────────────────────────────
LINEAR_SPEED  = 0.5    # m/s forward speed during straight segments
ANGULAR_SPEED = 0.3    # rad/s during turn segments
PAUSE_SECS    = 0.5    # stop duration between segments (seconds)
RECORD_HZ     = 10.0   # pose recording rate
OUTPUT_DIR    = os.path.expanduser('~/VIO_Compare')

# ── built-in fallback routes (used when vio_test_routes.yaml is missing) ─────
BUILTIN_ROUTES = {
    'A': [('straight', 20.0), ('turn', -90.0), ('straight', 20.0)],
    'B': [('straight', 10.0), ('turn', -90.0),
          ('straight', 10.0), ('turn', -90.0),
          ('straight', 10.0), ('turn', -90.0),
          ('straight', 10.0)],
    'C': [('straight', 10.0), ('turn',  45.0),
          ('straight', 14.14), ('turn', -90.0),
          ('straight', 14.14), ('turn',  45.0),
          ('straight', 10.0)],
}

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
    WAITING = auto()   # waiting for both /ground_truth/odom and /vo
    RUNNING = auto()   # executing a segment
    PAUSING = auto()   # brief stop between segments
    DONE    = auto()   # route complete


class GeometricRouteRunner(Node):

    def __init__(self):
        super().__init__('geometric_route_runner')

        self.declare_parameter('route', 'A')
        route_name = str(self.get_parameter('route').value).upper()

        self._route      = self._load_route(route_name)
        self._route_name = route_name
        self._seg_idx    = 0

        self._state        = State.WAITING
        self._pause_start  = 0.0
        self._seg_start_x  = 0.0
        self._seg_start_y  = 0.0
        self._seg_start_yaw = 0.0

        self._gt_msg: Odometry | None = None
        self._vo_msg: Odometry | None = None

        # Recording — poses relative to route start
        self._t0: float | None = None
        self._gt_x0 = self._gt_y0 = 0.0
        self._vo_x0 = self._vo_y0 = 0.0
        self._records: list[tuple] = []   # (t, gt_x, gt_y, vo_x, vo_y)

        Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self._gt_cb, _BE_QOS)
        self.create_subscription(Odometry, '/vo',                self._vo_cb, _BE_QOS)

        self.create_timer(0.05,          self._ctrl_tick)
        self.create_timer(1.0/RECORD_HZ, self._record_tick)

        self.get_logger().info(
            f'geometric_route_runner  route={route_name}  '
            f'{len(self._route)} segments\n'
            '  Waiting for /ground_truth/odom and /vo ...\n'
            '  (VIO needs IMU init — this may take a few seconds)')

    # ── route loading ─────────────────────────────────────────────────────────

    def _load_route(self, name: str) -> list[tuple[str, float]]:
        try:
            pkg = get_package_share_directory('dcvg_robot')
            with open(os.path.join(pkg, 'config', 'vio_test_routes.yaml')) as f:
                data = yaml.safe_load(f)
            return [(s['type'], float(s['value'])) for s in data[name]]
        except Exception:
            self.get_logger().warn(
                f'vio_test_routes.yaml not found — using built-in route {name}')
            return BUILTIN_ROUTES.get(name, BUILTIN_ROUTES['A'])

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _gt_cb(self, msg: Odometry): self._gt_msg = msg
    def _vo_cb(self, msg: Odometry): self._vo_msg = msg

    # ── control loop (20 Hz) ─────────────────────────────────────────────────

    def _ctrl_tick(self):
        if self._state == State.DONE:
            return

        if self._state == State.WAITING:
            if self._gt_msg is not None and self._vo_msg is not None:
                self.get_logger().info('Both topics live — starting route')
                self._start_segment()
            return

        if self._state == State.PAUSING:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._pause_start >= PAUSE_SECS:
                self._start_segment()
            return

        # RUNNING
        gt = self._gt_msg
        if gt is None:
            return

        seg_type, seg_val = self._route[self._seg_idx]
        x   = gt.pose.pose.position.x
        y   = gt.pose.pose.position.y
        yaw = _yaw(gt.pose.pose.orientation)
        cmd = Twist()

        if seg_type == 'straight':
            dist = math.hypot(x - self._seg_start_x, y - self._seg_start_y)
            if dist >= seg_val:
                self._finish_segment()
                return
            cmd.linear.x = LINEAR_SPEED

        else:  # turn
            turned     = _angle_diff(yaw, self._seg_start_yaw)
            target_rad = math.radians(seg_val)
            done = (seg_val > 0 and turned >= target_rad - 0.01) or \
                   (seg_val < 0 and turned <= target_rad + 0.01)
            if done:
                self._finish_segment()
                return
            cmd.angular.z = ANGULAR_SPEED if seg_val > 0 else -ANGULAR_SPEED

        self._cmd_pub.publish(cmd)

    def _start_segment(self):
        if self._seg_idx >= len(self._route):
            self._finish_route()
            return

        gt = self._gt_msg
        self._seg_start_x   = gt.pose.pose.position.x
        self._seg_start_y   = gt.pose.pose.position.y
        self._seg_start_yaw = _yaw(gt.pose.pose.orientation)

        if self._t0 is None:
            self._t0   = self.get_clock().now().nanoseconds * 1e-9
            self._gt_x0 = self._seg_start_x
            self._gt_y0 = self._seg_start_y
            vo = self._vo_msg
            self._vo_x0 = vo.pose.pose.position.x
            self._vo_y0 = vo.pose.pose.position.y

        seg_type, seg_val = self._route[self._seg_idx]
        unit = 'm' if seg_type == 'straight' else '°'
        self.get_logger().info(
            f'  [{self._seg_idx+1}/{len(self._route)}] '
            f'{seg_type} {seg_val}{unit}')
        self._state = State.RUNNING

    def _finish_segment(self):
        self._cmd_pub.publish(Twist())   # stop
        self._seg_idx += 1
        if self._seg_idx >= len(self._route):
            self._finish_route()
        else:
            self._state       = State.PAUSING
            self._pause_start = self.get_clock().now().nanoseconds * 1e-9

    def _finish_route(self):
        self._cmd_pub.publish(Twist())
        self._state = State.DONE
        self.get_logger().info('Route complete — saving results ...')
        self.save_and_plot()
        raise SystemExit(0)

    # ── recording (RECORD_HZ) ─────────────────────────────────────────────────

    def _record_tick(self):
        if self._state not in (State.RUNNING, State.PAUSING):
            return
        if self._gt_msg is None or self._vo_msg is None or self._t0 is None:
            return

        t  = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        gx = self._gt_msg.pose.pose.position.x - self._gt_x0
        gy = self._gt_msg.pose.pose.position.y - self._gt_y0
        vx = self._vo_msg.pose.pose.position.x - self._vo_x0
        vy = self._vo_msg.pose.pose.position.y - self._vo_y0
        self._records.append((t, gx, gy, vx, vy))

    # ── output ────────────────────────────────────────────────────────────────

    def save_and_plot(self):
        if len(self._records) < 5:
            self.get_logger().warn('Too few samples — nothing to plot')
            return

        arr    = np.array(self._records)
        ts     = arr[:, 0]
        gt_x, gt_y = arr[:, 1], arr[:, 2]
        vo_x, vo_y = arr[:, 3], arr[:, 4]
        errors = np.sqrt((vo_x - gt_x)**2 + (vo_y - gt_y)**2)

        mean_e = float(np.mean(errors))
        rmse   = float(np.sqrt(np.mean(errors**2)))
        max_e  = float(np.max(errors))
        label  = f'Route{self._route_name}'
        stamp  = datetime.now().strftime('%Y%m%d_%H%M%S')

        self._plot_route(gt_x, gt_y, vo_x, vo_y, label, stamp)
        self._plot_error(ts, errors, mean_e, rmse, max_e, label, stamp)
        self._save_csv(ts, gt_x, gt_y, vo_x, vo_y, errors, label, stamp)

        self.get_logger().info(
            f'\n  Route {self._route_name} results:\n'
            f'    Mean error : {mean_e:.3f} m\n'
            f'    RMSE       : {rmse:.3f} m\n'
            f'    Max error  : {max_e:.3f} m\n'
            f'    Samples    : {len(errors)}\n'
            f'    Output dir : {OUTPUT_DIR}/')

    def _plot_route(self, gt_x, gt_y, vo_x, vo_y, label, stamp):
        fig, ax = plt.subplots(figsize=(10, 8))
        fig.suptitle(f'VIO Accuracy — {label}', fontsize=14, fontweight='bold')
        ax.plot(gt_x, gt_y, color='#2ecc71', lw=2.5, label='Ground truth', zorder=3)
        ax.plot(vo_x, vo_y, color='#e74c3c', lw=2.0, ls='--',
                label='VIO (/vo)', zorder=2)
        ax.scatter([0], [0], color='#2c3e50', s=80, zorder=5, label='Start')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title('2D Trajectory: Ground Truth vs VIO', fontsize=12)
        ax.legend(); ax.grid(True, alpha=0.35); ax.set_aspect('equal')
        plt.tight_layout()
        out = f'{OUTPUT_DIR}/route_{label}_{stamp}.png'
        fig.savefig(out, dpi=150, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f'  Route plot → {out}')

    def _plot_error(self, ts, errors, mean_e, rmse, max_e, label, stamp):
        fig, ax = plt.subplots(figsize=(10, 6))
        fig.suptitle(f'VIO Position Error — {label}', fontsize=14, fontweight='bold')
        ax.plot(ts, errors, color='#3498db', lw=1.5, label='Position error')
        ax.axhline(mean_e, color='#e67e22', ls='--', lw=1.5,
                   label=f'Mean {mean_e:.2f} m')
        ax.axhline(rmse,   color='#c0392b', ls=':',  lw=1.5,
                   label=f'RMSE {rmse:.2f} m')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Error (m)')
        ax.set_title('Position Error over Time', fontsize=12)
        ax.set_ylim(0, max(10.0, max_e * 1.2))
        ax.legend(); ax.grid(True, alpha=0.35)
        plt.tight_layout()
        out = f'{OUTPUT_DIR}/error_{label}_{stamp}.png'
        fig.savefig(out, dpi=150, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f'  Error plot → {out}')

    def _save_csv(self, ts, gt_x, gt_y, vo_x, vo_y, errors, label, stamp):
        out = f'{OUTPUT_DIR}/route_{label}_{stamp}.csv'
        with open(out, 'w') as f:
            f.write('time,gt_x,gt_y,vo_x,vo_y,error\n')
            for i in range(len(ts)):
                f.write(f'{ts[i]:.3f},{gt_x[i]:.4f},{gt_y[i]:.4f},'
                        f'{vo_x[i]:.4f},{vo_y[i]:.4f},{errors[i]:.4f}\n')
        self.get_logger().info(f'  CSV        → {out}')


def main(args=None):
    rclpy.init(args=args)
    node = GeometricRouteRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node._state != State.DONE:
            node._cmd_pub.publish(Twist())   # safety stop
            node.save_and_plot()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
