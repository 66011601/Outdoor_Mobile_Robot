"""
odom_accuracy_plot.py

Subscribes to /ground_truth/odom (Gazebo) and /odometry/local (robot_localization EKF)
to evaluate localization accuracy. On Ctrl+C, saves two separate PNG files plus a CSV:

  odom_route_<label>_<stamp>.png  — 2D trajectory: ground truth vs /odometry/local
  odom_error_<label>_<stamp>.png  — position error over time  (Y-axis fixed 0–10 m)
  odom_accuracy_<label>_<stamp>.csv

Run twice to compare modes:
  # Mode 1 — GPS + IMU only (comment out /vo in dual_ekf_navsat_params.yaml first)
  ros2 run dcvg_robot odom_accuracy_plot --ros-args -p label:=GPS_IMU_only

  # Mode 2 — GPS + IMU + VIO (uncomment /vo blocks)
  ros2 run dcvg_robot odom_accuracy_plot --ros-args -p label:=GPS_IMU_VIO
"""

# ═══════════════════════════════════════════════════════════════════════════════
# Appearance — edit here to restyle all output, no need to touch the class
# ═══════════════════════════════════════════════════════════════════════════════

ROUTE_FIGSIZE   = (10, 8)       # (width, height) inches — route comparison plot
ERROR_FIGSIZE   = (10, 6)       # (width, height) inches — error over time plot
FIGURE_DPI      = 150           # output PNG resolution

COLOR_GT        = '#2ecc71'     # ground truth trajectory line
COLOR_EST       = '#e74c3c'     # /odometry/local (EKF estimate) line
COLOR_ERROR     = '#3498db'     # error curve
COLOR_MEAN      = '#e67e22'     # mean-error horizontal line
COLOR_RMSE      = '#c0392b'     # RMSE horizontal line
COLOR_START     = '#2c3e50'     # start-point dot (both plots)
COLOR_GRID      = '#cccccc'     # grid line colour

LW_GT           = 1.5           # linewidth — ground truth
LW_EST          = 1.25           # linewidth — estimate
LW_ERROR        = 1.5           # linewidth — error curve
LW_STAT         = 1.5           # linewidth — mean / RMSE reference lines

ERROR_YMIN      = 0.0           # error plot Y-axis lower bound (m)
ERROR_YMAX      = 100.0          # error plot Y-axis upper bound (m) — keep fixed for comparability
GRID_ALPHA      = 0.35          # grid line transparency  (0 = invisible, 1 = solid)

FONT_SUPTITLE   = 14            # figure-level title font size
FONT_AX_TITLE   = 12            # axes subtitle font size

TOPIC_GT        = '/ground_truth/odom'  # Gazebo ground-truth odometry
TOPIC_EST       = '/odometry/local'     # robot_localization EKF output

OUTPUT_DIR      = '/home/thana/VIO_Compare'                # directory for saved files

# ═══════════════════════════════════════════════════════════════════════════════

import math
from datetime import datetime

import matplotlib
matplotlib.use('Agg')  # headless — no display needed
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomAccuracyPlot(Node):

    def __init__(self):
        super().__init__('odom_accuracy_plot')

        self.declare_parameter('label', 'estimate')
        self.declare_parameter('output_dir', OUTPUT_DIR)
        self._label = str(self.get_parameter('label').value)
        self._output_dir = str(self.get_parameter('output_dir').value)

        # Raw storage — collected at each topic's own rate
        self._gt:  list[tuple[float, float, float]] = []   # (t, x, y)
        self._est: list[tuple[float, float, float]] = []

        self._gt_latest  = None
        self._est_latest = None
        self._t0: float | None = None

        # Initial position offset — both trajectories normalised to start at (0, 0)
        self._gt_x0  = None
        self._est_x0 = None

        self._gt_sub = self.create_subscription(
            Odometry, TOPIC_GT, self._gt_cb, 20)
        self._est_sub = self.create_subscription(
            Odometry, TOPIC_EST, self._est_cb, 20)

        # Record matched pairs at 10 Hz
        self.create_timer(0.1, self._record_tick)

        self.get_logger().info(
            f"[odom_accuracy_plot] recording '{self._label}'\n"
            f"  ground truth : {TOPIC_GT}\n"
            f"  estimate     : {TOPIC_EST}\n"
            f"  Press Ctrl+C to stop and save plots.")

    # ── callbacks ────────────────────────────────────────────────────────────

    def _gt_cb(self, msg: Odometry):
        self._gt_latest = msg
        if self._t0 is None:
            self._t0 = self.get_clock().now().nanoseconds * 1e-9

    def _est_cb(self, msg: Odometry):
        self._est_latest = msg

    def _record_tick(self):
        if self._gt_latest is None or self._est_latest is None:
            return

        t = self.get_clock().now().nanoseconds * 1e-9 - (self._t0 or 0.0)

        gx = self._gt_latest.pose.pose.position.x
        gy = self._gt_latest.pose.pose.position.y
        ex = self._est_latest.pose.pose.position.x
        ey = self._est_latest.pose.pose.position.y

        if self._gt_x0 is None:
            self._gt_x0  = (gx, gy)
            self._est_x0 = (ex, ey)

        gx -= self._gt_x0[0];  gy -= self._gt_x0[1]
        ex -= self._est_x0[0]; ey -= self._est_x0[1]

        self._gt.append((t, gx, gy))
        self._est.append((t, ex, ey))

    # ── output ───────────────────────────────────────────────────────────────

    def save_and_plot(self):
        if len(self._gt) < 5:
            self.get_logger().warn("Too few samples — nothing to plot")
            return

        ts_gt  = np.array([r[0] for r in self._gt])
        gt_x   = np.array([r[1] for r in self._gt])
        gt_y   = np.array([r[2] for r in self._gt])
        ts_est = np.array([r[0] for r in self._est])
        est_x  = np.array([r[1] for r in self._est])
        est_y  = np.array([r[2] for r in self._est])

        # Interpolate ground truth onto estimate timestamps for error
        gt_xi  = np.interp(ts_est, ts_gt, gt_x)
        gt_yi  = np.interp(ts_est, ts_gt, gt_y)
        errors = np.sqrt((est_x - gt_xi)**2 + (est_y - gt_yi)**2)

        mean_err = float(np.mean(errors))
        max_err  = float(np.max(errors))
        rmse     = float(np.sqrt(np.mean(errors**2)))

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._save_route_plot(gt_x, gt_y, est_x, est_y, stamp)
        self._save_error_plot(ts_est, errors, mean_err, rmse, stamp)
        self._save_csv(ts_est, gt_xi, gt_yi, est_x, est_y, errors, stamp)

        self.get_logger().info(
            f"Stats → mean {mean_err:.3f} m  RMSE {rmse:.3f} m  "
            f"max {max_err:.3f} m  samples {len(errors)}")

    # ── route comparison ─────────────────────────────────────────────────────

    def _save_route_plot(self, gt_x, gt_y, est_x, est_y, stamp):
        fig, ax = plt.subplots(figsize=ROUTE_FIGSIZE)
        fig.suptitle(
            f'Route Comparison — {self._label}',
            fontsize=FONT_SUPTITLE, fontweight='bold')

        ax.plot(gt_x, gt_y,
                color=COLOR_GT, linewidth=LW_GT,
                alpha=0.5, label='Ground truth', zorder=3)
        ax.plot(est_x, est_y,
                color=COLOR_EST, linewidth=LW_EST, linestyle='--',
                label=f'/odometry/local  ({self._label})', zorder=2)
        ax.scatter([0], [0],
                   color=COLOR_START, s=80, zorder=5, label='Start')

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('2D Trajectory', fontsize=FONT_AX_TITLE)
        ax.legend()
        ax.grid(True, color=COLOR_GRID, alpha=GRID_ALPHA)
        ax.set_aspect('equal')
        plt.tight_layout()

        path = f'{self._output_dir}/odom_route_{self._label}_{stamp}.png'
        fig.savefig(path, dpi=FIGURE_DPI, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f"Route plot → {path}")

    # ── error over time ───────────────────────────────────────────────────────

    def _save_error_plot(self, ts_est, errors, mean_err, rmse, stamp):
        fig, ax = plt.subplots(figsize=ERROR_FIGSIZE)
        fig.suptitle(
            f'Position Error — {self._label}',
            fontsize=FONT_SUPTITLE, fontweight='bold')

        ax.plot(ts_est, errors,
                color=COLOR_ERROR, linewidth=LW_ERROR,
                label='Position error', zorder=3)
        ax.axhline(mean_err,
                   color=COLOR_MEAN, linestyle='--', linewidth=LW_STAT,
                   label=f'Mean  {mean_err:.2f} m', zorder=2)
        ax.axhline(rmse,
                   color=COLOR_RMSE, linestyle=':', linewidth=LW_STAT,
                   label=f'RMSE  {rmse:.2f} m', zorder=2)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error (m)')
        ax.set_title('Position Error over Time', fontsize=FONT_AX_TITLE)
        ax.set_ylim(ERROR_YMIN, ERROR_YMAX)
        ax.legend()
        ax.grid(True, color=COLOR_GRID, alpha=GRID_ALPHA)
        plt.tight_layout()

        path = f'{self._output_dir}/odom_error_{self._label}_{stamp}.png'
        fig.savefig(path, dpi=FIGURE_DPI, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f"Error plot → {path}")

    # ── CSV ──────────────────────────────────────────────────────────────────

    def _save_csv(self, ts_est, gt_xi, gt_yi, est_x, est_y, errors, stamp):
        path = f'{self._output_dir}/odom_accuracy_{self._label}_{stamp}.csv'
        with open(path, 'w') as f:
            f.write('time,gt_x,gt_y,est_x,est_y,error\n')
            for i in range(len(ts_est)):
                f.write(f"{ts_est[i]:.3f},{gt_xi[i]:.4f},{gt_yi[i]:.4f},"
                        f"{est_x[i]:.4f},{est_y[i]:.4f},{errors[i]:.4f}\n")
        self.get_logger().info(f"CSV        → {path}")

def main():
    rclpy.init()
    node = OdomAccuracyPlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_and_plot()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
