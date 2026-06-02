"""
odom_accuracy_plot.py

Records /odometry/global vs /ground_truth/odom, computes position error,
and saves a PNG plot + CSV on Ctrl+C.

Run twice to compare modes:
  # Mode 1 — GPS + IMU only (comment out /vo in dual_ekf_navsat_params.yaml first)
  ros2 run dcvg_robot odom_accuracy_plot --ros-args -p label:=GPS_IMU_only

  # Mode 2 — GPS + IMU + VIO (uncomment /vo blocks)
  ros2 run dcvg_robot odom_accuracy_plot --ros-args -p label:=GPS_IMU_VIO

Results saved to /tmp/odom_accuracy_<label>_<timestamp>.png / .csv
"""

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
        self._label = str(self.get_parameter('label').value)

        # Raw storage — collected at each topic's own rate
        self._gt:   list[tuple[float, float, float]] = []   # (t, x, y)
        self._est:  list[tuple[float, float, float]] = []

        self._gt_latest  = None
        self._est_latest = None
        self._t0: float | None = None

        # Initial position offset (calibrated from first reading so both
        # trajectories start at 0,0 regardless of Gazebo/map frame offset)
        self._gt_x0  = None
        self._est_x0 = None

        self._gt_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self._gt_cb, 20)
        self._est_sub = self.create_subscription(
            Odometry, '/odometry/global', self._est_cb, 20)

        # Record matched pairs at 10 Hz
        self.create_timer(0.1, self._record_tick)

        self.get_logger().info(
            f"[odom_accuracy_plot] recording '{self._label}' — press Ctrl+C to stop and plot")

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

        # Calibrate initial offset on first sample so both start at (0, 0)
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

        # Interpolate ground truth onto estimate timestamps for error calculation
        gt_xi = np.interp(ts_est, ts_gt, gt_x)
        gt_yi = np.interp(ts_est, ts_gt, gt_y)
        errors = np.sqrt((est_x - gt_xi)**2 + (est_y - gt_yi)**2)

        mean_err = float(np.mean(errors))
        max_err  = float(np.max(errors))
        rmse     = float(np.sqrt(np.mean(errors**2)))

        # ── figure ────────────────────────────────────────────────────────────
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle(f'Odometry Accuracy — {self._label}', fontsize=14, fontweight='bold')

        # Left: 2D trajectory
        ax1.plot(gt_x,  gt_y,  'g-',  linewidth=2.0, label='Ground truth', zorder=3)
        ax1.plot(est_x, est_y, 'r--', linewidth=1.5, label=self._label,    zorder=2)
        ax1.scatter([0], [0], c='black', s=80, zorder=5, label='Start')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('2D Trajectory')
        ax1.legend()
        ax1.grid(True, alpha=0.4)
        ax1.set_aspect('equal')

        # Right: error over time
        ax2.plot(ts_est, errors, 'b-', linewidth=1.5, label='Position error', zorder=3)
        ax2.axhline(mean_err, color='orange', linestyle='--', linewidth=1.5,
                    label=f'Mean  {mean_err:.2f} m', zorder=2)
        ax2.axhline(rmse, color='red', linestyle=':', linewidth=1.5,
                    label=f'RMSE  {rmse:.2f} m', zorder=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Error (m)')
        ax2.set_title('Position Error over Time')
        ax2.legend()
        ax2.grid(True, alpha=0.4)
        ax2.set_ylim(bottom=0)

        plt.tight_layout()

        # ── save ──────────────────────────────────────────────────────────────
        stamp    = datetime.now().strftime('%Y%m%d_%H%M%S')
        png_path = f'/tmp/odom_accuracy_{self._label}_{stamp}.png'
        csv_path = f'/tmp/odom_accuracy_{self._label}_{stamp}.csv'

        plt.savefig(png_path, dpi=150, bbox_inches='tight')

        with open(csv_path, 'w') as f:
            f.write('time,gt_x,gt_y,est_x,est_y,error\n')
            for i in range(len(ts_est)):
                f.write(f"{ts_est[i]:.3f},{gt_xi[i]:.4f},{gt_yi[i]:.4f},"
                        f"{est_x[i]:.4f},{est_y[i]:.4f},{errors[i]:.4f}\n")

        self.get_logger().info(f"Plot  → {png_path}")
        self.get_logger().info(f"CSV   → {csv_path}")
        self.get_logger().info(
            f"Stats → mean {mean_err:.3f} m  RMSE {rmse:.3f} m  "
            f"max {max_err:.3f} m  samples {len(errors)}")


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
