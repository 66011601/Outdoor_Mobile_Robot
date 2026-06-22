"""
vo_plotter.py  (REAL HARDWARE — VIO inspection)

Live XY-plane plot of a visual-odometry trajectory, for eyeballing how good the
odometry is while you move the camera/robot by hand. Unlike the sim-only
dcvg_robot/odom_accuracy_plot.py, this needs NO ground truth — it just draws the
raw path from an Odometry topic (default /vo) as it arrives.

  - Live matplotlib window: trajectory, start dot, current position, equal aspect
  - Live stats in the title: samples, path length, straight-line displacement
  - On window close (or Ctrl+C): saves a PNG + CSV of the run

Run (with test_stereo_vio.launch.py already publishing /vo):
  ros2 run dcvg_bringup vo_plotter
  ros2 run dcvg_bringup vo_plotter --ros-args -p topic:=/odometry/local -p label:=ekf

A clean VIO trace looks smooth and returns near (0,0) when you bring the camera
back to start; jumps/spikes mean tracking loss, big return error means drift.
"""

# ═══════════════════════════════════════════════════════════════════════════════
# Appearance — edit here to restyle, no need to touch the class
# ═══════════════════════════════════════════════════════════════════════════════

FIGSIZE      = (9, 8)        # (width, height) inches
FIGURE_DPI   = 150           # saved-PNG resolution
REFRESH_MS   = 200           # live redraw period (ms)

COLOR_PATH   = '#e74c3c'     # trajectory line
COLOR_START  = '#2c3e50'     # start-point dot
COLOR_NOW    = '#2980b9'     # current-position dot
COLOR_GRID   = '#cccccc'     # grid lines

LW_PATH      = 1.5           # trajectory linewidth
MARGIN_M     = 0.5           # padding (m) around the path when autoscaling

OUTPUT_DIR   = '~/vio_plots'  # saved PNG/CSV directory (created if missing)

# ═══════════════════════════════════════════════════════════════════════════════

import math
import os
import threading
from datetime import datetime

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class VoPlotter(Node):

    def __init__(self):
        super().__init__('vo_plotter')

        self.declare_parameter('topic', '/vo')
        self.declare_parameter('label', 'vo')
        self.declare_parameter('output_dir', OUTPUT_DIR)
        self._topic = str(self.get_parameter('topic').value)
        self._label = str(self.get_parameter('label').value)
        self._output_dir = os.path.expanduser(
            str(self.get_parameter('output_dir').value))

        # Shared with the GUI thread — guard every access with the lock.
        self._lock = threading.Lock()
        self._ts: list[float] = []
        self._xs: list[float] = []
        self._ys: list[float] = []
        self._x0: float | None = None   # first sample → normalise path to (0,0)
        self._y0: float | None = None
        self._t0: float | None = None

        self.create_subscription(Odometry, self._topic, self._cb, 50)
        self.get_logger().info(
            f"[vo_plotter] plotting '{self._topic}' (label '{self._label}'). "
            f"Close the window or Ctrl+C to save.")

    def _cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            if self._x0 is None:
                self._x0, self._y0, self._t0 = x, y, t
            self._xs.append(x - self._x0)
            self._ys.append(y - self._y0)
            self._ts.append(t - self._t0)

    def snapshot(self):
        """Thread-safe copy of the path so far."""
        with self._lock:
            return list(self._ts), list(self._xs), list(self._ys)

    # ── stats ──────────────────────────────────────────────────────────────────

    @staticmethod
    def path_length(xs, ys):
        return sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
                   for i in range(1, len(xs)))

    # ── save on exit ─────────────────────────────────────────────────────────────

    def save(self):
        ts, xs, ys = self.snapshot()
        if len(xs) < 2:
            self.get_logger().warn("Too few samples — nothing to save")
            return
        os.makedirs(self._output_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        length = self.path_length(xs, ys)
        disp = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])

        fig, ax = plt.subplots(figsize=FIGSIZE)
        ax.plot(xs, ys, color=COLOR_PATH, linewidth=LW_PATH, label=self._label)
        ax.scatter([xs[0]], [ys[0]], color=COLOR_START, s=80, zorder=5, label='Start')
        ax.scatter([xs[-1]], [ys[-1]], color=COLOR_NOW, s=60, zorder=5, label='End')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'VO trajectory — {self._label}\n'
                     f'samples {len(xs)}  path {length:.2f} m  '
                     f'displacement {disp:.2f} m')
        ax.set_aspect('equal')
        ax.grid(True, color=COLOR_GRID, alpha=0.35)
        ax.legend()
        fig.tight_layout()

        png = f'{self._output_dir}/vo_route_{self._label}_{stamp}.png'
        fig.savefig(png, dpi=FIGURE_DPI, bbox_inches='tight')
        plt.close(fig)

        csv = f'{self._output_dir}/vo_route_{self._label}_{stamp}.csv'
        with open(csv, 'w') as f:
            f.write('time,x,y\n')
            for i in range(len(xs)):
                f.write(f'{ts[i]:.3f},{xs[i]:.4f},{ys[i]:.4f}\n')

        self.get_logger().info(
            f"Saved → {png}\n        {csv}\n"
            f"  path {length:.2f} m  displacement {disp:.2f} m  samples {len(xs)}")


def main():
    rclpy.init()
    node = VoPlotter()

    # rclpy spins in a background thread; matplotlib owns the main thread so its
    # GUI event loop stays responsive.
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    fig, ax = plt.subplots(figsize=FIGSIZE)
    (line,) = ax.plot([], [], color=COLOR_PATH, linewidth=LW_PATH, label=node._label)
    start = ax.scatter([], [], color=COLOR_START, s=80, zorder=5, label='Start')
    now = ax.scatter([], [], color=COLOR_NOW, s=60, zorder=5, label='Current')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_aspect('equal')
    ax.grid(True, color=COLOR_GRID, alpha=0.35)
    ax.legend(loc='upper right')

    def update(_frame):
        _, xs, ys = node.snapshot()
        if not xs:
            return line, start, now
        line.set_data(xs, ys)
        start.set_offsets([[xs[0], ys[0]]])
        now.set_offsets([[xs[-1], ys[-1]]])

        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
        span = max(xmax - xmin, ymax - ymin, 1.0)  # keep equal aspect framing
        cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
        half = span / 2 + MARGIN_M
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)

        length = node.path_length(xs, ys)
        disp = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        ax.set_title(f'VO live ({node._topic})  —  samples {len(xs)}  '
                     f'path {length:.2f} m  displacement {disp:.2f} m')
        return line, start, now

    # Keep a handle so the animation isn't garbage-collected.
    _ani = FuncAnimation(fig, update, interval=REFRESH_MS, cache_frame_data=False)

    try:
        plt.show()           # blocks until the window is closed
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
