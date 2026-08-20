"""
vo_plotter.py  (REAL HARDWARE — VIO inspection)

Live multi-panel inspection of a visual-odometry stream, for eyeballing how good
the odometry is while you move the camera/robot by hand. Needs NO ground truth —
it just reads an Odometry topic (default /vo) as it arrives and draws:

  - XY trajectory (start dot, current dot, reset markers)
  - Linear velocity (twist.linear.x) vs time
  - Angular velocity (twist.angular.z) vs time
  - Cumulative odometry RESETS vs time — how often VIO tracking jumps/relocalises

The velocity panels matter most on the real robot: once GPS anchors position, the
velocity estimate is what the EKF actually leans on the VIO for. The reset panel
tells you *how often* tracking breaks, so you can correlate spikes with what the
robot was doing (fast turn, low texture, motion blur, sun glare…).

A reset is flagged when EITHER the pose covariance spikes (RTAB-Map sets it to
~9999 on tracking loss) OR the position jumps more than `jump_thresh` metres
between two consecutive samples. Consecutive lost frames count as ONE reset
(rising edge only).

  - Live matplotlib window with all four panels
  - On window close (or Ctrl+C): saves a PNG + CSV of the run

Run (with test_stereo_vio.launch.py already publishing /vo):
  ros2 run dcvg_bringup vo_plotter
  ros2 run dcvg_bringup vo_plotter --ros-args -p topic:=/odometry/local -p label:=ekf
  ros2 run dcvg_bringup vo_plotter --ros-args -p jump_thresh:=0.5 -p cov_reset_thresh:=1000.0

SSH / headless mode (no display):
  ros2 run dcvg_bringup vo_plotter --ros-args -p headless:=true
  Collects data without opening any window; press Ctrl+C to save PNG + CSV and exit.

A clean VIO trace looks smooth and returns near (0,0) when you bring the camera
back to start; jumps/spikes mean tracking loss, big return error means drift, and
a steeply climbing reset count means VIO is unreliable in that condition.
"""

# ═══════════════════════════════════════════════════════════════════════════════
# Appearance — edit here to restyle, no need to touch the class
# ═══════════════════════════════════════════════════════════════════════════════

FIGSIZE      = (14, 8)       # (width, height) inches
FIGURE_DPI   = 150           # saved-PNG resolution
REFRESH_MS   = 200           # live redraw period (ms)

COLOR_PATH   = '#e74c3c'     # trajectory line
COLOR_START  = '#2c3e50'     # start-point dot
COLOR_NOW    = '#2980b9'     # current-position dot
COLOR_RESET  = '#8e44ad'     # reset markers / reset-count line
COLOR_VLIN   = '#27ae60'     # linear-velocity line
COLOR_VANG   = '#e67e22'     # angular-velocity line
COLOR_GRID   = '#cccccc'     # grid lines

LW_PATH      = 1.5           # trajectory linewidth
LW_VEL       = 1.2           # velocity linewidth
MARGIN_M     = 0.5           # padding (m) around the path when autoscaling

# Reset-detection defaults (overridable as ROS params)
JUMP_THRESH      = 0.6       # m, position jump between consecutive samples = reset
COV_RESET_THRESH = 1000.0    # pose covariance[0] above this = tracking lost (RTAB-Map ~9999)

OUTPUT_DIR   = '~/vio_plots'  # saved PNG/CSV directory (created if missing)

# ═══════════════════════════════════════════════════════════════════════════════

import math
import os
import threading
from datetime import datetime

import numpy as np
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
        self.declare_parameter('jump_thresh', JUMP_THRESH)
        self.declare_parameter('cov_reset_thresh', COV_RESET_THRESH)
        self.declare_parameter('headless', False)
        self._topic = str(self.get_parameter('topic').value)
        self._label = str(self.get_parameter('label').value)
        self._output_dir = os.path.expanduser(
            str(self.get_parameter('output_dir').value))
        self._jump_thresh = float(self.get_parameter('jump_thresh').value)
        self._cov_thresh = float(self.get_parameter('cov_reset_thresh').value)
        self._headless = bool(self.get_parameter('headless').value)

        # Shared with the GUI thread — guard every access with the lock.
        self._lock = threading.Lock()
        self._ts: list[float] = []
        self._xs: list[float] = []
        self._ys: list[float] = []
        self._vlin: list[float] = []   # twist.linear.x  (forward velocity)
        self._vlat: list[float] = []   # twist.linear.y  (kept for CSV only)
        self._vang: list[float] = []   # twist.angular.z (yaw rate)
        self._lost: list[bool] = []    # per-sample tracking-loss flag (CSV)
        self._reset_ts: list[float] = []   # time of each reset event (rising edge)
        self._reset_xs: list[float] = []   # where on the path the reset happened
        self._reset_ys: list[float] = []
        self._prev_lost = False
        self._x0: float | None = None   # first sample → normalise path to (0,0)
        self._y0: float | None = None
        self._t0: float | None = None

        self.create_subscription(Odometry, self._topic, self._cb, 50)
        mode = 'headless' if self._headless else 'live window'
        self.get_logger().info(
            f"[vo_plotter] plotting '{self._topic}' (label '{self._label}', {mode}). "
            f"reset if jump>{self._jump_thresh} m or cov>{self._cov_thresh}. "
            f"Ctrl+C to save.")

    def _cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vlin = msg.twist.twist.linear.x
        vlat = msg.twist.twist.linear.y
        vang = msg.twist.twist.angular.z
        cov = msg.pose.covariance[0]

        with self._lock:
            if self._x0 is None:
                self._x0, self._y0, self._t0 = x, y, t
            nx = x - self._x0
            ny = y - self._y0
            nt = t - self._t0

            # ── reset detection ──────────────────────────────────────────────
            lost = cov >= self._cov_thresh
            if self._xs:
                jump = math.hypot(nx - self._xs[-1], ny - self._ys[-1])
                if jump >= self._jump_thresh:
                    lost = True
            # count only the rising edge: a run of lost frames = one reset
            if lost and not self._prev_lost:
                self._reset_ts.append(nt)
                self._reset_xs.append(nx)
                self._reset_ys.append(ny)
            self._prev_lost = lost

            self._xs.append(nx)
            self._ys.append(ny)
            self._ts.append(nt)
            self._vlin.append(vlin)
            self._vlat.append(vlat)
            self._vang.append(vang)
            self._lost.append(lost)

    def snapshot(self):
        """Thread-safe copy of everything collected so far."""
        with self._lock:
            return {
                'ts': list(self._ts),
                'xs': list(self._xs),
                'ys': list(self._ys),
                'vlin': list(self._vlin),
                'vlat': list(self._vlat),
                'vang': list(self._vang),
                'lost': list(self._lost),
                'reset_ts': list(self._reset_ts),
                'reset_xs': list(self._reset_xs),
                'reset_ys': list(self._reset_ys),
            }

    # ── stats ──────────────────────────────────────────────────────────────────

    @staticmethod
    def path_length(xs, ys):
        return sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
                   for i in range(1, len(xs)))

    @staticmethod
    def reset_step(reset_ts, t_end):
        """Build a steps-post (time, cumulative-count) line for the reset panel."""
        if not reset_ts:
            return [0.0, t_end], [0, 0]
        step_t = [0.0] + list(reset_ts) + [t_end]
        step_c = [0] + list(range(1, len(reset_ts) + 1)) + [len(reset_ts)]
        return step_t, step_c

    # ── save on exit ─────────────────────────────────────────────────────────────

    def save(self):
        s = self.snapshot()
        xs, ys, ts = s['xs'], s['ys'], s['ts']
        if len(xs) < 2:
            self.get_logger().warn("Too few samples — nothing to save")
            return
        os.makedirs(self._output_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        length = self.path_length(xs, ys)
        disp = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        n_reset = len(s['reset_ts'])
        duration = ts[-1] - ts[0] if ts else 0.0
        rate = (n_reset / duration * 60.0) if duration > 0 else 0.0  # resets/min

        fig, (ax_traj, ax_vlin, ax_vang, ax_reset) = _make_axes()

        _draw_traj(ax_traj, s, self._label)
        ax_traj.set_title(f'VO trajectory — {self._label}\n'
                          f'samples {len(xs)}  path {length:.2f} m  '
                          f'displacement {disp:.2f} m')

        _draw_velocity(ax_vlin, ts, s['vlin'], 'Linear vx (m/s)', COLOR_VLIN)
        _draw_velocity(ax_vang, ts, s['vang'], 'Angular wz (rad/s)', COLOR_VANG)
        _draw_resets(ax_reset, self.reset_step(s['reset_ts'], ts[-1]),
                     s['reset_ts'], n_reset, rate)

        fig.suptitle(f"VIO inspection — {self._label}", fontsize=12)
        fig.tight_layout(rect=(0, 0, 1, 0.97))

        png = f'{self._output_dir}/vo_route_{self._label}_{stamp}.png'
        fig.savefig(png, dpi=FIGURE_DPI, bbox_inches='tight')
        plt.close(fig)

        csv = f'{self._output_dir}/vo_route_{self._label}_{stamp}.csv'
        with open(csv, 'w') as f:
            f.write('time,x,y,vlin_x,vlin_y,vang_z,reset\n')
            for i in range(len(xs)):
                f.write(f"{ts[i]:.3f},{xs[i]:.4f},{ys[i]:.4f},"
                        f"{s['vlin'][i]:.4f},{s['vlat'][i]:.4f},"
                        f"{s['vang'][i]:.4f},{int(s['lost'][i])}\n")

        self.get_logger().info(
            f"Saved → {png}\n        {csv}\n"
            f"  path {length:.2f} m  displacement {disp:.2f} m  samples {len(xs)}  "
            f"resets {n_reset} ({rate:.1f}/min)")


# ═══════════════════════════════════════════════════════════════════════════════
# Figure helpers — shared by the live window and the saved PNG
# ═══════════════════════════════════════════════════════════════════════════════

def _make_axes():
    """Trajectory on the left (full height), three time-series on the right."""
    fig = plt.figure(figsize=FIGSIZE)
    gs = fig.add_gridspec(3, 2, width_ratios=[1.3, 1.0])
    ax_traj = fig.add_subplot(gs[:, 0])
    ax_vlin = fig.add_subplot(gs[0, 1])
    ax_vang = fig.add_subplot(gs[1, 1], sharex=ax_vlin)
    ax_reset = fig.add_subplot(gs[2, 1], sharex=ax_vlin)

    ax_traj.set_xlabel('X (m)')
    ax_traj.set_ylabel('Y (m)')
    ax_traj.set_aspect('equal')
    ax_traj.grid(True, color=COLOR_GRID, alpha=0.35)

    for ax in (ax_vlin, ax_vang, ax_reset):
        ax.grid(True, color=COLOR_GRID, alpha=0.35)
    ax_reset.set_xlabel('time (s)')
    return fig, (ax_traj, ax_vlin, ax_vang, ax_reset)


def _draw_traj(ax, s, label):
    xs, ys = s['xs'], s['ys']
    ax.plot(xs, ys, color=COLOR_PATH, linewidth=LW_PATH, label=label, zorder=2)
    ax.scatter([xs[0]], [ys[0]], color=COLOR_START, s=80, zorder=5, label='Start')
    ax.scatter([xs[-1]], [ys[-1]], color=COLOR_NOW, s=60, zorder=5, label='Current')
    if s['reset_xs']:
        ax.scatter(s['reset_xs'], s['reset_ys'], color=COLOR_RESET, s=70,
                   marker='x', linewidths=2, zorder=6,
                   label=f"Resets ({len(s['reset_xs'])})")
    ax.legend(loc='upper right')


def _draw_velocity(ax, ts, vs, ylabel, color):
    ax.plot(ts, vs, color=color, linewidth=LW_VEL)
    ax.axhline(0.0, color=COLOR_GRID, linewidth=0.8)
    ax.set_ylabel(ylabel)


def _draw_resets(ax, step, reset_ts, n_reset, rate):
    step_t, step_c = step
    ax.step(step_t, step_c, where='post', color=COLOR_RESET, linewidth=1.4)
    if reset_ts:
        ax.scatter(reset_ts, range(1, len(reset_ts) + 1),
                   color=COLOR_RESET, s=30, zorder=5)
    ax.set_ylabel('cumulative resets')
    ax.set_title(f'resets: {n_reset}  ({rate:.1f}/min)', fontsize=10)
    ax.set_ylim(bottom=0)


def main():
    rclpy.init()
    node = VoPlotter()

    if node._headless:
        # No display available (SSH). Switch to a non-interactive backend so that
        # node.save() can still render and write the PNG without a GUI.
        plt.switch_backend('agg')
        node.get_logger().info('[vo_plotter] headless — collecting data, Ctrl+C to save')
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
        return

    # rclpy spins in a background thread; matplotlib owns the main thread so its
    # GUI event loop stays responsive.
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    fig, (ax_traj, ax_vlin, ax_vang, ax_reset) = _make_axes()

    # Persistent artists, updated in place each frame.
    (line,) = ax_traj.plot([], [], color=COLOR_PATH, linewidth=LW_PATH,
                           label=node._label, zorder=2)
    start = ax_traj.scatter([], [], color=COLOR_START, s=80, zorder=5, label='Start')
    now = ax_traj.scatter([], [], color=COLOR_NOW, s=60, zorder=5, label='Current')
    resets = ax_traj.scatter([], [], color=COLOR_RESET, s=70, marker='x',
                             linewidths=2, zorder=6, label='Resets')
    ax_traj.legend(loc='upper right')

    (vlin_line,) = ax_vlin.plot([], [], color=COLOR_VLIN, linewidth=LW_VEL)
    ax_vlin.axhline(0.0, color=COLOR_GRID, linewidth=0.8)
    ax_vlin.set_ylabel('Linear vx (m/s)')

    (vang_line,) = ax_vang.plot([], [], color=COLOR_VANG, linewidth=LW_VEL)
    ax_vang.axhline(0.0, color=COLOR_GRID, linewidth=0.8)
    ax_vang.set_ylabel('Angular wz (rad/s)')

    (reset_line,) = ax_reset.step([], [], where='post', color=COLOR_RESET,
                                  linewidth=1.4)
    reset_pts = ax_reset.scatter([], [], color=COLOR_RESET, s=30, zorder=5)
    ax_reset.set_ylabel('cumulative resets')
    ax_reset.set_ylim(bottom=0)

    def update(_frame):
        s = node.snapshot()
        xs, ys, ts = s['xs'], s['ys'], s['ts']
        if not xs:
            return line, start, now, resets, vlin_line, vang_line, reset_line, reset_pts

        # ── trajectory ──────────────────────────────────────────────────────
        line.set_data(xs, ys)
        start.set_offsets([[xs[0], ys[0]]])
        now.set_offsets([[xs[-1], ys[-1]]])
        if s['reset_xs']:
            resets.set_offsets(np.column_stack((s['reset_xs'], s['reset_ys'])))
        else:
            resets.set_offsets(np.empty((0, 2)))

        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
        span = max(xmax - xmin, ymax - ymin, 1.0)  # keep equal aspect framing
        cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
        half = span / 2 + MARGIN_M
        ax_traj.set_xlim(cx - half, cx + half)
        ax_traj.set_ylim(cy - half, cy + half)

        length = node.path_length(xs, ys)
        disp = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        n_reset = len(s['reset_ts'])
        ax_traj.set_title(f'VO live ({node._topic})  —  samples {len(xs)}  '
                          f'path {length:.2f} m  disp {disp:.2f} m  '
                          f'resets {n_reset}')

        # ── velocities ──────────────────────────────────────────────────────
        vlin_line.set_data(ts, s['vlin'])
        ax_vlin.relim(); ax_vlin.autoscale_view()
        vang_line.set_data(ts, s['vang'])
        ax_vang.relim(); ax_vang.autoscale_view()

        # ── resets ──────────────────────────────────────────────────────────
        step_t, step_c = node.reset_step(s['reset_ts'], ts[-1])
        reset_line.set_data(step_t, step_c)
        if s['reset_ts']:
            reset_pts.set_offsets(
                np.column_stack((s['reset_ts'], range(1, n_reset + 1))))
        else:
            reset_pts.set_offsets(np.empty((0, 2)))
        ax_reset.set_xlim(0, max(ts[-1], 1.0))
        ax_reset.set_ylim(0, max(n_reset, 1) + 0.5)

        duration = ts[-1] - ts[0] if ts else 0.0
        rate = (n_reset / duration * 60.0) if duration > 0 else 0.0
        ax_reset.set_title(f'resets: {n_reset}  ({rate:.1f}/min)', fontsize=10)

        return line, start, now, resets, vlin_line, vang_line, reset_line, reset_pts

    # Keep a handle so the animation isn't garbage-collected.
    _ani = FuncAnimation(fig, update, interval=REFRESH_MS, cache_frame_data=False)
    fig.tight_layout()

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
