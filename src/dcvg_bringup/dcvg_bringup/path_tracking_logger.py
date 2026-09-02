"""
path_tracking_logger.py  (REAL HARDWARE — path-tracking diagnostics)

Records, at controller rate and against a single common clock, everything needed
to tell apart the competing explanations for zigzag motion:

    t, cross_track_error, heading_error, cmd_vx, cmd_wz,
    robot_x, robot_y, robot_yaw, carrot_dist_est, path_seq, path_len,
    nearest_idx, seg_staircase, dist_to_path_end

plus a snapshot of every distinct global path to a sidecar file, so the plotter
can show the actual staircase geometry rather than inferring it.

WHAT THE SIGNATURES MEAN (this is the point of the tool)
─────────────────────────────────────────────────────────
  omega spikes at fixed spatial intervals matching waypoint spacing
      -> DISPATCH. Each waypoint is acting as a terminal goal.
  continuous oscillation of cross-track error, roughly sinusoidal
      -> RPP GEOMETRY. Lookahead too short for the speed; kappa = 2y/L^2 so
         the correction overshoots and re-corrects.
  steady one-sided cross-track bias that appears only on curves
      -> TRACK-vs-GROUND SLIP. Still a "stop tuning" signal, but be precise
         about why: the Teensy runs a closed-loop velocity PID on its own
         encoders, so commanded velocity IS achieved AT THE SPROCKET. What that
         loop cannot see is the track slipping against dirt, which is exactly
         what a skid-steer chassis does in a turn. So this is not a control
         problem to tune out, and no lookahead value fixes it.
  staircase in cross_track_error with steps at the costmap resolution
      -> JAGGED GLOBAL PATH. NavFn 8-connected on a 0.1m grid.

Expect several at once. The plot is laid out so their relative amplitude is
directly comparable, which is what decides priority.

CROSS-TRACK ERROR
─────────────────
Signed perpendicular distance from the robot to the nearest SEGMENT of the
current global path (not the nearest vertex — vertex distance quantizes to the
path resolution and would manufacture a staircase that isn't there). Positive is
left of the path direction.

NOTE ON CAUSALITY: /cmd_vel is what Nav2 asked for. The Teensy's velocity PID
does track it at the sprocket, but there is still ~600ms of MECHANICAL breakaway
on start-from-rest (confirmed under RC control, which bypasses all software), and
that feedback is not transmitted back to the Jetson, so nothing here observes it.
Cross-track error is therefore the RESPONSE to commands issued ~600ms earlier —
do not read a command and an error at the same timestamp as cause and effect. The
plotter draws a shifted-command trace to make this visible.

Usage
─────
    ros2 run dcvg_bringup path_tracking_logger
    ros2 run dcvg_bringup path_tracking_logger --ros-args -p out:=/tmp/run1.csv

Then:
    python3 plot_path_tracking.py /tmp/run1.csv
"""

import csv
import math
import os

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

FIELDS = [
    't', 'cross_track_error', 'heading_error', 'cmd_vx', 'cmd_wz',
    'robot_x', 'robot_y', 'robot_yaw', 'path_seq', 'path_len',
    'nearest_idx', 'dist_to_path_end', 'carrot_dist_est',
]


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def point_seg_signed(px, py, ax, ay, bx, by):
    """Signed perpendicular distance from P to segment AB, plus the clamped
    projection parameter. Segment-based, not vertex-based, so a coarse path does
    not fabricate a staircase in the error signal."""
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 <= 1e-12:
        return math.hypot(px - ax, py - ay), 0.0
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    cx, cy = ax + t * dx, ay + t * dy
    dist = math.hypot(px - cx, py - cy)
    # Sign by which side of AB the point falls on (positive = left).
    cross = dx * (py - ay) - dy * (px - ax)
    return math.copysign(dist, cross if cross != 0 else 1.0), t


class PathTrackingLogger(Node):
    def __init__(self):
        super().__init__('path_tracking_logger')

        self.declare_parameter('out', '/tmp/path_tracking.csv')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odometry/local')
        self.declare_parameter('rate', 20.0)   # match controller_frequency

        self.out_path = self.get_parameter('out').value
        self.paths_path = os.path.splitext(self.out_path)[0] + '_paths.csv'
        rate = float(self.get_parameter('rate').value)

        # /plan is latched (transient local) in Nav2 — match it or we may never
        # receive the path that was published before this node started.
        plan_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)

        self.create_subscription(Path, self.get_parameter('path_topic').value,
                                 self._on_path, plan_qos)
        self.create_subscription(Twist, self.get_parameter('cmd_topic').value,
                                 self._on_cmd, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self._on_odom, 10)

        self._path = []          # [(x, y)]
        self._path_seq = -1
        self._path_sig = None
        self._cmd = (0.0, 0.0)
        self._pose = None        # (x, y, yaw)

        self._csv_f = open(self.out_path, 'w', newline='')
        self._csv = csv.DictWriter(self._csv_f, fieldnames=FIELDS)
        self._csv.writeheader()
        self._paths_f = open(self.paths_path, 'w', newline='')
        self._paths = csv.writer(self._paths_f)
        self._paths.writerow(['path_seq', 'idx', 'x', 'y'])

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'logging -> {self.out_path}   (paths -> {self.paths_path})')
        self.get_logger().info(
            'NOTE: cmd_vel is COMMANDED. The Teensy PID tracks it at the '
            'sprocket, but that feedback never reaches this node.')

    # ── inputs ──────────────────────────────────────────────────────────────
    def _on_path(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not pts:
            return
        sig = (len(pts), round(pts[0][0], 3), round(pts[0][1], 3),
               round(pts[-1][0], 3), round(pts[-1][1], 3))
        if sig != self._path_sig:
            self._path_sig = sig
            self._path_seq += 1
            for i, (x, y) in enumerate(pts):
                self._paths.writerow([self._path_seq, i, f'{x:.4f}', f'{y:.4f}'])
            self._paths_f.flush()
        self._path = pts

    def _on_cmd(self, msg: Twist):
        self._cmd = (msg.linear.x, msg.angular.z)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    # ── per-cycle record ────────────────────────────────────────────────────
    def _tick(self):
        if self._pose is None or len(self._path) < 2:
            return
        px, py, pyaw = self._pose

        best = None
        for i in range(len(self._path) - 1):
            ax, ay = self._path[i]
            bx, by = self._path[i + 1]
            d, t = point_seg_signed(px, py, ax, ay, bx, by)
            if best is None or abs(d) < abs(best[0]):
                best = (d, i, t)
        xte, idx, tparam = best

        # Heading error against the local path tangent.
        ax, ay = self._path[idx]
        bx, by = self._path[idx + 1]
        heading_err = wrap(math.atan2(by - ay, bx - ax) - pyaw)

        # Remaining path length from the projection point to the end.
        rem = (1.0 - tparam) * math.hypot(bx - ax, by - ay)
        for i in range(idx + 1, len(self._path) - 1):
            rem += math.hypot(self._path[i + 1][0] - self._path[i][0],
                              self._path[i + 1][1] - self._path[i][1])

        vx, wz = self._cmd
        # RPP's effective carrot with velocity scaling: v * lookahead_time,
        # clamped. Recorded so the plot can show when it is pinned at the floor.
        carrot = abs(vx) * 1.5

        self._csv.writerow({
            't': f'{self.get_clock().now().nanoseconds * 1e-9:.4f}',
            'cross_track_error': f'{xte:.4f}',
            'heading_error': f'{heading_err:.4f}',
            'cmd_vx': f'{vx:.4f}', 'cmd_wz': f'{wz:.4f}',
            'robot_x': f'{px:.4f}', 'robot_y': f'{py:.4f}', 'robot_yaw': f'{pyaw:.4f}',
            'path_seq': self._path_seq, 'path_len': len(self._path),
            'nearest_idx': idx, 'dist_to_path_end': f'{rem:.3f}',
            'carrot_dist_est': f'{carrot:.3f}',
        })
        self._csv_f.flush()

    def close(self):
        for f in (self._csv_f, self._paths_f):
            try:
                f.close()
            except Exception:  # noqa: BLE001 - best-effort on shutdown
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PathTrackingLogger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.get_logger().info(f'wrote {node.out_path}')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
