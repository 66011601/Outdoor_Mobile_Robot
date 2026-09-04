"""
heading_check.py  (REAL HARDWARE — heading source diagnostics)

Answers one question: is the robot's heading WRONG-AND-GETTING-WORSE (drift /
divergence), or is it NOISY-BUT-CENTRED (jitter)? Those look similar from the
driver's seat and have completely different causes and fixes.

It logs every yaw in the stack against a common clock and compares them:

  gps_yaw     /gps/heading/imu       the ONLY absolute heading source on this
                                     robot (simpleRTK2B moving baseline, two F9P
                                     antennas). No magnetometer exists.
  local_yaw   /odometry/local        ekf_local  (odom frame)
  global_yaw  /odometry/global       ekf_global (map frame)
  tf_yaw      map -> base_footprint  what Nav2 ACTUALLY steers with, i.e. the
                                     composition of both EKFs

The signal that matters is (tf_yaw - gps_yaw) over time:

  slope ~ 0, spread small      -> heading is healthy
  slope ~ 0, spread large      -> NOISE. Heading is centred but jittery. Look at
                                  RTK quality, antenna baseline length, multipath.
  slope clearly non-zero       -> DIVERGENCE. The estimate is walking away from
                                  the only absolute reference. No controller
                                  tuning can fix this: RPP is being fed a heading
                                  that is increasingly wrong, so it will keep
                                  approaching waypoints at the wrong angle no
                                  matter what the lookahead is.
  constant non-zero offset,
  slope ~ 0                    -> heading_offset_deg is uncalibrated. That is a
                                  CONSTANT error, so it cannot explain motion
                                  that starts fine and degrades.

gps_yaw missing entirely      -> gps_heading_to_imu only publishes when
                                 FLAGS_GNSS_FIX_OK and FLAGS_REL_POS_HEAD_VALID
                                 are both set. If it is silent, there is NO
                                 absolute heading at all and everything is gyro
                                 integration — guaranteed drift. Indoors this is
                                 the expected state, which is why this tool needs
                                 to be run OUTSIDE with sky view.

Usage
─────
    ros2 run dcvg_bringup heading_check
    ros2 run dcvg_bringup heading_check --ros-args -p duration:=120.0 \
        -p out:=/tmp/heading.csv

Drive a straight line for the test — the divergence shows up as the run
progresses. Sitting still will not reveal it.
"""

import csv
import math
import statistics

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import Buffer, TransformListener

FIELDS = ['t', 'gps_yaw', 'local_yaw', 'global_yaw', 'tf_yaw',
          'tf_minus_gps', 'local_minus_gps', 'global_minus_gps',
          'gps_age_s', 'fix_status']


def yaw_of(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def deg(a):
    return None if a is None else math.degrees(a)


class HeadingCheck(Node):
    def __init__(self):
        super().__init__('heading_check')
        self.declare_parameter('out', '/tmp/heading_check.csv')
        self.declare_parameter('duration', 0.0)      # 0 = until Ctrl-C
        self.declare_parameter('rate', 10.0)

        self.out_path = self.get_parameter('out').value
        self.duration = float(self.get_parameter('duration').value)

        self._gps = None          # (t, yaw)
        self._local = None
        self._global = None
        self._fix = None

        self.create_subscription(Imu, '/gps/heading/imu', self._on_gps,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/local', self._on_local, 10)
        self.create_subscription(Odometry, '/odometry/global', self._on_global, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self._on_fix,
                                 qos_profile_sensor_data)

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        self._f = open(self.out_path, 'w', newline='')
        self._csv = csv.DictWriter(self._f, fieldnames=FIELDS)
        self._csv.writeheader()
        self._rows = []
        self._t0 = self._now()

        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)
        self.get_logger().info(f'heading_check logging -> {self.out_path}')
        self.get_logger().info(
            'Drive a STRAIGHT LINE outdoors. Divergence only appears over distance.')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_gps(self, m: Imu):
        self._gps = (self._now(), yaw_of(m.orientation))

    def _on_local(self, m: Odometry):
        self._local = yaw_of(m.pose.pose.orientation)

    def _on_global(self, m: Odometry):
        self._global = yaw_of(m.pose.pose.orientation)

    def _on_fix(self, m: NavSatFix):
        self._fix = m.status.status

    def _tf_yaw(self):
        try:
            tr = self._tf_buf.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            return yaw_of(tr.transform.rotation)
        except Exception:  # noqa: BLE001 - TF not ready yet is normal at startup
            return None

    def _tick(self):
        t = self._now() - self._t0
        tf_yaw = self._tf_yaw()
        gps_t, gps_yaw = self._gps if self._gps else (None, None)
        row = {
            't': f'{t:.3f}',
            'gps_yaw': '' if gps_yaw is None else f'{gps_yaw:.5f}',
            'local_yaw': '' if self._local is None else f'{self._local:.5f}',
            'global_yaw': '' if self._global is None else f'{self._global:.5f}',
            'tf_yaw': '' if tf_yaw is None else f'{tf_yaw:.5f}',
            'tf_minus_gps': '', 'local_minus_gps': '', 'global_minus_gps': '',
            'gps_age_s': '' if gps_t is None else f'{self._now()-gps_t:.2f}',
            'fix_status': '' if self._fix is None else self._fix,
        }
        if gps_yaw is not None:
            if tf_yaw is not None:
                row['tf_minus_gps'] = f'{wrap(tf_yaw-gps_yaw):.5f}'
            if self._local is not None:
                row['local_minus_gps'] = f'{wrap(self._local-gps_yaw):.5f}'
            if self._global is not None:
                row['global_minus_gps'] = f'{wrap(self._global-gps_yaw):.5f}'
        self._csv.writerow(row)
        self._f.flush()
        self._rows.append((t, tf_yaw, gps_yaw,
                           None if (tf_yaw is None or gps_yaw is None)
                           else wrap(tf_yaw - gps_yaw)))

        if self.duration > 0 and t >= self.duration:
            self.report()
            raise SystemExit(0)

    # ── verdict ─────────────────────────────────────────────────────────────
    def report(self):
        print('\n' + '=' * 72)
        print(' HEADING CHECK')
        print('=' * 72)
        have_gps = [r for r in self._rows if r[2] is not None]
        if not have_gps:
            print('  /gps/heading/imu NEVER PUBLISHED.')
            print('  gps_heading_to_imu only publishes with FLAGS_GNSS_FIX_OK and')
            print('  FLAGS_REL_POS_HEAD_VALID both set. With no absolute heading,')
            print('  yaw is pure gyro integration and WILL drift without bound.')
            print('  Indoors this is expected — re-run outdoors with sky view.')
            print('=' * 72)
            return

        pairs = [(t, d) for t, _, _, d in self._rows if d is not None]
        if len(pairs) < 20:
            print(f'  only {len(pairs)} paired samples — run longer while driving.')
            print('=' * 72)
            return

        ts = [p[0] for p in pairs]
        ds = [p[1] for p in pairs]
        # least-squares slope of (tf_yaw - gps_yaw) vs time
        mt, md = statistics.mean(ts), statistics.mean(ds)
        num = sum((t - mt) * (d - md) for t, d in pairs)
        den = sum((t - mt) ** 2 for t in ts) or 1e-9
        slope = num / den                      # rad/s
        spread = statistics.pstdev(ds)
        span = ts[-1] - ts[0]

        print(f'  samples {len(pairs)} over {span:.0f}s')
        print(f'  mean offset (tf - gps) : {deg(md):+.1f} deg')
        print(f'  spread (1 sigma)       : {deg(spread):.1f} deg')
        print(f'  DRIFT RATE             : {deg(slope)*60:+.2f} deg/min')
        print(f'  projected over 10 min  : {deg(slope)*600:+.1f} deg')
        print()
        drift_per_min = abs(deg(slope) * 60)
        if drift_per_min > 1.0:
            print('  VERDICT: DIVERGENCE. The fused heading is walking away from the')
            print('    only absolute reference. This is not a tuning problem — RPP is')
            print('    being handed a heading that is increasingly wrong, so it will')
            print('    keep meeting waypoints at the wrong angle whatever the lookahead.')
            print('    Prime suspect: absolute yaw fused into ekf_global (imu1 in')
            print('    ekf_filter_node_map). This config file\'s own header says that')
            print('    was root-caused on hardware as a divergence source and should')
            print('    live in ekf_local only.')
        elif deg(spread) > 5.0:
            print('  VERDICT: NOISE, not drift. Heading is centred but jittery.')
            print('    Look at RTK solution quality, antenna baseline length and')
            print('    multipath — not at the EKF or the controller.')
        elif abs(deg(md)) > 5.0:
            print('  VERDICT: CONSTANT OFFSET. Heading is stable but biased.')
            print('    Calibrate heading_offset_deg (bringup.launch.py arg) — this is')
            print(f'    the antenna mounting offset, currently reading {deg(md):+.1f} deg.')
            print('    NOTE: a constant offset cannot explain motion that starts')
            print('    straight and degrades over the run.')
        else:
            print('  VERDICT: heading looks healthy. Cause of the zigzag is elsewhere.')
        print('=' * 72)

    def close(self):
        try:
            self._f.close()
        except Exception:  # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HeadingCheck()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            try:
                node.report()
            except Exception:  # noqa: BLE001
                pass
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
