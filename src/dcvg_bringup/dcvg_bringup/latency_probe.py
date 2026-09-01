"""
latency_probe.py  (REAL HARDWARE — actuation dead-time / ramp measurement)

Measures how long the tracks take to actually respond to a velocity command, by
publishing a STEP cmd_vel (bypassing Nav2) and timestamping real motion.
Reports, separately:

  START dead time  command published -> first real acceleration
  START ramp time  first motion      -> 90% of commanded speed
  STOP  dead time  zero commanded    -> speed begins dropping
  STOP  time       zero commanded    -> effectively stopped (<10% of plateau)

Stop latency matters more than start latency for collision avoidance: it is the
blind distance travelled after a stop command.

ONSET DETECTION — read this before changing it
───────────────────────────────────────────────
An earlier version of this probe was BROKEN in a way worth recording. It
thresholded the horizontal acceleration MAGNITUDE, hypot(a.x, a.y), against an
adaptive 6-sigma baseline. Two compounding faults:

  1. If gravity lies in the axes being combined, the baseline magnitude is
     ~9.81 and a forward acceleration is PERPENDICULAR to it. A real 0.5 m/s^2
     step then changes the magnitude by only
         sqrt(9.81^2 + 0.5^2) - 9.81 = 0.0127 m/s^2
     which is below even a clean 6-sigma threshold (~0.06). The probe could
     never see the signal at ANY baseline quality — it was triggering on
     chassis vibration, not acceleration.
  2. The adaptive threshold scaled with baseline noise, so a trial started
     before the robot settled got a threshold of several m/s^2, which nothing
     could cross. Negative ramp times were the signature: "onset" was detected
     after the speed plateau had already been reached.

Both are fixed here by:
  - subtracting the per-axis baseline MEAN (removes the gravity bias whatever
    the IMU mounting) and thresholding the SIGNED deviation of ONE axis,
  - using a FIXED absolute threshold, not a multiple of measured noise,
  - refusing to start a trial until the baseline is genuinely quiet,
  - rejecting any trial whose event ordering is impossible.

The forward axis depends on IMU mounting. The probe reports all three baseline
axis means, identifies which one holds gravity, and refuses to run if the axis
you selected is the gravity axis.

WHICH TOPIC — this is the experiment
─────────────────────────────────────
  controller_server --/cmd_vel_nav--> velocity_smoother --/cmd_vel-->
      low_level_command --ASCII serial--> Teensy --PWM--> Pololu G2 --> tracks

  --topic /cmd_vel       BYPASSES velocity_smoother (actuation path only)
  --topic /cmd_vel_nav   THROUGH velocity_smoother (adds its accel ramp)

(No Pixhawk/MAVLink/ArduPilot exists in this build — see CLAUDE.md.)

SPEED SWEEP — distinguishes slew limiting from a fixed lag
───────────────────────────────────────────────────────────
Pass several speeds. Ramp time that scales with target speed means SLEW-RATE
limiting (a fixed m/s^2 cap, in velocity_smoother or the Teensy sketch). Ramp
time that stays constant regardless of target means a fixed time constant —
filtering, or drivetrain spin-up. One speed cannot tell these apart.

SAFETY — this MOVES THE ROBOT
──────────────────────────────
Tracks OFF THE GROUND or area clear, kill switch in hand. STOP is published
after every trial, on exit and on Ctrl-C.

Usage
─────
    ros2 run dcvg_bringup latency_probe --ros-args -p topic:=/cmd_vel
    ros2 run dcvg_bringup latency_probe --ros-args -p topic:=/cmd_vel_nav
    ros2 run dcvg_bringup latency_probe --ros-args -p "speeds:=[0.2,0.4,0.6]"
"""

import statistics
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

GRAVITY = 9.81


def _fmt(v, unit='ms', scale=1000.0):
    return '     n/a' if v is None else f'{v * scale:7.1f}{unit}'


def _median_iqr(vals):
    """Median and interquartile range — robust to the outliers a mis-fired
    trial produces, which is why this replaced mean/stdev."""
    if not vals:
        return None, None, None
    med = statistics.median(vals)
    if len(vals) < 4:
        return med, min(vals), max(vals)
    q1, _, q3 = statistics.quantiles(vals, n=4)
    return med, q1, q3


class LatencyProbe(Node):
    def __init__(self):
        super().__init__('latency_probe')

        self.declare_parameter('topic', '/cmd_vel')
        self.declare_parameter('speeds', [0.3])       # step magnitudes [m/s]
        self.declare_parameter('hold', 2.0)           # [s] hold each step
        self.declare_parameter('settle', 2.0)         # [s] baseline window
        self.declare_parameter('trials', 15)
        self.declare_parameter('rate', 20.0)          # [Hz] = controller_frequency
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odometry/local')
        self.declare_parameter('axis', 'x')           # IMU forward axis
        # FIXED absolute onset threshold. A 0.3 m/s step reaching speed in a few
        # hundred ms is ~0.5-1.5 m/s^2; IMU noise at rest is ~0.02-0.05. 0.15
        # sits clear of both.
        self.declare_parameter('onset_threshold', 0.15)   # [m/s^2]
        # Hard settling gate — a trial may not START above this baseline noise.
        self.declare_parameter('max_baseline_sd', 0.05)   # [m/s^2]
        self.declare_parameter('settle_retries', 5)
        self.declare_parameter('timeout', 3.0)        # [s] wait for motion
        self.declare_parameter('yes', False)

        g = self.get_parameter
        self.topic = g('topic').value
        self.speeds = [float(s) for s in g('speeds').value]
        self.hold = float(g('hold').value)
        self.settle = float(g('settle').value)
        self.trials = int(g('trials').value)
        self.rate = float(g('rate').value)
        self.axis = str(g('axis').value).lower()
        self.onset_threshold = float(g('onset_threshold').value)
        self.max_baseline_sd = float(g('max_baseline_sd').value)
        self.settle_retries = int(g('settle_retries').value)
        self.timeout = float(g('timeout').value)

        if self.axis not in ('x', 'y', 'z'):
            raise ValueError(f"axis must be x, y or z (got {self.axis})")

        self.pub = self.create_publisher(Twist, self.topic, 10)
        self.create_subscription(Imu, g('imu_topic').value,
                                 self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, g('odom_topic').value,
                                 self._on_odom, 10)

        self._imu = None      # (t, ax, ay, az)
        self._odom = None     # (t, |linear.x|)

    # ── feedback ────────────────────────────────────────────────────────────
    def _on_imu(self, msg: Imu):
        a = msg.linear_acceleration
        self._imu = (self._now(), a.x, a.y, a.z)

    def _on_odom(self, msg: Odometry):
        self._odom = (self._now(), abs(msg.twist.twist.linear.x))

    def _axis_val(self, sample):
        return sample[{'x': 1, 'y': 2, 'z': 3}[self.axis]]

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _spin(self, duration):
        end = self._now() + duration
        while rclpy.ok() and self._now() < end:
            rclpy.spin_once(self, timeout_sec=0.002)

    def _publish(self, vx):
        t = Twist()
        t.linear.x = float(vx)
        self.pub.publish(t)

    def stop(self):
        for _ in range(5):
            self._publish(0.0)
            self._spin(0.01)

    # ── baseline with a hard settling gate ──────────────────────────────────
    def _baseline(self):
        """Collect a quiet baseline. Returns (means, sd_on_axis) or None if the
        robot will not settle — we refuse to run on a dirty baseline rather
        than silently producing a garbage threshold."""
        for attempt in range(1, self.settle_retries + 1):
            samples, last_t = [], None
            end = self._now() + self.settle
            while rclpy.ok() and self._now() < end:
                self._publish(0.0)
                self._spin(1.0 / self.rate)
                if self._imu and self._imu[0] != last_t:
                    last_t = self._imu[0]
                    samples.append(self._imu)

            if len(samples) < 20:
                self.get_logger().error(
                    f'    only {len(samples)} IMU samples — is '
                    f'{self.get_parameter("imu_topic").value} publishing?')
                return None

            means = {ax: statistics.mean(s[i] for s in samples)
                     for ax, i in (('x', 1), ('y', 2), ('z', 3))}
            sd = statistics.pstdev([self._axis_val(s) for s in samples])

            if sd <= self.max_baseline_sd:
                return means, sd
            self.get_logger().warn(
                f'    NOT SETTLED (attempt {attempt}/{self.settle_retries}): '
                f'baseline sd {sd:.4f} > limit {self.max_baseline_sd:.4f} m/s^2 '
                f'— waiting, not proceeding with a dirty baseline')
            self.stop()
            self._spin(2.0)

        self.get_logger().error(
            f'    ABORT trial: robot never settled below '
            f'{self.max_baseline_sd:.4f} m/s^2 in {self.settle_retries} attempts.')
        return None

    # ── one trial ───────────────────────────────────────────────────────────
    def run_trial(self, n, speed):
        base = self._baseline()
        if base is None:
            return None
        means, sd = base

        # Refuse to measure along the gravity axis — that is the exact fault
        # that invalidated the previous version of this probe.
        grav_axis = max(means, key=lambda k: abs(means[k]))
        if grav_axis == self.axis and abs(means[self.axis]) > 5.0:
            self.get_logger().error(
                f'    axis "{self.axis}" holds gravity (mean '
                f'{means[self.axis]:.2f} m/s^2). Forward acceleration is '
                f'perpendicular to it and will not be visible. Pick another '
                f'axis with -p axis:=  (means: ' +
                ', '.join(f'{k}={v:+.2f}' for k, v in means.items()) + ')')
            return None

        mu = means[self.axis]
        t_cmd = self._now()
        t_onset = t_plateau = None
        target90 = 0.9 * speed
        deadline = t_cmd + self.timeout + self.hold

        while rclpy.ok() and self._now() < deadline:
            self._publish(speed)
            self._spin(1.0 / self.rate)
            if t_onset is None and self._imu \
                    and abs(self._axis_val(self._imu) - mu) > self.onset_threshold:
                t_onset = self._imu[0]
            # Ordering guard: the plateau cannot precede onset. The old probe
            # produced negative ramp times exactly because this was missing.
            if t_onset is not None and t_plateau is None and self._odom \
                    and self._odom[0] > t_onset and self._odom[1] >= target90:
                t_plateau = self._odom[0]
            if self._now() - t_cmd > self.hold and t_onset is not None:
                break

        if t_onset is None:
            self.stop()
            self.get_logger().error(
                f'    no motion detected in {self.timeout:.1f}s at {speed} m/s '
                f'(threshold {self.onset_threshold} m/s^2). If the robot DID '
                f'move, the step may be below breakaway — raise speed.')
            return None

        plateau_speed = self._odom[1] if self._odom else 0.0

        # --- stop latency: command zero, measure the decay ---
        t_zero = self._now()
        t_decel = t_stopped = None
        while rclpy.ok() and self._now() < t_zero + self.timeout:
            self._publish(0.0)
            self._spin(1.0 / self.rate)
            if self._odom and self._odom[0] > t_zero:
                if t_decel is None and self._odom[1] < 0.9 * plateau_speed:
                    t_decel = self._odom[0]
                if self._odom[1] < 0.1 * plateau_speed:
                    t_stopped = self._odom[0]
                    break
        self.stop()

        res = {
            'trial': n, 'speed': speed, 'sd': sd,
            'start_dead': t_onset - t_cmd,
            'start_ramp': (t_plateau - t_onset) if t_plateau else None,
            'stop_dead': (t_decel - t_zero) if t_decel else None,
            'stop_total': (t_stopped - t_zero) if t_stopped else None,
        }
        self.get_logger().info(
            f'    sd={sd:.4f}  start dead={_fmt(res["start_dead"])} '
            f'ramp={_fmt(res["start_ramp"])}  '
            f'stop dead={_fmt(res["stop_dead"])} total={_fmt(res["stop_total"])}')
        return res

    # ── driver ──────────────────────────────────────────────────────────────
    def run(self):
        print('\n' + '=' * 78)
        print(' ACTUATION LATENCY PROBE — THIS WILL MOVE THE ROBOT')
        print('=' * 78)
        bypass = 'BYPASSES' if self.topic.rstrip('/').endswith('cmd_vel') else 'THROUGH'
        print(f'  topic   : {self.topic}   ({bypass} velocity_smoother)')
        print(f'  speeds  : {self.speeds} m/s     trials: {self.trials} each')
        print(f'  onset   : FIXED {self.onset_threshold} m/s^2 on IMU axis '
              f'{self.axis} (baseline mean subtracted)')
        print(f'  gate    : baseline sd must be <= {self.max_baseline_sd} m/s^2')
        print('\n  Tracks OFF THE GROUND or area clear. Kill switch in hand.')
        print('=' * 78)
        if not bool(self.get_parameter('yes').value):
            try:
                if input('  Type "go" to start: ').strip().lower() != 'go':
                    print('  aborted.')
                    return
            except EOFError:
                print('  no TTY — re-run with -p yes:=true if intended.')
                return

        all_res, rejected = [], 0
        for speed in self.speeds:
            self.get_logger().info(f'=== speed {speed} m/s ===')
            for i in range(1, self.trials + 1):
                r = self.run_trial(i, speed)
                if r:
                    all_res.append(r)
                else:
                    rejected += 1
                self._spin(0.5)
        self.stop()
        self._report(all_res, rejected)

    def _report(self, res, rejected):
        print('\n' + '=' * 78)
        print(f' RESULTS — topic {self.topic}')
        print(f' {len(res)} usable trial(s), {rejected} rejected '
              f'(dirty baseline / no motion / bad axis)')
        print('=' * 78)
        if not res:
            print('  No usable trials — nothing can be concluded. Do not treat')
            print('  a rejected run as a measurement.')
            print('=' * 78)
            return

        # Per-trial table so a poisoned trial is visible without the full log.
        print(f'  {"trial":>5} {"speed":>6} {"base sd":>9} {"st dead":>9} '
              f'{"st ramp":>9} {"sp dead":>9} {"sp total":>9}')
        for r in res:
            print(f'  {r["trial"]:>5} {r["speed"]:>6.2f} {r["sd"]:>9.4f} '
                  f'{_fmt(r["start_dead"])} {_fmt(r["start_ramp"])} '
                  f'{_fmt(r["stop_dead"])} {_fmt(r["stop_total"])}')

        print('\n  --- median [IQR] by speed ---')
        ramp_by_speed = {}
        for speed in self.speeds:
            sub = [r for r in res if r['speed'] == speed]
            if not sub:
                continue
            for label, key in (('start dead', 'start_dead'), ('start ramp', 'start_ramp'),
                               ('stop dead', 'stop_dead'), ('stop total', 'stop_total')):
                vals = [r[key] for r in sub if r[key] is not None]
                med, lo, hi = _median_iqr(vals)
                if med is None:
                    continue
                print(f'  {speed:>4.2f} m/s  {label:<11} {med * 1000:7.1f} ms '
                      f'[{lo * 1000:6.1f} - {hi * 1000:6.1f}]  (n={len(vals)})')
                if key == 'start_ramp':
                    ramp_by_speed[speed] = med
            print()

        # Slew-rate vs fixed-lag discrimination needs >= 2 speeds.
        if len(ramp_by_speed) >= 2:
            sp = sorted(ramp_by_speed)
            lo_s, hi_s = sp[0], sp[-1]
            ratio_speed = hi_s / lo_s if lo_s else 0
            ratio_ramp = ramp_by_speed[hi_s] / ramp_by_speed[lo_s] if ramp_by_speed[lo_s] else 0
            print(f'  RAMP vs SPEED: speed x{ratio_speed:.2f} -> ramp x{ratio_ramp:.2f}')
            if ratio_ramp > 0.7 * ratio_speed:
                print('    Ramp scales with target speed => SLEW-RATE limiting')
                print('    (a fixed m/s^2 cap). Implied accel = speed/ramp:')
                for s in sp:
                    print(f'      {s:.2f} m/s / {ramp_by_speed[s]:.3f}s = '
                          f'{s / ramp_by_speed[s]:.2f} m/s^2')
                print('    Compare against velocity_smoother max_accel (2.5). If the')
                print('    implied value is well below it, the limit is downstream —')
                print('    i.e. in the Teensy sketch, not in ROS.')
            else:
                print('    Ramp is roughly CONSTANT across speeds => fixed time')
                print('    constant, not slew limiting. Look at filtering or')
                print('    drivetrain spin-up (track tension, backlash), not accel caps.')
        else:
            print('  Only one speed tested — cannot separate slew-rate limiting from a')
            print('  fixed lag. Re-run with -p "speeds:=[0.2,0.4,0.6]".')

        print('=' * 78)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LatencyProbe()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.stop()
            except Exception:  # noqa: BLE001 - best-effort stop on shutdown
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
