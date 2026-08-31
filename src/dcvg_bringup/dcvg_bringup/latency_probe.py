"""
latency_probe.py  (REAL HARDWARE — actuation dead-time / ramp measurement)

Measures how long it takes the tracks to actually respond to a velocity command,
by publishing a STEP cmd_vel directly (bypassing Nav2) and timestamping the first
detected motion. Reports DEAD TIME and RAMP TIME separately, because they have
different causes and different fixes:

  dead time = command published  -> first detectable motion
              (serial/firmware/scheduling latency, plus any part of a velocity
               ramp that sits below the tracks' breakaway friction)
  ramp time = first motion       -> 90% of the commanded speed
              (acceleration limiting: velocity_smoother max_accel, or whatever
               slew limiting the Teensy sketch applies)

WHICH TOPIC TO PUBLISH ON — this is the whole experiment
─────────────────────────────────────────────────────────
The real command chain on this robot is:

  controller_server --/cmd_vel_nav--> velocity_smoother --/cmd_vel-->
      low_level_command --ASCII serial--> Teensy --PWM--> Pololu G2 --> tracks

(There is no Pixhawk, MAVLink or ArduPilot in this build — motor commands go
straight to the Teensy. See CLAUDE.md and low_level_command.py.)

So:
  --topic /cmd_vel       BYPASSES velocity_smoother. Measures only
                         low_level_command + serial + Teensy + motors.
  --topic /cmd_vel_nav   goes THROUGH velocity_smoother. Adds its accel ramp.

Run BOTH and subtract. If /cmd_vel is fast and /cmd_vel_nav is slow by ~0.5s,
the delay is velocity_smoother's max_accel ramp, not the actuation path.
If BOTH are slow, the delay is in the Teensy firmware / serial / scheduling.

MOTION FEEDBACK — this robot has no wheel encoders
───────────────────────────────────────────────────
Onset is detected from the IMU (fastest, ~200Hz, sees acceleration immediately).
/vo and /odometry/local are also recorded, but both lag: VIO needs a camera frame
pair, and the EKF adds filter lag. Speed for the ramp measurement comes from
odometry since the IMU cannot give absolute velocity.

SAFETY — this MOVES THE ROBOT
──────────────────────────────
Run with the tracks OFF THE GROUND or in a clear area, with the kill switch in
hand. Defaults are deliberately slow (0.3 m/s) and short. A STOP is published on
exit, on Ctrl-C, and after every trial.

Usage
─────
    # actuation path only (no smoother)
    ros2 run dcvg_bringup latency_probe --ros-args -p topic:=/cmd_vel

    # full chain including velocity_smoother
    ros2 run dcvg_bringup latency_probe --ros-args -p topic:=/cmd_vel_nav

    # slower/gentler, more trials
    ros2 run dcvg_bringup latency_probe --ros-args \
        -p topic:=/cmd_vel -p speed:=0.2 -p trials:=8 -p yes:=true
"""

import math
import statistics
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class LatencyProbe(Node):
    def __init__(self):
        super().__init__('latency_probe')

        self.declare_parameter('topic', '/cmd_vel')
        self.declare_parameter('speed', 0.3)          # [m/s] step magnitude
        self.declare_parameter('angular', 0.0)        # [rad/s] optional yaw step
        self.declare_parameter('hold', 1.5)           # [s] hold the step
        self.declare_parameter('settle', 2.0)         # [s] baseline before step
        self.declare_parameter('trials', 5)
        self.declare_parameter('rate', 20.0)          # [Hz] matches controller_frequency
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odometry/local')
        self.declare_parameter('vo_topic', '/vo')
        self.declare_parameter('sigma', 6.0)          # onset threshold in baseline sigmas
        self.declare_parameter('timeout', 3.0)        # [s] give up waiting for motion
        self.declare_parameter('yes', False)          # skip the safety prompt

        self.topic = self.get_parameter('topic').value
        self.speed = float(self.get_parameter('speed').value)
        self.angular = float(self.get_parameter('angular').value)
        self.hold = float(self.get_parameter('hold').value)
        self.settle = float(self.get_parameter('settle').value)
        self.trials = int(self.get_parameter('trials').value)
        self.rate = float(self.get_parameter('rate').value)
        self.sigma = float(self.get_parameter('sigma').value)
        self.timeout = float(self.get_parameter('timeout').value)

        self.pub = self.create_publisher(Twist, self.topic, 10)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value,
                                 self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self._on_odom, 10)
        self.create_subscription(Odometry, self.get_parameter('vo_topic').value,
                                 self._on_vo, 10)

        # Latest samples as (receipt_time, value)
        self._imu = None      # forward acceleration magnitude
        self._odom = None     # |linear.x|
        self._vo = None       # |linear.x|

    # ── feedback callbacks ──────────────────────────────────────────────────
    def _on_imu(self, msg: Imu):
        a = msg.linear_acceleration
        # Use the horizontal magnitude: the forward axis depends on camera
        # mounting, and gravity dominates whichever axis is vertical.
        self._imu = (self._now(), math.hypot(a.x, a.y))

    def _on_odom(self, msg: Odometry):
        self._odom = (self._now(), abs(msg.twist.twist.linear.x))

    def _on_vo(self, msg: Odometry):
        self._vo = (self._now(), abs(msg.twist.twist.linear.x))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _spin(self, duration):
        """Spin for `duration` seconds, returning collected samples."""
        end = self._now() + duration
        while rclpy.ok() and self._now() < end:
            rclpy.spin_once(self, timeout_sec=0.002)

    def _publish(self, vx, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.pub.publish(t)

    def stop(self):
        for _ in range(5):
            self._publish(0.0, 0.0)
            self._spin(0.01)

    # ── one trial ───────────────────────────────────────────────────────────
    def run_trial(self, n):
        self.get_logger().info(f'--- trial {n}: settling {self.settle:.1f}s (commanding zero) ---')
        baseline = []
        end = self._now() + self.settle
        last_stamp = None
        while rclpy.ok() and self._now() < end:
            self._publish(0.0, 0.0)
            self._spin(1.0 / self.rate)
            if self._imu and self._imu[0] != last_stamp:
                last_stamp = self._imu[0]
                baseline.append(self._imu[1])

        if len(baseline) < 10:
            self.get_logger().error(
                f'Only {len(baseline)} IMU samples during settle — is '
                f'{self.get_parameter("imu_topic").value} publishing? Cannot detect onset.')
            return None

        mu = statistics.mean(baseline)
        sd = statistics.pstdev(baseline) or 1e-6
        thresh = self.sigma * sd
        self.get_logger().info(
            f'    IMU baseline: mean={mu:.4f} sd={sd:.4f} -> onset threshold '
            f'{thresh:.4f} m/s^2 above baseline ({len(baseline)} samples)')

        # --- step ---
        t_cmd = self._now()
        self._publish(self.speed, self.angular)
        t_onset = None
        t_ramp = None
        target = 0.9 * self.speed
        deadline = t_cmd + self.timeout + self.hold

        while rclpy.ok() and self._now() < deadline:
            self._publish(self.speed, self.angular)
            self._spin(1.0 / self.rate)

            if t_onset is None and self._imu and abs(self._imu[1] - mu) > thresh:
                t_onset = self._imu[0]
            if t_onset is not None and t_ramp is None and self._odom \
                    and self._odom[1] >= target:
                t_ramp = self._odom[0]
            if self._now() - t_cmd > self.hold and t_onset is not None:
                break

        self.stop()

        if t_onset is None:
            self.get_logger().error(
                f'    NO MOTION DETECTED within {self.timeout:.1f}s at {self.speed} m/s. '
                f'Either the robot did not move (check the Teensy/kill switch) or the '
                f'step is below breakaway friction — retry with a larger --speed.')
            return None

        dead = t_onset - t_cmd
        ramp = (t_ramp - t_onset) if t_ramp is not None else None
        self.get_logger().info(
            f'    dead time = {dead * 1000:7.1f} ms' +
            (f'   ramp to 90% = {ramp * 1000:7.1f} ms' if ramp is not None
             else '   ramp = NOT REACHED (never hit 90% of target)'))
        return dead, ramp

    # ── all trials + verdict ────────────────────────────────────────────────
    def run(self):
        print()
        print('=' * 72)
        print(' ACTUATION LATENCY PROBE — THIS WILL MOVE THE ROBOT')
        print('=' * 72)
        print(f'  topic          : {self.topic}' +
              ('   (BYPASSES velocity_smoother)' if self.topic.rstrip("/").endswith("cmd_vel")
               else '   (THROUGH velocity_smoother)'))
        print(f'  step           : {self.speed} m/s, {self.angular} rad/s for {self.hold}s')
        print(f'  trials         : {self.trials}')
        print()
        print('  Tracks OFF THE GROUND or area clear. Kill switch in hand.')
        print('=' * 72)
        if not bool(self.get_parameter('yes').value):
            try:
                if input('  Type "go" to start: ').strip().lower() != 'go':
                    print('  aborted.')
                    return
            except EOFError:
                print('  no TTY — re-run with -p yes:=true if this is intended.')
                return

        results = []
        for i in range(1, self.trials + 1):
            r = self.run_trial(i)
            if r:
                results.append(r)
            self._spin(0.5)
        self.stop()

        if not results:
            self.get_logger().error('No usable trials.')
            return

        deads = [d for d, _ in results]
        ramps = [r for _, r in results if r is not None]

        print()
        print('=' * 72)
        print(f' RESULTS  ({len(results)}/{self.trials} usable trials, topic {self.topic})')
        print('=' * 72)
        print(f'  DEAD TIME  median {statistics.median(deads) * 1000:7.1f} ms   '
              f'min {min(deads) * 1000:7.1f}   max {max(deads) * 1000:7.1f}')
        if len(deads) > 1:
            print(f'             stdev  {statistics.pstdev(deads) * 1000:7.1f} ms')
        if ramps:
            print(f'  RAMP TIME  median {statistics.median(ramps) * 1000:7.1f} ms   '
                  f'min {min(ramps) * 1000:7.1f}   max {max(ramps) * 1000:7.1f}')
        else:
            print('  RAMP TIME  never reached 90% of target — check odom_topic is live')
        print()

        # Constant vs intermittent is the key discriminator between a
        # deterministic cause (accel ramp / firmware slew) and CPU starvation.
        if len(deads) > 1:
            sd = statistics.pstdev(deads)
            if sd < 0.05:
                print('  VERDICT: dead time is CONSISTENT (stdev < 50ms).')
                print('           Points to a DETERMINISTIC cause — an acceleration ramp')
                print('           (velocity_smoother max_accel, or Teensy slew limiting),')
                print('           NOT CPU scheduling. Re-run on the other topic to see')
                print('           which side of velocity_smoother it comes from.')
            else:
                print('  VERDICT: dead time is VARIABLE (stdev > 50ms).')
                print('           Points to CPU scheduling starvation or serial blocking.')
                print('           Check load from RTAB-Map VIO while this runs, and see')
                print('           the cmd_vel queue-depth note in low_level_command.py.')
        print('=' * 72)


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
