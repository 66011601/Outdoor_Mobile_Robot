"""
cmd_vel_to_teensy.py  (REAL HARDWARE — cmd_vel → serial motor bridge)

Alternative to the MAVROS motor path (see mavros.launch.py): instead of handing
/cmd_vel to the Pixhawk, this drives the two motors directly through a Teensy over
a USB-serial link. Nav2 (or teleop) publishes geometry_msgs/Twist on /cmd_vel; this
node applies differential-drive inverse kinematics to split that body twist into a
LEFT and RIGHT wheel command, then streams both over serial to the Teensy, which
does the actual PWM to the Pololu G2 drivers.

Differential-drive inverse kinematics
─────────────────────────────────────
Body twist (v = linear.x [m/s], w = angular.z [rad/s]) maps to wheel *ground*
speeds via the wheel separation L:

    v_left  = v - w * L / 2      [m/s]
    v_right = v + w * L / 2      [m/s]

Those are converted to wheel *angular* speeds with the wheel radius r
(omega = v_wheel / r [rad/s]) so what goes on the wire is independent of tyre size,
then clamped to ±max_wheel_speed. If clamping would distort a turn (one wheel
saturates), both wheels are scaled down together so the turn geometry is preserved.

Serial protocol (ASCII, one line per command)
──────────────────────────────────────────────
    L<left> R<right>\n         e.g.  "L-1.20 R3.40\n"

left/right are wheel angular speeds in rad/s, fixed to 2 decimals. ASCII keeps the
Teensy sketch trivial to parse (sscanf / manual split) and the link easy to debug
with a serial monitor. Change `format_command()` if your firmware wants raw PWM,
a binary frame, or a checksum.

Safety
──────
  - Watchdog: if no /cmd_vel arrives within `cmd_timeout` seconds, a STOP
    ("L0.00 R0.00") is sent so the robot halts on a dead publisher.
  - On shutdown a STOP is sent and the port closed.
  - This bypasses the Pixhawk/ArduPilot failsafe chain — keep an independent RC or
    hardware kill-switch in reach during outdoor testing.

Run
───
    ros2 run dcvg_bringup cmd_vel_to_teensy
    ros2 run dcvg_bringup cmd_vel_to_teensy --ros-args \
        -p port:=/dev/ttyACM0 -p baud:=115200 \
        -p wheel_separation:=0.55 -p wheel_radius:=0.12 -p max_wheel_speed:=15.0

Requires pyserial (`pip install pyserial` / `sudo apt install python3-serial`).
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import serial
except ImportError:  # pragma: no cover - surfaced at runtime with a clear message
    serial = None


class CmdVelToTeensy(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_teensy')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheel_separation', 0.8009616629101749)   # [m] left↔right wheel distance (TBD — measure)
        self.declare_parameter('wheel_radius', 0.2)       # [m] wheel radius (TBD — measure)
        self.declare_parameter('max_wheel_speed', 15.0)    # [rad/s] clamp per wheel
        self.declare_parameter('cmd_timeout', 0.5)         # [s] watchdog: stop if no cmd_vel
        self.declare_parameter('serial_timeout', 0.1)      # [s] serial write timeout

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        if self.wheel_radius <= 0.0:
            raise ValueError('wheel_radius must be > 0')

        # ── Serial link to the Teensy ───────────────────────────────────────
        if serial is None:
            raise ImportError(
                'pyserial not installed — run "sudo apt install python3-serial" '
                'or "pip install pyserial"')
        try:
            self.ser = serial.Serial(
                self.port, self.baud,
                timeout=float(self.get_parameter('serial_timeout').value),
                write_timeout=float(self.get_parameter('serial_timeout').value))
        except serial.SerialException as exc:
            raise RuntimeError(f'Could not open Teensy serial port {self.port}: {exc}')
        self.get_logger().info(f'Teensy serial open on {self.port} @ {self.baud} baud')

        # ── ROS interface ───────────────────────────────────────────────────
        self.sub = self.create_subscription(Twist, cmd_vel_topic, self.on_cmd_vel, 10)
        self.last_cmd_time = self.get_clock().now()
        self.stopped = True
        # Watchdog runs faster than the timeout so a stall is caught promptly.
        self.create_timer(self.cmd_timeout / 2.0, self.watchdog)
        self.get_logger().info(
            f'Listening on {cmd_vel_topic}; L={self.wheel_separation} m, '
            f'r={self.wheel_radius} m, max={self.max_wheel_speed} rad/s')

    # ------------------------------------------------------------------------
    def on_cmd_vel(self, msg: Twist):
        v = msg.linear.x          # [m/s]
        w = msg.angular.z         # [rad/s]

        # Differential-drive inverse kinematics → wheel ground speeds [m/s]
        v_left = v - w * self.wheel_separation / 2.0
        v_right = v + w * self.wheel_separation / 2.0

        # Ground speed → wheel angular speed [rad/s]
        left = v_left / self.wheel_radius
        right = v_right / self.wheel_radius

        left, right = self.clamp_preserving_turn(left, right)

        self.last_cmd_time = self.get_clock().now()
        self.stopped = False
        self.send(left, right)

    def clamp_preserving_turn(self, left, right):
        """Clamp both wheels to ±max_wheel_speed. If either exceeds the limit,
        scale both down by the same factor so the turn geometry is preserved."""
        peak = max(abs(left), abs(right))
        if peak > self.max_wheel_speed and peak > 0.0:
            scale = self.max_wheel_speed / peak
            left *= scale
            right *= scale
        return left, right

    def watchdog(self):
        """Send a STOP if no cmd_vel has arrived within cmd_timeout."""
        if self.stopped:
            return
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            self.get_logger().warn(
                f'No cmd_vel for {elapsed:.2f}s — stopping motors', throttle_duration_sec=5.0)
            self.send(0.0, 0.0)
            self.stopped = True

    @staticmethod
    def format_command(left, right):
        """Wire format the Teensy parses. Edit here to change the protocol."""
        return f'L{left:.2f} R{right:.2f}\n'

    def send(self, left, right):
        line = self.format_command(left, right)
        try:
            self.ser.write(line.encode('ascii'))
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write failed: {exc}')

    def stop_and_close(self):
        try:
            if self.ser.is_open:
                self.send(0.0, 0.0)
                self.ser.flush()
                self.ser.close()
        except Exception:  # noqa: BLE001 - best-effort cleanup on shutdown
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CmdVelToTeensy()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop_and_close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
