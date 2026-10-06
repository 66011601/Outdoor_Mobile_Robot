import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

MAX_RX_LINE_BYTES = 1024
READ_PERIOD_S = 0.05


def seconds(duration):
    return duration.nanoseconds * 1e-9


class LowLevelCommand(Node):

    def __init__(self):
        super().__init__('low_level_command')
        self.declare_parameter('port', '/dev/ttyTeensy')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_separation', 0.8009616629101749)
        self.declare_parameter('wheel_radius', 0.2)
        self.declare_parameter('max_wheel_speed', 15.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('startup_hold', 2.0)
        self.declare_parameter('probe_settle_delay', 2.0)
        self.declare_parameter('probe_down_delay', 8.0)
        self.declare_parameter('probe_raise_delay', 3.0)
        self.declare_parameter('auto_confirm', True)
        p = lambda name: self.get_parameter(name).value
        self.wheel_separation = float(p('wheel_separation'))
        self.wheel_radius = float(p('wheel_radius'))
        self.max_wheel_speed = float(p('max_wheel_speed'))
        self.cmd_timeout = float(p('cmd_timeout'))
        self.startup_hold = float(p('startup_hold'))
        self.probe_settle_delay = float(p('probe_settle_delay'))
        self.probe_down_delay = float(p('probe_down_delay'))
        self.probe_raise_delay = float(p('probe_raise_delay'))
        self.auto_confirm = bool(p('auto_confirm'))

        port = p('port')
        self.ser = serial.Serial(port, int(p('baud')), timeout=0.1, write_timeout=0.1)
        try:
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warn(f'Could not flush serial buffers: {exc}')
        self.get_logger().info(f'Teensy serial open on {port}')

        self.armed = False
        self.stopped = True
        self.start_time = self.get_clock().now()
        self.last_rx_time = self.start_time
        self.last_cmd_time = self.start_time
        self.probe_up = 0
        self.probe_down = 0
        self.probe_timer = None
        self.rx_buf = bytearray()
        self.seen_feedback = False
        self.send(0.0, 0.0)

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(String, '/dcvg/stop_reached', self.on_stop_reached, 10)
        self.stop_confirm_pub = self.create_publisher(String, '/dcvg/stop_confirm', 10)
        self.feedback_pub = self.create_publisher(String, '/teensy/feedback', 10)
        self.create_timer(self.cmd_timeout / 2.0, self.watchdog)
        self.create_timer(READ_PERIOD_S, self.read_serial)
        self.get_logger().info(f'Startup lock: motors held at STOP for >= {self.startup_hold}s')

    def on_cmd_vel(self, msg):
        v, w = msg.linear.x, msg.angular.z
        now = self.get_clock().now()
        self.last_rx_time = now
        if not self.armed:
            if abs(v) < 1e-3 and abs(w) < 1e-3 and self.hold_elapsed(now):
                self.arm('received a zero /cmd_vel')
            else:
                self.get_logger().warn(f'Startup lock: ignoring /cmd_vel (v={v:.2f}, w={w:.2f})',
                                       throttle_duration_sec=2.0)
                self.send(0.0, 0.0)
                return

        left = (v - w * self.wheel_separation / 2.0) / self.wheel_radius
        right = (v + w * self.wheel_separation / 2.0) / self.wheel_radius
        peak = max(abs(left), abs(right))
        if peak > self.max_wheel_speed:
            left *= self.max_wheel_speed / peak
            right *= self.max_wheel_speed / peak

        self.last_cmd_time = self.get_clock().now()
        self.stopped = False
        self.send(left, right)

    def hold_elapsed(self, now):
        return seconds(now - self.start_time) >= self.startup_hold

    def arm(self, reason):
        self.armed = True
        self.stopped = True
        self.send(0.0, 0.0)
        self.get_logger().info(f'Startup lock released ({reason}) — motors ARMED')

    def watchdog(self):
        now = self.get_clock().now()
        if not self.armed:
            self.send(0.0, 0.0)
            quiet = seconds(now - self.last_rx_time)
            if self.hold_elapsed(now) and quiet > self.cmd_timeout:
                self.arm(f'/cmd_vel quiet for {quiet:.2f}s')
            return
        elapsed = seconds(now - self.last_cmd_time)
        if not self.stopped and elapsed > self.cmd_timeout:
            self.get_logger().warn(f'No cmd_vel for {elapsed:.2f}s — stopping motors',
                                   throttle_duration_sec=5.0)
            self.send(0.0, 0.0)
            self.stopped = True

    def schedule(self, delay, callback):
        self.cancel_probe_timer()
        self.probe_timer = self.create_timer(delay, callback)

    def cancel_probe_timer(self):
        if self.probe_timer is not None:
            self.probe_timer.cancel()
            self.destroy_timer(self.probe_timer)
            self.probe_timer = None

    def on_stop_reached(self, msg):
        self.get_logger().info(f"stop_reached ('{msg.data}') — settling {self.probe_settle_delay}s")
        self.schedule(self.probe_settle_delay, self.lower_probe)

    def lower_probe(self):
        self.probe_down, self.probe_up = 1, 0
        self.get_logger().info(f'Probe DOWN, holding {self.probe_down_delay}s')
        self.send(0.0, 0.0)
        self.schedule(self.probe_down_delay, self.raise_probe)

    def raise_probe(self):
        self.probe_down, self.probe_up = 0, 1
        self.get_logger().info(f'Probe UP, allowing {self.probe_raise_delay}s to retract')
        self.send(0.0, 0.0)
        self.schedule(self.probe_raise_delay, self.finish_probe_cycle)

    def finish_probe_cycle(self):
        self.cancel_probe_timer()
        if not self.auto_confirm:
            self.get_logger().info('Probe retracted — waiting for the operator')
            return
        self.stop_confirm_pub.publish(String(data='probe_cycle_complete'))
        self.get_logger().info('Probe retracted — published stop confirmation')

    def send(self, left, right):
        line = f'${left:.2f},{right:.2f},{self.probe_up},{self.probe_down}#\n'
        self.get_logger().debug(line)
        try:
            self.ser.write(line.encode('ascii'))
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write failed: {exc}')

    def read_serial(self):
        try:
            pending = self.ser.in_waiting
            if not pending:
                return
            self.rx_buf += self.ser.read(pending)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Serial read failed: {exc}', throttle_duration_sec=5.0)
            return

        while (idx := self.rx_buf.find(b'\n')) >= 0:
            line = bytes(self.rx_buf[:idx]).decode('ascii', errors='replace').strip()
            del self.rx_buf[:idx + 1]
            if line.startswith('$'):
                self.feedback_pub.publish(String(data=line))
                if not self.seen_feedback:
                    self.seen_feedback = True
                    self.get_logger().info(f'First Teensy frame: {line!r}')

        if len(self.rx_buf) > MAX_RX_LINE_BYTES:
            self.get_logger().warn(f'Dropping {len(self.rx_buf)} bytes without newline '
                                   f'(baud mismatch?)', throttle_duration_sec=5.0)
            self.rx_buf.clear()

    def close(self):
        try:
            if self.ser.is_open:
                self.send(0.0, 0.0)
                self.ser.flush()
                self.ser.close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = None
    try:
        node = LowLevelCommand()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
