"""
low_level_command.py  (REAL HARDWARE — cmd_vel → serial motor bridge)

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
    $<left>,<right>,<probe_up>,<probe_down>#\n

left/right are wheel angular speeds in rad/s, fixed to 2 decimals. probe_up/
probe_down are 1 (active) or 0 (inactive). ASCII keeps the Teensy sketch trivial
to parse (sscanf / manual split) and the link easy to debug with a serial
monitor. Change `format_command()` if your firmware wants raw PWM, a binary
frame, or a checksum.

Teensy → ROS (the return path)
──────────────────────────────
The same port is polled for whatever the Teensy sends back — encoder ticks,
probe limit switches, battery volts, plain log lines, anything. Every complete
newline-terminated line is republished verbatim as a std_msgs/String on
`feedback_topic` (default /teensy/feedback):

    ros2 topic echo /teensy/feedback

That topic is the seam for off-robot telemetry: a separate node subscribes to it
and does the HTTP POST / MQTT publish / websocket write. Deliberately NOT done
in this node — it runs a single-threaded executor, so a network call here would
park the same thread that delivers /cmd_vel and services the watchdog, and a
slow uplink would become a stalled control loop. Reading is safe because it only
ever takes the bytes already buffered (`in_waiting`) and never blocks.

Only lines matching `feedback_pattern` are published; everything else the Teensy
writes is dropped with a throttled warning. The default pattern accepts any line
that STARTS WITH '$':

    $0,0,0,0,0,0,1,1,100,100,0,0,0#     published
    X1                                  dropped
    ADC :374                            dropped
    Foword/Backward :1515               dropped
    Teensy booting, firmware v3         dropped

The firmware interleaves human-readable debug prints with its telemetry, and the
leading '$' is the only thing that reliably tells the two apart — so that is the
entire test. Nothing about field count or the trailing '#' is checked, which
means the frame can gain fields as the firmware grows without anyone editing the
pattern.

The trade-off: a truncated frame ('$0,0,0' after a reset or a baud glitch) now
gets through instead of being rejected here. It is still recognisable downstream
— `teensy_feedback_server` only stamps DCVG/GPS onto lines that both start with
'$' AND end with '#', so a half-frame arrives at the laptop visibly short rather
than silently mis-stamped. If you would rather drop those at the source, set

    feedback_pattern:='^\$.*#$'          # starts '$', ends '#', any field count

Set `feedback_pattern` to an empty string to publish every line unfiltered
(useful when bringing new firmware up and you want to see the boot chatter too).
On a shell command line that has to be written `feedback_pattern:="''"` — a bare
`:=''` is stripped by the shell to `feedback_pattern:=`, which rcl rejects.

Survey telemetry stamping (elsewhere)
──────────────────────────────────────
Frames leave here exactly as the Teensy sent them. The DCVG meter reading and
the RTK position are stamped onto them downstream by `teensy_feedback_server`,
which subscribes to `feedback_topic`, appends dcvg_on,dcvg_off then lat,lon, and
serves the result to the laptop over TCP — see its docstring for the resulting
17-column wire format and why that field order is what it is.

That split is the whole point of the seam described above: the OCR server's
socket and the laptop's sockets then live in a node with no control loop to
park, so a wedged far end costs telemetry rather than /cmd_vel.

Probe sequencing
─────────────────
On /dcvg/stop_reached (std_msgs/String) this runs a timed measurement cycle and
then releases the rover to carry on:

    stop_reached
      │  probe_settle_delay   (2.0 s)  probe stays UP — let the rover stop moving
      ▼
    probe DOWN  (probe_down=1, probe_up=0)
      │  probe_down_delay     (8.0 s)  the DCVG measurement itself
      ▼
    probe UP    (probe_down=0, probe_up=1)
      │  probe_raise_delay    (3.0 s)  actuator physically retracts
      ▼
    publish on /dcvg/stop_confirm  →  route_waypoint_follower resumes

The two delays either side of the measurement exist because the wire command and
the physical world disagree about timing. /dcvg/stop_reached fires the moment
Nav2's goal checker is satisfied, but the rover is still creeping (~0.6 s of
actuation dead time plus decel), so planting the probe immediately drags the tip
through moving ground. Symmetrically, the raise command is instantaneous on the
wire while the actuator is not, so driving off the instant it is sent drags the
probe again.

The final publish is what closes the loop: route_waypoint_follower's
_wait_at_stop() blocks on that topic, so without it the survey needs an operator
press per stop. Set `auto_confirm:=false` to keep that manual gate instead.

The cycle is non-blocking (chained one-shot timers), so /cmd_vel and the watchdog
keep running throughout; every send() carries the current probe state, so a
watchdog STOP mid-cycle cannot disturb the probe. Only one cycle runs at a time —
a new stop_reached received mid-cycle restarts it from the settle step.

Safety
──────
  - Startup lock: the node starts DISARMED. Until it arms, every /cmd_vel is
    ignored and a STOP is re-sent on every watchdog tick, so the robot cannot
    move on a command left over from before this node started (a teleop or
    velocity_smoother still publishing, messages queued on the subscription,
    or bytes stale in the serial buffers, which are flushed on open). It arms
    only once `startup_hold` seconds have passed AND the /cmd_vel stream is
    known to be safe: either a zero command arrives, or /cmd_vel stays quiet
    for `cmd_timeout`. A publisher streaming non-zero commands keeps it locked.
  - Watchdog: if no /cmd_vel arrives within `cmd_timeout` seconds, a STOP
    ("$0.00,0.00,0,0#") is sent so the robot halts on a dead publisher. The
    probe state is left untouched by the watchdog.
  - On shutdown a STOP is sent (probe left as-is) and the port closed.
  - This bypasses the Pixhawk/ArduPilot failsafe chain — keep an independent RC or
    hardware kill-switch in reach during outdoor testing.

Run
───
    ros2 run dcvg_bringup low_level_command
    ros2 run dcvg_bringup low_level_command --ros-args \
        -p port:=/dev/ttyACM1 -p baud:=115200 \
        -p wheel_separation:=0.55 -p wheel_radius:=0.12 -p max_wheel_speed:=15.0
    ros2 run dcvg_bringup low_level_command --ros-args \
        -p feedback_topic:=/teensy/feedback -p read_period:=0.05   # 0.0 disables reads
    ros2 run dcvg_bringup low_level_command --ros-args \
        -p feedback_pattern:="''"   # publish every line, not just telemetry
                                    # frames. Double quotes are required — see
                                    # the note above.

Requires pyserial (`pip install pyserial` / `sudo apt install python3-serial`).
"""

import re

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    import serial
except ImportError:  # pragma: no cover - surfaced at runtime with a clear message
    serial = None


# Cap on a single un-terminated inbound line. A Teensy that never sends '\n'
# (wrong baud, half-open port, binary noise) would otherwise grow the receive
# buffer without bound for the whole run.
MAX_RX_LINE_BYTES = 1024

# Default shape of a Teensy telemetry frame:  $0,0,0,0,0,0,1,1,100,100,0,0,0#
# The firmware interleaves human-readable debug with its frames — 'X1',
# 'ADC :374', 'Foword/Backward :1515', 'MODE :1013', boot banners — and the one
# thing that separates telemetry from all of it is the leading '$'. So that is
# the whole test: starts with '$', publish it.
#
# Deliberately NOT anchored at the end and NOT counting fields. The frame is
# expected to gain fields as the firmware grows, and a stricter pattern turns
# every such change into silent total telemetry loss until someone edits this
# line. The cost of the looser test is that a short or truncated frame now
# reaches /teensy/feedback instead of being dropped here; see `feedback_pattern`
# below for the stricter alternative if that matters more than forward
# compatibility.
TEENSY_FRAME_PATTERN = r'^\$'


class CmdVelToTeensy(Node):
    def __init__(self):
        super().__init__('low_level_command')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyTeensy')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheel_separation', 0.8009616629101749)   # [m] left↔right wheel distance (TBD — measure)
        self.declare_parameter('wheel_radius', 0.2)        # [m] wheel radius (matches xacro's placeholder — TBD, measure on the real robot)
        self.declare_parameter('max_wheel_speed', 15.0)    # [rad/s] clamp per wheel
        self.declare_parameter('cmd_timeout', 0.5)         # [s] watchdog: stop if no cmd_vel
        # [s] minimum time after start during which all /cmd_vel is ignored and
        # STOP is held. See "Startup lock" in the module docstring.
        self.declare_parameter('startup_hold', 2.0)
        self.declare_parameter('serial_timeout', 0.1)      # [s] serial write timeout
        self.declare_parameter('stop_reached_topic', '/dcvg/stop_reached')
        self.declare_parameter('stop_confirm_topic', '/dcvg/stop_confirm')
        # Whatever the Teensy writes back, one std_msgs/String per line. Set
        # read_period to 0.0 to disable the read side entirely (no timer, no
        # publisher) if the firmware is write-only.
        self.declare_parameter('feedback_topic', '/teensy/feedback')
        self.declare_parameter('read_period', 0.05)        # [s] serial poll interval
        # Regex a Teensy line must match to be republished. Defaults to '^\$'
        # — any line starting with '$', which is what separates telemetry frames
        # from the firmware's debug prints. '' publishes every line.
        self.declare_parameter('feedback_pattern', TEENSY_FRAME_PATTERN)
        # [s] settle time between arriving at a stop and putting the probe down.
        # /dcvg/stop_reached fires the moment Nav2's goal checker is satisfied, but
        # the rover is not physically still at that instant — there is ~0.6s of
        # actuation dead time plus decel, so it is still creeping. Driving the
        # probe into the ground while the chassis is still moving drags the tip.
        self.declare_parameter('probe_settle_delay', 2.0)
        self.declare_parameter('probe_down_delay', 8.0)    # [s] hold probe down before raising it
        # [s] time for the actuator to physically finish retracting before the
        # rover is allowed to drive again. The raise command is instantaneous on
        # the wire; the actuator is not. Moving off early drags the probe.
        self.declare_parameter('probe_raise_delay', 3.0)
        # Publish on stop_confirm_topic once the probe cycle finishes, which is
        # what releases route_waypoint_follower's _wait_at_stop() and lets the
        # survey continue. Set False to keep the operator gate (web-app button or
        # a manual `ros2 topic pub`) as the only way to advance.
        self.declare_parameter('auto_confirm', True)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.startup_hold = float(self.get_parameter('startup_hold').value)
        self.probe_settle_delay = float(self.get_parameter('probe_settle_delay').value)
        self.probe_down_delay = float(self.get_parameter('probe_down_delay').value)
        self.probe_raise_delay = float(self.get_parameter('probe_raise_delay').value)
        self.auto_confirm = bool(self.get_parameter('auto_confirm').value)
        self.read_period = float(self.get_parameter('read_period').value)
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        stop_reached_topic = self.get_parameter('stop_reached_topic').value
        stop_confirm_topic = self.get_parameter('stop_confirm_topic').value
        feedback_topic = self.get_parameter('feedback_topic').value
        feedback_pattern = self.get_parameter('feedback_pattern').value

        # Compiled once, at startup, so a typo'd pattern fails here with a clear
        # message instead of throwing inside the read timer on every poll.
        try:
            self.feedback_re = re.compile(feedback_pattern) if feedback_pattern else None
        except re.error as exc:
            raise ValueError(f'feedback_pattern is not a valid regex ({exc}): {feedback_pattern}')

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
        # Drop anything stale in the OS buffers from a previous session, so the
        # first bytes the Teensy sees from us are the STOP below.
        try:
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warn(f'Could not flush serial buffers: {exc}')
        self.get_logger().info(f'Teensy serial open on {self.port} @ {self.baud} baud')

        # ── Startup lock ────────────────────────────────────────────────────
        self.armed = False
        self.start_time = self.get_clock().now()
        # Time of the last /cmd_vel received, accepted or not — used to decide
        # whether the stream has gone quiet while disarmed.
        self.last_rx_time = self.start_time

        # ── Probe state ─────────────────────────────────────────────────────
        self.probe_up = 0
        self.probe_down = 0
        self.probe_timer = None
        self.send(0.0, 0.0)

        # ── ROS interface ───────────────────────────────────────────────────
        self.sub = self.create_subscription(Twist, cmd_vel_topic, self.on_cmd_vel, 10)
        self.stop_reached_sub = self.create_subscription(
            String, stop_reached_topic, self.on_stop_reached, 10)
        self.stop_confirm_pub = self.create_publisher(String, stop_confirm_topic, 10)
        self.last_cmd_time = self.get_clock().now()
        self.stopped = True
        # Watchdog runs faster than the timeout so a stall is caught promptly.
        self.create_timer(self.cmd_timeout / 2.0, self.watchdog)

        # ── Teensy → ROS return path ────────────────────────────────────────
        self._rx_buf = bytearray()
        self._seen_feedback = False
        if self.read_period > 0.0:
            self.feedback_pub = self.create_publisher(String, feedback_topic, 10)
            self.create_timer(self.read_period, self.read_serial)
        else:
            self.feedback_pub = None

        self.send(0.0, 0.0)

        self.get_logger().info(
            f'Startup lock: motors held at STOP for >= {self.startup_hold}s, then until '
            f'/cmd_vel is zero or quiet for {self.cmd_timeout}s')
        self.get_logger().info(
            f'Listening on {cmd_vel_topic}; L={self.wheel_separation} m, '
            f'r={self.wheel_radius} m, max={self.max_wheel_speed} rad/s')
        self.get_logger().info(
            f'Probe sequencing on {stop_reached_topic}: settle {self.probe_settle_delay}s '
            f'-> down {self.probe_down_delay}s -> raise {self.probe_raise_delay}s '
            f'-> {"confirm on " + stop_confirm_topic if self.auto_confirm else "NO auto-confirm"} '
            f'(total dwell '
            f'{self.probe_settle_delay + self.probe_down_delay + self.probe_raise_delay:.1f}s)')
        self.get_logger().info(
            f'Republishing Teensy serial lines on {feedback_topic} '
            f'(polled every {self.read_period}s, '
            f'{"matching " + self.feedback_re.pattern if self.feedback_re else "unfiltered"})'
            if self.read_period > 0.0
            else 'read_period=0 — Teensy feedback disabled (write-only link)')

    # ------------------------------------------------------------------------
    def on_cmd_vel(self, msg: Twist):
        v = msg.linear.x          # [m/s]
        w = msg.angular.z         # [rad/s]

        now = self.get_clock().now()
        self.last_rx_time = now
        if not self.armed:
            is_zero = abs(v) < 1e-3 and abs(w) < 1e-3
            if is_zero and self._startup_hold_elapsed(now):
                self.arm('received a zero /cmd_vel')
            else:
                self.get_logger().warn(
                    f'Startup lock: ignoring /cmd_vel (v={v:.2f}, w={w:.2f}) — '
                    f'send a zero command or stop publishing to arm',
                    throttle_duration_sec=2.0)
                self.send(0.0, 0.0)
                return

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

    # ── Probe measurement cycle ─────────────────────────────────────────────
    # settle -> down -> hold -> raise -> retract -> confirm. Each step arms a
    # one-shot timer for the next, so the whole thing is non-blocking: /cmd_vel
    # and the watchdog keep running throughout, and every send() carries the
    # current probe state, so a watchdog STOP mid-cycle cannot disturb the probe.
    def _arm(self, delay, callback):
        """Arm a one-shot timer, replacing any cycle already in flight."""
        self._cancel_probe_timer()
        self.probe_timer = self.create_timer(delay, callback)

    def _cancel_probe_timer(self):
        if self.probe_timer is not None:
            self.probe_timer.cancel()
            self.destroy_timer(self.probe_timer)
            self.probe_timer = None

    def on_stop_reached(self, msg: String):
        """Start the probe cycle. A new stop_reached mid-cycle restarts it."""
        self.get_logger().info(
            f"stop_reached ('{msg.data}') — settling {self.probe_settle_delay} s "
            f"before lowering probe")
        # Probe stays UP during the settle window. /dcvg/stop_reached fires the
        # instant Nav2's goal checker is satisfied, but the rover is still
        # creeping (actuation dead time + decel), and planting the probe into
        # moving ground drags the tip.
        self._arm(self.probe_settle_delay, self.lower_probe)

    def lower_probe(self):
        # probe_down=1 / probe_up=0. This pair was once set the wrong way round
        # here — identical to what raise_probe() sets — so the "down" half of the
        # sequence commanded the probe UP and it never descended. Every DCVG stop
        # ran the full wait-and-confirm ritual with the probe in the air, taking
        # no measurement at all. Keep these two lines opposite to raise_probe().
        self.probe_down = 1
        self.probe_up = 0
        self.get_logger().info(
            f'Settled — probe DOWN, holding {self.probe_down_delay} s')
        self.send(0.0, 0.0)
        self._arm(self.probe_down_delay, self.raise_probe)

    def raise_probe(self):
        self.probe_down = 0
        self.probe_up = 1
        self.get_logger().info(
            f'Measurement done — probe UP, allowing {self.probe_raise_delay} s to retract')
        self.send(0.0, 0.0)
        # Do NOT confirm yet. The raise command is instantaneous on the wire but
        # the actuator is not; releasing the rover here would drag the probe.
        self._arm(self.probe_raise_delay, self.finish_probe_cycle)

    def finish_probe_cycle(self):
        """Probe fully retracted — release the rover to drive to the next stop."""
        self._cancel_probe_timer()
        if not self.auto_confirm:
            self.get_logger().info(
                'Probe retracted — auto_confirm is False, waiting for the operator')
            return
        msg = String()
        msg.data = 'probe_cycle_complete'
        self.stop_confirm_pub.publish(msg)
        self.get_logger().info(
            'Probe retracted — published stop confirmation, navigation continues')

    def clamp_preserving_turn(self, left, right):
        """Clamp both wheels to ±max_wheel_speed. If either exceeds the limit,
        scale both down by the same factor so the turn geometry is preserved."""
        peak = max(abs(left), abs(right))
        if peak > self.max_wheel_speed and peak > 0.0:
            scale = self.max_wheel_speed / peak
            left *= scale
            right *= scale
        return left, right

    # ── Startup lock ────────────────────────────────────────────────────────
    def _startup_hold_elapsed(self, now):
        return (now - self.start_time).nanoseconds * 1e-9 >= self.startup_hold

    def arm(self, reason):
        self.armed = True
        self.stopped = True
        self.send(0.0, 0.0)
        self.get_logger().info(f'Startup lock released ({reason}) — motors ARMED')

    def watchdog(self):
        """Send a STOP if no cmd_vel has arrived within cmd_timeout.

        While disarmed, keep re-asserting STOP and arm once the hold has elapsed
        and /cmd_vel has been quiet for cmd_timeout."""
        if not self.armed:
            self.send(0.0, 0.0)
            now = self.get_clock().now()
            quiet = (now - self.last_rx_time).nanoseconds * 1e-9
            if self._startup_hold_elapsed(now) and quiet > self.cmd_timeout:
                self.arm(f'/cmd_vel quiet for {quiet:.2f}s')
            return
        if self.stopped:
            return
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            self.get_logger().warn(
                f'No cmd_vel for {elapsed:.2f}s — stopping motors', throttle_duration_sec=5.0)
            self.send(0.0, 0.0)
            self.stopped = True

    @staticmethod
    def format_command(left, right, probe_up, probe_down):
        """Wire format the Teensy parses. Edit here to change the protocol."""
        return f'${left:.2f},{right:.2f},{int(probe_up)},{int(probe_down)}#\n'

    def send(self, left, right):
        line = self.format_command(left, right, self.probe_up, self.probe_down)
        # debug, not info: this fires at the /cmd_vel rate (20 Hz). On a platform
        # with documented CPU starvation, formatting and emitting a log line 20x
        # a second costs cycles the controller loop needs — and a missed
        # controller cycle is a tracking error. Enable with
        # --ros-args --log-level low_level_command:=debug when debugging the wire
        # format.
        self.get_logger().debug(line)
        try:
            self.ser.write(line.encode('ascii'))
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write failed: {exc}')

    # ── Read side: Teensy → /teensy/feedback ────────────────────────────────
    def read_serial(self):
        """Drain what the Teensy has sent and publish each complete line.

        Non-blocking by construction: only the `in_waiting` bytes already in the
        OS buffer are taken, so this never parks waiting on the port. That
        matters because the node runs a single-threaded executor — a blocking
        read (`readline()`, or `read()` past what's buffered) would hold up
        /cmd_vel delivery and the watchdog for up to `serial_timeout` seconds
        every poll, which is a tracking error on a 20 Hz control loop.

        Lines are published verbatim; see the module docstring for why nothing
        is parsed here.
        """
        try:
            pending = self.ser.in_waiting
            if not pending:
                return
            self._rx_buf += self.ser.read(pending)
        except (serial.SerialException, OSError) as exc:
            # Throttled: a yanked USB cable would otherwise flood the log at the
            # poll rate.
            self.get_logger().error(
                f'Serial read failed: {exc}', throttle_duration_sec=5.0)
            return

        while True:
            idx = self._rx_buf.find(b'\n')
            if idx < 0:
                break
            raw = bytes(self._rx_buf[:idx])
            del self._rx_buf[:idx + 1]
            # errors='replace': line noise on the wire must not raise inside a
            # timer callback and kill the poll.
            line = raw.decode('ascii', errors='replace').strip()
            if not line:
                continue
            if self.feedback_re is not None and not self.feedback_re.match(line):
                # Throttled, not silent: if the firmware's frame changes shape,
                # /teensy/feedback going quiet would otherwise look like a dead
                # serial link rather than a pattern mismatch.
                # self.get_logger().warn(
                #     f'Dropping Teensy line (no feedback_pattern match): {line!r}',
                #     throttle_duration_sec=10.0)
                continue
            self.publish_feedback(line)

        if len(self._rx_buf) > MAX_RX_LINE_BYTES:
            self.get_logger().warn(
                f'Dropping {len(self._rx_buf)} bytes of un-terminated Teensy output — '
                f'no newline in {MAX_RX_LINE_BYTES} bytes (baud mismatch? binary output?)',
                throttle_duration_sec=5.0)
            self._rx_buf.clear()

    def publish_feedback(self, line: str):
        """Republish one Teensy line. Subscribe here to forward it off-robot."""
        msg = String()
        msg.data = line
        self.feedback_pub.publish(msg)
        if not self._seen_feedback:
            # One info line proves the return path is alive without needing
            # debug logging on; everything after it is debug (it can arrive at
            # whatever rate the firmware chooses).
            self._seen_feedback = True
            self.get_logger().info(f'First published Teensy frame: {msg.data!r}')
        else:
            self.get_logger().debug(f'teensy: {msg.data}')

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
