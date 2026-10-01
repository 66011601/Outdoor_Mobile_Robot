"""
teensy_feedback_server.py  (REAL HARDWARE — survey telemetry stamper + TCP uplink)

Takes the Teensy's telemetry frames off /teensy/feedback, appends the DCVG meter
reading (read off the meter's LCD by `dcvg_ocr`) and the RTK position, and serves
the result to the laptop over TCP. Port of ~/Desktop/DCVG_Robot/main/
main_gps_r2_withautosensors.py, minus the parts `low_level_command` and the
ublox driver already own.

Wire format
───────────
In, from the Teensy — opaque, comma-separated, '$'…'#' delimited:

    $0,0,0,0,0,0,1,1,100,100,0,0,0#                                    13 fields

Out, four fields appended:

    $0,0,0,0,0,0,1,1,100,100,0,0,0,52,-310,51.05112340,-114.08529870#  17 fields
                                   └ on └ off └ lat       └ lon

The laptop indexes those four from the END, so the Teensy half can grow without
the laptop changing. `expected_fields` (17) is the total it currently expects; a
frame that does not total exactly that is dropped, because a short frame shifts
every column in the survey log. Set it to 0 when the firmware's field count
changes. No trailing newline — the laptop splits on the delimiters.

Topics
──────
    in   /teensy/feedback   (String)          from low_level_command
    in   /gps/fix           (NavSatFix)       from ublox_gps
    out  /dcvg/telemetry    (String)          exactly what goes on the wire
    out  /dcvg/reading      (Int32MultiArray) [dcvg_on, dcvg_off]

Parameters
──────────
    feedback_topic   /teensy/feedback
    gps_topic        /gps/fix
    tcp_host         0.0.0.0        interface to serve the laptop on
    tcp_port         5005
    expected_fields  17             0 disables the field-count check
    enable_ocr       True           False → no camera needed, stamps 0,0
    camera_device    by-id path to the meter webcam
    ocr_period       0.1            seconds; a cap, not an added delay
    log_period       10.0           seconds between the counters line

Everything else is a constant below — tuning values, not knobs.

Debugging
─────────
    ros2 run dcvg_bringup teensy_feedback_server
    ros2 run dcvg_bringup teensy_feedback_server --ros-args -p enable_ocr:=false
    ros2 topic echo /dcvg/telemetry     # the wire, without needing the laptop
    ros2 topic echo /dcvg/reading       # the OCR pair alone

The counters line (every `log_period`) is the first thing to read:

    13 frames in, 13 stamped, 7 sent, 0 dropped

  frames in == 0      low_level_command is not publishing — check it is running
                      and that its `feedback_pattern` is not filtering everything
  stamped  < in       frames malformed or truncated; see the WARN above it
  sent     < stamped  no laptop connected (frames are dropped, not queued
                      forever), or it disconnected
  dropped  > 0        the laptop cannot keep up; oldest frames are being lost

A stale or missing DCVG/GPS input is a throttled WARN, not a failure: the last
known value keeps being stamped so the survey continues.

Notes for whoever changes this
──────────────────────────────
  - Three threads. The ROS executor only does string work. The OCR worker is
    separate because easyocr is 10s-100s of ms per frame; the TCP sender is
    separate because sendall() blocks on a half-open laptop. Keep it that way:
    a stall in either must not reach the executor.
  - The uplink is not a precondition. With no laptop attached the rover still
    surveys and frames are simply dropped; a client connecting later gets
    current telemetry, not a replay.
  - OCR is optional at runtime. A missing camera, missing easyocr or a failed
    model load logs an error and degrades to stamping 0,0 — it never stops the
    node from starting.

Requires opencv-python + easyocr when `enable_ocr` is true; both are imported
inside the OCR thread so the node still runs where they are absent.
"""

import collections
import math
import socket
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Int32MultiArray, String

# The meter webcam. A by-id path, not /dev/videoN: that index depends on
# enumeration order and would swap with the RealSense on a replug.
DEFAULT_CAMERA = ('/dev/v4l/by-id/'
                  'usb-Pico_Instruments_LLC_Pico_iCubie_Webcam_689076698948-video-index0')

# 13 Teensy fields + dcvg_on, dcvg_off, lat, lon.
DEFAULT_EXPECTED_FIELDS = 17

# dcvg_ocr's geometry defaults are keyed to this frame size — changing it needs
# every region in OcrGeometry re-derived, so it is not a parameter.
CAMERA_SIZE = (320, 240)
OCR_LANGUAGES = ['en']

POSITION_FMT = '%.8f'       # RTK is cm-level; fewer decimals throw that away
QUEUE_SIZE = 200            # frames buffered for a slow laptop, then drop-oldest
CLIENT_TIMEOUT = 5.0        # s in sendall() before a laptop is declared gone
DCVG_TIMEOUT = 5.0          # s before a reading is called stale in the log
GPS_TIMEOUT = 3.0


class _Slot:
    """The latest value from one async source, with its age.

    Written by a producer (OCR thread, GPS callback), read by the stamper. The
    lock covers only the assignment, so a slow producer never delays a read.
    """

    def __init__(self, value):
        self._lock = threading.Lock()
        self._value = value
        self._stamp = None

    def set(self, value):
        with self._lock:
            self._value = value
            self._stamp = time.monotonic()

    def get(self):
        """(value, age_seconds). age is None if nothing has been written yet."""
        with self._lock:
            if self._stamp is None:
                return self._value, None
            return self._value, time.monotonic() - self._stamp


class _TelemetrySender(threading.Thread):
    """Serves stamped frames to one laptop at a time. `send()` never blocks."""

    def __init__(self, host, port, logger):
        super().__init__(name='telemetry_sender', daemon=True)
        self._host = host
        self._port = port
        self._log = logger
        # Drop-oldest: this is a live feed, so what to lose under back-pressure
        # is the backlog, not the present.
        self._queue = collections.deque(maxlen=QUEUE_SIZE)
        self._wake = threading.Condition()
        self._halt = threading.Event()
        self.sent = 0
        self.dropped = 0

    def send(self, line: str):
        with self._wake:
            if len(self._queue) == QUEUE_SIZE:
                self.dropped += 1
            self._queue.append(line)
            self._wake.notify()

    def stop(self):
        self._halt.set()
        with self._wake:
            self._wake.notify()

    def run(self):
        listener = self._listen()
        if listener is None:
            return
        conn = None
        try:
            while not self._halt.is_set():
                if conn is None:
                    conn = self._accept(listener)
                    continue
                batch = self._take()
                if batch and not self._write(conn, batch):
                    conn.close()
                    conn = None
        finally:
            if conn is not None:
                conn.close()
            listener.close()
            # Not self._log: on Ctrl-C rclpy's signal handler kills the context
            # before this thread is joined, and a logger call past that point
            # prints "publisher's context is invalid" instead of the message.
            note = f'Telemetry uplink closed ({self.sent} sent, {self.dropped} dropped)'
            if rclpy.ok():
                self._log.info(note)
            else:
                print(f'[teensy_feedback_server] {note}', flush=True)

    def _listen(self):
        try:
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((self._host, self._port))
            listener.listen(1)
            listener.settimeout(0.5)     # so shutdown is not stuck in accept()
        except OSError as exc:
            self._log.error(f'Cannot listen on {self._host}:{self._port} ({exc}) — '
                            f'uplink disabled, the node keeps running')
            return None
        self._log.info(f'TCP telemetry server listening on {self._host}:{self._port}')
        return listener

    def _accept(self, listener):
        try:
            conn, addr = listener.accept()
        except socket.timeout:
            return None
        except OSError as exc:
            if not self._halt.is_set():
                self._log.error(f'accept() failed: {exc}')
            return None
        conn.settimeout(CLIENT_TIMEOUT)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._log.info(f'Laptop connected from {addr[0]}:{addr[1]}')
        # Whatever queued up while nobody was attached is backlog by definition.
        with self._wake:
            self._queue.clear()
        return conn

    def _take(self):
        """Everything queued, waiting briefly if empty. May return []."""
        with self._wake:
            if not self._queue and not self._halt.is_set():
                self._wake.wait(timeout=0.5)
            batch = list(self._queue)
            self._queue.clear()
        return batch

    def _write(self, conn, batch):
        """False means the client is gone and should be dropped."""
        try:
            conn.sendall(''.join(batch).encode('utf-8'))
        except socket.timeout:
            self._log.warning(f'Laptop stopped reading for {CLIENT_TIMEOUT:.0f}s '
                              f'(half-open link?) — waiting for a reconnect')
            return False
        except OSError as exc:
            self._log.warning(f'Laptop disconnected ({exc}) — waiting for a reconnect')
            return False
        self.sent += len(batch)
        return True


class _OcrWorker(threading.Thread):
    """Keeps `slot` at the meter's latest (on, off).

    Camera open and the ~5 s easyocr model load happen in here, so any failure
    degrades the node to stamping 0,0 rather than stopping it from starting.
    """

    def __init__(self, slot, logger, device, period):
        super().__init__(name='dcvg_ocr', daemon=True)
        self._slot = slot
        self._log = logger
        self._device = device
        self._period = period
        self._halt = threading.Event()
        self.failures = 0

    def stop(self):
        self._halt.set()

    def run(self):
        setup = self._setup()
        if setup is None:
            return
        cap, ocr = setup
        try:
            while not self._halt.is_set():
                started = time.monotonic()
                self._tick(cap, ocr)
                # Pace off time actually spent, so period is a cap rather than
                # something added to the inference time.
                self._halt.wait(max(0.0, self._period - (time.monotonic() - started)))
        finally:
            cap.release()

    def _setup(self):
        """(cap, ocr), or None with an error logged."""
        try:
            import cv2
            import easyocr
            from dcvg_bringup.dcvg_ocr import DcvgOcr
        except ImportError as exc:
            self._log.error(f'OCR disabled — {exc}. Install opencv-python and easyocr, '
                            f'or set enable_ocr:=false. DCVG columns will read 0,0.')
            return None

        cap = cv2.VideoCapture(self._device)
        if not cap.isOpened():
            self._log.error(f'OCR disabled — cannot open camera {self._device}. Check '
                            f'the path (ls /dev/v4l/by-id/) and that nothing else holds '
                            f'it; V4L2 is exclusive. DCVG columns will read 0,0.')
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_SIZE[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_SIZE[1])
        actual = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                  int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if actual != CAMERA_SIZE:
            self._log.warning(f'Camera gave {actual[0]}x{actual[1]}, not '
                              f'{CAMERA_SIZE[0]}x{CAMERA_SIZE[1]} — the OCR regions are '
                              f'keyed to that size and will not line up.')
        try:
            self._log.info('Loading easyocr model (a few seconds)...')
            ocr = DcvgOcr(easyocr.Reader(OCR_LANGUAGES, gpu=True))
        except Exception as exc:       # noqa: BLE001 - model / template load
            self._log.error(f'OCR disabled — setup failed: {exc}. '
                            f'DCVG columns will read 0,0.')
            cap.release()
            return None
        self._log.info(f'DCVG OCR running on {self._device} at '
                       f'{1.0 / self._period:.1f} Hz max')
        return cap, ocr

    def _tick(self, cap, ocr):
        ok, frame = cap.read()
        if not ok:
            self.failures += 1
            self._log.warning('Camera read failed — retrying', throttle_duration_sec=5.0)
            return
        try:
            reading = ocr.read(frame)
        except Exception as exc:       # noqa: BLE001 - keep the survey alive
            self.failures += 1
            self._log.warning(f'OCR failed on a frame: {exc}', throttle_duration_sec=5.0)
            return
        # Only a full reading refreshes the slot: pairing this frame's ON with a
        # stale OFF would look like a plausible measurement rather than a miss.
        if reading.confident:
            self._slot.set((reading.on, reading.off))


class TeensyFeedbackServer(Node):

    def __init__(self):
        super().__init__('teensy_feedback_server')

        self.declare_parameter('feedback_topic', '/teensy/feedback')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('tcp_host', '0.0.0.0')
        self.declare_parameter('tcp_port', 5005)
        self.declare_parameter('expected_fields', DEFAULT_EXPECTED_FIELDS)
        self.declare_parameter('enable_ocr', True)
        self.declare_parameter('camera_device', DEFAULT_CAMERA)
        self.declare_parameter('ocr_period', 0.1)
        self.declare_parameter('log_period', 10.0)

        p = self.get_parameter
        feedback_topic = p('feedback_topic').value
        gps_topic = p('gps_topic').value
        host, port = p('tcp_host').value, p('tcp_port').value
        self._expected_fields = p('expected_fields').value
        self._log_period = p('log_period').value

        # Both start at zero so a frame can always be stamped; `age is None`
        # then distinguishes "never received" from "genuinely reading zero".
        self._dcvg = _Slot((0, 0))
        self._gps = _Slot((0.0, 0.0))
        self._frames = self._stamped = self._malformed = 0

        self._sender = _TelemetrySender(host, port, self.get_logger())
        self._sender.start()

        self._ocr = None
        if p('enable_ocr').value:
            self._ocr = _OcrWorker(self._dcvg, self.get_logger(),
                                   p('camera_device').value, p('ocr_period').value)
            self._ocr.start()
        else:
            self.get_logger().info('enable_ocr:=false — DCVG columns will read 0,0')

        self._telemetry_pub = self.create_publisher(String, '/dcvg/telemetry', 10)
        self._reading_pub = self.create_publisher(Int32MultiArray, '/dcvg/reading', 10)
        self.create_subscription(NavSatFix, gps_topic, self.on_fix, 10)
        self.create_subscription(String, feedback_topic, self.on_feedback, 50)
        self.create_timer(self._log_period, self.report)

        self.get_logger().info(
            f'Stamping {feedback_topic} with DCVG + {gps_topic} → TCP {host}:{port} '
            + (f'({self._expected_fields} fields expected)' if self._expected_fields
               else '(field count unchecked)'))

    # ── Inputs ──────────────────────────────────────────────────────────────
    def on_fix(self, msg: NavSatFix):
        # NaN is what the F9P publishes before a fix, and would otherwise reach
        # the laptop as the literal string 'nan'.
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        self._gps.set((msg.latitude, msg.longitude))

    def on_feedback(self, msg: String):
        line = msg.data.strip()
        if not line:
            return
        self._frames += 1

        if not line.startswith('$'):
            # Teensy boot chatter. Never forwarded: nothing delimits records on
            # the wire, so it would concatenate onto the fragment before it and
            # break the laptop's '$'-split. Normally low_level_command's
            # feedback_pattern has already dropped these.
            self.get_logger().info(f'Ignoring non-telemetry line: {line!r}',
                                   throttle_duration_sec=self._log_period)
            return

        if not line.endswith('#'):
            # A truncated frame (Teensy reset mid-write). Forwarded unstamped so
            # it arrives visibly short rather than silently mis-stamped.
            self._emit(line)
            return

        (dcvg_on, dcvg_off), dcvg_age = self._dcvg.get()
        (lat, lon), gps_age = self._gps.get()
        self._warn_if_stale(dcvg_age, gps_age)

        stamped = (f'${line[1:-1]},{dcvg_on},{dcvg_off},'
                   f'{POSITION_FMT % lat},{POSITION_FMT % lon}#')
        fields = stamped[1:-1].split(',')
        if self._expected_fields and (len(fields) != self._expected_fields
                                      or any(f == '' for f in fields)):
            # Dropped, not sent: a wrong field count shifts every column in the
            # laptop's log, which is worse than a missing sample.
            self._malformed += 1
            self.get_logger().warning(
                f'Dropping frame — {len(fields)} fields, expected '
                f'{self._expected_fields}: {stamped}', throttle_duration_sec=5.0)
            return

        self._stamped += 1
        self._emit(stamped)
        self._reading_pub.publish(Int32MultiArray(data=[dcvg_on, dcvg_off]))

    # ── Output ──────────────────────────────────────────────────────────────
    def _emit(self, line: str):
        self._telemetry_pub.publish(String(data=line))
        self._sender.send(line)

    def _warn_if_stale(self, dcvg_age, gps_age):
        notes = []
        if self._ocr is not None:
            if dcvg_age is None:
                notes.append('no DCVG reading yet')
            elif dcvg_age > DCVG_TIMEOUT:
                notes.append(f'DCVG reading {dcvg_age:.1f}s stale (camera on the meter?)')
        if gps_age is None:
            notes.append('no GPS fix yet')
        elif gps_age > GPS_TIMEOUT:
            notes.append(f'GPS fix {gps_age:.1f}s stale')
        if notes:
            self.get_logger().warning('Stamping anyway — ' + ', '.join(notes),
                                      throttle_duration_sec=self._log_period)

    def report(self):
        extra = f', {self._malformed} malformed' if self._malformed else ''
        if self._ocr is not None and self._ocr.failures:
            extra += f', {self._ocr.failures} OCR failures'
        self.get_logger().info(
            f'{self._frames} frames in, {self._stamped} stamped, '
            f'{self._sender.sent} sent, {self._sender.dropped} dropped{extra}')

    def shutdown(self):
        if self._ocr is not None:
            self._ocr.stop()
            self._ocr.join(timeout=2.0)
        self._sender.stop()
        self._sender.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = TeensyFeedbackServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C under `ros2 run` arrives as ExternalShutdownException, not
        # KeyboardInterrupt — rclpy's SIGINT handler tears the context down
        # first. Catching only the latter leaves a traceback on every stop.
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
