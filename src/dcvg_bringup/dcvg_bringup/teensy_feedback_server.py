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

CAMERA_DEVICE = '/dev/v4l/by-id/usb-Pico_Instruments_LLC_Pico_iCubie_Webcam_689076698948-video-index0'
CAMERA_SIZE = (320, 240)
OCR_PERIOD_S = 0.1
EXPECTED_FIELDS = 17
QUEUE_SIZE = 200
CLIENT_TIMEOUT_S = 5.0
DCVG_STALE_S = 5.0
GPS_STALE_S = 3.0
LOG_PERIOD_S = 10.0


class Latest:

    def __init__(self, value):
        self.lock = threading.Lock()
        self.value = value
        self.stamp = None

    def set(self, value):
        with self.lock:
            self.value = value
            self.stamp = time.monotonic()

    def get(self):
        with self.lock:
            age = None if self.stamp is None else time.monotonic() - self.stamp
            return self.value, age


class TelemetrySender(threading.Thread):

    def __init__(self, port, logger):
        super().__init__(daemon=True)
        self.port = port
        self.log = logger
        self.queue = collections.deque(maxlen=QUEUE_SIZE)
        self.wake = threading.Condition()
        self.halt = threading.Event()
        self.sent = 0
        self.dropped = 0

    def send(self, line):
        with self.wake:
            if len(self.queue) == QUEUE_SIZE:
                self.dropped += 1
            self.queue.append(line)
            self.wake.notify()

    def stop(self):
        self.halt.set()
        with self.wake:
            self.wake.notify()

    def run(self):
        try:
            listener = socket.create_server(('0.0.0.0', self.port))
            listener.settimeout(0.5)
        except OSError as exc:
            self.log.error(f'Cannot listen on port {self.port} ({exc}) — uplink disabled')
            return
        self.log.info(f'TCP telemetry server listening on port {self.port}')
        conn = None
        try:
            while not self.halt.is_set():
                if conn is None:
                    conn = self.accept(listener)
                    continue
                with self.wake:
                    if not self.queue:
                        self.wake.wait(timeout=0.5)
                    batch = list(self.queue)
                    self.queue.clear()
                if batch and not self.write(conn, batch):
                    conn.close()
                    conn = None
        finally:
            if conn is not None:
                conn.close()
            listener.close()

    def accept(self, listener):
        try:
            conn, addr = listener.accept()
        except OSError:
            return None
        conn.settimeout(CLIENT_TIMEOUT_S)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.log.info(f'Laptop connected from {addr[0]}:{addr[1]}')
        with self.wake:
            self.queue.clear()
        return conn

    def write(self, conn, batch):
        try:
            conn.sendall(''.join(batch).encode('utf-8'))
        except OSError as exc:
            self.log.warning(f'Laptop disconnected ({exc}) — waiting for a reconnect')
            return False
        self.sent += len(batch)
        return True


class OcrWorker(threading.Thread):

    def __init__(self, reading, logger, device):
        super().__init__(daemon=True)
        self.reading = reading
        self.log = logger
        self.device = device
        self.halt = threading.Event()
        self.failures = 0

    def stop(self):
        self.halt.set()

    def run(self):
        try:
            import cv2
            import easyocr
            from dcvg_bringup.dcvg_ocr import DcvgOcr
        except ImportError as exc:
            self.log.error(f'OCR disabled — {exc}. DCVG columns will read 0,0.')
            return
        cap = cv2.VideoCapture(self.device)
        if not cap.isOpened():
            self.log.error(f'OCR disabled — cannot open camera {self.device}. '
                           f'DCVG columns will read 0,0.')
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_SIZE[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_SIZE[1])
        try:
            self.log.info('Loading easyocr model...')
            ocr = DcvgOcr(easyocr.Reader(['en'], gpu=True))
        except Exception as exc:
            self.log.error(f'OCR disabled — setup failed: {exc}. DCVG columns will read 0,0.')
            cap.release()
            return
        self.log.info(f'DCVG OCR running on {self.device}')
        try:
            while not self.halt.is_set():
                started = time.monotonic()
                self.tick(cap, ocr)
                self.halt.wait(max(0.0, OCR_PERIOD_S - (time.monotonic() - started)))
        finally:
            cap.release()

    def tick(self, cap, ocr):
        ok, frame = cap.read()
        if not ok:
            self.failures += 1
            self.log.warning('Camera read failed — retrying', throttle_duration_sec=5.0)
            return
        try:
            on, off = ocr.read(frame)
        except Exception as exc:
            self.failures += 1
            self.log.warning(f'OCR failed on a frame: {exc}', throttle_duration_sec=5.0)
            return
        if on is not None and off is not None:
            self.reading.set((on, off))


class TeensyFeedbackServer(Node):

    def __init__(self):
        super().__init__('teensy_feedback_server')
        self.declare_parameter('tcp_port', 5005)
        self.declare_parameter('enable_ocr', True)
        self.declare_parameter('camera_device', CAMERA_DEVICE)

        self.dcvg = Latest((0, 0))
        self.gps = Latest((0.0, 0.0))
        self.frames = self.stamped = self.malformed = 0

        self.sender = TelemetrySender(self.get_parameter('tcp_port').value, self.get_logger())
        self.sender.start()
        self.ocr = None
        if self.get_parameter('enable_ocr').value:
            self.ocr = OcrWorker(self.dcvg, self.get_logger(),
                                 self.get_parameter('camera_device').value)
            self.ocr.start()

        self.telemetry_pub = self.create_publisher(String, '/dcvg/telemetry', 10)
        self.reading_pub = self.create_publisher(Int32MultiArray, '/dcvg/reading', 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.on_fix, 10)
        self.create_subscription(String, '/teensy/feedback', self.on_feedback, 50)
        self.create_timer(LOG_PERIOD_S, self.report)

    def on_fix(self, msg):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        self.gps.set((msg.latitude, msg.longitude))

    def on_feedback(self, msg):
        line = msg.data.strip()
        if not line:
            return
        self.frames += 1
        if not line.startswith('$'):
            return
        if not line.endswith('#'):
            self.emit(line)
            return

        (dcvg_on, dcvg_off), dcvg_age = self.dcvg.get()
        (lat, lon), gps_age = self.gps.get()
        self.warn_if_stale(dcvg_age, gps_age)

        stamped = f'${line[1:-1]},{dcvg_on},{dcvg_off},{lat:.8f},{lon:.8f}#'
        fields = stamped[1:-1].split(',')
        if len(fields) != EXPECTED_FIELDS or '' in fields:
            self.malformed += 1
            self.get_logger().warning(f'Dropping frame — {len(fields)} fields, expected '
                                      f'{EXPECTED_FIELDS}: {stamped}', throttle_duration_sec=5.0)
            return
        self.stamped += 1
        self.emit(stamped)
        self.reading_pub.publish(Int32MultiArray(data=[dcvg_on, dcvg_off]))

    def emit(self, line):
        self.telemetry_pub.publish(String(data=line))
        self.sender.send(line)

    def warn_if_stale(self, dcvg_age, gps_age):
        notes = []
        if self.ocr is not None and (dcvg_age is None or dcvg_age > DCVG_STALE_S):
            notes.append('no fresh DCVG reading')
        if gps_age is None or gps_age > GPS_STALE_S:
            notes.append('no fresh GPS fix')
        if notes:
            self.get_logger().warning('Stamping anyway — ' + ', '.join(notes),
                                      throttle_duration_sec=LOG_PERIOD_S)

    def report(self):
        ocr_failures = self.ocr.failures if self.ocr else 0
        self.get_logger().info(f'{self.frames} frames in, {self.stamped} stamped, '
                               f'{self.sender.sent} sent, {self.sender.dropped} dropped, '
                               f'{self.malformed} malformed, {ocr_failures} OCR failures')

    def shutdown(self):
        if self.ocr is not None:
            self.ocr.stop()
            self.ocr.join(timeout=2.0)
        self.sender.stop()
        self.sender.join(timeout=2.0)


def main():
    rclpy.init()
    node = TeensyFeedbackServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
