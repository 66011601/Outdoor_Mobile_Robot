import os

import rclpy
from nmea_msgs.msg import Sentence
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rtcm_msgs.msg import Message as RtcmMessage
from ublox_msgs.msg import NavPVT

STATS_PERIOD_S = 10.0
CARRIER_NAMES = {
    NavPVT.CARRIER_PHASE_NO_SOLUTION: 'NONE (no RTK)',
    NavPVT.CARRIER_PHASE_FLOAT: 'RTK FLOAT',
    NavPVT.CARRIER_PHASE_FIXED: 'RTK FIXED',
}


def make_gga(msg):
    lat, lon = msg.lat * 1e-7, msg.lon * 1e-7
    alat, alon = abs(lat), abs(lon)
    lat_dmm = f'{int(alat):02d}{(alat - int(alat)) * 60:010.7f}'
    lon_dmm = f'{int(alon):03d}{(alon - int(alon)) * 60:010.7f}'

    carrier = msg.flags & NavPVT.FLAGS_CARRIER_PHASE_MASK
    if carrier == NavPVT.CARRIER_PHASE_FIXED:
        quality = 4
    elif carrier == NavPVT.CARRIER_PHASE_FLOAT:
        quality = 5
    elif msg.flags & NavPVT.FLAGS_DIFF_SOLN:
        quality = 2
    else:
        quality = 1

    utc = f'{msg.hour:02d}{msg.min:02d}{msg.sec:02d}.{max(0, msg.nano) // 10_000_000:02d}'
    body = (f"GPGGA,{utc},{lat_dmm},{'N' if lat >= 0 else 'S'},{lon_dmm},"
            f"{'E' if lon >= 0 else 'W'},{quality},{msg.num_sv:02d},{msg.p_dop * 0.01:.1f},"
            f"{msg.h_msl * 1e-3:.1f},M,{(msg.height - msg.h_msl) * 1e-3:.1f},M,,")
    checksum = 0
    for ch in body:
        checksum ^= ord(ch)
    return f'${body}*{checksum:02X}\r\n'


class NtripBridge(Node):

    def __init__(self):
        super().__init__('ntrip_bridge')
        self.declare_parameter('device', '/dev/ttyGPS_movingbase')
        self.declare_parameter('gga_period', 1.0)
        self.device = str(self.get_parameter('device').value)
        self.gga_period = float(self.get_parameter('gga_period').value)
        self.fd = None
        self.rtcm_count = 0
        self.rtcm_bytes = 0
        self.gga_count = 0
        self.last_gga = 0.0
        self.last_carrier = None
        self.nmea_pub = self.create_publisher(Sentence, '/nmea', 10)
        self.create_subscription(RtcmMessage, '/rtcm', self.on_rtcm, 10)
        self.create_subscription(NavPVT, '/navpvt', self.on_navpvt, 10)
        self.create_timer(STATS_PERIOD_S, self.log_stats)

    def on_rtcm(self, msg):
        if self.fd is None:
            try:
                self.fd = os.open(self.device, os.O_WRONLY | os.O_NOCTTY)
                self.get_logger().info(f'Opened {self.device} write-only for RTCM')
            except OSError as e:
                self.get_logger().warning(f'Cannot open {self.device} ({e.strerror}) — RTCM dropped',
                                          throttle_duration_sec=5.0)
                return
        try:
            os.write(self.fd, bytes(msg.message))
            self.rtcm_count += 1
            self.rtcm_bytes += len(msg.message)
        except OSError as e:
            self.get_logger().warning(f'Write to {self.device} failed ({e.strerror}) — reopening')
            os.close(self.fd)
            self.fd = None

    def on_navpvt(self, msg):
        carrier = msg.flags & NavPVT.FLAGS_CARRIER_PHASE_MASK
        if carrier != self.last_carrier:
            self.get_logger().info(f"RTK carrier: {CARRIER_NAMES.get(self.last_carrier, 'startup')}"
                                   f" → {CARRIER_NAMES.get(carrier, '?')}")
            self.last_carrier = carrier

        now = self.get_clock().now()
        if msg.fix_type < NavPVT.FIX_TYPE_2D or now.nanoseconds * 1e-9 - self.last_gga < self.gga_period:
            return
        self.last_gga = now.nanoseconds * 1e-9
        out = Sentence()
        out.header.stamp = now.to_msg()
        out.header.frame_id = 'gps_link'
        out.sentence = make_gga(msg)
        self.nmea_pub.publish(out)
        self.gga_count += 1

    def log_stats(self):
        if self.rtcm_count == 0:
            self.get_logger().warning(f'No RTCM from caster yet (GGA sent: {self.gga_count}). '
                                      f'Check ntrip_client login/mountpoint and /navpvt.')
        else:
            self.get_logger().info(f'RTCM {self.rtcm_count} msgs / {self.rtcm_bytes / 1024:.1f} KB '
                                   f'→ {self.device}, GGA sent {self.gga_count}, carrier: '
                                   f"{CARRIER_NAMES.get(self.last_carrier, '?')}")


def main():
    rclpy.init()
    node = NtripBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.fd is not None:
            os.close(node.fd)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
