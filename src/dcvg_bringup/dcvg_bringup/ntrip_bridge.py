"""
ntrip_bridge.py  (REAL HARDWARE — RTK corrections)

Glue between ntrip_client and the F9P on the simpleRTK2B Heading. Two jobs:

  1. /rtcm (rtcm_msgs/Message, from ntrip_client) → raw RTCM3 bytes written to
     the F9P serial port.  The humble ublox_gps driver has NO rtcm-in topic, so
     corrections must be written straight onto the port. ublox_gps already owns
     /dev/ttyACM0 read/write, but Linux ttys allow a second WRITE-ONLY open —
     we open O_WRONLY|O_NOCTTY and never read, so we don't steal UBX bytes
     from the driver.

  2. /navpvt (ublox_msgs/NavPVT, from ublox_gps) → NMEA GGA sentence on /nmea
     (nmea_msgs/Sentence, consumed by ntrip_client).  The caster mountpoint is
     a VRS — it sends NOTHING until it receives a GGA with our approximate
     position.  ntrip_client's built-in fix→GGA fallback spams at full fix
     rate with a fake altitude, so we publish our own: real altitude, real
     RTK quality field, throttled to gga_period.

Also logs every RTK carrier-solution transition (NONE → FLOAT → FIXED), so the
terminal shows at a glance whether corrections are taking effect.

Requires: RTCM3 enabled as USB *input* protocol on the F9P (factory default;
check CFG-USBINPROT-RTCM3X in u-center if FLOAT never appears).

Run (next to ublox_gps.launch.py; normally started by ntrip.launch.py):
  ros2 run dcvg_bringup ntrip_bridge
  ros2 run dcvg_bringup ntrip_bridge --ros-args -p device:=/dev/serial/by-id/usb-u-blox_...-if00
"""

import os

import rclpy
from rclpy.node import Node

from nmea_msgs.msg import Sentence
from rtcm_msgs.msg import Message as RtcmMessage
from ublox_msgs.msg import NavPVT

DEVICE = '/dev/ttyGPS_movingbase'   # position/base board — must match ublox_gps.launch.py's `device`
GGA_PERIOD = 1.0          # s between GGA uploads to the caster (1 Hz is plenty)
STATS_PERIOD = 10.0       # s between "corrections flowing?" summary logs

CARR_NAME = {NavPVT.CARRIER_PHASE_NO_SOLUTION: 'NONE (no RTK)',
             NavPVT.CARRIER_PHASE_FLOAT: 'RTK FLOAT',
             NavPVT.CARRIER_PHASE_FIXED: 'RTK FIXED'}


class NtripBridge(Node):

    def __init__(self):
        super().__init__('ntrip_bridge')

        self.declare_parameter('device', DEVICE)
        self.declare_parameter('gga_period', GGA_PERIOD)
        self._device = str(self.get_parameter('device').value)
        self._gga_period = float(self.get_parameter('gga_period').value)

        self._fd: int | None = None       # write-only fd to the F9P port
        self._rtcm_count = 0
        self._rtcm_bytes = 0
        self._gga_count = 0
        self._last_gga_stamp = 0.0
        self._last_carr = None

        self._nmea_pub = self.create_publisher(Sentence, '/nmea', 10)
        self.create_subscription(RtcmMessage, '/rtcm', self._rtcm_cb, 10)
        self.create_subscription(NavPVT, '/navpvt', self._navpvt_cb, 10)
        self.create_timer(STATS_PERIOD, self._stats_cb)

        self.get_logger().info(
            f"[ntrip_bridge] RTCM /rtcm → {self._device} (write-only), "
            f"GGA from /navpvt → /nmea every {self._gga_period:.1f} s")

    # ── RTCM → serial ────────────────────────────────────────────────────────

    def _ensure_port(self) -> int | None:
        if self._fd is not None:
            return self._fd
        try:
            self._fd = os.open(self._device, os.O_WRONLY | os.O_NOCTTY)
            self.get_logger().info(f"Opened {self._device} write-only for RTCM")
        except OSError as e:
            self.get_logger().warning(
                f"Cannot open {self._device} ({e.strerror}) — RTCM dropped. "
                f"Is the receiver plugged in / device arg correct?",
                throttle_duration_sec=5.0)
        return self._fd

    def _rtcm_cb(self, msg: RtcmMessage):
        fd = self._ensure_port()
        if fd is None:
            return
        try:
            os.write(fd, bytes(msg.message))
            self._rtcm_count += 1
            self._rtcm_bytes += len(msg.message)
        except OSError as e:
            self.get_logger().warning(
                f"Write to {self._device} failed ({e.strerror}) — reopening")
            os.close(fd)
            self._fd = None

    # ── NavPVT → GGA ─────────────────────────────────────────────────────────

    def _navpvt_cb(self, msg: NavPVT):
        carr = msg.flags & NavPVT.FLAGS_CARRIER_PHASE_MASK
        if carr != self._last_carr:
            self.get_logger().info(
                f"RTK carrier solution: {CARR_NAME.get(self._last_carr, 'startup')} "
                f"→ {CARR_NAME.get(carr, '?')}")
            self._last_carr = carr

        if msg.fix_type < NavPVT.FIX_TYPE_2D:
            return                        # no position yet — nothing to tell the VRS
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_gga_stamp < self._gga_period:
            return
        self._last_gga_stamp = now

        out = Sentence()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'gps_link'
        out.sentence = self._make_gga(msg)
        self._nmea_pub.publish(out)
        self._gga_count += 1

    @staticmethod
    def _make_gga(msg: NavPVT) -> str:
        lat = msg.lat * 1e-7
        lon = msg.lon * 1e-7
        lat_hemi = 'N' if lat >= 0 else 'S'
        lon_hemi = 'E' if lon >= 0 else 'W'
        alat, alon = abs(lat), abs(lon)
        lat_dmm = f"{int(alat):02d}{(alat - int(alat)) * 60:010.7f}"
        lon_dmm = f"{int(alon):03d}{(alon - int(alon)) * 60:010.7f}"

        carr = msg.flags & NavPVT.FLAGS_CARRIER_PHASE_MASK
        if carr == NavPVT.CARRIER_PHASE_FIXED:
            quality = 4                   # RTK fixed
        elif carr == NavPVT.CARRIER_PHASE_FLOAT:
            quality = 5                   # RTK float
        elif msg.flags & NavPVT.FLAGS_DIFF_SOLN:
            quality = 2                   # DGNSS
        else:
            quality = 1                   # plain GNSS

        hundredths = max(0, msg.nano) // 10_000_000
        utc = f"{msg.hour:02d}{msg.min:02d}{msg.sec:02d}.{hundredths:02d}"
        hdop = msg.p_dop * 0.01           # pDOP as HDOP stand-in — fine for a VRS
        alt_msl = msg.h_msl * 1e-3
        geoid_sep = (msg.height - msg.h_msl) * 1e-3

        body = (f"GPGGA,{utc},{lat_dmm},{lat_hemi},{lon_dmm},{lon_hemi},"
                f"{quality},{msg.num_sv:02d},{hdop:.1f},{alt_msl:.1f},M,"
                f"{geoid_sep:.1f},M,,")
        checksum = 0
        for ch in body:
            checksum ^= ord(ch)
        return f"${body}*{checksum:02X}\r\n"

    # ── heartbeat ────────────────────────────────────────────────────────────

    def _stats_cb(self):
        if self._rtcm_count == 0:
            self.get_logger().warning(
                f"No RTCM from caster yet (GGA sent: {self._gga_count}). "
                f"Check ntrip_client login/mountpoint and that /navpvt is publishing.")
        else:
            self.get_logger().info(
                f"RTCM {self._rtcm_count} msgs / {self._rtcm_bytes / 1024:.1f} KB "
                f"→ {self._device}   GGA sent {self._gga_count}   "
                f"carrier: {CARR_NAME.get(self._last_carr, '?')}")


def main():
    rclpy.init()
    node = NtripBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._fd is not None:
            os.close(node._fd)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
