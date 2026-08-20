"""
gps_heading_to_imu.py  (REAL HARDWARE — GPS moving-baseline heading → EKF)

robot_localization only accepts nav_msgs/Odometry, geometry_msgs/Pose/TwistWithCovariance,
or sensor_msgs/Imu — it cannot subscribe to ublox_msgs/NavRELPOSNED9 directly. This node
bridges the heading board's raw moving-baseline vector onto a plain sensor_msgs/Imu
(orientation-only) so it can be fused as an imu1 input in dual_ekf_navsat_params.yaml.

Source: /gps/heading/navrelposned (ublox_msgs/NavRELPOSNED9, from the heading board —
see ublox_gps.launch.py). rel_pos_heading is the bearing of the base→rover vector in the
local NED frame: 0 = North, increasing clockwise, units 1e-5 deg. That's a *true* heading
(no magnetometer involved, no magnetic declination to correct for) but it is the bearing
of the antenna baseline, not necessarily the robot's forward axis — `heading_offset_deg`
corrects for whatever the physical antenna mounting is relative to base_link's +X.

NED compass bearing → ENU yaw (REP-103, 0 = East, CCW+): yaw = pi/2 - bearing, then the
mounting offset is added and the result normalized to [-pi, pi].

Output: /gps/heading/imu (sensor_msgs/Imu), orientation quaternion (yaw only, roll=pitch=0),
angular_velocity/linear_acceleration covariance[0] = -1 (per REP-145, "field not populated")
so nothing downstream mistakes this for a real IMU reading. orientation_covariance's yaw
term comes from acc_heading (u-blox's own estimate, already degrades appropriately between
RTK FIXED/FLOAT), floored at min_heading_std_deg so a lucky short-baseline estimate can't
make the EKF overconfident.

Only publishes when FLAGS_GNSS_FIX_OK and FLAGS_REL_POS_HEAD_VALID are both set — an
invalid heading is worse than a missing one (dead reckoning fills the gap; a wrong
absolute yaw fusion doesn't self-correct).

Run (normally started by ublox_gps.launch.py, right after the heading board node):
  ros2 run dcvg_bringup gps_heading_to_imu
  ros2 run dcvg_bringup gps_heading_to_imu --ros-args -p heading_offset_deg:=90.0
"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from ublox_msgs.msg import NavRELPOSNED9

DEG_TO_RAD = math.pi / 180.0
HEADING_SCALE = 1e-5           # rel_pos_heading / acc_heading units → deg
MIN_HEADING_STD_DEG = 2.0      # floor on reported accuracy — u-blox can be optimistic.
# Was temporarily raised to 15.0 to damp a feedback loop in ekf_filter_node_map
# (world_frame=map): it used to fuse odom0 (/odometry/gps, frame_id="odom") via
# robot_localization's absolute-pose path, which looked up its OWN just-published
# map->odom TF to interpret each new GPS position fix — any yaw correction perturbed
# that lookup, which corrupted the next position fix, compounding without bound
# (confirmed on hardware: position ran away past 10^7 m from a ~4 m GPS input within
# under a minute, even parked). That was a structural bug, not something a covariance
# floor could actually fix — damping only slowed the runaway. It's now fixed properly
# in localization.launch.py (navsat_transform's odometry/filtered input remapped to
# odometry/global so /odometry/gps publishes directly in the "map" frame, making the
# lookup unnecessary), confirmed on hardware to hold position stable. That structural
# fix means this floor no longer needs to be inflated for ekf_filter_node_map's sake —
# and ekf_filter_node_odom (world_frame=odom) never had this feedback risk at all, yet
# was equally damped by the shared 15 deg floor on this topic, measured on hardware as
# a persistent ~15 deg gap between /gps/heading/imu's true heading and ekf_local's
# fused yaw (the absolute correction was too weak to out-pull VIO's and the camera
# gyro's competing vyaw fusion). Back to a realistic sensor-accuracy floor.

REQUIRED_FLAGS = (NavRELPOSNED9.FLAGS_GNSS_FIX_OK
                   | NavRELPOSNED9.FLAGS_REL_POS_HEAD_VALID)

CARR_NAME = {
    NavRELPOSNED9.FLAGS_CARR_SOLN_NONE: 'NONE (no RTK)',
    NavRELPOSNED9.FLAGS_CARR_SOLN_FLOAT: 'RTK FLOAT',
    NavRELPOSNED9.FLAGS_CARR_SOLN_FIXED: 'RTK FIXED',
}


class GpsHeadingToImu(Node):

    def __init__(self):
        super().__init__('gps_heading_to_imu')

        self.declare_parameter('heading_offset_deg', 0.0)
        self.declare_parameter('min_heading_std_deg', MIN_HEADING_STD_DEG)
        self._offset_rad = float(self.get_parameter('heading_offset_deg').value) * DEG_TO_RAD
        self._min_std_rad = float(self.get_parameter('min_heading_std_deg').value) * DEG_TO_RAD

        self._pub = self.create_publisher(Imu, '/gps/heading/imu', 10)
        self.create_subscription(
            NavRELPOSNED9, '/gps/heading/navrelposned', self._relposned_cb, 10)

        self._last_carr = None
        self._dropped = 0

        self.get_logger().info(
            f"[gps_heading_to_imu] /gps/heading/navrelposned → /gps/heading/imu "
            f"(offset={math.degrees(self._offset_rad):.1f} deg, "
            f"min_std={math.degrees(self._min_std_rad):.1f} deg)")

    def _relposned_cb(self, msg: NavRELPOSNED9):
        carr = msg.flags & NavRELPOSNED9.FLAGS_CARR_SOLN_MASK
        if carr != self._last_carr:
            self.get_logger().info(
                f"GPS heading carrier solution: "
                f"{CARR_NAME.get(self._last_carr, 'startup')} → {CARR_NAME.get(carr, '?')}")
            self._last_carr = carr

        if (msg.flags & REQUIRED_FLAGS) != REQUIRED_FLAGS:
            self._dropped += 1
            self.get_logger().warning(
                f"GPS heading invalid (flags=0x{msg.flags:x}) — not publishing "
                f"({self._dropped} dropped so far)", throttle_duration_sec=5.0)
            return

        bearing_rad = (msg.rel_pos_heading * HEADING_SCALE) * DEG_TO_RAD
        yaw = _normalize(math.pi / 2.0 - bearing_rad + self._offset_rad)
        # yaw = _normalize(bearing_rad + self._offset_rad)

        std_rad = max(self._min_std_rad,
                       (msg.acc_heading * HEADING_SCALE) * DEG_TO_RAD)
        yaw_var = std_rad * std_rad

        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'gps_heading_link'

        half_yaw = yaw / 2.0
        out.orientation.x = 0.0
        out.orientation.y = 0.0
        out.orientation.z = math.sin(half_yaw)
        out.orientation.w = math.cos(half_yaw)
        out.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, yaw_var,
        ]

        # Not populated — tells consumers (and robot_localization) to ignore these.
        out.angular_velocity_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0

        self._pub.publish(out)


def _normalize(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    rclpy.init()
    node = GpsHeadingToImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
