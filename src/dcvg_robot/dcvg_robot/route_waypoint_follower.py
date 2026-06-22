"""
route_waypoint_follower.py

Reads GPS waypoints from waypoints.yaml and follows them one by one.
Reads stop points from stops.yaml (same directory) — robot waits at each
stop until a confirmation message arrives on the configured topic.

Commands written to /tmp/dcvg_nav_cmd:
  run   — navigate normally
  pause — cancel current goal and wait in place
  stop  — cancel and exit

Usage:
  ros2 run dcvg_robot route_waypoint_follower
  ros2 run dcvg_robot route_waypoint_follower /path/to/waypoints.yaml

To confirm a stop from the terminal (first draft):
  ros2 topic pub --once /dcvg/stop_confirm std_msgs/msg/String "data: 'go'"
"""

import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_localization.srv import FromLL
from std_msgs.msg import String

from geometry_msgs.msg import Quaternion

_NAV_CMD_FILE      = '/tmp/dcvg_nav_cmd'
_STOP_STATUS_FILE  = '/tmp/dcvg_stop_status'
_STOP_CONFIRM_FILE = '/tmp/dcvg_stop_confirm'
_NAV_TARGET_FILE   = '/tmp/dcvg_nav_target'   # current goal the robot is driving to


def _read_cmd():
    try:
        with open(_NAV_CMD_FILE) as f:
            return f.read().strip()
    except FileNotFoundError:
        return 'run'


def _write_cmd(cmd):
    with open(_NAV_CMD_FILE, 'w') as f:
        f.write(cmd)


def _write_stop_status(msg: str):
    with open(_STOP_STATUS_FILE, 'w') as f:
        f.write(msg)


def _write_nav_target(orig_idx: int, seq: int, total: int, lat: float, lon: float):
    """Tell the web app which waypoint the robot is currently driving to.

    orig_idx — index into the full waypoints.yaml list (matches the app's array)
    seq/total — human-readable progress (waypoint seq of total, after filtering)
    """
    import json
    with open(_NAV_TARGET_FILE, 'w') as f:
        json.dump({'index': orig_idx, 'seq': seq, 'total': total,
                   'latitude': lat, 'longitude': lon}, f)


def _clear_nav_target():
    try:
        os.remove(_NAV_TARGET_FILE)
    except FileNotFoundError:
        pass


def _check_file_confirm() -> bool:
    """Return True if the web app wrote a confirm signal, and clear it."""
    try:
        with open(_STOP_CONFIRM_FILE) as f:
            val = f.read().strip()
        if val == 'confirm':
            os.remove(_STOP_CONFIRM_FILE)
            return True
    except FileNotFoundError:
        pass
    return False

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


class RouteWaypointFollower(Node):

    def __init__(self, wps_file_path):
        super().__init__('route_waypoint_follower')
        self.navigator = BasicNavigator('basic_navigator')
        self.from_ll_client = self.create_client(FromLL, '/fromLL')

        # Robot position — updated by odometry so we can skip close initial waypoints
        self._robot_x = None
        self._robot_y = None
        self._odom_sub = self.create_subscription(
            Odometry, '/odometry/local', self._odom_cb, 10)

        # Minimum distances (metres) — tunable via ROS parameters
        self.declare_parameter('min_start_distance', 5.0)
        self.declare_parameter('min_wp_distance',    2.0)
        self.min_start_dist: float = float(self.get_parameter('min_start_distance').value)
        self.min_wp_dist:    float = float(self.get_parameter('min_wp_distance').value)

        with open(wps_file_path, 'r') as f:
            data = yaml.safe_load(f)
        self.waypoints = data['waypoints']
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {wps_file_path}")

        # --- Load stops configuration ---
        self.stops = {}            # {waypoint_index: {index, latitude, longitude}}
        self.confirmation_topic = '/dcvg/stop_confirm'
        self.reached_topic      = '/dcvg/stop_reached'
        self._stop_confirmed    = False

        stops_file = os.path.join(os.path.dirname(wps_file_path), 'stops.yaml')
        if os.path.exists(stops_file):
            with open(stops_file) as f:
                sd = yaml.safe_load(f) or {}
            self.confirmation_topic = sd.get('confirmation_topic', self.confirmation_topic)
            self.reached_topic      = sd.get('reached_topic',      self.reached_topic)
            for s in sd.get('stops', []):
                self.stops[int(s['index'])] = s
            self.get_logger().info(
                f"Loaded {len(self.stops)} stop(s) — "
                f"confirm: '{self.confirmation_topic}', reached: '{self.reached_topic}'"
            )
        else:
            self.get_logger().info("No stops.yaml found — no stop points configured")

        if self.stops:
            self._stop_pub = self.create_publisher(String, self.reached_topic, 10)
            self._confirm_sub = self.create_subscription(
                String, self.confirmation_topic, self._confirm_cb, 10)

    def _odom_cb(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    def _confirm_cb(self, msg):
        self.get_logger().info(f"Stop confirmation received: '{msg.data}'")
        self._stop_confirmed = True

    def _convert_to_posestamped(self, lat, lon, yaw=0.0):
        if not self.from_ll_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/fromLL service not available — is navsat_transform_node running?')
            return None

        req = FromLL.Request()
        req.ll_point.latitude  = lat
        req.ll_point.longitude = lon
        req.ll_point.altitude  = 0.0

        future = self.from_ll_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None:
            self.get_logger().error(f'fromLL conversion failed for ({lat:.6f}, {lon:.6f})')
            return None

        mp = result.map_point
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = mp.x
        pose.pose.position.y = mp.y
        pose.pose.position.z = mp.z
        pose.pose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
        return pose

    def _wait_at_stop(self, waypoint_idx):
        """Publish stop-reached event and block until confirmation or stop command."""
        sorted_keys = sorted(self.stops.keys())
        stop_num = sorted_keys.index(waypoint_idx) + 1
        s = self.stops[waypoint_idx]

        text = (f"Stop{stop_num}, "
                f"lat {float(s['latitude']):.6f}, "
                f"long {float(s['longitude']):.6f}")

        # Publish to ROS topic
        ros_msg = String()
        ros_msg.data = text
        self._stop_pub.publish(ros_msg)

        # Write status file so the web app can show the popup
        _write_stop_status(text)
        # Clear any stale confirm from a previous stop
        _check_file_confirm()

        self.get_logger().info(
            f"★ Reached {text} — waiting for confirmation on '{self.confirmation_topic}'"
        )

        self._stop_confirmed = False
        while not self._stop_confirmed:
            cmd = _read_cmd()
            if cmd == 'stop':
                _write_stop_status('')
                self.get_logger().info("Navigation stopped at stop point.")
                return 'stop'
            # Accept confirm from web app (file) OR ROS topic
            if _check_file_confirm():
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        _write_stop_status('')
        self.get_logger().info(f"★ Stop {stop_num} confirmed — continuing")
        return 'continue'

    def start(self):
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')

        self.get_logger().info("Converting GPS waypoints to map frame...")
        # poses: list of (original_waypoint_index, PoseStamped)
        poses = []
        for i, wp in enumerate(self.waypoints):
            pose = self._convert_to_posestamped(
                wp['latitude'], wp['longitude'], wp.get('yaw', 0.0))
            if pose is None:
                self.get_logger().warn(f"Skipping waypoint {i + 1} — conversion failed")
                continue
            poses.append((i, pose))
            self.get_logger().info(
                f"  [{i + 1:>3}/{len(self.waypoints)}] "
                f"GPS ({wp['latitude']:.6f}, {wp['longitude']:.6f}) → "
                f"map ({pose.pose.position.x:.1f}, {pose.pose.position.y:.1f})"
                + (" [STOP]" if i in self.stops else "")
            )

        if not poses:
            self.get_logger().error("No valid waypoints — aborting")
            return

        # ── Step 1: filter out consecutive waypoints that are too close ──────
        filtered = [poses[0]]
        for orig_idx, pose in poses[1:]:
            px = filtered[-1][1].pose.position.x
            py = filtered[-1][1].pose.position.y
            cx = pose.pose.position.x
            cy = pose.pose.position.y
            if math.hypot(cx - px, cy - py) >= self.min_wp_dist:
                filtered.append((orig_idx, pose))
        removed = len(poses) - len(filtered)
        if removed:
            self.get_logger().info(
                f"Filtered {removed} waypoint(s) closer than {self.min_wp_dist:.1f} m apart")
        poses = filtered

        # ── Step 2: skip initial waypoints too close to the robot's start ────
        # Spin briefly to get a current odometry reading
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._robot_x is not None:
                break

        if self._robot_x is not None:
            robot_x = float(self._robot_x)
            robot_y = float(self._robot_y)
            skipped = 0
            first_dist = 0.0
            while poses:
                cx = poses[0][1].pose.position.x
                cy = poses[0][1].pose.position.y
                first_dist = math.hypot(cx - robot_x, cy - robot_y)
                if first_dist < self.min_start_dist:
                    self.get_logger().info(
                        f"Skipping waypoint {poses[0][0] + 1} — "
                        f"{first_dist:.1f} m from robot start (< {self.min_start_dist:.1f} m)")
                    poses.pop(0)
                    skipped += 1
                else:
                    break
            if skipped:
                self.get_logger().info(
                    f"Skipped {skipped} near-start waypoint(s); "
                    f"first target now {first_dist:.1f} m away")
        else:
            self.get_logger().warn(
                "No odometry received — skipping proximity filter")

        if not poses:
            self.get_logger().error("All waypoints filtered out — aborting")
            return

        # ── Step 3: compute look-ahead yaw so robot faces the next waypoint ──
        for i in range(len(poses)):
            if i < len(poses) - 1:
                nx = poses[i + 1][1].pose.position.x
                ny = poses[i + 1][1].pose.position.y
            else:
                # last waypoint keeps the same heading as the previous segment
                nx = poses[i][1].pose.position.x
                ny = poses[i][1].pose.position.y
                if i > 0:
                    nx += poses[i][1].pose.position.x - poses[i - 1][1].pose.position.x
                    ny += poses[i][1].pose.position.y - poses[i - 1][1].pose.position.y
            cx = poses[i][1].pose.position.x
            cy = poses[i][1].pose.position.y
            yaw = math.atan2(ny - cy, nx - cx)
            poses[i][1].pose.orientation = quaternion_from_euler(0.0, 0.0, yaw)

        _write_cmd('run')
        _clear_nav_target()   # drop any stale target from a previous run
        self.get_logger().info(
            f"Starting navigation: {len(poses)} waypoints, {len(self.stops)} stop(s)")

        j = 0
        while j < len(poses):
            orig_idx, pose = poses[j]

            # --- Check command before each waypoint ---
            while True:
                cmd = _read_cmd()
                if cmd == 'stop':
                    _clear_nav_target()
                    self.get_logger().info("Navigation stopped.")
                    return
                if cmd == 'pause':
                    rclpy.spin_once(self, timeout_sec=0.2)
                    continue
                break

            self.get_logger().info(f"→ Waypoint {j + 1}/{len(poses)}"
                                   + (" [STOP ahead]" if orig_idx in self.stops else ""))
            # Publish the current target so the web app can highlight it.
            # orig_idx maps to the same waypoints.yaml the app loaded.
            twp = self.waypoints[orig_idx]
            _write_nav_target(orig_idx, j + 1, len(poses),
                              float(twp['latitude']), float(twp['longitude']))
            self.navigator.goToPose(pose)

            # --- Monitor until waypoint reached or command received ---
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.05)
                cmd = _read_cmd()

                if cmd == 'stop':
                    self.navigator.cancelTask()
                    _clear_nav_target()
                    self.get_logger().info("Navigation stopped.")
                    return

                if cmd == 'pause':
                    self.navigator.cancelTask()
                    self.get_logger().info(f"Paused at waypoint {j + 1}")
                    cmd = 'pause'
                    while cmd == 'pause':
                        rclpy.spin_once(self, timeout_sec=0.2)
                        cmd = _read_cmd()
                    if cmd == 'stop':
                        _clear_nav_target()
                        self.get_logger().info("Navigation stopped while paused.")
                        return
                    self.get_logger().info(f"Resumed — re-navigating to waypoint {j + 1}")
                    self.navigator.goToPose(pose)

            result = self.navigator.getResult()
            self.get_logger().info(f"Waypoint {j + 1}/{len(poses)} complete — result: {result}")

            # --- Stop point: wait for confirmation before advancing ---
            if orig_idx in self.stops:
                outcome = self._wait_at_stop(orig_idx)
                if outcome == 'stop':
                    _clear_nav_target()
                    return

            j += 1

        _clear_nav_target()
        self.get_logger().info("All waypoints reached!")
        _write_cmd('stopped')


def main():
    rclpy.init()

    default_path = os.path.join(
        get_package_share_directory('dcvg_robot'),
        'config', 'waypoints.yaml')

    wps_file = sys.argv[1] if len(sys.argv) > 1 else default_path

    node = RouteWaypointFollower(wps_file)
    node.start()


if __name__ == '__main__':
    main()
