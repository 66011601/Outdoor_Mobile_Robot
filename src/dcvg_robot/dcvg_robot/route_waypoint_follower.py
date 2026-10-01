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
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
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

        # Robot position — updated by odometry so we can skip close initial waypoints.
        # Must be map-frame (ekf_global's output), not odom-frame: it's compared directly
        # against waypoint poses converted via /fromLL, which are in the map frame too.
        # odom and map coincide only near startup before drift accumulates — comparing
        # odom-frame position against map-frame waypoints would skip the wrong waypoints
        # (or fail to skip ones it should) once they diverge.
        self._robot_x = None
        self._robot_y = None
        self._odom_sub = self.create_subscription(
            Odometry, '/odometry/global', self._odom_cb, 10)

        # Minimum distances (metres) — tunable via ROS parameters
        self.declare_parameter('min_start_distance', 1.0)
        # 0.3, not 1.0. The live route is spaced a uniform 0.99m, so a 1.0
        # threshold sat directly ON the spacing: whether a point survived came
        # down to float noise in the fromLL projection, dropping points
        # nondeterministically and making runs inconsistent between attempts.
        # This matters more now that the points are fed as NavigateThroughPoses
        # path shape rather than individual goals — dropping half of them
        # coarsens the path the planner is asked to follow. 0.3 keeps every real
        # ~1m point while still collapsing true duplicates.
        self.declare_parameter('min_wp_distance',    0.3)
        # Route simplification: a waypoint is kept only where the route actually
        # turns by more than this. Straight runs of interpolated points collapse
        # to their endpoints. 12 deg sits in the middle of the 10-15 deg band
        # that reads as "still the same straight" on a dirt road. Set to 0 to
        # disable simplification entirely.
        self.declare_parameter('simplify_angle_deg', 8.0)
        # Hard cap on how long a collapsed straight may get. The global costmap
        # is rolling 100x100m (~50m around the robot), so a goal beyond that
        # falls outside it and planning to it fails.
        self.declare_parameter('simplify_max_run_m', 12.0)
        # Hard cap on how far the collapsed straight may deviate from the real
        # route. The angle test alone does not bound this — on a tight bend the
        # bearing can creep under tolerance while the chord cuts the corner by
        # metres (measured 1.9m on a 10m-radius arc).
        # 0.4 -> 0.15, and the angle test 12 -> 8 deg. The old values were sized
        # against a 1.0m localization noise floor, i.e. "the cut is smaller than
        # what we can measure anyway". That is no longer the standard: the rover
        # must hold better than 0.5m of the surveyed line, so a simplification
        # allowed to cut 0.4m would spend 80% of the whole budget before the
        # controller is even involved. 0.15 keeps route simplification an
        # optimisation rather than a source of error.
        # NOTE: with stops every ~2m the segments handed to _simplify are only
        # 2-3 poses long, so it returns early and these are inert TODAY. They
        # matter the moment stop spacing is widened — keep them honest.
        self.declare_parameter('simplify_max_deviation_m', 0.15)
        # What to do when a segment does not finish SUCCEEDED. 'hold' parks the
        # run and waits for the operator (the rover has already stopped — Nav2
        # recovery is non-evasive now, so it is sitting on the route line);
        # 'skip' is the previous behaviour, advancing to the next segment and
        # logging a data gap. 'hold' is the default because on a live road the
        # reason a segment failed is usually something in the way, and driving
        # on to a goal 2m further along means crossing whatever that was.
        self.declare_parameter('on_segment_failure', 'hold')
        self.min_start_dist: float = float(self.get_parameter('min_start_distance').value)
        self.min_wp_dist:    float = float(self.get_parameter('min_wp_distance').value)
        self.simplify_angle_deg: float = float(
            self.get_parameter('simplify_angle_deg').value)
        self.simplify_max_run_m: float = float(
            self.get_parameter('simplify_max_run_m').value)
        self.simplify_max_deviation_m: float = float(
            self.get_parameter('simplify_max_deviation_m').value)
        self.on_segment_failure: str = str(
            self.get_parameter('on_segment_failure').value).strip().lower()
        if self.on_segment_failure not in ('hold', 'skip'):
            self.get_logger().warn(
                f"on_segment_failure '{self.on_segment_failure}' not understood "
                f"— falling back to 'hold'")
            self.on_segment_failure = 'hold'

        with open(wps_file_path, 'r') as f:
            data = yaml.safe_load(f)
        self.waypoints = data['waypoints']
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {wps_file_path}")

        # --- Load stops configuration ---
        self.stops = {}            # {waypoint_index: {index, latitude, longitude}}
        # Waypoints that finished with a non-SUCCEEDED result. The run continues
        # past them, so they are survey data gaps and get reported at the end.
        self._skipped = []         # [(orig_idx, seq, TaskResult)]
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

    def _simplify(self, seg_poses):
        """Collapse runs of near-collinear waypoints into their endpoints.

        WHY THIS EXISTS
        ───────────────
        osm_route_generator's densify() LINEARLY INTERPOLATES between OSM nodes,
        so on a straight stretch the 1m points are exactly collinear — they carry
        no information their endpoints don't. Handing all of them to
        ComputePathThroughPoses as separate goals does not make the robot follow
        the road more accurately; it just gives the planner ~150 goals to thread
        for what is geometrically 4 corners, and gives RemovePassedGoals 150
        chances to disagree about which one is "next".

        Note this is the only lever available for "tell the planner which way to
        point". Intermediate pose ORIENTATIONS cannot do it: NavFn is a 2D grid
        planner and ComputePathThroughPoses only uses goal positions, so a yaw
        set on an intermediate waypoint is silently ignored. Which points exist
        is what shapes the path.

        A point is kept when the route actually turns there — bearing from the
        current anchor deviates by more than `simplify_angle_deg` — so straights
        become one long goal-to-goal leg the planner renders as a straight line,
        and corners stay corners.

        The run is also capped by `simplify_max_run_m` for a hard reason: the
        global costmap is ROLLING 100x100m, so it only covers ~50m around the
        robot. A goal further out than that falls outside the costmap and
        planning to it fails. The cap keeps every goal comfortably inside.

        First and last pose are always kept. Since segments are split AT stops,
        keeping the last pose means every DCVG measurement stop survives
        simplification untouched.
        """
        tol = math.radians(self.simplify_angle_deg)
        max_run = self.simplify_max_run_m
        if self.simplify_angle_deg <= 0.0 or len(seg_poses) <= 2:
            return seg_poses

        def xy(p):
            return p.pose.position.x, p.pose.position.y

        def bearing(a, b):
            (ax, ay), (bx, by) = xy(a), xy(b)
            return math.atan2(by - ay, bx - ax)

        def wrap(a):
            """Fold a bearing difference into (-pi, pi]. bearing() returns atan2's
            (-pi, pi], so a difference of two bearings spans (-2pi, 2pi): a route
            bending 2 deg across the +/-pi seam comes out as 358 and would read as
            a corner. In this ENU map frame that seam sits at due west, so without
            this every near-straight westbound run would be kept point-for-point."""
            return math.atan2(math.sin(a), math.cos(a))

        def dist(a, b):
            (ax, ay), (bx, by) = xy(a), xy(b)
            return math.hypot(bx - ax, by - ay)

        max_dev = self.simplify_max_deviation_m

        def chord_deviation(a_idx, b_idx):
            """Max perpendicular distance of the points between a and b from the
            straight chord a->b. The angle test alone does NOT bound this: on a
            tight bend, bearing can creep under the tolerance while the collapsed
            chord cuts the corner by metres. Measured at 1.9m on a 10m-radius arc
            before this cap existed, against a 1.0m localization noise floor —
            i.e. enough to put the rover off a dirt road."""
            (ax, ay), (bx, by) = xy(seg_poses[a_idx]), xy(seg_poses[b_idx])
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            worst = 0.0
            for k in range(a_idx + 1, b_idx):
                px, py = xy(seg_poses[k])
                if L2 <= 1e-12:
                    worst = max(worst, math.hypot(px - ax, py - ay))
                    continue
                t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
                worst = max(worst, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
            return worst

        out = [seg_poses[0]]
        anchor = 0
        i = 1
        while i < len(seg_poses) - 1:
            base = bearing(seg_poses[anchor], seg_poses[i])
            nxt = bearing(seg_poses[anchor], seg_poses[i + 1])
            turned = abs(wrap(nxt - base)) > tol
            too_long = dist(seg_poses[anchor], seg_poses[i + 1]) > max_run
            bulges = chord_deviation(anchor, i + 1) > max_dev
            if turned or too_long or bulges:
                out.append(seg_poses[i])     # keep the corner / cap the run
                anchor = i
            i += 1
        out.append(seg_poses[-1])

        if len(out) < len(seg_poses):
            self.get_logger().info(
                f"    simplified {len(seg_poses)} -> {len(out)} goal(s) "
                f"(straight runs collapsed at >{self.simplify_angle_deg:.0f} deg, "
                f"capped at {max_run:.0f} m)")
        return out

    def _poses_ahead(self, seg_poses):
        """Return the tail of seg_poses starting at the pose nearest the rover.

        Used when re-issuing a segment after a pause. RemovePassedGoals in the BT
        only prunes goals it is already close to, so anything left in front of the
        list that the rover has driven past would be navigated back to. Trimming
        by nearest-pose is enough here because a segment is a single forward run
        along the route — there is no branching to disambiguate.

        Falls back to the full segment if no pose estimate is available, since
        re-driving the segment is recoverable whereas sending an empty goal list
        is not.
        """
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._robot_x is not None:
                break
        if self._robot_x is None:
            self.get_logger().warn(
                'No pose available on resume — re-issuing the full segment. '
                'The rover may drive back toward its start.')
            return seg_poses

        rx, ry = float(self._robot_x), float(self._robot_y)
        nearest = min(
            range(len(seg_poses)),
            key=lambda i: math.hypot(seg_poses[i].pose.position.x - rx,
                                     seg_poses[i].pose.position.y - ry))
        # Keep the nearest pose itself rather than skipping to the next one: if
        # the rover is genuinely on top of it, RemovePassedGoals drops it on the
        # first tick, whereas skipping early would shortcut a real corner.
        # Always leave at least the final pose so the goal list is never empty.
        return seg_poses[nearest:] or seg_poses[-1:]

    def _hold_for_operator(self, seg_no, total, attempt, result):
        """Park the run after a failed segment and wait for the operator.

        Safe to hold here precisely BECAUSE Nav2 recovery is non-evasive now
        (BackUp and Spin were removed from both behavior trees): a failed
        segment leaves the rover stopped on the route line, not somewhere it
        improvised to. Holding is the right default on a live road — the usual
        reason a segment fails is that something is in the way, and advancing to
        the next stop ~2m further along means driving across whatever that was.

        The operator has exactly the two controls the web app already provides:
          Run  → retry this same segment (after clearing whatever blocked it)
          Stop → end the run
        To get the old advance-past-it behaviour instead, launch with
        `-p on_segment_failure:=skip`.

        Returns 'retry' or 'stop'.
        """
        text = (f"HELD — segment {seg_no}/{total} failed ({result}) on attempt "
                f"{attempt}. Robot is stopped on the route. Clear the "
                f"obstruction, then press Run to retry this segment, or Stop to "
                f"end the run.")
        _write_cmd('pause')
        _write_stop_status(text)
        self.get_logger().error(text)

        while True:
            cmd = _read_cmd()
            if cmd == 'stop':
                _write_stop_status('')
                self.get_logger().info("Run ended by the operator while held.")
                return 'stop'
            if cmd == 'run':
                _write_stop_status('')
                self.get_logger().info(
                    f"Operator resumed — retrying segment {seg_no}")
                return 'retry'
            rclpy.spin_once(self, timeout_sec=0.1)

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

        # ── Split the route into SEGMENTS ────────────────────────────────────
        # The route's ~1m points define path SHAPE, not places to stop. Sending
        # one NavigateToPose per point made every point a terminal goal — goal
        # checker, full stop, replan, restart — which is what made the motion
        # stop-start and zigzaggy between closely-spaced waypoints.
        # Instead, each segment is handed to Nav2 as ONE NavigateThroughPoses
        # plan, so the planner produces a single continuous path and the
        # controller flows through the intermediate points without ever
        # goal-checking at them. A segment ends only where the rover genuinely
        # must stop: a DCVG measurement stop, or the end of the route.
        # How close the rover passes each intermediate point is now governed by
        # RemovePassedGoals radius="0.7" in the navigate_through_poses BT — NOT
        # by xy_goal_tolerance, which now applies only at segment ends.
        segments = []          # [[(orig_idx, pose), ...], ...]
        current = []
        for entry in poses:
            current.append(entry)
            if entry[0] in self.stops:      # stop terminates the segment
                segments.append(current)
                current = []
        if current:
            segments.append(current)

        self.get_logger().info(
            f"Route split into {len(segments)} continuous segment(s) "
            f"({len(poses)} waypoints, {len(self.stops)} stop(s))")

        for seg_no, segment in enumerate(segments, start=1):
            # Dispatch the SIMPLIFIED point list, but keep `segment` intact for
            # data-gap accounting — a failed segment still stranded every
            # original waypoint in it, not just the ones we sent as goals.
            seg_poses = self._simplify([p for _, p in segment])
            end_orig_idx = segment[-1][0]
            ends_at_stop = end_orig_idx in self.stops

            # --- Check command before starting the segment ---
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

            # Attempt loop. Under on_segment_failure='hold' a failed segment is
            # retried as many times as the operator presses Run — there is no
            # automatic retry, so the rover never re-attempts a blocked segment
            # on its own. Under 'skip' this runs exactly once.
            attempt = 0
            while True:
                attempt += 1
                self.get_logger().info(
                    f"→ Segment {seg_no}/{len(segments)}: {len(seg_poses)} pose(s) "
                    f"flowing through to waypoint index {end_orig_idx + 1}"
                    + (f" [attempt {attempt}]" if attempt > 1 else "")
                    + (" [STOP at end]" if ends_at_stop else ""))

                # Publish the segment's end as the web app's highlighted target.
                twp = self.waypoints[end_orig_idx]
                _write_nav_target(end_orig_idx, seg_no, len(segments),
                                  float(twp['latitude']), float(twp['longitude']))
                # A retry starts from wherever the rover actually stopped, for
                # the same reason a resume does — see _poses_ahead.
                dispatch = seg_poses if attempt == 1 else self._poses_ahead(seg_poses)
                self.navigator.goThroughPoses(dispatch)

                # --- Monitor until segment complete or command received ---
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
                        self.get_logger().info(f"Paused during segment {seg_no}")
                        cmd = 'pause'
                        while cmd == 'pause':
                            rclpy.spin_once(self, timeout_sec=0.2)
                            cmd = _read_cmd()
                        if cmd == 'stop':
                            _clear_nav_target()
                            self.get_logger().info("Navigation stopped while paused.")
                            return
                        # Re-issue only the poses still AHEAD of the rover.
                        #
                        # This previously re-sent the whole segment, on the incorrect
                        # assumption that RemovePassedGoals would drop the ones
                        # already behind. It cannot: that node pops goals off the
                        # FRONT only while goal[0] is within its radius and stops at
                        # the first one that is not. Resuming at waypoint 20 of 37
                        # puts waypoint 1 at goal[0], ~19m away — nothing is pruned,
                        # and the rover drives all the way back to the start of the
                        # segment. Trim here instead of relying on the BT.
                        remaining = self._poses_ahead(seg_poses)
                        self.get_logger().info(
                            f"Resumed — re-issuing segment {seg_no} from the nearest "
                            f"pose ahead ({len(remaining)}/{len(seg_poses)} poses remain)")
                        self.navigator.goThroughPoses(remaining)

                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(
                        f"Segment {seg_no}/{len(segments)} complete — result: {result}")
                    break

                first_wp = self.waypoints[segment[0][0]]
                last_wp = self.waypoints[end_orig_idx]
                self.get_logger().error(
                    f"!!! SEGMENT FAILED !!! "
                    f"segment {seg_no}/{len(segments)} covering waypoints.yaml indices "
                    f"{segment[0][0] + 1}..{end_orig_idx + 1} "
                    f"(lat {float(first_wp['latitude']):.7f}, lon {float(first_wp['longitude']):.7f}"
                    f" → lat {float(last_wp['latitude']):.7f}, lon {float(last_wp['longitude']):.7f}) "
                    f"was NOT completed (result: {result}) on attempt {attempt}.")

                if self.on_segment_failure == 'skip':
                    # Previous behaviour: advance and log a data gap. With
                    # through-poses a failure can strand EVERY point in the
                    # segment, so all of them are recorded, not just one.
                    for orig_idx, _ in segment:
                        self._skipped.append((orig_idx, seg_no, result))
                    self.get_logger().error(
                        f"DATA GAP — survey continues to the next segment; "
                        f"{len(segment)} location(s) have no measurement. "
                        f"Total skipped so far: {len(self._skipped)}")
                    break

                if self._hold_for_operator(seg_no, len(segments), attempt, result) == 'stop':
                    _clear_nav_target()
                    return

            # --- Stop point: wait for confirmation before the next segment ---
            if ends_at_stop:
                outcome = self._wait_at_stop(end_orig_idx)
                if outcome == 'stop':
                    _clear_nav_target()
                    return

        _clear_nav_target()
        if self._skipped:
            self.get_logger().error(
                f"Route finished with {len(self._skipped)} SKIPPED waypoint(s) — "
                f"the survey has {len(self._skipped)} data gap(s):")
            for orig_idx, seq, res in self._skipped:
                wp = self.waypoints[orig_idx]
                self.get_logger().error(
                    f"  - waypoint {seq} (waypoints.yaml index {orig_idx + 1}) "
                    f"lat {float(wp['latitude']):.7f}, lon {float(wp['longitude']):.7f} "
                    f"({res})")
        else:
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
