import json
import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_localization.srv import FromLL
from std_msgs.msg import String

from dcvg_robot.common import (NAV_TARGET_FILE, STOP_CONFIRM_FILE, STOP_STATUS_FILE,
                               read_cmd, read_file, remove_file, write_cmd, write_file)

STOP_REACHED_TOPIC = '/dcvg/stop_reached'
STOP_CONFIRM_TOPIC = '/dcvg/stop_confirm'


def yaw_quaternion(yaw):
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def xy(pose):
    return pose.pose.position.x, pose.pose.position.y


def dist(a, b):
    (ax, ay), (bx, by) = xy(a), xy(b)
    return math.hypot(bx - ax, by - ay)


def bearing(a, b):
    (ax, ay), (bx, by) = xy(a), xy(b)
    return math.atan2(by - ay, bx - ax)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def write_nav_target(orig_idx, seq, total, lat, lon):
    write_file(NAV_TARGET_FILE, json.dumps({'index': orig_idx, 'seq': seq, 'total': total,
                                            'latitude': lat, 'longitude': lon}))


def take_file_confirm():
    if read_file(STOP_CONFIRM_FILE) == 'confirm':
        remove_file(STOP_CONFIRM_FILE)
        return True
    return False


class RouteWaypointFollower(Node):

    def __init__(self, wps_file_path):
        super().__init__('route_waypoint_follower')
        self.navigator = BasicNavigator('basic_navigator')
        self.from_ll_client = self.create_client(FromLL, '/fromLL')
        self.robot_xy = None
        self.create_subscription(Odometry, '/odometry/global', self.odom_cb, 10)

        self.declare_parameter('min_start_distance', 1.0)
        self.declare_parameter('min_wp_distance', 0.3)
        self.declare_parameter('simplify_angle_deg', 8.0)
        self.declare_parameter('simplify_max_run_m', 12.0)
        self.declare_parameter('simplify_max_deviation_m', 0.15)
        self.declare_parameter('on_segment_failure', 'hold')
        p = lambda name: self.get_parameter(name).value
        self.min_start_dist = float(p('min_start_distance'))
        self.min_wp_dist = float(p('min_wp_distance'))
        self.simplify_angle_deg = float(p('simplify_angle_deg'))
        self.simplify_max_run_m = float(p('simplify_max_run_m'))
        self.simplify_max_deviation_m = float(p('simplify_max_deviation_m'))
        self.on_segment_failure = str(p('on_segment_failure')).strip().lower()
        if self.on_segment_failure not in ('hold', 'skip'):
            self.get_logger().warn(f"on_segment_failure '{self.on_segment_failure}' "
                                   f"not understood — falling back to 'hold'")
            self.on_segment_failure = 'hold'

        with open(wps_file_path) as f:
            self.waypoints = yaml.safe_load(f)['waypoints']
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {wps_file_path}')

        self.stops = {}
        stops_file = os.path.join(os.path.dirname(wps_file_path), 'stops.yaml')
        if os.path.exists(stops_file):
            with open(stops_file) as f:
                for s in (yaml.safe_load(f) or {}).get('stops', []):
                    self.stops[int(s['index'])] = s
        self.get_logger().info(f'Loaded {len(self.stops)} stop(s)')

        self.skipped = []
        self.stop_confirmed = False
        self.stop_pub = self.create_publisher(String, STOP_REACHED_TOPIC, 10)
        self.create_subscription(String, STOP_CONFIRM_TOPIC, self.confirm_cb, 10)

    def odom_cb(self, msg):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def confirm_cb(self, msg):
        self.get_logger().info(f"Stop confirmation received: '{msg.data}'")
        self.stop_confirmed = True

    def wait_for_pose(self):
        for _ in range(20):
            if self.robot_xy is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.robot_xy

    def to_map_pose(self, lat, lon):
        if not self.from_ll_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/fromLL not available — is navsat_transform running?')
            return None
        req = FromLL.Request()
        req.ll_point.latitude = lat
        req.ll_point.longitude = lon
        future = self.from_ll_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error(f'fromLL conversion failed for ({lat:.6f}, {lon:.6f})')
            return None
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = future.result().map_point
        return pose

    def chord_deviation(self, poses, a_idx, b_idx):
        (ax, ay), (bx, by) = xy(poses[a_idx]), xy(poses[b_idx])
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        worst = 0.0
        for k in range(a_idx + 1, b_idx):
            px, py = xy(poses[k])
            t = 0.0 if length2 <= 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
            worst = max(worst, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
        return worst

    def simplify(self, poses):
        if self.simplify_angle_deg <= 0.0 or len(poses) <= 2:
            return poses
        tol = math.radians(self.simplify_angle_deg)
        out = [poses[0]]
        anchor = 0
        for i in range(1, len(poses) - 1):
            turned = abs(wrap(bearing(poses[anchor], poses[i + 1])
                              - bearing(poses[anchor], poses[i]))) > tol
            too_long = dist(poses[anchor], poses[i + 1]) > self.simplify_max_run_m
            bulges = self.chord_deviation(poses, anchor, i + 1) > self.simplify_max_deviation_m
            if turned or too_long or bulges:
                out.append(poses[i])
                anchor = i
        out.append(poses[-1])
        if len(out) < len(poses):
            self.get_logger().info(f'    simplified {len(poses)} -> {len(out)} goal(s)')
        return out

    def poses_ahead(self, poses):
        if self.wait_for_pose() is None:
            self.get_logger().warn('No pose available on resume — re-issuing the full segment.')
            return poses
        rx, ry = self.robot_xy
        nearest = min(range(len(poses)),
                      key=lambda i: math.hypot(xy(poses[i])[0] - rx, xy(poses[i])[1] - ry))
        return poses[nearest:] or poses[-1:]

    def hold_for_operator(self, seg_no, total, attempt, result):
        text = (f'HELD — segment {seg_no}/{total} failed ({result}) on attempt {attempt}. '
                f'Robot is stopped on the route. Clear the obstruction, then press Run to '
                f'retry this segment, or Stop to end the run.')
        write_cmd('pause')
        write_file(STOP_STATUS_FILE, text)
        self.get_logger().error(text)
        while True:
            cmd = read_cmd()
            if cmd in ('stop', 'run'):
                write_file(STOP_STATUS_FILE, '')
                self.get_logger().info('Run ended while held.' if cmd == 'stop'
                                       else f'Operator resumed — retrying segment {seg_no}')
                return 'stop' if cmd == 'stop' else 'retry'
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_at_stop(self, waypoint_idx):
        stop_num = sorted(self.stops).index(waypoint_idx) + 1
        s = self.stops[waypoint_idx]
        text = (f"Stop{stop_num}, lat {float(s['latitude']):.6f}, "
                f"long {float(s['longitude']):.6f}")
        self.stop_pub.publish(String(data=text))
        write_file(STOP_STATUS_FILE, text)
        take_file_confirm()
        self.get_logger().info(f'★ Reached {text} — waiting for confirmation')

        self.stop_confirmed = False
        while not self.stop_confirmed:
            if read_cmd() == 'stop':
                write_file(STOP_STATUS_FILE, '')
                self.get_logger().info('Navigation stopped at stop point.')
                return 'stop'
            if take_file_confirm():
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        write_file(STOP_STATUS_FILE, '')
        self.get_logger().info(f'★ Stop {stop_num} confirmed — continuing')
        return 'continue'

    def prepare_poses(self):
        poses = []
        for i, wp in enumerate(self.waypoints):
            pose = self.to_map_pose(wp['latitude'], wp['longitude'])
            if pose is None:
                self.get_logger().warn(f'Skipping waypoint {i + 1} — conversion failed')
                continue
            poses.append((i, pose))
        if not poses:
            return []

        filtered = [poses[0]]
        for entry in poses[1:]:
            if dist(filtered[-1][1], entry[1]) >= self.min_wp_dist:
                filtered.append(entry)
        poses = filtered

        if self.wait_for_pose() is not None:
            robot = PoseStamped()
            robot.pose.position.x, robot.pose.position.y = self.robot_xy
            while poses and dist(robot, poses[0][1]) < self.min_start_dist:
                self.get_logger().info(f'Skipping waypoint {poses[0][0] + 1} — too close to start')
                poses.pop(0)
        else:
            self.get_logger().warn('No odometry received — skipping proximity filter')

        for i, (_, pose) in enumerate(poses):
            if i < len(poses) - 1:
                yaw = bearing(pose, poses[i + 1][1])
            elif i > 0:
                yaw = bearing(poses[i - 1][1], pose)
            else:
                yaw = 0.0
            pose.pose.orientation = yaw_quaternion(yaw)
        return poses

    def split_segments(self, poses):
        segments, current = [], []
        for entry in poses:
            current.append(entry)
            if entry[0] in self.stops:
                segments.append(current)
                current = []
        if current:
            segments.append(current)
        return segments

    def wait_while_paused(self):
        cmd = read_cmd()
        while cmd == 'pause':
            rclpy.spin_once(self, timeout_sec=0.2)
            cmd = read_cmd()
        return cmd

    def run_segment(self, seg_no, total, segment):
        seg_poses = self.simplify([p for _, p in segment])
        end_idx = segment[-1][0]
        attempt = 0
        while True:
            attempt += 1
            self.get_logger().info(f'→ Segment {seg_no}/{total}: {len(seg_poses)} pose(s) to '
                                   f'waypoint {end_idx + 1} (attempt {attempt})')
            wp = self.waypoints[end_idx]
            write_nav_target(end_idx, seg_no, total, float(wp['latitude']), float(wp['longitude']))
            self.navigator.goThroughPoses(seg_poses if attempt == 1 else self.poses_ahead(seg_poses))

            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.05)
                cmd = read_cmd()
                if cmd == 'stop':
                    self.navigator.cancelTask()
                    return 'stop'
                if cmd == 'pause':
                    self.navigator.cancelTask()
                    self.get_logger().info(f'Paused during segment {seg_no}')
                    if self.wait_while_paused() == 'stop':
                        return 'stop'
                    self.navigator.goThroughPoses(self.poses_ahead(seg_poses))

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                return 'ok'
            self.get_logger().error(f'SEGMENT FAILED: segment {seg_no}/{total} (waypoints '
                                    f'{segment[0][0] + 1}..{end_idx + 1}) result {result}, '
                                    f'attempt {attempt}')
            if self.on_segment_failure == 'skip':
                self.skipped.extend(idx for idx, _ in segment)
                return 'ok'
            if self.hold_for_operator(seg_no, total, attempt, result) == 'stop':
                return 'stop'

    def start(self):
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')
        poses = self.prepare_poses()
        if not poses:
            self.get_logger().error('No valid waypoints — aborting')
            return
        write_cmd('run')
        remove_file(NAV_TARGET_FILE)
        segments = self.split_segments(poses)
        self.get_logger().info(f'Route: {len(poses)} waypoints, {len(self.stops)} stop(s), '
                               f'{len(segments)} segment(s)')

        for seg_no, segment in enumerate(segments, start=1):
            if self.wait_while_paused() == 'stop':
                break
            if self.run_segment(seg_no, len(segments), segment) == 'stop':
                break
            if segment[-1][0] in self.stops and self.wait_at_stop(segment[-1][0]) == 'stop':
                break
        else:
            if self.skipped:
                self.get_logger().error(f'Route finished with {len(self.skipped)} skipped '
                                        f'waypoint(s): {[i + 1 for i in self.skipped]}')
            else:
                self.get_logger().info('All waypoints reached!')
            write_cmd('stopped')
        remove_file(NAV_TARGET_FILE)
        self.get_logger().info('Navigation finished.')


def main():
    rclpy.init()
    default_path = os.path.join(get_package_share_directory('dcvg_robot'), 'config',
                                'waypoints.yaml')
    node = RouteWaypointFollower(sys.argv[1] if len(sys.argv) > 1 else default_path)
    node.start()


if __name__ == '__main__':
    main()
