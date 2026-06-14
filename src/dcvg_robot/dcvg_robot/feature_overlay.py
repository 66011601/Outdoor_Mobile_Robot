"""
feature_overlay.py

Draws RTAB-Map's detected keypoints onto the D455 RGB image and publishes
the annotated result on /d455/image_features.

Uses /odom_sensor_data/features for 2D keypoints — these are already in
image pixel coordinates so no projection or TF is needed.

The key overhead guard is the FPS throttle: drawing is capped at max_fps
(default 5 Hz) so OpenCV work never competes with rtabmap's 15 Hz odom
thread for CPU time.

What you see:
  Yellow ring  — GFTT keypoint (ring radius = feature scale)
  Tick line    — keypoint orientation
  Status bar   — [OK/WARN/LOST]  inliers / matches / ratio
  Green banner — NEW KEYFRAME when rgbd_odometry adds a reference frame

Topics consumed:
  <image_topic>              sensor_msgs/Image       — base image (param, see below)
  /odom_sensor_data/features rtabmap_msgs/SensorData — 2D keypoints (image coords)
  /odom_info                 rtabmap_msgs/OdomInfo   — stats + keyframe flag

Topic published:
  <pub_topic>                sensor_msgs/Image       — annotated image (param, see below)

Run:
  ros2 run dcvg_robot feature_overlay
  ros2 run rqt_image_view rqt_image_view /d455/image_features

Params:
  max_fps     (default 5.0)                    — cap drawing rate; lower = less CPU competition
  image_topic (default /d455/image_raw)        — source image; set to /d457/left/image_raw in stereo mode
  pub_topic   (default /d455/image_features)   — output topic; set to /d457/left/image_features in stereo mode
"""

import math
import time

import cv2
from cv_bridge import CvBridge
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from rtabmap_msgs.msg import OdomInfo, SensorData

# ── visual constants ────────────────────────────────────────────────────────
KP_COLOR    = (0, 220, 255)   # BGR yellow-cyan
KP_CENTER_R = 1
KP_SCALE    = 0.15            # ring_radius = size * scale, min 5 px
STATUS_OK   = (40, 220, 40)
STATUS_WARN = (0, 180, 255)
STATUS_LOST = (40, 40, 230)
KF_BG       = (30, 140, 30)
FONT        = cv2.FONT_HERSHEY_SIMPLEX
INLIER_WARN = 10

_BE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)


class FeatureOverlay(Node):

    def __init__(self):
        super().__init__('feature_overlay')
        self._bridge = CvBridge()
        self._latest_info: OdomInfo | None = None
        self._last_draw: float = 0.0

        self.declare_parameter('max_fps',    5.0)
        self.declare_parameter('image_topic', '/d455/image_raw')       # [ADDED] configurable source image
        self.declare_parameter('pub_topic',   '/d455/image_features')  # [ADDED] configurable output topic
        fps         = self.get_parameter('max_fps').value    or 5.0
        image_topic = self.get_parameter('image_topic').value or '/d455/image_raw'      # [ADDED]
        pub_topic   = self.get_parameter('pub_topic').value   or '/d455/image_features' # [ADDED]
        self._min_interval = 1.0 / max(float(fps), 0.1)

        self.create_subscription(OdomInfo, '/odom_info', self._info_cb, 20)

        img_sub  = message_filters.Subscriber(
            self, Image,       image_topic,                  qos_profile=_BE_QOS)  # [CHANGED] was hard-coded '/d455/image_raw'
        feat_sub = message_filters.Subscriber(
            self, SensorData,  '/odom_sensor_data/features', qos_profile=_BE_QOS)
        sync = message_filters.ApproximateTimeSynchronizer(
            [img_sub, feat_sub], queue_size=4, slop=0.1)
        sync.registerCallback(self._draw_cb)

        self._pub = self.create_publisher(Image, pub_topic, 10)  # [CHANGED] was hard-coded '/d455/image_features'
        self.get_logger().info(
            f'feature_overlay ready  max_fps={fps}\n'
            f'  src: {image_topic}\n'   # [ADDED]
            f'  out: {pub_topic}')

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _info_cb(self, msg: OdomInfo) -> None:
        self._latest_info = msg

    def _draw_cb(self, img_msg: Image, feat_msg: SensorData) -> None:
        # FPS throttle — skip frames between draw intervals
        now = time.monotonic()
        if now - self._last_draw < self._min_interval:
            return
        self._last_draw = now

        img = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        h, w = img.shape[:2]

        self._draw_keypoints(img, feat_msg.key_points, w, h)

        info = self._latest_info
        if info is not None:
            self._draw_status(img, info, len(feat_msg.key_points))
            if info.key_frame_added:
                self._draw_keyframe_banner(img, w, h)

        self._pub.publish(self._bridge.cv2_to_imgmsg(img, encoding='bgr8'))

    # ── drawing helpers ────────────────────────────────────────────────────────

    def _draw_keypoints(self, img, key_points, w: int, h: int) -> None:
        for kp in key_points:
            x, y = int(kp.pt.x), int(kp.pt.y)
            if not (0 <= x < w and 0 <= y < h):
                continue
            r = max(5, int(kp.size * KP_SCALE))
            cv2.circle(img, (x, y), r,          KP_COLOR, 1,  cv2.LINE_AA)
            cv2.circle(img, (x, y), KP_CENTER_R, KP_COLOR, -1, cv2.LINE_AA)
            if kp.angle >= 0:
                rad = math.radians(kp.angle)
                ex = int(x + r * math.cos(rad))
                ey = int(y + r * math.sin(rad))
                cv2.line(img, (x, y), (ex, ey), KP_COLOR, 1, cv2.LINE_AA)

    def _draw_status(self, img, info: OdomInfo, n_kp: int) -> None:
        ratio = info.inliers / info.matches if info.matches > 0 else 0.0
        if info.lost:
            color, tag = STATUS_LOST, 'LOST'
        elif info.inliers < INLIER_WARN:
            color, tag = STATUS_WARN, 'WARN'
        else:
            color, tag = STATUS_OK, 'OK'

        lines = [
            f'[{tag}]  inliers={info.inliers}  matches={info.matches}  ratio={ratio:.2f}',
            f'keypoints={n_kp}  features={info.features}'
            f'  local_map={info.local_map_size}  dt={info.interval * 1000:.0f}ms',
        ]
        for i, line in enumerate(lines):
            y = 28 + i * 26
            cv2.putText(img, line, (10, y + 1), FONT, 0.3, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(img, line, (10, y),     FONT, 0.3, color,     1, cv2.LINE_AA)

    def _draw_keyframe_banner(self, img, w: int, h: int) -> None:
        text = ' NEW KEYFRAME — reference frame updated '
        (tw, th), _ = cv2.getTextSize(text, FONT, 0.8, 2)
        bx, by = (w - tw) // 2, h - 18
        cv2.rectangle(img, (bx - 6, by - th - 8), (bx + tw + 6, by + 6), KF_BG, -1)
        cv2.putText(img, text, (bx, by), FONT, 0.8, (255, 255, 255), 2, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = FeatureOverlay()
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
