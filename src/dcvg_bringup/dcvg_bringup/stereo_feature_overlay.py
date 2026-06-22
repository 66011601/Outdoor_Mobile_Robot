"""
stereo_feature_overlay.py

Left camera image with depth-coloured keypoints from stereo_odometry.

Each keypoint is coloured by its stereo-triangulated depth (from SensorData.points — Point3f[]):
  Red   — near  (< ~2 m)
  Green — mid   (~5 m)
  Blue  — far   (> ~12 m)
  Grey  — no valid depth (behind camera or triangulation failed)

A colour legend with depth scale is drawn in the bottom-right corner.
This gives an immediate visual read of what depths RTAB-Map is tracking,
without needing to understand the stereo geometry.

Topics consumed:
  <left_topic>               sensor_msgs/Image       (default /d457/left/image_raw)
  /odom_sensor_data/features rtabmap_msgs/SensorData — 2D keypoints + 3D positions
  /odom_info                 rtabmap_msgs/OdomInfo   — inliers / matches / keyframe flag

Topic published:
  <pub_topic>                sensor_msgs/Image       (default /d457/stereo_features)

Params:
  max_fps    (default 5.0)
  left_topic (default /d457/left/image_raw)
  pub_topic  (default /d457/stereo_features)
  max_depth  (default 6.0)  — depth in metres mapped to far end of colour scale
                              raise this for open outdoor scenes (e.g. 15.0)
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

# ── visual constants ─────────────────────────────────────────────────────────
KP_COLOR_FALLBACK = (0, 220, 255)   # yellow-cyan used when no 3D depth available
KP_CENTER_R       = 2
KP_SCALE          = 0.15
STATUS_OK         = (40, 220, 40)
STATUS_WARN       = (0, 180, 255)
STATUS_LOST       = (40, 40, 230)
KF_BG             = (30, 140, 30)
FONT              = cv2.FONT_HERSHEY_SIMPLEX
INLIER_WARN       = 10
LEGEND_W          = 18   # pixels wide for the depth colour bar
LEGEND_H          = 120  # pixels tall

_BE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)


def _depth_to_bgr(z: float, max_depth: float) -> tuple[int, int, int]:
    """Red (near) → Green (mid) → Blue (far), Grey for invalid depth."""
    if z <= 0.0 or z > max_depth * 1.5:
        return (128, 128, 128)          # grey — no valid triangulation
    t = min(z / max_depth, 1.0)         # 0 = nearest, 1 = farthest
    if t < 0.5:                         # near half: red → green
        s = t * 2.0
        return (0, int(255 * s), int(255 * (1.0 - s)))
    else:                               # far half: green → blue
        s = (t - 0.5) * 2.0
        return (int(255 * s), int(255 * (1.0 - s)), 0)


class StereoFeatureOverlay(Node):

    def __init__(self):
        super().__init__('stereo_feature_overlay')
        self._bridge = CvBridge()
        self._last_draw: float = 0.0

        self.declare_parameter('max_fps',    5.0)
        self.declare_parameter('left_topic', '/camera/camera/infra1/image_rect_raw')
        self.declare_parameter('pub_topic',  '/d457/stereo_features')
        self.declare_parameter('max_depth',  6.0)   # matches Vis/MaxDepth in stereo_odometry

        fps         = float(self.get_parameter('max_fps').value   or 5.0)
        left_topic  = str(self.get_parameter('left_topic').value  or '/camera/camera/infra1/image_rect_raw')
        pub_topic   = str(self.get_parameter('pub_topic').value   or '/d457/stereo_features')
        self._max_depth = float(self.get_parameter('max_depth').value or 6.0)
        self._min_interval = 1.0 / max(fps, 0.1)

        img_sub  = message_filters.Subscriber(
            self, Image,      left_topic,                   qos_profile=_BE_QOS)
        feat_sub = message_filters.Subscriber(
            self, SensorData, '/odom_sensor_data/features', qos_profile=_BE_QOS)
        sync = message_filters.ApproximateTimeSynchronizer(
            [img_sub, feat_sub], queue_size=4, slop=0.1)
        sync.registerCallback(self._draw_cb)

        self._pub = self.create_publisher(Image, pub_topic, 10)
        self.get_logger().info(
            f'stereo_feature_overlay ready  max_fps={fps}  max_depth={self._max_depth} m\n'
            f'  src: {left_topic}\n'
            f'  out: {pub_topic}')


    def _draw_cb(self, img_msg: Image, feat_msg: SensorData) -> None:
        now = time.monotonic()
        if now - self._last_draw < self._min_interval:
            return
        self._last_draw = now

        img = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        h, w = img.shape[:2]

        kp3d = feat_msg.points   # rtabmap_msgs/Point3f[], same length as key_points when available
        self._draw_keypoints(img, feat_msg.key_points, kp3d, w, h)
        
        self._draw_keyframe_banner(img, w, h)

        self._draw_depth_legend(img, w, h)
        self._pub.publish(self._bridge.cv2_to_imgmsg(img, encoding='bgr8'))

    # ── drawing helpers ────────────────────────────────────────────────────────

    def _draw_keypoints(self, img, key_points, key_points_3d, w: int, h: int) -> None:
        has_depth = len(key_points_3d) == len(key_points)
        for i, kp in enumerate(key_points):
            x, y = int(kp.pt.x), int(kp.pt.y)
            if not (0 <= x < w and 0 <= y < h):
                continue
            r = max(5, int(kp.size * KP_SCALE))

            if has_depth:
                color = _depth_to_bgr(float(key_points_3d[i].z), self._max_depth)
            else:
                color = KP_COLOR_FALLBACK

            # cv2.circle(img, (x, y), r,           color, 1,  cv2.LINE_AA)
            cv2.circle(img, (x, y), KP_CENTER_R, color, -1, cv2.LINE_AA)
            if kp.angle >= 0:
                rad = math.radians(kp.angle)
                ex  = int(x + r * math.cos(rad))
                ey  = int(y + r * math.sin(rad))
                cv2.line(img, (x, y), (ex, ey), color, 1, cv2.LINE_AA)

    def _draw_keyframe_banner(self, img, w: int, h: int) -> None:
        text = ' NEW KEYFRAME — reference frame updated '
        (tw, th), _ = cv2.getTextSize(text, FONT, 0.8, 2)
        bx, by = (w - tw) // 2, h - 18
        cv2.rectangle(img, (bx - 6, by - th - 8), (bx + tw + 6, by + 6), KF_BG, -1)
        cv2.putText(img, text, (bx, by), FONT, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

    def _draw_depth_legend(self, img, w: int, h: int) -> None:
        """Vertical colour bar in bottom-right corner with depth labels."""
        margin = 6
        bx = w - LEGEND_W - margin - 30   # leave room for text
        by = h - LEGEND_H - margin

        # Draw gradient bar top (far=blue) → bottom (near=red)
        for row in range(LEGEND_H):
            t       = 1.0 - row / (LEGEND_H - 1)   # 1.0 at top (far), 0.0 at bottom (near)
            z       = t * self._max_depth
            b, g, r = _depth_to_bgr(z, self._max_depth)
            cv2.line(img, (bx, by + row), (bx + LEGEND_W - 1, by + row), (b, g, r), 1)

        cv2.rectangle(img, (bx, by), (bx + LEGEND_W, by + LEGEND_H), (200, 200, 200), 1)

        # Labels at top, middle, bottom
        tx = bx + LEGEND_W + 3
        labels = [
            (by + 8,           f'{self._max_depth:.0f}m'),
            (by + LEGEND_H//2, f'{self._max_depth/2:.0f}m'),
            (by + LEGEND_H - 2, '0m'),
        ]
        for ly, lbl in labels:
            cv2.putText(img, lbl, (tx, ly), FONT, 0.3, (0, 0, 0),       1, cv2.LINE_AA)
            cv2.putText(img, lbl, (tx, ly), FONT, 0.3, (200, 200, 200), 1, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = StereoFeatureOverlay()
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
