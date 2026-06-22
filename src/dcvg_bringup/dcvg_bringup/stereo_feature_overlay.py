"""
stereo_feature_overlay.py  (REAL HARDWARE — depth-camera / RGBD debug view)

Draws RTAB-Map's tracked keypoints onto the RealSense D455 COLOR image, each
keypoint coloured by the depth RTAB-Map measured for it. This is the RGBD
counterpart to the IR-stereo overlay: keypoints are detected on the colour
image and their depth comes from the aligned depth frame, so the overlay shows
*which real element each feature landed on* and *how far away it is*.

Keypoint colour (from SensorData.points — Point3f[], same order as key_points):
  Red   — near  (< ~⅓ max_depth)
  Green — mid
  Blue  — far   (→ max_depth)
  Grey  — no valid depth (hole / out of range / triangulation failed)

Stats panel (top-left) — the numbers to analyse tracking quality:
  keypoints    — total features detected this frame
  with-depth   — how many got a valid depth (the rest are grey / unusable)
  near/mid/far — depth distribution of the valid ones
  inliers/matches/ratio + features/local_map — from /odom_info (if available)

A depth colour legend is drawn bottom-right. A NEW KEYFRAME banner appears only
when rgbd_odometry actually adds a reference frame.

Topics consumed:
  <image_topic>              sensor_msgs/Image       (default /camera/camera/color/image_raw)
  /odom_sensor_data/features rtabmap_msgs/SensorData — 2D keypoints + 3D positions
  /odom_info                 rtabmap_msgs/OdomInfo   — inliers / matches / keyframe flag

Topic published:
  <pub_topic>                sensor_msgs/Image       (default /d455/depth_features)

Run:
  ros2 run dcvg_bringup stereo_feature_overlay
  ros2 run rqt_image_view rqt_image_view /d455/depth_features

Params:
  max_fps     (default 5.0)  — cap drawing rate so OpenCV never starves odom
  image_topic (default /camera/camera/color/image_raw)
  pub_topic   (default /d455/depth_features)
  max_depth   (default 6.0)  — depth (m) mapped to far end of colour scale;
                               D455 RGBD maxes ~6 m outdoors. Match Vis/MaxDepth.
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


def _is_valid_depth(z: float) -> bool:
    """A stereo feature has usable depth only if it was triangulated to a
    finite, positive z. Failed disparity matches arrive as 0.0 (or NaN/inf),
    which is genuinely 'depth unknown' — not a far point."""
    return math.isfinite(z) and z > 0.0


def _depth_to_bgr(z: float, max_depth: float) -> tuple[int, int, int]:
    """Red (near) → Green (mid) → Blue (far). Grey ONLY when depth is missing.

    Far points beyond max_depth are clamped to blue rather than greyed: in
    stereo VO a large triangulated depth is still a real 'far' measurement
    (just less precise), so it should read blue, not 'unknown'."""
    if not _is_valid_depth(z):
        return (128, 128, 128)          # grey — no depth triangulated for this feature
    t = min(z / max_depth, 1.0)         # 0 = nearest, 1 = farthest (clamped)
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
        self._latest_info: OdomInfo | None = None
        self._last_draw: float = 0.0

        self.declare_parameter('max_fps',     5.0)
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('pub_topic',   '/d455/depth_features')
        self.declare_parameter('max_depth',   6.0)   # matches Vis/MaxDepth in rgbd_odometry

        fps          = float(self.get_parameter('max_fps').value    or 5.0)
        image_topic  = str(self.get_parameter('image_topic').value  or '/camera/camera/color/image_raw')
        pub_topic    = str(self.get_parameter('pub_topic').value    or '/d455/depth_features')
        self._max_depth    = float(self.get_parameter('max_depth').value or 6.0)
        self._min_interval = 1.0 / max(fps, 0.1)

        self.create_subscription(OdomInfo, '/odom_info', self._info_cb, 20)

        img_sub  = message_filters.Subscriber(
            self, Image,      image_topic,                  qos_profile=_BE_QOS)
        feat_sub = message_filters.Subscriber(
            self, SensorData, '/odom_sensor_data/features', qos_profile=_BE_QOS)
        sync = message_filters.ApproximateTimeSynchronizer(
            [img_sub, feat_sub], queue_size=4, slop=0.1)
        sync.registerCallback(self._draw_cb)

        self._pub = self.create_publisher(Image, pub_topic, 10)
        self.get_logger().info(
            f'stereo_feature_overlay (RGBD) ready  max_fps={fps}  max_depth={self._max_depth} m\n'
            f'  src: {image_topic}\n'
            f'  out: {pub_topic}')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _info_cb(self, msg: OdomInfo) -> None:
        self._latest_info = msg

    def _draw_cb(self, img_msg: Image, feat_msg: SensorData) -> None:
        # FPS throttle — skip frames between draw intervals so OpenCV drawing
        # never competes with rtabmap's odom thread for CPU.
        now = time.monotonic()
        if now - self._last_draw < self._min_interval:
            return
        self._last_draw = now

        img = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        h, w = img.shape[:2]

        kp3d = feat_msg.points   # rtabmap_msgs/Point3f[], same length as key_points when available
        counts = self._draw_keypoints(img, feat_msg.key_points, kp3d, w, h)

        info = self._latest_info
        self._draw_status(img, counts, info)
        if info is not None and info.key_frame_added:
            self._draw_keyframe_banner(img, w, h)

        self._draw_depth_legend(img, w, h)
        self._pub.publish(self._bridge.cv2_to_imgmsg(img, encoding='bgr8'))

    # ── drawing helpers ────────────────────────────────────────────────────────

    def _draw_keypoints(self, img, key_points, key_points_3d, w: int, h: int) -> dict:
        """Draw each keypoint coloured by depth; return a count breakdown."""
        has_depth = len(key_points_3d) == len(key_points)
        counts = {'total': len(key_points), 'depth': 0,
                  'near': 0, 'mid': 0, 'far': 0, 'nodepth': 0}

        for i, kp in enumerate(key_points):
            x, y = int(kp.pt.x), int(kp.pt.y)
            if not (0 <= x < w and 0 <= y < h):
                continue
            r = max(5, int(kp.size * KP_SCALE))

            z = float(key_points_3d[i].z) if has_depth else 0.0
            if has_depth and _is_valid_depth(z):
                counts['depth'] += 1
                t = min(z / self._max_depth, 1.0)   # clamp far → blue, never grey
                if t < 1.0 / 3.0:
                    counts['near'] += 1
                elif t < 2.0 / 3.0:
                    counts['mid'] += 1
                else:
                    counts['far'] += 1
                color = _depth_to_bgr(z, self._max_depth)
            else:
                counts['nodepth'] += 1
                # grey = depth not triangulated; yellow-cyan = no 3D array at all
                color = (128, 128, 128) if has_depth else KP_COLOR_FALLBACK

            cv2.circle(img, (x, y), KP_CENTER_R, color, -1, cv2.LINE_AA)
            if kp.angle >= 0:
                rad = math.radians(kp.angle)
                ex  = int(x + r * math.cos(rad))
                ey  = int(y + r * math.sin(rad))
                cv2.line(img, (x, y), (ex, ey), color, 1, cv2.LINE_AA)

        return counts

    def _draw_status(self, img, counts: dict, info: OdomInfo | None) -> None:
        """Top-left panel: keypoint counts + odom stats — the numbers to analyse."""
        if info is not None and info.lost:
            color, tag = STATUS_LOST, 'LOST'
        elif info is not None and info.inliers < INLIER_WARN:
            color, tag = STATUS_WARN, 'WARN'
        else:
            color, tag = STATUS_OK, 'OK'

        pct = (100.0 * counts['depth'] / counts['total']) if counts['total'] else 0.0
        lines = [
            f"[{tag}]  keypoints={counts['total']}  "
            f"with-depth={counts['depth']} ({pct:.0f}%)  no-depth={counts['nodepth']}",
            f"near={counts['near']}  mid={counts['mid']}  far={counts['far']}",
        ]
        if info is not None:
            ratio = info.inliers / info.matches if info.matches > 0 else 0.0
            lines.append(
                f'inliers={info.inliers}  matches={info.matches}  ratio={ratio:.2f}'
                f'  features={info.features}  local_map={info.local_map_size}'
                f'  dt={info.interval * 1000:.0f}ms')

        for i, line in enumerate(lines):
            y = 22 + i * 18
            cv2.putText(img, line, (10, y + 1), FONT, 0.45, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(img, line, (10, y),     FONT, 0.45, color,     1, cv2.LINE_AA)

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
            (by + 8,            f'{self._max_depth:.0f}m'),
            (by + LEGEND_H // 2, f'{self._max_depth / 2:.0f}m'),
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
