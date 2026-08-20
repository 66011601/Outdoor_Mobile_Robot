"""
osm_route_app.py — Interactive web UI for route planning and waypoint editing.

Talks to the real-hardware navigation stack (`dcvg_bringup bringup.launch.py`)
purely over its ROS topic contract (`/gps/fix`, `/odometry/global`, `/cmd_vel`,
a camera image topic) — see the "Sensor Topics" table in CLAUDE.md. It also
spawns/kills `route_waypoint_follower` itself (via Start/Pause/Stop in the UI).

The default camera topic, `/camera/color/image_raw`, matches dcvg_bringup's
RGBD default (`rtabmap.launch.py`'s `rgb_topic`). Stereo mode
(`use_stereo:=true`) uses `/camera/infra1/image_rect_raw` instead — that's a
ROS param, not a code change:

Usage:
  ros2 run dcvg_robot osm_route_app
  ros2 run dcvg_robot osm_route_app --ros-args -p camera_topic:=/camera/infra1/image_rect_raw  # stereo mode
  Then open:  http://localhost:5000
"""

import math as _math
import os
import subprocess
import sys
import threading
import time
import traceback
import yaml

_NAV_CMD_FILE      = '/tmp/dcvg_nav_cmd'
_STOP_STATUS_FILE  = '/tmp/dcvg_stop_status'
_STOP_CONFIRM_FILE = '/tmp/dcvg_stop_confirm'
_NAV_TARGET_FILE   = '/tmp/dcvg_nav_target'   # current goal published by route_waypoint_follower
_nav_proc = None
_nav_state = 'stopped'  # 'stopped' | 'running' | 'paused'

# ---------------------------------------------------------------------------
# Robot pose monitoring (background ROS2 subscriber thread)
# ---------------------------------------------------------------------------
_robot_pose = {'available': False, 'lat': 0.0, 'lon': 0.0, 'heading': 0.0,
               'lin': 0.0, 'ang': 0.0}

# Latest camera frame (JPEG bytes) for the monitoring screen. We only encode
# frames while a viewer is connected (want_ts kept fresh by the stream endpoint).
#
# Default matches dcvg_bringup/launch/rtabmap.launch.py's rgb_topic default.
# Override with a ROS param for stereo VIO (use_stereo:=true) — see the module
# docstring above for the exact topic.
_DEFAULT_CAMERA_TOPIC = '/camera/color/image_raw'
_camera = {'jpeg': None, 'ts': 0.0, 'want_ts': 0.0, 'topic': _DEFAULT_CAMERA_TOPIC}

# ---------------------------------------------------------------------------
# Manual teleop (publishes /cmd_vel for over-control when nav is paused/stopped)
# ---------------------------------------------------------------------------
_TELEOP_DEADMAN_S = 0.5     # stop the robot if no fresh command within this window
_teleop = {'active': False, 'lin': 0.0, 'ang': 0.0, 'ts': 0.0, 'disengage_at': 0.0}
_ros = {'cmd_pub': None}    # filled in once the ROS monitor node is up

# ---------------------------------------------------------------------------
# Stall watchdog — auto-pause + alert if the robot stops moving mid-route
# (stuck / blocked) while it is supposed to be navigating.
# ---------------------------------------------------------------------------
_STALL_LIN       = 0.03     # m/s   below this (and below _STALL_ANG) = "not moving"
_STALL_ANG       = 0.05     # rad/s (spins/recoveries exceed this, so they don't trip it)
_STALL_TIMEOUT_S = 7.0      # motionless this long while navigating → auto-pause
_STALL_GRACE_S   = 6.0      # ignore stalls right after a (re)start while Nav2 plans
_stall = {'since': None, 'notified': False, 'grace_until': 0.0}
_nav_alert = {'active': False, 'message': ''}


def _start_ros_monitor():
    def _spin():
        try:
            import time
            import rclpy
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import NavSatFix, Image
            from geometry_msgs.msg import Twist
            rclpy.init(args=sys.argv)   # so --ros-args -p camera_topic:=... is honored
            node = rclpy.create_node('osm_route_monitor')

            node.declare_parameter('camera_topic', _DEFAULT_CAMERA_TOPIC)
            _camera['topic'] = str(node.get_parameter('camera_topic').value)

            def gps_cb(msg):
                if msg.status.status >= 0:
                    _robot_pose['lat'] = msg.latitude
                    _robot_pose['lon'] = msg.longitude
                    _robot_pose['available'] = True

            def odom_cb(msg):
                q = msg.pose.pose.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                _robot_pose['heading'] = _math.atan2(siny, cosy)
                _robot_pose['lin'] = msg.twist.twist.linear.x
                _robot_pose['ang'] = msg.twist.twist.angular.z

            # Change '/gps/fix' if your GPS topic name differs
            node.create_subscription(NavSatFix, '/gps/fix', gps_cb, 10)
            # ekf_local (odom frame), not ekf_global: dual_ekf_navsat_params.yaml fuses
            # the accurate GPS moving-baseline heading (gps_heading_to_imu) into
            # ekf_local's imu1 specifically because ekf_global can't safely carry a
            # continuously-updating absolute yaw source (see that file's imu0 comment
            # under ekf_filter_node_map for the root-caused reason — a feedback loop
            # through the map->odom transform, unrelated to sensor accuracy).
            # ekf_global's own orientation is therefore NOT accurate — only rate-bounded.
            # Twist (lin/ang below) is reported in the body frame either way.
            node.create_subscription(Odometry, '/odometry/local', odom_cb, 10)

            # Camera → JPEG for the monitoring screen. Encoding only runs while a
            # browser is actually watching the stream (cheap drop otherwise).
            try:
                from cv_bridge import CvBridge
                import cv2
                _bridge = CvBridge()

                def image_cb(msg):
                    if time.monotonic() - _camera['want_ts'] > 3.0:
                        return                      # nobody watching → skip encode
                    if time.monotonic() - _camera['ts'] < 0.05:
                        return                      # cap at ~20 fps
                    try:
                        cv_img = _bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                        ok, jpg = cv2.imencode(
                            '.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                        if ok:
                            _camera['jpeg'] = jpg.tobytes()
                            _camera['ts'] = time.monotonic()
                    except Exception:
                        pass

                node.create_subscription(Image, _camera['topic'], image_cb, 5)
            except Exception as cam_exc:
                print(f'[osm_route_app] camera stream disabled: {cam_exc}')

            # /cmd_vel publisher for manual teleop. Only publishes while teleop is
            # engaged — leaving Nav2 in sole control of /cmd_vel otherwise.
            _ros['cmd_pub'] = node.create_publisher(Twist, '/cmd_vel', 10)

            def _teleop_tick():
                t = _teleop
                if not t['active']:
                    return
                now = time.monotonic()
                lin, ang = t['lin'], t['ang']
                # Deadman: a frozen tab / dropped network stops the robot.
                if now - t['ts'] > _TELEOP_DEADMAN_S:
                    lin = ang = 0.0
                # Disengaging: send zeros briefly, then release /cmd_vel entirely.
                if t['disengage_at'] and now >= t['disengage_at']:
                    lin = ang = 0.0
                    t['active'] = False
                    t['disengage_at'] = 0.0
                msg = Twist()
                msg.linear.x = float(lin)
                msg.angular.z = float(ang)
                _ros['cmd_pub'].publish(msg)

            node.create_timer(0.1, _teleop_tick)   # 10 Hz keep-alive / watchdog

            def _stall_tick():
                global _nav_state
                st = _stall
                if _nav_state != 'running':
                    st['since'] = None
                    st['notified'] = False
                    return
                now = time.monotonic()
                if now < st['grace_until']:
                    st['since'] = None          # Nav2 still spinning up after start
                    return
                # Don't flag a deliberate wait at a stop point.
                try:
                    with open(_STOP_STATUS_FILE) as f:
                        if f.read().strip():
                            st['since'] = None
                            return
                except FileNotFoundError:
                    pass
                moving = (abs(_robot_pose.get('lin', 0.0)) > _STALL_LIN or
                          abs(_robot_pose.get('ang', 0.0)) > _STALL_ANG)
                if moving:
                    st['since'] = None
                    return
                if st['since'] is None:
                    st['since'] = now
                elif (now - st['since'] > _STALL_TIMEOUT_S) and not st['notified']:
                    _write_nav_cmd('pause')     # stop the robot in place
                    _nav_state = 'paused'
                    _nav_alert['active'] = True
                    _nav_alert['message'] = (
                        f"Robot hasn't moved for {int(_STALL_TIMEOUT_S)}s — it may be "
                        "stuck or blocked. Navigation auto-paused. Take manual control "
                        "or resume when clear.")
                    st['notified'] = True
                    print('[osm_route_app] STALL detected → auto-paused navigation')

            node.create_timer(0.5, _stall_tick)    # 2 Hz stall watchdog
            rclpy.spin(node)
        except Exception as exc:
            print(f'[osm_route_app] ROS monitor unavailable: {exc}')

    threading.Thread(target=_spin, daemon=True).start()


_start_ros_monitor()


def _write_nav_cmd(cmd):
    with open(_NAV_CMD_FILE, 'w') as f:
        f.write(cmd)


def _clear_nav_alert():
    _nav_alert['active'] = False
    _nav_alert['message'] = ''


def _arm_stall_watchdog():
    """Reset the stall detector and grant a grace window (used on (re)start)."""
    _stall['since'] = None
    _stall['notified'] = False
    _stall['grace_until'] = time.monotonic() + _STALL_GRACE_S
    _clear_nav_alert()


def _refresh_nav_state():
    global _nav_proc, _nav_state
    if _nav_proc is not None and _nav_proc.poll() is not None:
        _nav_proc = None
        _nav_state = 'stopped'

try:
    from flask import Flask, request, jsonify, Response
except ImportError:
    print("ERROR: flask is not installed.  pip install flask")
    sys.exit(1)

app = Flask(__name__)

# ---------------------------------------------------------------------------
# Package path helpers
# ---------------------------------------------------------------------------

def _pkg_paths():
    """Return (share_dir, src_config_dir). Either may be None."""
    share_dir = src_dir = None
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('dcvg_robot')
        ws_root = os.path.normpath(os.path.join(share_dir, '..', '..', '..', '..'))
        src_dir  = os.path.join(ws_root, 'src',
                                'dcvg_robot', 'config')
    except Exception:
        pass
    return share_dir, src_dir


def _find_config(filename):
    share_dir, src_dir = _pkg_paths()
    for d in [src_dir, share_dir and os.path.join(share_dir, 'config')]:
        if d:
            p = os.path.join(d, filename)
            if os.path.isfile(p):
                return p
    return None


def _save_yaml(data, filename):
    share_dir, src_dir = _pkg_paths()
    saved = []
    for d in [src_dir, share_dir and os.path.join(share_dir, 'config')]:
        if d and os.path.isdir(d):
            p = os.path.join(d, filename)
            with open(p, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            saved.append(p)
    return saved


# ---------------------------------------------------------------------------
# API endpoints
# ---------------------------------------------------------------------------

@app.route('/api/generate', methods=['POST'])
def api_generate():
    body = request.json or {}
    stops      = body.get('stops', [])
    cfg_params = body.get('config', {})

    if len(stops) < 2:
        return jsonify({'error': 'Need at least 2 stops'}), 400

    cfg = {
        'network_type':     cfg_params.get('network_type', 'none (straight line)'),
        'custom_filter':    cfg_params.get('custom_filter') or None,
        'buffer_m':         float(cfg_params.get('buffer_m', 200.0)),
        'max_gap_m':        float(cfg_params.get('max_gap_m', 25.0)),
        'lateral_offset_m': float(cfg_params.get('lateral_offset_m', 0.0)),
        'offset_side':      cfg_params.get('offset_side', 'right'),
    }

    try:
        from dcvg_robot.osm_route_generator import haversine_distance
        if cfg['network_type'] == 'none':
            from dcvg_robot.osm_route_generator import generate_straight_route
            wps = generate_straight_route(stops, cfg)
        else:
            from dcvg_robot.osm_route_generator import generate_route_from_stops
            wps = generate_route_from_stops(stops, cfg)
        total_m = sum(
            haversine_distance(wps[i]['latitude'],   wps[i]['longitude'],
                               wps[i+1]['latitude'], wps[i+1]['longitude'])
            for i in range(len(wps) - 1)
        )
        output = {'waypoints': [
            {'latitude':  round(float(w['latitude']),  8),
             'longitude': round(float(w['longitude']), 8),
             'yaw':       round(float(w.get('yaw', 0.0)), 6)}
            for w in wps
        ]}
        saved = _save_yaml(output, 'waypoints.yaml')
        return jsonify({'waypoints': wps, 'total_m': round(total_m, 1),
                        'saved': saved[-1] if saved else None})
    except Exception as exc:
        traceback.print_exc()
        return jsonify({'error': str(exc)}), 500



@app.route('/api/save', methods=['POST'])
def api_save():
    body = request.json or {}
    wps  = body.get('waypoints', [])
    if not wps:
        return jsonify({'error': 'No waypoints provided'}), 400
    output = {'waypoints': [
        {'latitude':  round(float(w['latitude']),  8),
         'longitude': round(float(w['longitude']), 8),
         'yaw':       round(float(w.get('yaw', 0.0)), 6)}
        for w in wps
    ]}
    saved = _save_yaml(output, 'waypoints.yaml')
    if not saved:
        return jsonify({'error': 'Could not determine save path'}), 500
    return jsonify({'status': 'ok', 'path': saved[-1], 'count': len(wps)})



@app.route('/api/stops')
def api_stops():
    """Return start/end stops from osm_route_config.yaml so the UI can pre-populate them."""
    path = _find_config('osm_route_config.yaml')
    if not path:
        return jsonify({'stops': []})
    with open(path) as f:
        data = yaml.safe_load(f)
    r = data.get('route', {})
    stops = []
    for key in ('start', 'end'):
        pt = r.get(key)
        if pt and 'latitude' in pt and 'longitude' in pt:
            stops.append({'latitude': float(pt['latitude']),
                          'longitude': float(pt['longitude'])})
    return jsonify({'stops': stops})


@app.route('/api/stops_config', methods=['GET', 'POST'])
def api_stops_config():
    if request.method == 'GET':
        path = _find_config('stops.yaml')
        if not path:
            return jsonify({'confirmation_topic': '/dcvg/stop_confirm',
                            'reached_topic': '/dcvg/stop_reached', 'stops': []})
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return jsonify({
            'confirmation_topic': data.get('confirmation_topic', '/dcvg/stop_confirm'),
            'reached_topic':      data.get('reached_topic',      '/dcvg/stop_reached'),
            'stops': data.get('stops', [])
        })
    body = request.json or {}
    data = {
        'confirmation_topic': body.get('confirmation_topic', '/dcvg/stop_confirm'),
        'reached_topic':      body.get('reached_topic',      '/dcvg/stop_reached'),
        'stops': [{'index':     int(s['index']),
                   'latitude':  round(float(s['latitude']),  8),
                   'longitude': round(float(s['longitude']), 8)}
                  for s in body.get('stops', [])]
    }
    saved = _save_yaml(data, 'stops.yaml')
    if not saved:
        return jsonify({'error': 'Could not save'}), 500
    return jsonify({'status': 'ok', 'count': len(data['stops'])})


_WAYPOINT_SANITY_RADIUS_M = 2000.0   # beyond this, Start requires confirmation


def _check_waypoint_sanity():
    """Return the great-circle distance (m) from the robot's last GPS fix to
    the current waypoints.yaml's first waypoint, if it's implausibly far —
    else None. Guards against Start silently driving toward a stale/leftover
    route left over from a different site or session."""
    if not _robot_pose['available']:
        return None
    path = _find_config('waypoints.yaml')
    if not path:
        return None
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        wps = data.get('waypoints', [])
        if not wps:
            return None
        first = wps[0]
        from dcvg_robot.osm_route_generator import haversine_distance
        d = haversine_distance(_robot_pose['lat'], _robot_pose['lon'],
                                float(first['latitude']), float(first['longitude']))
        return d if d > _WAYPOINT_SANITY_RADIUS_M else None
    except Exception:
        return None


@app.route('/api/nav/start', methods=['POST'])
def api_nav_start():
    global _nav_proc, _nav_state
    _refresh_nav_state()
    # get_json(silent=True): a bare POST (no body/content-type — e.g. a stale
    # cached tab predating this endpoint's confirm_far body, or curl -X POST
    # with nothing else) must still start navigation, not 500 on a safety-
    # critical endpoint. request.json raises on that instead of returning None.
    body = request.get_json(silent=True) or {}
    if _nav_state != 'paused' and not body.get('confirm_far'):
        far_m = _check_waypoint_sanity()
        if far_m is not None:
            return jsonify({
                'error': 'far_start',
                'message': f'Route start is {far_m / 1000:.1f} km from the '
                           f'robot\'s last GPS fix.',
                'distance_m': far_m,
            }), 409
    if _nav_state == 'paused':
        _write_nav_cmd('run')
        _nav_state = 'running'
        _arm_stall_watchdog()
        return jsonify({'state': 'running'})
    if _nav_proc and _nav_proc.poll() is None:
        _write_nav_cmd('stop')
        try:
            _nav_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            _nav_proc.terminate()
    _write_nav_cmd('run')
    _nav_proc = subprocess.Popen(['ros2', 'run', 'dcvg_robot', 'route_waypoint_follower'])
    _nav_state = 'running'
    _arm_stall_watchdog()
    return jsonify({'state': 'running'})


@app.route('/api/nav/pause', methods=['POST'])
def api_nav_pause():
    global _nav_state
    _refresh_nav_state()
    if _nav_state != 'running':
        return jsonify({'error': 'Not running'}), 400
    _write_nav_cmd('pause')
    _nav_state = 'paused'
    _clear_nav_alert()
    return jsonify({'state': 'paused'})


@app.route('/api/nav/stop', methods=['POST'])
def api_nav_stop():
    global _nav_proc, _nav_state
    _write_nav_cmd('stop')
    if _nav_proc and _nav_proc.poll() is None:
        try:
            _nav_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            _nav_proc.terminate()
    _nav_proc = None
    _nav_state = 'stopped'
    _clear_nav_alert()
    # Drop the target highlight if the follower was killed before clearing it
    try:
        os.remove(_NAV_TARGET_FILE)
    except FileNotFoundError:
        pass
    return jsonify({'state': 'stopped'})


@app.route('/api/nav/status')
def api_nav_status():
    _refresh_nav_state()
    return jsonify({'state': _nav_state, 'alert': dict(_nav_alert)})


@app.route('/api/nav/ack_alert', methods=['POST'])
def api_nav_ack_alert():
    _clear_nav_alert()
    return jsonify({'status': 'ok'})


# ---------------------------------------------------------------------------
# Manual teleop — over-control. Navigation must yield /cmd_vel first, so
# engaging manual mode auto-pauses an active route.
# ---------------------------------------------------------------------------
@app.route('/api/teleop/engage', methods=['POST'])
def api_teleop_engage():
    global _nav_state
    import time
    if _ros['cmd_pub'] is None:
        return jsonify({'error': 'cmd_vel publisher not ready (is ROS running?)'}), 503
    _refresh_nav_state()
    # Take over: pause the follower so Nav2 stops driving before we do.
    if _nav_state == 'running':
        _write_nav_cmd('pause')
        _nav_state = 'paused'
    _clear_nav_alert()   # taking manual control acknowledges any stall alert
    _teleop.update(active=True, lin=0.0, ang=0.0,
                   ts=time.monotonic(), disengage_at=0.0)
    return jsonify({'state': 'manual', 'nav_state': _nav_state})


@app.route('/api/teleop', methods=['POST'])
def api_teleop_drive():
    import time
    if not _teleop['active']:
        return jsonify({'error': 'Manual control not engaged'}), 409
    body = request.json or {}
    _teleop.update(lin=float(body.get('lin', 0.0)),
                   ang=float(body.get('ang', 0.0)),
                   ts=time.monotonic())
    return jsonify({'ok': True})


@app.route('/api/teleop/disengage', methods=['POST'])
def api_teleop_disengage():
    import time
    # Coast to a stop (zeros) for a moment, then release /cmd_vel back to Nav2.
    now = time.monotonic()
    _teleop.update(lin=0.0, ang=0.0, ts=now, disengage_at=now + 0.4)
    return jsonify({'state': 'idle'})


@app.route('/api/robot_pose')
def api_robot_pose():
    return jsonify(_robot_pose)


@app.route('/api/camera_topic')
def api_camera_topic():
    """Topic the monitoring screen is actually subscribed to (see camera_topic
    ROS param — differs between D455 RGBD / D457 stereo and any real-HW override)."""
    return jsonify({'topic': _camera['topic']})


@app.route('/api/camera')
def api_camera():
    """MJPEG stream of the robot camera for the monitoring screen."""
    import time

    def gen():
        last_ts = 0.0
        while True:
            _camera['want_ts'] = time.monotonic()   # keep the encoder alive
            frame, ts = _camera['jpeg'], _camera['ts']
            if frame and ts != last_ts:
                last_ts = ts
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + frame + b'\r\n')
            time.sleep(0.04)

    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/stop_status')
def api_stop_status():
    try:
        with open(_STOP_STATUS_FILE) as f:
            msg = f.read().strip()
        return jsonify({'waiting': bool(msg), 'message': msg})
    except FileNotFoundError:
        return jsonify({'waiting': False, 'message': ''})


@app.route('/api/nav_target')
def api_nav_target():
    """Return the waypoint the follower is currently driving to, or active:False."""
    try:
        with open(_NAV_TARGET_FILE) as f:
            data = yaml.safe_load(f) or {}
        return jsonify({'active': True, **data})
    except FileNotFoundError:
        return jsonify({'active': False})
    except Exception:
        return jsonify({'active': False})


@app.route('/api/stop_confirm', methods=['POST'])
def api_stop_confirm():
    with open(_STOP_CONFIRM_FILE, 'w') as f:
        f.write('confirm')
    return jsonify({'status': 'ok'})


@app.route('/api/config')
def api_config():
    path = _find_config('osm_route_config.yaml')
    if not path:
        return jsonify({'config': {}})
    with open(path) as f:
        data = yaml.safe_load(f)
    r = data.get('route', {})
    return jsonify({'config': {
        'network_type':     r.get('network_type', 'none (straight line)'),
        'custom_filter':    r.get('custom_filter') or '',
        'buffer_m':         r.get('buffer_m', 200.0),
        'max_gap_m':        r.get('max_gap_m', 25.0),
        'lateral_offset_m': r.get('lateral_offset_m', 0.0),
        'offset_side':      r.get('offset_side', 'right'),
    }})


# ---------------------------------------------------------------------------
# Frontend
# ---------------------------------------------------------------------------

@app.route('/')
def index():
    return _HTML


_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>OSM Route Planner</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;font-size:13px;display:flex;height:100vh;overflow:hidden;background:#f0f0f0}

/* ── Sidebar ── */
#sidebar{width:310px;min-width:310px;background:#fff;border-right:1px solid #ddd;display:flex;flex-direction:column;overflow:hidden;box-shadow:2px 0 6px rgba(0,0,0,.07)}
#sidebar-header{padding:13px 16px;background:#2c3e50;color:#fff;font-weight:700;font-size:14px;letter-spacing:.3px}
#sidebar-header small{display:block;font-weight:400;font-size:11px;opacity:.7;margin-top:1px}

/* ── Tabs ── */
#tabs{display:flex;border-bottom:1px solid #ddd}
.tab{flex:1;padding:9px 4px;border:none;background:#f7f7f7;cursor:pointer;font-size:12px;font-weight:600;color:#888;border-bottom:2px solid transparent;transition:all .15s}
.tab.active{background:#fff;color:#2980b9;border-bottom-color:#2980b9}
.tab:disabled{opacity:.4;cursor:not-allowed}

/* ── Scrollable body ── */
#sidebar-body{flex:1;overflow-y:auto;padding:14px 12px}
.section{margin-bottom:18px}
.sec-title{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.8px;color:#aaa;margin-bottom:8px}

/* ── Form ── */
.row{display:flex;align-items:center;margin-bottom:7px;gap:8px}
.row label{width:108px;min-width:108px;color:#555;font-size:12px}
.row input,.row select{flex:1;padding:5px 7px;border:1px solid #ddd;border-radius:5px;font-size:12px;background:#fafafa}
.row input:focus,.row select:focus{outline:none;border-color:#2980b9;background:#fff}

/* ── Stop list ── */
.hint{color:#aaa;font-size:11px;font-style:italic;margin-bottom:8px}
#stop-list{list-style:none;margin-bottom:8px}
.stop-item{display:flex;align-items:center;gap:7px;padding:5px 0;border-bottom:1px solid #f2f2f2}
.badge{width:22px;height:22px;border-radius:50%;display:flex;align-items:center;justify-content:center;color:#fff;font-weight:800;font-size:10px;flex-shrink:0}
.stop-coords{flex:1;font-size:11px;color:#555;font-family:monospace;line-height:1.4}
.stop-rm{cursor:pointer;color:#ccc;font-size:18px;line-height:1;padding:0 2px;transition:color .1s}
.stop-rm:hover{color:#e74c3c}

/* ── Waypoint summary ── */
#wp-info{font-size:12px;color:#555;padding:7px 10px;background:#f8f8f8;border-radius:5px;margin-bottom:8px;line-height:1.6}
#wp-info b{color:#2c3e50}

/* ── Footer ── */
#sidebar-footer{padding:10px 12px;border-top:1px solid #eee;display:flex;flex-direction:column;gap:7px}
.btn{padding:8px 12px;border:none;border-radius:5px;cursor:pointer;font-size:12px;font-weight:700;width:100%;text-align:center;transition:opacity .15s}
.btn:disabled{opacity:.35;cursor:not-allowed}
.btn-blue{background:#2980b9;color:#fff}.btn-blue:hover:not(:disabled){background:#2471a3}
.btn-green{background:#27ae60;color:#fff}.btn-green:hover:not(:disabled){background:#229954}
.btn-gray{background:#ecf0f1;color:#555}.btn-gray:hover:not(:disabled){background:#dde4e6}
.btn-row{display:flex;gap:6px}
.btn-row .btn{flex:1}
.btn-red{background:#e74c3c;color:#fff}.btn-red:hover:not(:disabled){background:#cb4335}
.btn-orange{background:#e67e22;color:#fff}.btn-orange:hover:not(:disabled){background:#d35400}
#nav-state-badge{display:inline-block;padding:3px 8px;border-radius:10px;font-size:10px;font-weight:700;letter-spacing:.4px;text-transform:uppercase}

/* ── Target waypoint highlight ── */
@keyframes target-pulse{0%{box-shadow:0 0 0 0 rgba(230,126,34,.7)}70%{box-shadow:0 0 0 12px rgba(230,126,34,0)}100%{box-shadow:0 0 0 0 rgba(230,126,34,0)}}
.wp-target{width:20px;height:20px;border-radius:50%;background:#e67e22;border:3px solid #fff;box-shadow:0 2px 6px rgba(0,0,0,.4);animation:target-pulse 1.4s ease-out infinite}
#nav-target-card{display:none;padding:7px 10px;background:#fef6ec;border:1.5px solid #e67e22;border-radius:6px;margin-bottom:4px}
#nav-target-card .tc-label{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.6px;color:#d35400;margin-bottom:3px}
#nav-target-card .tc-body{font-size:11px;color:#555;font-family:monospace;line-height:1.7}
#nav-target-card .tc-body b{color:#d35400;font-size:13px}

/* ── Teleop pad ── */
#teleop-card{border-top:1px solid #eee;margin-top:6px;padding-top:8px}
#teleop-card .tp-head{display:flex;align-items:center;justify-content:space-between;margin-bottom:6px}
#teleop-card .tp-title{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.8px;color:#aaa}
#btn-teleop{font-size:10px;padding:3px 8px;border-radius:10px;border:none;cursor:pointer;font-weight:700;background:#34495e;color:#fff}
#btn-teleop.on{background:#16a085}
#teleop-pad{display:none;margin-top:4px}
#teleop-pad.show{display:block}
.tp-grid{display:grid;grid-template-columns:repeat(3,1fr);grid-gap:5px;max-width:170px;margin:0 auto}
.tp-grid button{padding:10px 0;font-size:16px;border:1px solid #ccc;border-radius:6px;background:#f7f9fb;cursor:pointer;user-select:none;touch-action:none}
.tp-grid button:active{background:#16a085;color:#fff;border-color:#16a085}
.tp-grid button.tp-stop{background:#c0392b;color:#fff;border-color:#c0392b;font-size:12px;font-weight:700}
.tp-grid .tp-empty{visibility:hidden}
.tp-speed{display:flex;align-items:center;gap:6px;margin:8px 2px 2px}
.tp-speed label{font-size:10px;color:#888;width:34px}
.tp-speed input{flex:1}
.tp-speed .tp-val{font-size:10px;color:#16a085;font-family:monospace;width:54px;text-align:right}
#teleop-pad .tp-hint{font-size:10px;color:#aaa;margin-top:6px;text-align:center}
#teleop-locked{font-size:10px;color:#aaa;margin-top:4px;line-height:1.4}

/* ── Status ── */
#status{padding:6px 8px;border-radius:4px;font-size:11px;min-height:22px;line-height:1.4}
.s-ok{background:#eafaf1;color:#1e8449}
.s-err{background:#fdedec;color:#c0392b}
.s-info{background:#eaf4fb;color:#1a5276}

/* ── Map ── */
#map{flex:1 1 auto}

/* ── Spinner overlay ── */
#spinner{display:none;position:fixed;inset:0;background:rgba(0,0,0,.45);z-index:9999;align-items:center;justify-content:center}
#spinner.show{display:flex}
.spin-box{background:#fff;padding:20px 28px;border-radius:10px;text-align:center;box-shadow:0 4px 20px rgba(0,0,0,.25)}
.spin-box p{margin-top:10px;font-size:13px;color:#555}
.spinner-ring{width:36px;height:36px;border:4px solid #ddd;border-top-color:#2980b9;border-radius:50%;animation:spin .7s linear infinite;margin:0 auto}
@keyframes spin{to{transform:rotate(360deg)}}

/* ── Leaflet custom icons ── */
.stop-icon{display:flex;align-items:center;justify-content:center;width:26px;height:26px;border-radius:50%;color:#fff;font-weight:800;font-size:11px;border:2px solid #fff;box-shadow:0 1px 5px rgba(0,0,0,.4)}
.stop-icon.passed{opacity:.4;filter:grayscale(1)}
.stop-icon.next{border-color:#e67e22;animation:stop-pulse 1.4s ease-out infinite}
@keyframes stop-pulse{0%{box-shadow:0 0 0 0 rgba(230,126,34,.7)}70%{box-shadow:0 0 0 13px rgba(230,126,34,0)}100%{box-shadow:0 0 0 0 rgba(230,126,34,0)}}
.wp-dot{width:10px;height:10px;border-radius:50%;background:#2980b9;border:2px solid #fff;box-shadow:0 1px 4px rgba(0,0,0,.35);cursor:grab;transition:transform .1s}
.wp-dot:hover{transform:scale(1.5);background:#1a6090}

/* ── Stop points ── */
.stop-item-ui{display:flex;align-items:center;gap:6px;padding:4px 0;border-bottom:1px solid #f2f2f2}
.stop-diamond-sm{width:11px;height:11px;background:#c0392b;border:2px solid #fff;border-radius:2px;transform:rotate(45deg);flex-shrink:0;box-shadow:0 1px 3px rgba(0,0,0,.3)}
#btn-add-stop.active{background:#27ae60 !important}

/* ── Search ── */
#search-wrap{display:flex;gap:6px;padding:10px 12px;border-bottom:1px solid #eee;background:#fafafa}
#search-input{flex:1;padding:6px 9px;border:1px solid #ddd;border-radius:5px;font-size:12px;background:#fff}
#search-input:focus{outline:none;border-color:#2980b9}
#btn-search{padding:6px 10px;border:none;border-radius:5px;background:#2980b9;color:#fff;font-size:12px;font-weight:700;cursor:pointer;white-space:nowrap}
#btn-search:hover{background:#2471a3}
#search-results{position:absolute;z-index:2000;background:#fff;border:1px solid #ddd;border-radius:5px;box-shadow:0 4px 12px rgba(0,0,0,.12);max-height:200px;overflow-y:auto;width:calc(310px - 24px);left:12px;top:0;display:none}
#search-results .sr-item{padding:8px 10px;cursor:pointer;font-size:12px;border-bottom:1px solid #f2f2f2;line-height:1.4}
#search-results .sr-item:last-child{border-bottom:none}
#search-results .sr-item:hover{background:#eaf4fb}
#search-results .sr-name{color:#2c3e50;font-weight:600}
#search-results .sr-detail{color:#888;font-size:10px}

/* ── Legend ── */
#legend{position:absolute;bottom:22px;right:10px;z-index:1000;background:rgba(255,255,255,.92);backdrop-filter:blur(4px);padding:10px 14px;border-radius:7px;border:1px solid #ddd;font-size:12px;line-height:2;box-shadow:0 2px 8px rgba(0,0,0,.1)}
.leg{display:flex;align-items:center;gap:7px}
.leg-line{width:22px;height:3px;border-radius:2px}

/* ── Planning UI locked while navigating (observe only) ── */
.locked{position:relative;pointer-events:none;opacity:.5;filter:grayscale(.7)}
#planning-lock-note{display:none;padding:7px 10px;margin:0 12px 8px;background:#eef2f6;border:1px solid #d6dde4;border-radius:6px;font-size:11px;color:#667;line-height:1.4}
#planning-lock-note.show{display:block}

/* ── Monitoring screen (camera + telemetry) over the map ── */
#monitor{display:none;position:absolute;inset:0;z-index:1200;background:#11151a;flex-direction:column}
#monitor.show{display:flex}
#monitor .cam-wrap{flex:1 1 auto;position:relative;display:flex;align-items:center;justify-content:center;overflow:hidden;background:#000}
#monitor #cam-stream{max-width:100%;max-height:100%;object-fit:contain}
#monitor .cam-off{position:absolute;color:#7a8694;font-size:13px;text-align:center;line-height:1.6}
#mon-dest{position:absolute;top:14px;left:14px;right:14px;display:flex;align-items:center;gap:10px;background:rgba(20,24,30,.82);border:1px solid rgba(230,126,34,.55);border-radius:8px;padding:9px 13px;color:#fff;backdrop-filter:blur(4px)}
#mon-dest .md-icon{font-size:18px}
#mon-dest .md-text{font-size:14px;font-weight:600}
#mon-dest .md-text b{color:#f0922b;font-size:18px}
#mon-hud{display:flex;gap:0;background:#1a2027;border-top:1px solid #2a323c}
#mon-hud .hud-cell{flex:1;padding:10px 12px;text-align:center;border-right:1px solid #2a323c}
#mon-hud .hud-cell:last-child{border-right:none}
#mon-hud .hud-label{font-size:10px;letter-spacing:.6px;text-transform:uppercase;color:#7a8694;margin-bottom:3px}
#mon-hud .hud-val{font-size:17px;font-weight:700;color:#eaf0f6;font-family:monospace}
#mon-hud .hud-val span{font-size:11px;color:#7a8694;font-weight:400}
#mon-toggle{position:absolute;top:12px;right:12px;z-index:1300;background:rgba(20,24,30,.85);color:#fff;border:1px solid #3a434e;border-radius:6px;padding:6px 11px;font-size:12px;font-weight:600;cursor:pointer;display:none}
#mon-toggle.show{display:block}

/* ── Stall alert banner (robot stuck / not moving) ── */
#nav-alert{display:none;position:fixed;top:16px;left:50%;transform:translateX(-50%);z-index:3000;max-width:540px;width:calc(100% - 40px);background:#fff;border:2px solid #c0392b;border-radius:10px;box-shadow:0 8px 30px rgba(0,0,0,.3);overflow:hidden;animation:alert-in .25s ease-out}
#nav-alert.show{display:block}
@keyframes alert-in{from{opacity:0;transform:translate(-50%,-12px)}to{opacity:1;transform:translate(-50%,0)}}
#nav-alert .na-head{display:flex;align-items:center;gap:9px;background:#c0392b;color:#fff;padding:9px 14px;font-weight:700;font-size:14px}
#nav-alert .na-head .na-pulse{width:11px;height:11px;border-radius:50%;background:#fff;animation:alert-blink 1s ease-in-out infinite}
@keyframes alert-blink{0%,100%{opacity:1}50%{opacity:.25}}
#nav-alert .na-body{padding:12px 14px;font-size:13px;color:#444;line-height:1.5}
#nav-alert .na-actions{display:flex;gap:8px;padding:0 14px 13px}
#nav-alert .na-actions button{flex:1;padding:9px 0;border:none;border-radius:6px;font-size:12.5px;font-weight:700;cursor:pointer}
#nav-alert .na-take{background:#16a085;color:#fff}
#nav-alert .na-resume{background:#27ae60;color:#fff}
#nav-alert .na-dismiss{background:#ecf0f1;color:#666;flex:0 0 auto;padding:9px 14px}
</style>
</head>
<body>

<!-- ── Stall alert (robot not moving / stuck) ── -->
<div id="nav-alert">
  <div class="na-head"><span class="na-pulse"></span> Robot may be stuck</div>
  <div class="na-body" id="nav-alert-msg"></div>
  <div class="na-actions">
    <button class="na-take"    onclick="alertTakeControl()">🕹 Take manual control</button>
    <button class="na-resume"  onclick="alertResume()">▶ Resume</button>
    <button class="na-dismiss" onclick="alertDismiss()">Dismiss</button>
  </div>
</div>

<!-- ── Sidebar ── -->
<div id="sidebar">
  <div id="sidebar-header">
    OSM Route Planner
    <small>Click map to add stops &rarr; Generate &rarr; Edit</small>
  </div>

  <!-- ── Search bar ── -->
  <div id="search-wrap" style="position:relative">
    <input id="search-input" type="text" placeholder="Search location…"
           onkeydown="if(event.key==='Enter')searchLocation()"
           oninput="if(!this.value)closeSearch()"/>
    <button id="btn-search" onclick="searchLocation()">Search</button>
    <button id="btn-find-robot" onclick="_goToRobot()" title="Go to robot" style="padding:6px 8px;border:none;border-radius:5px;background:#e74c3c;color:#fff;font-size:13px;cursor:pointer;white-space:nowrap">🤖</button>
    <div id="search-results"></div>
  </div>

  <div id="tabs">
    <button class="tab active" onclick="switchTab('define')" id="tab-define">Define Route</button>
    <button class="tab" onclick="switchTab('draw')" id="tab-draw">Draw Path</button>
    <button class="tab" onclick="switchTab('edit')" id="tab-edit" disabled>Edit Waypoints</button>
  </div>

  <div id="planning-lock-note">🔒 Navigating — route editing is locked. Stop navigation to edit again.</div>

  <div id="sidebar-body">

    <!-- Config (always visible) -->
    <div class="section">
      <div class="sec-title">Config</div>
      <div class="row"><label>Network type</label>
        <select id="cfg-network" onchange="onNetworkChange()"><option value="none">none (straight line)</option><option value="walk">walk</option><option value="bike">bike</option><option value="drive">drive</option><option value="drive_service">drive_service</option><option value="all">all</option><option value="all_private">all_private</option></select>
      </div>
      <div class="row"><label>Max gap (m)</label>
        <input type="number" id="cfg-gap" value="25" step="0.5" min="1"/>
      </div>
      <div class="row"><label>Lateral offset</label>
        <input type="number" id="cfg-offset" value="0" step="0.5"/>
      </div>
      <div class="row"><label>Offset side</label>
        <select id="cfg-side"><option value="right">right</option><option value="left">left</option></select>
      </div>
      <div id="osm-fields">
      <div class="row"><label>Buffer (m)</label>
        <input type="number" id="cfg-buffer" value="200" step="50" min="50"/>
      </div>
      <div class="row"><label>Custom filter</label>
        <input type="text" id="cfg-filter" placeholder='["highway"~"raceway"]'/>
      </div>
      </div>
    </div>

    <!-- Define panel -->
    <div id="panel-define">
      <div class="section">
        <div class="sec-title">Route Stops &nbsp;<span id="stop-count" style="color:#2980b9">0</span></div>
        <div class="hint">Click the map to place stops in order. Drag to reposition.</div>
        <ul id="stop-list"></ul>
        <div class="btn-row" style="margin-top:6px">
          <button class="btn btn-gray" onclick="clearStops()">Clear All</button>
          <button class="btn btn-gray" onclick="reloadConfigStops()" title="Reset stops to coordinates in osm_route_config.yaml">↺ From Config</button>
        </div>
      </div>
    </div>

    <!-- Draw panel -->
    <div id="panel-draw" style="display:none">
      <div class="section">
        <div class="sec-title">Free-Draw Path &nbsp;<span id="draw-count" style="color:#8e44ad">0</span> pts</div>
        <div class="hint">Click the map to place waypoints in sequence.<br>Right-click to undo the last point.</div>
        <div class="btn-row" style="margin-top:10px">
          <button class="btn btn-gray" onclick="clearDraw()">Clear</button>
          <button class="btn" style="background:#8e44ad;color:#fff" onclick="finishDraw()">&#10003; Use as Route</button>
        </div>
      </div>
    </div>

    <!-- Edit panel -->
    <div id="panel-edit" style="display:none">
      <div class="section">
        <div class="sec-title">Generated Waypoints</div>
        <div id="wp-info">No waypoints yet.</div>
        <div class="hint">Drag <b style="color:#2980b9">●</b> to move &nbsp;·&nbsp; Right-click to remove &nbsp;·&nbsp; Click <span style="color:#2980b9">route line</span> to add</div>
      </div>
      <div class="section">
        <div class="sec-title">Stop Points &nbsp;<span id="stop-pts-count" style="color:#c0392b">0</span></div>
        <div class="hint">Robot waits at each ◆ until it receives a confirmation message.</div>
        <div class="row"><label>Interval (m)</label>
          <input type="number" id="stop-interval" value="50" min="5" step="5"/>
        </div>
        <div class="btn-row" style="margin-bottom:8px">
          <button class="btn btn-gray" onclick="autoGenStops()">↺ Auto-generate</button>
          <button class="btn" id="btn-add-stop" onclick="toggleStopMode()" style="background:#8e44ad;color:#fff">+ Add Stop</button>
        </div>
        <div class="btn-row" style="margin-bottom:10px">
          <button class="btn btn-gray" onclick="clearAllStops()">Clear Stops</button>
          <button class="btn btn-gray" onclick="saveStopsConfig()">💾 Save Config</button>
        </div>
        <div class="sec-title">ROS Topics</div>
        <div class="row"><label>Confirm on</label>
          <input type="text" id="stop-confirm-topic" value="/dcvg/stop_confirm" style="font-family:monospace;font-size:10px"/>
        </div>
        <div class="row"><label>Reached on</label>
          <input type="text" id="stop-reached-topic" value="/dcvg/stop_reached" style="font-family:monospace;font-size:10px"/>
        </div>
        <ul id="stops-list-ui" style="list-style:none;margin-top:8px"></ul>
      </div>
    </div>

  </div><!-- sidebar-body -->

  <div id="sidebar-footer">
    <button class="btn btn-blue" id="btn-gen" onclick="generateRoute()">Generate Route</button>
    <div style="border-top:1px solid #eee;margin:4px 0;padding-top:8px">
      <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:6px">
        <span style="font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.8px;color:#aaa">Navigation</span>
        <span id="nav-state-badge" style="background:#ecf0f1;color:#888">Stopped</span>
      </div>
      <div class="btn-row">
        <button class="btn btn-green"  id="btn-nav-start" onclick="navStart()">▶ Start</button>
        <button class="btn btn-orange" id="btn-nav-pause" onclick="navPause()" disabled>⏸ Pause</button>
        <button class="btn btn-red"    id="btn-nav-stop"  onclick="navStop()"  disabled>⏹ Stop</button>
      </div>
    </div>
    <div id="nav-target-card">
      <div class="tc-label">🎯 Heading to</div>
      <div class="tc-body" id="nav-target-info">—</div>
    </div>

    <div id="teleop-card">
      <div class="tp-head">
        <span class="tp-title">🕹 Manual Control</span>
        <button id="btn-teleop" onclick="toggleTeleop()">Take Over</button>
      </div>
      <div id="teleop-locked">Navigation has control. Take over to drive the robot manually — this pauses the route first.</div>
      <div id="teleop-pad">
        <div class="tp-grid">
          <div class="tp-empty"></div>
          <button data-dir="fwd"  title="Forward (W / ↑)">▲</button>
          <div class="tp-empty"></div>
          <button data-dir="left" title="Turn left (A / ←)">◀</button>
          <button class="tp-stop" data-dir="stop" title="Stop (Space)">STOP</button>
          <button data-dir="right" title="Turn right (D / →)">▶</button>
          <div class="tp-empty"></div>
          <button data-dir="back" title="Reverse (S / ↓)">▼</button>
          <div class="tp-empty"></div>
        </div>
        <div class="tp-speed">
          <label>Speed</label>
          <input type="range" id="tp-lin" min="0.1" max="1.5" step="0.1" value="0.5">
          <span class="tp-val" id="tp-lin-val">0.5 m/s</span>
        </div>
        <div class="tp-speed">
          <label>Turn</label>
          <input type="range" id="tp-ang" min="0.1" max="1.0" step="0.1" value="0.6">
          <span class="tp-val" id="tp-ang-val">0.6 rad/s</span>
        </div>
        <div class="tp-hint">Hold a button or use W A S D / arrows · Space = stop</div>
      </div>
    </div>

    <div id="status"></div>
  </div>
</div><!-- sidebar -->

<!-- ── Map ── -->
<div id="map">
  <div id="legend">
    <div class="leg"><div class="leg-line" style="background:#2980b9"></div> Generated route</div>
    <div class="leg"><div style="width:13px;height:13px;background:#e67e22;border:2px solid #fff;border-radius:50%;box-shadow:0 1px 3px rgba(0,0,0,.3)"></div> Current target</div>
    <div class="leg"><div style="width:11px;height:11px;background:#c0392b;border:2px solid #fff;border-radius:2px;transform:rotate(45deg);box-shadow:0 1px 3px rgba(0,0,0,.3)"></div> Stop point</div>
  </div>

  <button id="mon-toggle" onclick="toggleMonitorView()">🗺 Map view</button>

  <!-- Live monitoring screen — shown while navigating -->
  <div id="monitor">
    <div id="mon-dest">
      <span class="md-icon">🎯</span>
      <span class="md-text" id="mon-dest-text">Waiting for destination…</span>
    </div>
    <div class="cam-wrap">
      <div class="cam-off" id="cam-off">📷 Waiting for camera stream…<br><span style="font-size:11px">(topic <span id="cam-off-topic">/camera/color/image_raw</span>)</span></div>
      <img id="cam-stream" alt="" onload="document.getElementById('cam-off').style.display='none'"/>
    </div>
    <div id="mon-hud">
      <div class="hud-cell">
        <div class="hud-label">Linear speed</div>
        <div class="hud-val" id="hud-lin">0.00 <span>m/s</span></div>
      </div>
      <div class="hud-cell">
        <div class="hud-label">Turn rate</div>
        <div class="hud-val" id="hud-ang">0.00 <span>rad/s</span></div>
      </div>
      <div class="hud-cell">
        <div class="hud-label">Destination</div>
        <div class="hud-val" id="hud-dest">—</div>
      </div>
    </div>
  </div>
</div>


<!-- ── Spinner ── -->
<div id="spinner">
  <div class="spin-box">
    <div class="spinner-ring"></div>
    <p id="spin-msg">Querying OSM, please wait…</p>
  </div>
</div>

<script>
// ═══════════════════════════════════════════════════════════════════
// State
// ═══════════════════════════════════════════════════════════════════
let mode = 'define';
let stops     = [];   // {lat, lon, marker}
let waypoints = [];   // {latitude, longitude, yaw, marker}
let routeLine = null;
let drawPts   = [];   // {lat, lon} raw free-draw points
let drawLine  = null;
let drawDots  = [];
let _robotMarker  = null;
let _passedLine   = null;
let _passedWpIdx  = 0;
let _targetMarker = null;   // pulsing ring over the waypoint the robot is driving to
let _targetWpIdx  = -1;     // its index into waypoints[] (-1 = no active target)
let _navHighlighted = false; // true while nav markers are recoloured (so clear is idempotent)
let _stopPts      = [];   // {index, latitude, longitude, marker}
let _stopMode     = false;
let _userWps      = [];   // user-defined numbered anchors mapped onto the dense route:
                          // {num, lat, lon, wpIdx} — wpIdx = nearest dense waypoint index

// ═══════════════════════════════════════════════════════════════════
// Map
// ═══════════════════════════════════════════════════════════════════
const map = L.map('map', {zoomControl: true, maxZoom: 21}).setView([13.75, 100.5], 13);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '© <a href="https://openstreetmap.org">OpenStreetMap</a>',
  maxZoom: 21,
  maxNativeZoom: 19
}).addTo(map);

map.on('click', e => {
  if (mode === 'define') addStop(e.latlng.lat, e.latlng.lng);
  else if (mode === 'draw') addDrawPoint(e.latlng.lat, e.latlng.lng);
  else if (mode === 'edit' && _stopMode) _addStopAtLatLng(e.latlng.lat, e.latlng.lng);
});

map.on('contextmenu', e => {
  if (mode === 'draw') { L.DomEvent.preventDefault(e); undoDrawPoint(); }
});

// ═══════════════════════════════════════════════════════════════════
// Tabs
// ═══════════════════════════════════════════════════════════════════
function switchTab(t) {
  mode = t;
  document.getElementById('tab-define').classList.toggle('active', t === 'define');
  document.getElementById('tab-draw').classList.toggle('active',   t === 'draw');
  document.getElementById('tab-edit').classList.toggle('active',   t === 'edit');
  document.getElementById('panel-define').style.display = t === 'define' ? '' : 'none';
  document.getElementById('panel-draw').style.display   = t === 'draw'   ? '' : 'none';
  document.getElementById('panel-edit').style.display   = t === 'edit'   ? '' : 'none';
  waypoints.forEach(wp => {
    if (!wp.marker) return;
    if (t === 'edit') wp.marker.addTo(map); else wp.marker.remove();
  });
}

// ═══════════════════════════════════════════════════════════════════
// Stop markers
// ═══════════════════════════════════════════════════════════════════
function stopColor(i) {
  if (i === 0)               return '#27ae60';
  if (i === stops.length - 1) return '#e74c3c';
  return '#f39c12';
}

function makeStopIcon(i, state) {
  const cls = state ? ' ' + state : '';   // '' | 'passed' | 'next'
  return L.divIcon({
    className: '',
    html: `<div class="stop-icon${cls}" style="background:${stopColor(i)}">${i+1}</div>`,
    iconSize: [26,26], iconAnchor: [13,13]
  });
}

function addStop(lat, lon) {
  const stop = {lat, lon, marker: null};
  const idx = stops.length;
  stops.push(stop);

  const m = L.marker([lat, lon], {draggable: true, icon: makeStopIcon(idx),
                                   zIndexOffset: 500}).addTo(map);
  stop.marker = m;

  m.on('dragend', () => {
    const p = m.getLatLng();
    stop.lat = p.lat; stop.lon = p.lng;
    refreshStopList();
  });

  refreshStopList();
  refreshStopColors();
}

function removeStop(i) {
  stops[i].marker.remove();
  stops.splice(i, 1);
  stops.forEach((s,j) => s.marker.setIcon(makeStopIcon(j)));
  refreshStopList();
}

function clearStops() {
  stops.forEach(s => s.marker.remove());
  stops = [];
  refreshStopList();
  waypoints.forEach(wp => wp.marker && wp.marker.remove());
  waypoints = [];
  if (routeLine)  { routeLine.remove();  routeLine  = null; }
  if (_passedLine){ _passedLine.remove(); _passedLine = null; }
  if (_targetMarker){ _targetMarker.remove(); _targetMarker = null; }
  _passedWpIdx = 0;
  _targetWpIdx = -1;
  document.getElementById('nav-target-card').style.display = 'none';
  clearAllStops();
  document.getElementById('tab-edit').disabled = true;
  document.getElementById('wp-info').innerHTML = 'No waypoints yet.';
}

async function reloadConfigStops() {
  try {
    const r = await fetch('/api/stops');
    const d = await r.json();
    if (!d.stops || d.stops.length < 2) { status('No stops found in osm_route_config.yaml', 'err'); return; }
    clearStops();
    d.stops.forEach(s => addStop(s.latitude, s.longitude));
    const bounds = L.latLngBounds(stops.map(s => [s.lat, s.lon]));
    map.fitBounds(bounds, {padding:[80,80], maxZoom:17});
    status(`Loaded ${d.stops.length} stops from config`, 'ok');
  } catch(e) {
    status('✗ ' + e.message, 'err');
  }
}

function refreshStopColors() {
  stops.forEach((s,i) => s.marker.setIcon(makeStopIcon(i)));
}

function refreshStopList() {
  document.getElementById('stop-count').textContent = stops.length;
  const ul = document.getElementById('stop-list');
  ul.innerHTML = '';
  stops.forEach((s, i) => {
    const color = i === 0 ? '#27ae60' : (i === stops.length-1 ? '#e74c3c' : '#f39c12');
    const li = document.createElement('li');
    li.className = 'stop-item';
    li.innerHTML =
      `<div class="badge" style="background:${color}">${i+1}</div>` +
      `<div class="stop-coords">${s.lat.toFixed(8)}<br>${s.lon.toFixed(8)}</div>` +
      `<span class="stop-rm" onclick="removeStop(${i})" title="Remove">×</span>`;
    ul.appendChild(li);
  });
}

// ═══════════════════════════════════════════════════════════════════
// Free-draw mode
// ═══════════════════════════════════════════════════════════════════
function addDrawPoint(lat, lon) {
  drawPts.push({lat, lon});
  const dot = L.circleMarker([lat, lon], {radius:6, color:'#8e44ad',
    fillColor:'#8e44ad', fillOpacity:.85, interactive:false}).addTo(map);
  drawDots.push(dot);
  refreshDrawLine();
  document.getElementById('draw-count').textContent = drawPts.length;
}

function undoDrawPoint() {
  if (!drawPts.length) return;
  drawPts.pop();
  const dot = drawDots.pop();
  if (dot) dot.remove();
  refreshDrawLine();
  document.getElementById('draw-count').textContent = drawPts.length;
}

function clearDraw() {
  drawPts = [];
  drawDots.forEach(d => d.remove()); drawDots = [];
  if (drawLine) { drawLine.remove(); drawLine = null; }
  document.getElementById('draw-count').textContent = 0;
}

function refreshDrawLine() {
  if (drawLine) drawLine.remove();
  if (drawPts.length < 2) { drawLine = null; return; }
  drawLine = L.polyline(drawPts.map(p => [p.lat, p.lon]),
    {color:'#8e44ad', weight:3, opacity:.9}).addTo(map);
}

function finishDraw() {
  if (drawPts.length < 2) { status('Draw at least 2 points first.', 'err'); return; }
  const wps = drawPts.map((p, i) => {
    let yaw = 0;
    if (i < drawPts.length - 1)
      yaw = bearingRad(p.lat, p.lon, drawPts[i+1].lat, drawPts[i+1].lon);
    else
      yaw = bearingRad(drawPts[i-1].lat, drawPts[i-1].lon, p.lat, p.lon);
    return {latitude: p.lat, longitude: p.lon, yaw};
  });
  clearDraw();
  loadWaypoints(wps);
  document.getElementById('tab-edit').disabled = false;
  switchTab('edit');
  fetch('/api/save', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({waypoints: wps.map(wp => ({latitude: wp.latitude, longitude: wp.longitude, yaw: wp.yaw ?? 0.0}))})
  }).then(r => r.json()).then(d => {
    status(`✓ ${wps.length} drawn waypoints saved — drag to adjust`, 'ok');
  }).catch(() => {
    status(`✓ ${wps.length} drawn waypoints — drag to adjust`, 'ok');
  });
}

// ═══════════════════════════════════════════════════════════════════
// Route generation
// ═══════════════════════════════════════════════════════════════════
async function generateRoute() {
  if (stops.length < 2) { status('Add at least 2 stops first.', 'err'); return; }

  const isNone = document.getElementById('cfg-network').value === 'none';
  showSpinner(true, isNone ? 'Building straight-line route…' : 'Querying OSM and building route…');
  status('Generating…', 'info');

  const cfg = {
    network_type:     document.getElementById('cfg-network').value,
    custom_filter:    document.getElementById('cfg-filter').value.trim() || null,
    buffer_m:         parseFloat(document.getElementById('cfg-buffer').value),
    max_gap_m:        parseFloat(document.getElementById('cfg-gap').value),
    lateral_offset_m: parseFloat(document.getElementById('cfg-offset').value),
    offset_side:      document.getElementById('cfg-side').value,
  };

  try {
    const r = await fetch('/api/generate', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({
        stops: stops.map(s => ({latitude: s.lat, longitude: s.lon})),
        config: cfg
      })
    });
    const d = await r.json();
    if (!r.ok) throw new Error(d.error || 'Server error');

    loadWaypoints(d.waypoints);
    const savedMsg = d.saved ? ` · saved` : '';
    status(`✓ ${d.waypoints.length} waypoints · ${d.total_m} m${savedMsg}`, 'ok');
    document.getElementById('tab-edit').disabled = false;
    switchTab('edit');
  } catch(e) {
    status('✗ ' + e.message, 'err');
  } finally {
    showSpinner(false);
  }
}

// ═══════════════════════════════════════════════════════════════════
// Waypoint management
// ═══════════════════════════════════════════════════════════════════
function makeWpIcon() {
  return L.divIcon({
    className: '', html: '<div class="wp-dot"></div>',
    iconSize: [10,10], iconAnchor: [5,5]
  });
}

function loadWaypoints(wps) {
  waypoints.forEach(wp => wp.marker && wp.marker.remove());
  if (routeLine) routeLine.remove();
  if (_passedLine) { _passedLine.remove(); _passedLine = null; }
  _passedWpIdx = 0;
  clearAllStops();

  waypoints = wps.map(wp => ({...wp, marker: null}));

  // Map each user-defined numbered anchor (placed before generation) onto its
  // nearest dense waypoint, so navigation progress can be reported against those
  // few anchors instead of the hundreds of generated points.
  _userWps = stops.map((s, i) => ({
    num: i + 1, lat: s.lat, lon: s.lon, wpIdx: _nearestWpIndex(s.lat, s.lon)
  })).sort((a, b) => a.wpIdx - b.wpIdx);

  // Polyline
  routeLine = L.polyline(waypoints.map(wp => [wp.latitude, wp.longitude]),
                          {color:'#2980b9', weight:3, opacity:.9}).addTo(map);

  // Click polyline → insert waypoint
  routeLine.on('click', e => {
    if (mode !== 'edit') return;
    L.DomEvent.stopPropagation(e);
    const {lat, lng} = e.latlng;
    let bestIdx = 1, bestD = Infinity;
    for (let i = 0; i < waypoints.length - 1; i++) {
      const d = ptSegDist(lat, lng,
        waypoints[i].latitude,   waypoints[i].longitude,
        waypoints[i+1].latitude, waypoints[i+1].longitude);
      if (d < bestD) { bestD = d; bestIdx = i + 1; }
    }
    const nw = {latitude: lat, longitude: lng, yaw: 0.0, marker: null};
    waypoints.splice(bestIdx, 0, nw);
    addWpMarker(bestIdx);
    refreshRouteLine();
    updateWpInfo();
  });

  // Create a draggable marker per waypoint
  waypoints.forEach((_, i) => addWpMarker(i));

  // Hide markers until edit tab is active
  if (mode !== 'edit') waypoints.forEach(wp => wp.marker && wp.marker.remove());

  map.fitBounds(routeLine.getBounds(), {padding:[30,30]});
  updateWpInfo();
}

function addWpMarker(idx) {
  const wp = waypoints[idx];
  const m = L.marker([wp.latitude, wp.longitude],
                      {draggable: true, icon: makeWpIcon(), zIndexOffset: 100});
  if (mode === 'edit') m.addTo(map);

  m.on('dragend', () => {
    const p = m.getLatLng();
    wp.latitude = p.lat; wp.longitude = p.lng;
    refreshRouteLine();
    updateWpInfo();
  });

  // Right-click → remove
  m.on('contextmenu', e => {
    L.DomEvent.stopPropagation(e);
    m.remove();
    const i = waypoints.indexOf(wp);
    if (i !== -1) { waypoints.splice(i, 1); refreshRouteLine(); updateWpInfo(); }
  });

  wp.marker = m;
}

function refreshRouteLine() {
  if (!routeLine) return;
  // In edit mode always show full route so every waypoint is visible/draggable
  if (mode === 'edit') {
    routeLine.setLatLngs(waypoints.map(wp => [wp.latitude, wp.longitude]));
  } else {
    const remaining = waypoints.slice(_passedWpIdx).map(wp => [wp.latitude, wp.longitude]);
    routeLine.setLatLngs(remaining.length >= 2 ? remaining : waypoints.map(wp => [wp.latitude, wp.longitude]));
  }
}

function updateWpInfo() {
  const el = document.getElementById('wp-info');
  if (!waypoints.length) { el.innerHTML = 'No waypoints yet.'; return; }
  let dist = 0;
  for (let i = 0; i < waypoints.length - 1; i++)
    dist += haversineM(waypoints[i].latitude,   waypoints[i].longitude,
                       waypoints[i+1].latitude, waypoints[i+1].longitude);
  el.innerHTML = `<b>${waypoints.length}</b> waypoints &nbsp;·&nbsp; <b>${Math.round(dist)} m</b> total`;
}

// ═══════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════
function status(msg, type) {
  const el = document.getElementById('status');
  el.textContent = msg;
  el.className = 's-' + type;
}

function showSpinner(show, msg) {
  document.getElementById('spinner').classList.toggle('show', show);
  if (msg) document.getElementById('spin-msg').textContent = msg;
}

function haversineM(lat1,lon1,lat2,lon2) {
  const R=6371000, p1=lat1*Math.PI/180, p2=lat2*Math.PI/180;
  const dp=(lat2-lat1)*Math.PI/180, dl=(lon2-lon1)*Math.PI/180;
  const a=Math.sin(dp/2)**2+Math.cos(p1)*Math.cos(p2)*Math.sin(dl/2)**2;
  return 2*R*Math.asin(Math.sqrt(a));
}

function bearingRad(lat1, lon1, lat2, lon2) {
  const la1=lat1*Math.PI/180, la2=lat2*Math.PI/180;
  const dlon=(lon2-lon1)*Math.PI/180;
  const x=Math.sin(dlon)*Math.cos(la2);
  const y=Math.cos(la1)*Math.sin(la2)-Math.sin(la1)*Math.cos(la2)*Math.cos(dlon);
  let yaw=Math.PI/2-Math.atan2(x,y);
  while(yaw>Math.PI) yaw-=2*Math.PI;
  while(yaw<-Math.PI) yaw+=2*Math.PI;
  return yaw;
}

function onNetworkChange() {
  const isNone = document.getElementById('cfg-network').value === 'none';
  document.getElementById('osm-fields').style.display = isNone ? 'none' : '';
}

function ptSegDist(lat,lon,lat1,lon1,lat2,lon2) {
  const dx=lat2-lat1, dy=lon2-lon1;
  if(!dx&&!dy) return haversineM(lat,lon,lat1,lon1);
  const t=Math.max(0,Math.min(1,((lat-lat1)*dx+(lon-lon1)*dy)/(dx*dx+dy*dy)));
  return haversineM(lat,lon,lat1+t*dx,lon1+t*dy);
}

// ═══════════════════════════════════════════════════════════════════
// Location search (Nominatim)
// ═══════════════════════════════════════════════════════════════════
let _searchMarker = null;

function _goToRobot() {
  if (!_robotMarker) {
    status('Robot position not available yet — is the robot running?', 'err');
    return;
  }
  const ll = _robotMarker.getLatLng();
  map.setView(ll, Math.max(map.getZoom(), 18));
  closeSearch();
  document.getElementById('search-input').value = '';
  status(`Robot at ${ll.lat.toFixed(6)}, ${ll.lng.toFixed(6)}`, 'info');
}

const _ROBOT_KEYWORDS = ['robot', 'my robot', 'robot location', 'where is robot', 'robot position'];

async function searchLocation() {
  const q = document.getElementById('search-input').value.trim();
  if (!q) return;

  // Special case: jump to robot position
  if (_ROBOT_KEYWORDS.includes(q.toLowerCase())) {
    _goToRobot();
    return;
  }

  const box = document.getElementById('search-results');
  box.style.display = 'none';
  box.innerHTML = '<div class="sr-item"><span class="sr-name">Searching…</span></div>';
  box.style.display = 'block';

  try {
    const url = `https://nominatim.openstreetmap.org/search?q=${encodeURIComponent(q)}&format=json&limit=5&addressdetails=1`;
    const r = await fetch(url, {headers: {'Accept-Language': 'en'}});
    const results = await r.json();
    if (!results.length) {
      box.innerHTML = '<div class="sr-item"><span class="sr-name">No results found.</span></div>';
      return;
    }
    box.innerHTML = '';
    results.forEach(item => {
      const name    = item.display_name.split(',')[0];
      const detail  = item.display_name.split(',').slice(1, 3).join(',').trim();
      const div = document.createElement('div');
      div.className = 'sr-item';
      div.innerHTML = `<div class="sr-name">${name}</div><div class="sr-detail">${detail}</div>`;
      div.onclick = () => selectSearchResult(item);
      box.appendChild(div);
    });
  } catch(e) {
    box.innerHTML = `<div class="sr-item"><span class="sr-name" style="color:#c0392b">Error: ${e.message}</span></div>`;
  }
}

function selectSearchResult(item) {
  closeSearch();
  const lat = parseFloat(item.lat), lon = parseFloat(item.lon);

  if (_searchMarker) _searchMarker.remove();
  _searchMarker = L.marker([lat, lon], {
    icon: L.divIcon({
      className: '',
      html: '<div style="width:14px;height:14px;border-radius:50%;background:#9b59b6;border:2px solid #fff;box-shadow:0 1px 5px rgba(0,0,0,.4)"></div>',
      iconSize: [14,14], iconAnchor: [7,7]
    })
  }).addTo(map).bindPopup(item.display_name.split(',').slice(0,2).join(',')).openPopup();

  if (item.boundingbox) {
    const bb = item.boundingbox.map(Number);
    map.fitBounds([[bb[0],bb[2]],[bb[1],bb[3]]], {maxZoom: 17, padding: [40,40]});
  } else {
    map.setView([lat, lon], 16);
  }
  document.getElementById('search-input').value = item.display_name.split(',')[0];
}

function closeSearch() {
  const box = document.getElementById('search-results');
  box.style.display = 'none';
  box.innerHTML = '';
}

// Close dropdown when clicking elsewhere
document.addEventListener('click', e => {
  if (!e.target.closest('#search-wrap')) closeSearch();
});

// ═══════════════════════════════════════════════════════════════════
// Robot pose tracking
// ═══════════════════════════════════════════════════════════════════

function updateRobotOnMap(lat, lon, headingRad) {
  // ENU yaw (0=East, CCW+)  →  CSS rotation (0=North, CW+)
  const deg = (90 - headingRad * 180 / Math.PI).toFixed(1);
  // viewBox defines the polygon's coordinate space — always keep it matching the points.
  // Change width/height (and iconSize/iconAnchor) to resize the displayed icon.
  const W = 36, H = 48;
  const html =
    `<div style="transform:rotate(${deg}deg);transform-origin:50% 50%;` +
    `filter:drop-shadow(0 2px 4px rgba(0,0,0,.5))">` +
    `<svg viewBox="0 0 18 24" width="${W}" height="${H}">` +
    `<polygon points="9,2 17,22 9,17 1,22" fill="#e74c3c" stroke="white" stroke-width="2.5" stroke-linejoin="round"/>` +
    `</svg></div>`;
  const icon = L.divIcon({className:'', html, iconSize:[W,H], iconAnchor:[W/2,H/2]});

  if (!_robotMarker) {
    _robotMarker = L.marker([lat,lon], {icon, zIndexOffset:2000, interactive:false}).addTo(map);
  } else {
    _robotMarker.setLatLng([lat,lon]);
    _robotMarker.setIcon(icon);
  }
  // Progress (which waypoint we are driving to) is driven by the follower via
  // _applyNavTarget(), not by GPS-nearest guessing — the arrow only shows position.
}

// ═══════════════════════════════════════════════════════════════════
// Target waypoint highlight — authoritative, fed by route_waypoint_follower
// ═══════════════════════════════════════════════════════════════════

// Render the pulse ring + sidebar card + progress trail for the given waypoint.
function _applyNavTarget(data) {
  if (!data || !data.active || !waypoints.length) { _clearNavTarget(); return; }

  // Authoritative index from the follower; fall back to nearest-by-coords if the
  // app's waypoint list has drifted from the saved waypoints.yaml.
  let idx = (typeof data.index === 'number') ? data.index : -1;
  if ((idx < 0 || idx >= waypoints.length) && data.latitude != null && data.longitude != null) {
    let md = Infinity;
    waypoints.forEach((wp, i) => {
      const d = haversineM(data.latitude, data.longitude, wp.latitude, wp.longitude);
      if (d < md) { md = d; idx = i; }
    });
  }
  if (idx < 0 || idx >= waypoints.length) { _clearNavTarget(); return; }

  _targetWpIdx = idx;
  _navHighlighted = true;

  // Preferred view: track progress against the user-defined numbered anchors
  // (the few points the user placed before generating the dense route).
  if (_userWps.length) { _applyUserNavTarget(idx); return; }

  // Fallback (e.g. free-draw routes with no numbered anchors): highlight the
  // dense target waypoint directly.
  const wp = waypoints[idx];
  const ricon = L.divIcon({className:'', html:'<div class="wp-target"></div>',
                           iconSize:[20,20], iconAnchor:[10,10]});
  if (!_targetMarker) {
    _targetMarker = L.marker([wp.latitude, wp.longitude],
      {icon:ricon, zIndexOffset:1500, interactive:false}).addTo(map);
  } else {
    _targetMarker.setLatLng([wp.latitude, wp.longitude]);
    _targetMarker.setIcon(ricon);
  }

  const seq   = (data.seq   != null) ? data.seq   : (idx + 1);
  const total = (data.total != null) ? data.total : waypoints.length;
  document.getElementById('nav-target-card').style.display = '';
  document.getElementById('nav-target-info').innerHTML =
    `Waypoint <b>${seq}</b> / ${total}`;
  _setMonitorDest(seq, total);

  _renderProgress(idx);
}

// Update the destination message + HUD cell on the monitoring screen.
function _setMonitorDest(num, total) {
  const el = document.getElementById('mon-dest-text');
  if (el) el.innerHTML = `Heading to <b>Waypoint ${num}</b> of ${total}`;
  const hud = document.getElementById('hud-dest');
  if (hud) hud.innerHTML = `WP ${num} <span>/ ${total}</span>`;
}

// Highlight only the user-defined numbered anchors. The robot's current dense
// target (idx) maps to "the next anchor at or beyond it" — that anchor pulses,
// earlier anchors grey out, the rest stay normal.
function _applyUserNavTarget(idx) {
  let nd = _userWps.findIndex(u => u.wpIdx >= idx);
  if (nd < 0) nd = _userWps.length - 1;   // past the last anchor → final destination
  const dest = _userWps[nd];

  // The pulsing numbered marker is the destination cue — no separate ring needed.
  if (_targetMarker) { _targetMarker.remove(); _targetMarker = null; }

  document.getElementById('nav-target-card').style.display = '';
  document.getElementById('nav-target-info').innerHTML =
    `Waypoint <b>${dest.num}</b> / ${_userWps.length}`;
  _setMonitorDest(dest.num, _userWps.length);

  // Colour the numbered markers: passed = grey, next = pulsing, upcoming = normal.
  _userWps.forEach((u, j) => {
    const s = stops[u.num - 1];
    if (!s || !s.marker) return;
    const state = j < nd ? 'passed' : (j === nd ? 'next' : '');
    s.marker.setIcon(makeStopIcon(u.num - 1, state));
  });

  _renderPassedTrail(idx);
}

// Dashed grey trail along the dense route from the start up to the robot.
function _renderPassedTrail(targetIdx) {
  _passedWpIdx = targetIdx;
  if (_passedLine) { _passedLine.remove(); _passedLine = null; }
  if (targetIdx > 0) {
    const passed = waypoints.slice(0, targetIdx + 1).map(wp => [wp.latitude, wp.longitude]);
    if (passed.length >= 2)
      _passedLine = L.polyline(passed, {color:'#aaa', weight:3, opacity:.5, dashArray:'6 6'}).addTo(map);
  }
}

function _clearNavTarget() {
  _targetWpIdx = -1;
  // Idempotent: only touch markers on the running→stopped transition. Resetting
  // icons every poll would rebuild marker DOM and cancel an in-progress drag.
  if (!_navHighlighted) return;
  _navHighlighted = false;
  if (_targetMarker) { _targetMarker.remove(); _targetMarker = null; }
  const card = document.getElementById('nav-target-card');
  if (card) card.style.display = 'none';
  if (_passedLine) { _passedLine.remove(); _passedLine = null; }
  _passedWpIdx = 0;
  // restore default blue dots
  waypoints.forEach(wp => { if (wp.marker) wp.marker.setIcon(makeWpIcon()); });
  // restore numbered anchors to their normal look
  stops.forEach((s, i) => { if (s.marker) s.marker.setIcon(makeStopIcon(i)); });
}

// Nearest dense-waypoint index to a lat/lon (used to anchor numbered points).
function _nearestWpIndex(lat, lon) {
  let best = 0, bd = Infinity;
  waypoints.forEach((wp, i) => {
    const d = haversineM(lat, lon, wp.latitude, wp.longitude);
    if (d < bd) { bd = d; best = i; }
  });
  return best;
}

// Dashed trail for passed waypoints + recolour: passed=grey, target=orange, upcoming=blue
function _renderProgress(targetIdx) {
  _passedWpIdx = targetIdx;

  if (_passedLine) { _passedLine.remove(); _passedLine = null; }
  if (targetIdx > 0 && mode !== 'edit') {
    const passed = waypoints.slice(0, targetIdx + 1).map(wp => [wp.latitude, wp.longitude]);
    if (passed.length >= 2)
      _passedLine = L.polyline(passed, {color:'#aaa', weight:2, opacity:.45, dashArray:'5 5'}).addTo(map);
  }

  waypoints.forEach((wp, i) => {
    if (!wp.marker) return;
    let bg, border, sz, anchor;
    if (i < targetIdx)        { bg='#aaa';    border='#ccc'; sz=7;  anchor=3.5; }
    else if (i === targetIdx) { bg='#e67e22'; border='#fff'; sz=12; anchor=6;   }
    else                      { bg='#2980b9'; border='#fff'; sz=10; anchor=5;   }
    wp.marker.setIcon(L.divIcon({
      className:'',
      html:`<div style="width:${sz}px;height:${sz}px;border-radius:50%;background:${bg};` +
           `border:2px solid ${border};box-shadow:0 1px 4px rgba(0,0,0,.35);cursor:grab"></div>`,
      iconSize:[sz,sz], iconAnchor:[anchor,anchor]
    }));
  });
}

// Poll robot position every 500 ms
setInterval(async () => {
  try {
    const r = await fetch('/api/robot_pose');
    const d = await r.json();
    if (d.available) updateRobotOnMap(d.lat, d.lon, d.heading);
    if (typeof d.lin === 'number') {
      document.getElementById('hud-lin').innerHTML = d.lin.toFixed(2) + ' <span>m/s</span>';
      document.getElementById('hud-ang').innerHTML = d.ang.toFixed(2) + ' <span>rad/s</span>';
    }
  } catch {}
}, 500);

// Poll the follower's current target every 700 ms
setInterval(async () => {
  try {
    const r = await fetch('/api/nav_target');
    const d = await r.json();
    _applyNavTarget(d);
  } catch {}
}, 700);

// ═══════════════════════════════════════════════════════════════════
// Stop points
// ═══════════════════════════════════════════════════════════════════

function _stopDiamondHtml(num) {
  return `<div style="position:relative;width:44px;height:44px;display:flex;align-items:center;justify-content:center">` +
    `<div style="width:28px;height:28px;background:#c0392b;border:3px solid #fff;border-radius:5px;transform:rotate(45deg);box-shadow:0 3px 8px rgba(0,0,0,.5)"></div>` +
    `<span style="position:absolute;color:#fff;font-size:13px;font-weight:800;line-height:1">${num}</span></div>`;
}

function _attachStopMarker(sp) {
  const num = _stopPts.indexOf(sp) + 1;
  const m = L.marker([sp.latitude, sp.longitude], {
    icon: L.divIcon({className:'', html:_stopDiamondHtml(num), iconSize:[44,44], iconAnchor:[22,22]}),
    zIndexOffset: 300
  }).addTo(map);
  m.bindTooltip(`Stop ${num} (WP #${sp.index + 1})`, {direction:'top'});
  m.on('contextmenu', e => {
    L.DomEvent.stopPropagation(e);
    m.remove(); sp.marker = null;
    const i = _stopPts.indexOf(sp);
    if (i !== -1) { _stopPts.splice(i, 1); _rebuildStopMarkers(); _refreshStopListUI(); }
  });
  sp.marker = m;
}

function _rebuildStopMarkers() {
  _stopPts.forEach(sp => { if (sp.marker) { sp.marker.remove(); sp.marker = null; } });
  _stopPts.forEach(sp => _attachStopMarker(sp));
}

function _refreshStopListUI() {
  const el = document.getElementById('stop-pts-count');
  if (el) el.textContent = _stopPts.length;
  const ul = document.getElementById('stops-list-ui');
  if (!ul) return;
  ul.innerHTML = '';
  _stopPts.forEach((sp, i) => {
    const li = document.createElement('li');
    li.className = 'stop-item-ui';
    li.innerHTML =
      `<div class="stop-diamond-sm"></div>` +
      `<div style="flex:1;font-size:11px;color:#555;font-family:monospace;line-height:1.5">` +
      `Stop ${i+1} — WP #${sp.index+1}<br>${sp.latitude.toFixed(6)}, ${sp.longitude.toFixed(6)}</div>` +
      `<span style="cursor:pointer;color:#ccc;font-size:18px;line-height:1" onclick="removeStopAt(${i})" title="Remove">×</span>`;
    ul.appendChild(li);
  });
}

function removeStopAt(i) {
  if (_stopPts[i] && _stopPts[i].marker) _stopPts[i].marker.remove();
  _stopPts.splice(i, 1);
  _rebuildStopMarkers();
  _refreshStopListUI();
}

function clearAllStops() {
  _stopPts.forEach(sp => sp.marker && sp.marker.remove());
  _stopPts = [];
  if (_stopMode) toggleStopMode();
  _refreshStopListUI();
}

function _addStopByIndex(idx) {
  if (_stopPts.some(s => s.index === idx)) return;
  const wp = waypoints[idx];
  const sp = {index: idx, latitude: wp.latitude, longitude: wp.longitude, marker: null};
  _stopPts.push(sp);
  _stopPts.sort((a, b) => a.index - b.index);
  _attachStopMarker(sp);
  _refreshStopListUI();
}

function _addStopAtLatLng(lat, lng) {
  if (!waypoints.length) return;
  let minD = Infinity, nearestIdx = -1;
  waypoints.forEach((wp, i) => {
    const d = haversineM(lat, lng, wp.latitude, wp.longitude);
    if (d < minD) { minD = d; nearestIdx = i; }
  });
  if (nearestIdx >= 0) _addStopByIndex(nearestIdx);
}

function toggleStopMode() {
  _stopMode = !_stopMode;
  const btn = document.getElementById('btn-add-stop');
  if (!btn) return;
  if (_stopMode) {
    btn.textContent = '✓ Done';
    btn.classList.add('active');
    map.getContainer().style.cursor = 'crosshair';
  } else {
    btn.textContent = '+ Add Stop';
    btn.classList.remove('active');
    map.getContainer().style.cursor = '';
  }
}

function autoGenStops() {
  if (!waypoints.length) { status('Generate route first', 'err'); return; }
  const interval = parseFloat(document.getElementById('stop-interval').value);
  if (!interval || interval <= 0) { status('Enter a valid interval', 'err'); return; }
  _stopPts.forEach(sp => sp.marker && sp.marker.remove());
  _stopPts = [];
  let cumDist = 0, nextStop = interval;
  for (let i = 1; i < waypoints.length; i++) {
    cumDist += haversineM(waypoints[i-1].latitude, waypoints[i-1].longitude,
                          waypoints[i].latitude,   waypoints[i].longitude);
    if (cumDist >= nextStop) {
      const wp = waypoints[i];
      _stopPts.push({index: i, latitude: wp.latitude, longitude: wp.longitude, marker: null});
      nextStop = cumDist + interval;
    }
  }
  _stopPts.forEach(sp => _attachStopMarker(sp));
  _refreshStopListUI();
  status(`Generated ${_stopPts.length} stops every ${interval} m`, 'ok');
}

async function saveStopsConfig() {
  const body = {
    confirmation_topic: document.getElementById('stop-confirm-topic').value.trim(),
    reached_topic:      document.getElementById('stop-reached-topic').value.trim(),
    stops: _stopPts.map(s => ({index: s.index, latitude: s.latitude, longitude: s.longitude}))
  };
  try {
    const r = await fetch('/api/stops_config', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify(body)
    });
    const d = await r.json();
    if (!r.ok) throw new Error(d.error);
    status(`✓ ${d.count} stop(s) saved`, 'ok');
  } catch(e) { status('✗ ' + e.message, 'err'); }
}

async function loadStopsTopics() {
  try {
    const r = await fetch('/api/stops_config');
    if (!r.ok) return;
    const d = await r.json();
    const ct = document.getElementById('stop-confirm-topic');
    const rt = document.getElementById('stop-reached-topic');
    if (ct && d.confirmation_topic) ct.value = d.confirmation_topic;
    if (rt && d.reached_topic)      rt.value = d.reached_topic;
  } catch {}
}

// ═══════════════════════════════════════════════════════════════════
// Navigation control
// ═══════════════════════════════════════════════════════════════════
let _navState = 'stopped';

function _applyNavState(state) {
  _navState = state;
  const btnStart = document.getElementById('btn-nav-start');
  const btnPause = document.getElementById('btn-nav-pause');
  const btnStop  = document.getElementById('btn-nav-stop');
  const badge    = document.getElementById('nav-state-badge');

  btnStart.textContent = state === 'paused' ? '▶ Resume' : '▶ Start';
  btnStart.disabled    = state === 'running';
  btnPause.disabled    = state !== 'running';
  btnStop.disabled     = state === 'stopped';

  const cfg = {
    stopped: {label:'Stopped',  bg:'#ecf0f1', color:'#888'},
    running: {label:'Navigating', bg:'#d5f5e3', color:'#1e8449'},
    paused:  {label:'Paused',   bg:'#fdebd0', color:'#d35400'},
  }[state] || {label: state, bg:'#ecf0f1', color:'#888'};
  badge.textContent        = cfg.label;
  badge.style.background   = cfg.bg;
  badge.style.color        = cfg.color;

  // Lock the planning UI + show the monitoring screen while navigating.
  const active = (state === 'running' || state === 'paused');
  if (active !== _navActive) {
    _navActive = active;
    _setPlanningLocked(active);
    showMonitor(active);
  }
}

// ── Navigating mode: lock planning, show camera/telemetry screen ──
let _navActive   = false;
let _monitorView = false;   // true = camera screen up, false = map

function _setMarkersDraggable(on) {
  const apply = m => { if (m && m.dragging) { on ? m.dragging.enable() : m.dragging.disable(); } };
  waypoints.forEach(wp => apply(wp.marker));
  stops.forEach(s => apply(s.marker));
  _stopPts.forEach(sp => apply(sp.marker));
}

function _setPlanningLocked(locked) {
  document.getElementById('tabs').classList.toggle('locked', locked);
  document.getElementById('sidebar-body').classList.toggle('locked', locked);
  document.getElementById('planning-lock-note').classList.toggle('show', locked);
  _setMarkersDraggable(!locked);
}

function _setMonitorView(on) {
  _monitorView = on;
  const cam = document.getElementById('cam-stream');
  document.getElementById('monitor').classList.toggle('show', on);
  document.getElementById('mon-toggle').textContent = on ? '🗺 Map view' : '📹 Camera view';
  if (on) {
    if (!cam.getAttribute('src')) cam.src = '/api/camera?t=' + Date.now();
  } else {
    cam.removeAttribute('src');   // close the MJPEG connection
    document.getElementById('cam-off').style.display = '';
  }
}

function toggleMonitorView() { _setMonitorView(!_monitorView); }

function showMonitor(active) {
  document.getElementById('mon-toggle').classList.toggle('show', active);
  _setMonitorView(active);
  if (!active) {
    document.getElementById('mon-dest-text').textContent = 'Waiting for destination…';
    document.getElementById('hud-dest').textContent = '—';
  }
}

async function navStart(confirmFar) {
  if (_teleopOn) await teleopOff();   // hand /cmd_vel back to Nav2 before it drives
  await saveStopsConfig();   // persist latest stop config before launching follower
  try {
    const r = await fetch('/api/nav/start', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({confirm_far: !!confirmFar})
    });
    const d = await r.json();
    if (!r.ok) {
      if (d.error === 'far_start') {
        const km = (d.distance_m / 1000).toFixed(1);
        if (confirm(`This route starts ${km} km from the robot's last known GPS position. Start anyway?`)) {
          return navStart(true);
        }
        status('Start cancelled — route is far from the robot.', 'info');
        return;
      }
      status('✗ ' + (d.error || 'Start failed'), 'err'); return;
    }
    _applyNavState(d.state);
    status('Navigation started', 'ok');
  } catch(e) { status('✗ ' + e.message, 'err'); }
}

async function navPause() {
  try {
    const r = await fetch('/api/nav/pause', {method:'POST'});
    const d = await r.json();
    if (!r.ok) { status('✗ ' + (d.error || 'Pause failed'), 'err'); return; }
    _applyNavState(d.state);
    status('Navigation paused', 'info');
  } catch(e) { status('✗ ' + e.message, 'err'); }
}

async function navStop() {
  try {
    const r = await fetch('/api/nav/stop', {method:'POST'});
    const d = await r.json();
    _applyNavState(d.state);
    status('Navigation stopped', 'info');
  } catch(e) { status('✗ ' + e.message, 'err'); }
}

// Poll nav status every 1.5 s to catch process finishing + stall alerts
setInterval(async () => {
  try {
    const r = await fetch('/api/nav/status');
    const d = await r.json();
    if (d.state !== _navState) _applyNavState(d.state);
    _applyNavAlert(d.alert);
  } catch {}
}, 1500);

// ── Stall alert (server auto-paused because the robot stopped moving) ──
let _shownAlertMsg = null;

function _applyNavAlert(alert) {
  const banner = document.getElementById('nav-alert');
  if (alert && alert.active) {
    if (_shownAlertMsg === alert.message) return;   // already showing this one
    _shownAlertMsg = alert.message;
    document.getElementById('nav-alert-msg').textContent = alert.message;
    banner.classList.add('show');
  } else {
    _shownAlertMsg = null;
    banner.classList.remove('show');
  }
}

function _hideAlert() {
  _shownAlertMsg = null;
  document.getElementById('nav-alert').classList.remove('show');
}

async function alertResume() {
  _hideAlert();
  await navStart();                 // resumes (clears the alert server-side too)
}

async function alertTakeControl() {
  _hideAlert();
  await teleopOn();                 // engage manual driving (also clears the alert)
}

async function alertDismiss() {
  _hideAlert();
  try { await fetch('/api/nav/ack_alert', {method:'POST'}); } catch {}
}

// ═══════════════════════════════════════════════════════════════════
// Manual teleop — over-control when nav is paused/stopped
// ═══════════════════════════════════════════════════════════════════
let _teleopOn    = false;
let _teleopHeld  = null;   // direction currently held, or null
let _teleopTimer = null;   // keep-alive interval that feeds the deadman

const _TP_KEYS = {
  'w':'fwd', 'arrowup':'fwd',  's':'back',  'arrowdown':'back',
  'a':'left','arrowleft':'left','d':'right','arrowright':'right', ' ':'stop',
};

function _tpVel(dir) {
  const lin = parseFloat(document.getElementById('tp-lin').value);
  const ang = parseFloat(document.getElementById('tp-ang').value);
  switch (dir) {
    case 'fwd':   return {lin:  lin, ang:   0};
    case 'back':  return {lin: -lin, ang:   0};
    case 'left':  return {lin:    0, ang:  ang};
    case 'right': return {lin:    0, ang: -ang};
    default:      return {lin:    0, ang:   0};
  }
}

async function _tpSend(lin, ang) {
  try {
    await fetch('/api/teleop', {method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({lin, ang})});
  } catch {}
}

function teleopHold(dir) {
  if (!_teleopOn || dir === 'stop') { if (dir === 'stop') teleopRelease(); return; }
  if (_teleopHeld === dir) return;   // ignore key auto-repeat
  _teleopHeld = dir;
  const send = () => { const v = _tpVel(_teleopHeld); _tpSend(v.lin, v.ang); };
  send();
  if (_teleopTimer) clearInterval(_teleopTimer);
  _teleopTimer = setInterval(send, 150);   // < deadman window, keeps robot moving
}

function teleopRelease() {
  _teleopHeld = null;
  if (_teleopTimer) { clearInterval(_teleopTimer); _teleopTimer = null; }
  if (_teleopOn) _tpSend(0, 0);
}

async function teleopOn() {
  try {
    const r = await fetch('/api/teleop/engage', {method:'POST'});
    const d = await r.json();
    if (!r.ok) { status('✗ ' + (d.error || 'Cannot take over'), 'err'); return; }
    _teleopOn = true;
    if (d.nav_state) _applyNavState(d.nav_state);
    const btn = document.getElementById('btn-teleop');
    btn.textContent = 'Release'; btn.classList.add('on');
    document.getElementById('teleop-pad').classList.add('show');
    document.getElementById('teleop-locked').style.display = 'none';
    status('🕹 Manual control — navigation paused', 'info');
  } catch(e) { status('✗ ' + e.message, 'err'); }
}

async function teleopOff() {
  teleopRelease();
  _teleopOn = false;
  try { await fetch('/api/teleop/disengage', {method:'POST'}); } catch {}
  const btn = document.getElementById('btn-teleop');
  btn.textContent = 'Take Over'; btn.classList.remove('on');
  document.getElementById('teleop-pad').classList.remove('show');
  document.getElementById('teleop-locked').style.display = '';
  status('Manual control released', 'info');
}

function toggleTeleop() { _teleopOn ? teleopOff() : teleopOn(); }

// Wire pad buttons (mouse + touch via pointer events)
document.querySelectorAll('#teleop-pad .tp-grid button').forEach(btn => {
  const dir = btn.dataset.dir;
  btn.addEventListener('pointerdown',   e => { e.preventDefault(); teleopHold(dir); });
  btn.addEventListener('pointerup',     teleopRelease);
  btn.addEventListener('pointerleave',  teleopRelease);
  btn.addEventListener('pointercancel', teleopRelease);
});

// Speed slider labels
document.getElementById('tp-lin').addEventListener('input', e => {
  document.getElementById('tp-lin-val').textContent = (+e.target.value).toFixed(1) + ' m/s';
});
document.getElementById('tp-ang').addEventListener('input', e => {
  document.getElementById('tp-ang-val').textContent = (+e.target.value).toFixed(1) + ' rad/s';
});

// Keyboard driving (only while manual control is engaged)
window.addEventListener('keydown', e => {
  if (!_teleopOn) return;
  const tag = (e.target.tagName || '').toLowerCase();
  if (tag === 'input' || tag === 'textarea') return;
  const dir = _TP_KEYS[e.key.toLowerCase()];
  if (!dir) return;
  e.preventDefault();
  if (dir === 'stop') teleopRelease(); else teleopHold(dir);
});
window.addEventListener('keyup', e => {
  if (!_teleopOn) return;
  const dir = _TP_KEYS[e.key.toLowerCase()];
  if (dir && dir !== 'stop') teleopRelease();
});

// ═══════════════════════════════════════════════════════════════════
// Stop confirmation popup (Leaflet popup on the stop marker)
// ═══════════════════════════════════════════════════════════════════
let _stopPopup = null;

function showStopPopup(msg) {
  if (_stopPopup) return;  // already open
  // Parse "Stop1, lat 13.123456, long 100.234567"
  const m = msg.match(/Stop(\\d+),\\s*lat\\s*([\\d.-]+),\\s*long\\s*([\\d.-]+)/i);
  if (!m) return;
  const [, num, lat, lon] = m;
  const content =
    `<div style="text-align:center;min-width:160px;font-family:system-ui,sans-serif;padding:2px 0">` +
    `<div style="font-weight:700;font-size:13px;color:#c0392b;margin-bottom:4px">🛑 Stop ${num}</div>` +
    `<div style="font-size:11px;color:#888;font-family:monospace;margin-bottom:10px">` +
    `${parseFloat(lat).toFixed(6)}, ${parseFloat(lon).toFixed(6)}</div>` +
    `<button onclick="confirmStop()" style="padding:7px 0;background:#27ae60;color:#fff;` +
    `border:none;border-radius:6px;font-size:12px;font-weight:700;cursor:pointer;width:100%">` +
    `&#10003; Confirm</button></div>`;
  _stopPopup = L.popup({closeButton: false, autoClose: false, closeOnClick: false, maxWidth: 200})
    .setLatLng([parseFloat(lat), parseFloat(lon)])
    .setContent(content)
    .openOn(map);
  status(`Robot waiting at Stop ${num}`, 'info');
}

function hideStopPopup() {
  if (_stopPopup) { map.closePopup(_stopPopup); _stopPopup = null; }
}

async function confirmStop() {
  try {
    await fetch('/api/stop_confirm', {method: 'POST'});
    hideStopPopup();
    status('Stop confirmed — robot continuing', 'ok');
  } catch(e) { status('✗ Confirm failed: ' + e.message, 'err'); }
}

// Poll stop status every 1 s
setInterval(async () => {
  try {
    const r = await fetch('/api/stop_status');
    const d = await r.json();
    if (d.waiting) showStopPopup(d.message);
    else hideStopPopup();
  } catch {}
}, 1000);

// ═══════════════════════════════════════════════════════════════════
// Boot: load config + center on reference
// ═══════════════════════════════════════════════════════════════════
(async () => {
  await loadStopsTopics();

  // Camera topic in use can differ from the default (RGBD vs stereo, or a
  // real-HW override) — reflect whatever camera_topic the node actually resolved.
  try {
    const r = await fetch('/api/camera_topic');
    if (r.ok) {
      const {topic} = await r.json();
      if (topic) document.getElementById('cam-off-topic').textContent = topic;
    }
  } catch {}

  try {
    const r = await fetch('/api/config');
    if (r.ok) {
      const {config: c} = await r.json();
      if (c.network_type) { document.getElementById('cfg-network').value = c.network_type; onNetworkChange(); }
      if (c.max_gap_m != null)        document.getElementById('cfg-gap').value    = c.max_gap_m;
      if (c.lateral_offset_m != null) document.getElementById('cfg-offset').value = c.lateral_offset_m;
      if (c.offset_side)  document.getElementById('cfg-side').value   = c.offset_side;
      if (c.buffer_m != null)         document.getElementById('cfg-buffer').value = c.buffer_m;
      if (c.custom_filter)            document.getElementById('cfg-filter').value = c.custom_filter;
    }
  } catch {}

  // Pre-populate stops from osm_route_config.yaml
  try {
    const r = await fetch('/api/stops');
    if (r.ok) {
      const d = await r.json();
      if (d.stops && d.stops.length >= 2) {
        d.stops.forEach(s => addStop(s.latitude, s.longitude));
        status(`Config stops loaded — click Generate or adjust stops`, 'info');
        const bounds = L.latLngBounds(stops.map(s => [s.lat, s.lon]));
        map.fitBounds(bounds, {padding:[80,80], maxZoom:16});
      }
    }
  } catch {}

  // Fetch robot pose once on load — shows marker immediately if robot is active
  try {
    const r = await fetch('/api/robot_pose');
    const d = await r.json();
    if (d.available) {
      updateRobotOnMap(d.lat, d.lon, d.heading);
      status(`Robot active at ${d.lat.toFixed(6)}, ${d.lon.toFixed(6)}`, 'ok');
    }
  } catch {}

})();
</script>
</body>
</html>"""


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    port = 5000
    args = sys.argv[1:]
    if '--port' in args:
        i = args.index('--port')
        if i + 1 < len(args):
            port = int(args[i + 1])

    print("=" * 52)
    print("  OSM Route Planner Web UI")
    print("=" * 52)
    print(f"  Open in browser:  http://localhost:{port}")
    print("  Press Ctrl+C to stop.")
    print()

    app.run(host='0.0.0.0', port=port, debug=False, use_reloader=False,
            threaded=True)   # threaded: the camera MJPEG stream holds a worker


if __name__ == '__main__':
    main()
