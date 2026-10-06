import collections
import math
import os
import subprocess
import sys
import threading
import time
import traceback

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from flask import Flask, Response, jsonify, request
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, NavSatFix

from dcvg_robot.common import (NAV_TARGET_FILE, STOP_CONFIRM_FILE, STOP_STATUS_FILE,
                               haversine_distance, read_file, remove_file, write_cmd,
                               write_file)

PORT = int(os.environ.get('DCVG_PORT', '5010'))
MANUAL_SERVICE = 'dcvg.service'
DEFAULT_CAMERA_TOPIC = '/camera/color/image_raw'
BRINGUP_GRACE_S = 30.0
TELEOP_DEADMAN_S = 0.5
STALL_LIN = 0.03
STALL_ANG = 0.05
STALL_TIMEOUT_S = 7.0
STALL_GRACE_S = 6.0
WAYPOINT_SANITY_RADIUS_M = 2000.0

SHARE_DIR = get_package_share_directory('dcvg_robot')
SRC_CONFIG_DIR = os.path.normpath(
    os.path.join(SHARE_DIR, '..', '..', '..', '..', 'src', 'dcvg_robot', 'config'))
with open(os.path.join(SHARE_DIR, 'web', 'index.html')) as f:
    INDEX_HTML = f.read()

app = Flask(__name__)

nav = {'proc': None, 'state': 'stopped'}
bringup = {'proc': None, 'state': 'idle', 'log': collections.deque(maxlen=200), 'started_ts': 0.0}
robot_pose = {'available': False, 'lat': 0.0, 'lon': 0.0, 'heading': 0.0, 'lin': 0.0, 'ang': 0.0}
camera = {'jpeg': None, 'ts': 0.0, 'want_ts': 0.0, 'topic': DEFAULT_CAMERA_TOPIC}
teleop = {'active': False, 'lin': 0.0, 'ang': 0.0, 'ts': 0.0, 'disengage_at': 0.0}
stall = {'since': None, 'notified': False, 'grace_until': 0.0}
nav_alert = {'active': False, 'message': ''}
ros = {'cmd_pub': None}


def config_dirs():
    return [d for d in (SRC_CONFIG_DIR, os.path.join(SHARE_DIR, 'config')) if os.path.isdir(d)]


def find_config(filename):
    for d in config_dirs():
        path = os.path.join(d, filename)
        if os.path.isfile(path):
            return path
    return None


def save_yaml(data, filename):
    saved = []
    for d in config_dirs():
        path = os.path.join(d, filename)
        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        saved.append(path)
    return saved


def waypoints_yaml(wps):
    return {'waypoints': [{'latitude': round(float(w['latitude']), 8),
                           'longitude': round(float(w['longitude']), 8),
                           'yaw': round(float(w.get('yaw', 0.0)), 6)} for w in wps]}


def start_bringup():
    if bringup['proc'] is not None and bringup['proc'].poll() is None:
        return
    bringup['log'].clear()
    proc = subprocess.Popen(['ros2', 'launch', 'dcvg_bringup', 'bringup.launch.py'],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    bringup.update(proc=proc, state='starting', started_ts=time.monotonic())

    def reader():
        for line in proc.stdout:
            bringup['log'].append(line.rstrip('\n'))

    threading.Thread(target=reader, daemon=True).start()


def refresh_bringup_state():
    proc = bringup['proc']
    if proc is None:
        return
    if proc.poll() is None:
        if (bringup['state'] == 'starting'
                and time.monotonic() - bringup['started_ts'] >= BRINGUP_GRACE_S):
            bringup['state'] = 'running'
    else:
        bringup['proc'] = None
        if bringup['state'] != 'stopped':
            bringup['state'] = 'failed'


def stop_bringup():
    proc = bringup['proc']
    bringup['state'] = 'stopped'
    if proc and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
    bringup['proc'] = None


def manual_mode_active():
    return subprocess.run(['systemctl', 'is-active', '--quiet', MANUAL_SERVICE]).returncode == 0


def set_manual_service(action):
    r = subprocess.run(['sudo', '-n', '/usr/bin/systemctl', action, MANUAL_SERVICE],
                       capture_output=True, text=True, timeout=30)
    if r.returncode != 0:
        raise RuntimeError(r.stderr.strip() or f'systemctl {action} {MANUAL_SERVICE} failed')


def refresh_nav_state():
    if nav['proc'] is not None and nav['proc'].poll() is not None:
        nav['proc'] = None
        nav['state'] = 'stopped'


def stop_nav_proc():
    write_cmd('stop')
    proc = nav['proc']
    if proc and proc.poll() is None:
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.terminate()


def clear_nav_alert():
    nav_alert.update(active=False, message='')


def arm_stall_watchdog():
    stall.update(since=None, notified=False, grace_until=time.monotonic() + STALL_GRACE_S)
    clear_nav_alert()


def teleop_tick():
    if not teleop['active']:
        return
    now = time.monotonic()
    msg = Twist()
    if now - teleop['ts'] <= TELEOP_DEADMAN_S:
        msg.linear.x = float(teleop['lin'])
        msg.angular.z = float(teleop['ang'])
    if teleop['disengage_at'] and now >= teleop['disengage_at']:
        msg = Twist()
        teleop.update(active=False, disengage_at=0.0)
    ros['cmd_pub'].publish(msg)


def stall_tick():
    if nav['state'] != 'running':
        stall.update(since=None, notified=False)
        return
    now = time.monotonic()
    if now < stall['grace_until'] or read_file(STOP_STATUS_FILE):
        stall['since'] = None
        return
    if abs(robot_pose['lin']) > STALL_LIN or abs(robot_pose['ang']) > STALL_ANG:
        stall['since'] = None
        return
    if stall['since'] is None:
        stall['since'] = now
    elif now - stall['since'] > STALL_TIMEOUT_S and not stall['notified']:
        write_cmd('pause')
        nav['state'] = 'paused'
        nav_alert.update(active=True, message=(
            f"Robot hasn't moved for {int(STALL_TIMEOUT_S)}s — it may be stuck or blocked. "
            "Navigation auto-paused. Take manual control or resume when clear."))
        stall['notified'] = True
        print('[osm_route_app] STALL detected → auto-paused navigation')


def ros_monitor():
    try:
        rclpy.init(args=sys.argv)
        node = rclpy.create_node('osm_route_monitor')
        node.declare_parameter('camera_topic', DEFAULT_CAMERA_TOPIC)
        camera['topic'] = str(node.get_parameter('camera_topic').value)

        def gps_cb(msg):
            if msg.status.status >= 0:
                robot_pose.update(lat=msg.latitude, lon=msg.longitude, available=True)

        def odom_cb(msg):
            q = msg.pose.pose.orientation
            robot_pose['heading'] = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            robot_pose['lin'] = msg.twist.twist.linear.x
            robot_pose['ang'] = msg.twist.twist.angular.z

        node.create_subscription(NavSatFix, '/gps/fix', gps_cb, 10)
        node.create_subscription(Odometry, '/odometry/local', odom_cb, 10)

        try:
            import cv2
            from cv_bridge import CvBridge
            bridge = CvBridge()

            def image_cb(msg):
                now = time.monotonic()
                if now - camera['want_ts'] > 3.0 or now - camera['ts'] < 0.05:
                    return
                try:
                    ok, jpg = cv2.imencode('.jpg', bridge.imgmsg_to_cv2(msg, 'bgr8'),
                                           [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if ok:
                        camera.update(jpeg=jpg.tobytes(), ts=time.monotonic())
                except Exception:
                    pass

            node.create_subscription(Image, camera['topic'], image_cb, 5)
        except Exception as exc:
            print(f'[osm_route_app] camera stream disabled: {exc}')

        ros['cmd_pub'] = node.create_publisher(Twist, '/cmd_vel', 10)
        node.create_timer(0.1, teleop_tick)
        node.create_timer(0.5, stall_tick)
        rclpy.spin(node)
    except Exception as exc:
        print(f'[osm_route_app] ROS monitor unavailable: {exc}')


def far_start_distance():
    if not robot_pose['available']:
        return None
    path = find_config('waypoints.yaml')
    if not path:
        return None
    try:
        with open(path) as f:
            wps = (yaml.safe_load(f) or {}).get('waypoints', [])
        if not wps:
            return None
        d = haversine_distance(robot_pose['lat'], robot_pose['lon'],
                               float(wps[0]['latitude']), float(wps[0]['longitude']))
        return d if d > WAYPOINT_SANITY_RADIUS_M else None
    except Exception:
        return None


@app.route('/')
def index():
    return INDEX_HTML


@app.route('/api/generate', methods=['POST'])
def api_generate():
    body = request.json or {}
    stops = body.get('stops', [])
    params = body.get('config', {})
    if len(stops) < 2:
        return jsonify({'error': 'Need at least 2 stops'}), 400
    cfg = {
        'network_type': params.get('network_type', 'none'),
        'custom_filter': params.get('custom_filter') or None,
        'buffer_m': float(params.get('buffer_m', 500.0)),
        'max_gap_m': float(params.get('max_gap_m', 25.0)),
        'lateral_offset_m': float(params.get('lateral_offset_m', 0.0)),
        'offset_side': params.get('offset_side', 'right'),
    }
    try:
        from dcvg_robot import osm_route_generator as gen
        if cfg['network_type'] == 'none':
            wps = gen.generate_straight_route(stops, cfg)
        else:
            wps = gen.generate_route_from_stops(stops, cfg)
        total_m = sum(haversine_distance(wps[i]['latitude'], wps[i]['longitude'],
                                             wps[i + 1]['latitude'], wps[i + 1]['longitude'])
                      for i in range(len(wps) - 1))
        saved = save_yaml(waypoints_yaml(wps), 'waypoints.yaml')
        return jsonify({'waypoints': wps, 'total_m': round(total_m, 1),
                        'saved': saved[-1] if saved else None})
    except Exception as exc:
        traceback.print_exc()
        return jsonify({'error': str(exc)}), 500


@app.route('/api/save', methods=['POST'])
def api_save():
    wps = (request.json or {}).get('waypoints', [])
    if not wps:
        return jsonify({'error': 'No waypoints provided'}), 400
    saved = save_yaml(waypoints_yaml(wps), 'waypoints.yaml')
    if not saved:
        return jsonify({'error': 'Could not determine save path'}), 500
    return jsonify({'status': 'ok', 'path': saved[-1], 'count': len(wps)})


@app.route('/api/waypoints')
def api_waypoints():
    path = find_config('waypoints.yaml')
    if not path:
        return jsonify({'waypoints': []})
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except Exception as exc:
        return jsonify({'error': f'Could not read waypoints.yaml: {exc}'}), 500
    wps = []
    for w in data.get('waypoints') or []:
        try:
            wps.append({'latitude': float(w['latitude']), 'longitude': float(w['longitude']),
                        'yaw': float(w.get('yaw') or 0.0)})
        except (KeyError, TypeError, ValueError, AttributeError):
            continue
    return jsonify({'waypoints': wps, 'path': path})


@app.route('/api/stops_config', methods=['POST'])
def api_stops_config():
    data = {'stops': [{'index': int(s['index']),
                       'latitude': round(float(s['latitude']), 8),
                       'longitude': round(float(s['longitude']), 8)}
                      for s in (request.json or {}).get('stops', [])]}
    if not save_yaml(data, 'stops.yaml'):
        return jsonify({'error': 'Could not save'}), 500
    return jsonify({'status': 'ok', 'count': len(data['stops'])})


@app.route('/api/nav/start', methods=['POST'])
def api_nav_start():
    refresh_nav_state()
    body = request.get_json(silent=True) or {}
    if nav['state'] == 'paused':
        write_cmd('run')
        nav['state'] = 'running'
        arm_stall_watchdog()
        return jsonify({'state': 'running'})
    if not body.get('confirm_far'):
        far_m = far_start_distance()
        if far_m is not None:
            return jsonify({'error': 'far_start', 'distance_m': far_m,
                            'message': f"Route start is {far_m / 1000:.1f} km from the "
                                       f"robot's last GPS fix."}), 409
    stop_nav_proc()
    write_cmd('run')
    nav['proc'] = subprocess.Popen(['ros2', 'run', 'dcvg_robot', 'route_waypoint_follower'])
    nav['state'] = 'running'
    arm_stall_watchdog()
    return jsonify({'state': 'running'})


@app.route('/api/nav/pause', methods=['POST'])
def api_nav_pause():
    refresh_nav_state()
    if nav['state'] != 'running':
        return jsonify({'error': 'Not running'}), 400
    write_cmd('pause')
    nav['state'] = 'paused'
    clear_nav_alert()
    return jsonify({'state': 'paused'})


@app.route('/api/nav/stop', methods=['POST'])
def api_nav_stop():
    stop_nav_proc()
    nav.update(proc=None, state='stopped')
    clear_nav_alert()
    remove_file(NAV_TARGET_FILE)
    return jsonify({'state': 'stopped'})


@app.route('/api/nav/status')
def api_nav_status():
    refresh_nav_state()
    return jsonify({'state': nav['state'], 'alert': dict(nav_alert)})


@app.route('/api/nav/ack_alert', methods=['POST'])
def api_nav_ack_alert():
    clear_nav_alert()
    return jsonify({'status': 'ok'})


@app.route('/api/teleop/engage', methods=['POST'])
def api_teleop_engage():
    if ros['cmd_pub'] is None:
        return jsonify({'error': 'cmd_vel publisher not ready (is ROS running?)'}), 503
    refresh_nav_state()
    if nav['state'] == 'running':
        write_cmd('pause')
        nav['state'] = 'paused'
    clear_nav_alert()
    teleop.update(active=True, lin=0.0, ang=0.0, ts=time.monotonic(), disengage_at=0.0)
    return jsonify({'state': 'manual', 'nav_state': nav['state']})


@app.route('/api/teleop', methods=['POST'])
def api_teleop_drive():
    if not teleop['active']:
        return jsonify({'error': 'Manual control not engaged'}), 409
    body = request.json or {}
    teleop.update(lin=float(body.get('lin', 0.0)), ang=float(body.get('ang', 0.0)),
                  ts=time.monotonic())
    return jsonify({'ok': True})


@app.route('/api/teleop/disengage', methods=['POST'])
def api_teleop_disengage():
    now = time.monotonic()
    teleop.update(lin=0.0, ang=0.0, ts=now, disengage_at=now + 0.4)
    return jsonify({'state': 'idle'})


@app.route('/api/robot_pose')
def api_robot_pose():
    return jsonify(robot_pose)


@app.route('/api/camera_topic')
def api_camera_topic():
    return jsonify({'topic': camera['topic']})


@app.route('/api/camera')
def api_camera():
    def gen():
        last_ts = 0.0
        while True:
            camera['want_ts'] = time.monotonic()
            frame, ts = camera['jpeg'], camera['ts']
            if frame and ts != last_ts:
                last_ts = ts
                yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
            time.sleep(0.04)

    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/stop_status')
def api_stop_status():
    msg = read_file(STOP_STATUS_FILE)
    return jsonify({'waiting': bool(msg), 'message': msg})


@app.route('/api/nav_target')
def api_nav_target():
    try:
        with open(NAV_TARGET_FILE) as f:
            return jsonify({'active': True, **(yaml.safe_load(f) or {})})
    except Exception:
        return jsonify({'active': False})


@app.route('/api/stop_confirm', methods=['POST'])
def api_stop_confirm():
    write_file(STOP_CONFIRM_FILE, 'confirm')
    return jsonify({'status': 'ok'})


@app.route('/api/bringup/start', methods=['POST'])
def api_bringup_start():
    if manual_mode_active():
        return jsonify({'error': 'Manual mode is active', 'state': 'failed'}), 409
    refresh_bringup_state()
    if bringup['state'] not in ('starting', 'running'):
        start_bringup()
    return jsonify({'state': bringup['state']})


@app.route('/api/bringup/status')
def api_bringup_status():
    refresh_bringup_state()
    return jsonify({'state': bringup['state'], 'log': '\n'.join(bringup['log']),
                    'grace_s': BRINGUP_GRACE_S})


@app.route('/api/bringup/stop', methods=['POST'])
@app.route('/api/system/sleep', methods=['POST'])
def api_bringup_stop():
    stop_bringup()
    return jsonify({'state': bringup['state']})


@app.route('/api/system/shutdown', methods=['POST'])
def api_system_shutdown():
    stop_bringup()
    try:
        subprocess.Popen(['sudo', '-n', '/usr/sbin/poweroff'])
    except Exception as exc:
        return jsonify({'error': str(exc)}), 500
    return jsonify({'status': 'ok'})


@app.route('/api/mode')
def api_mode_status():
    return jsonify({'manual': manual_mode_active()})


@app.route('/api/mode', methods=['POST'])
def api_mode_set():
    mode = (request.get_json(silent=True) or {}).get('mode')
    try:
        if mode == 'manual':
            teleop.update(active=False, lin=0.0, ang=0.0)
            stop_nav_proc()
            nav.update(proc=None, state='stopped')
            clear_nav_alert()
            remove_file(NAV_TARGET_FILE)
            stop_bringup()
            set_manual_service('start')
        elif mode == 'auto':
            set_manual_service('stop')
        else:
            return jsonify({'error': 'mode must be manual or auto'}), 400
    except Exception as exc:
        return jsonify({'error': str(exc)}), 500
    return jsonify({'manual': manual_mode_active()})


@app.route('/api/wifi_status')
def api_wifi_status():
    try:
        r = subprocess.run(['nmcli', '-t', '-f', 'active,signal,ssid', 'dev', 'wifi'],
                           capture_output=True, text=True, timeout=2)
        for line in r.stdout.splitlines():
            parts = line.split(':', 2)
            if len(parts) == 3 and parts[0] == 'yes':
                return jsonify({'connected': True, 'signal': int(parts[1]),
                                'ssid': parts[2].replace('\\:', ':')})
    except Exception:
        pass
    return jsonify({'connected': False, 'signal': None, 'ssid': None})


def main():
    threading.Thread(target=ros_monitor, daemon=True).start()
    print(f'DCVG Web UI: http://dcvg-robot.local:{PORT}')
    app.run(host='0.0.0.0', port=PORT, debug=False, use_reloader=False, threaded=True)


if __name__ == '__main__':
    main()
