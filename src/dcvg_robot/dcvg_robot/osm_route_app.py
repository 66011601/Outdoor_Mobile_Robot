"""
osm_route_app.py — Interactive web UI for route planning and waypoint editing.

Usage:
  ros2 run dcvg_robot osm_route_app
  Then open:  http://localhost:5000
"""

import math as _math
import os
import subprocess
import sys
import threading
import traceback
import yaml

_NAV_CMD_FILE      = '/tmp/dcvg_nav_cmd'
_STOP_STATUS_FILE  = '/tmp/dcvg_stop_status'
_STOP_CONFIRM_FILE = '/tmp/dcvg_stop_confirm'
_nav_proc = None
_nav_state = 'stopped'  # 'stopped' | 'running' | 'paused'

# ---------------------------------------------------------------------------
# Robot pose monitoring (background ROS2 subscriber thread)
# ---------------------------------------------------------------------------
_robot_pose = {'available': False, 'lat': 0.0, 'lon': 0.0, 'heading': 0.0}


def _start_ros_monitor():
    def _spin():
        try:
            import rclpy
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import NavSatFix
            rclpy.init()
            node = rclpy.create_node('osm_route_monitor')

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

            # Change '/gps/fix' if your GPS topic name differs
            node.create_subscription(NavSatFix, '/gps/fix', gps_cb, 10)
            node.create_subscription(Odometry, '/odometry/local', odom_cb, 10)
            rclpy.spin(node)
        except Exception as exc:
            print(f'[osm_route_app] ROS monitor unavailable: {exc}')

    threading.Thread(target=_spin, daemon=True).start()


_start_ros_monitor()


def _write_nav_cmd(cmd):
    with open(_NAV_CMD_FILE, 'w') as f:
        f.write(cmd)


def _refresh_nav_state():
    global _nav_proc, _nav_state
    if _nav_proc is not None and _nav_proc.poll() is not None:
        _nav_proc = None
        _nav_state = 'stopped'

try:
    from flask import Flask, request, jsonify
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


@app.route('/api/nav/start', methods=['POST'])
def api_nav_start():
    global _nav_proc, _nav_state
    _refresh_nav_state()
    if _nav_state == 'paused':
        _write_nav_cmd('run')
        _nav_state = 'running'
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
    return jsonify({'state': 'running'})


@app.route('/api/nav/pause', methods=['POST'])
def api_nav_pause():
    global _nav_state
    _refresh_nav_state()
    if _nav_state != 'running':
        return jsonify({'error': 'Not running'}), 400
    _write_nav_cmd('pause')
    _nav_state = 'paused'
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
    return jsonify({'state': 'stopped'})


@app.route('/api/nav/status')
def api_nav_status():
    _refresh_nav_state()
    return jsonify({'state': _nav_state})


@app.route('/api/robot_pose')
def api_robot_pose():
    return jsonify(_robot_pose)


@app.route('/api/stop_status')
def api_stop_status():
    try:
        with open(_STOP_STATUS_FILE) as f:
            msg = f.read().strip()
        return jsonify({'waiting': bool(msg), 'message': msg})
    except FileNotFoundError:
        return jsonify({'waiting': False, 'message': ''})


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
</style>
</head>
<body>

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

  <div id="sidebar-body">

    <!-- Config (always visible) -->
    <div class="section">
      <div class="sec-title">Config</div>
      <div class="row"><label>Network type</label>
        <select id="cfg-network" onchange="onNetworkChange()"><option value="walk">walk</option><option value="bike">bike</option><option value="drive">drive</option><option value="drive_service">drive_service</option><option value="all">all</option><option value="all_private">all_private</option><option value="none">none (straight line)</option></select>
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
    <div id="status"></div>
  </div>
</div><!-- sidebar -->

<!-- ── Map ── -->
<div id="map">
  <div id="legend">
    <div class="leg"><div class="leg-line" style="background:#2980b9"></div> Generated route</div>
    <div class="leg"><div style="width:11px;height:11px;background:#c0392b;border:2px solid #fff;border-radius:2px;transform:rotate(45deg);box-shadow:0 1px 3px rgba(0,0,0,.3)"></div> Stop point</div>
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
let _stopPts      = [];   // {index, latitude, longitude, marker}
let _stopMode     = false;

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

function makeStopIcon(i) {
  return L.divIcon({
    className: '',
    html: `<div class="stop-icon" style="background:${stopColor(i)}">${i+1}</div>`,
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
  _passedWpIdx = 0;
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

  if (!waypoints.length) return;

  // Find nearest waypoint to determine progress
  let minDist = Infinity, nearestIdx = 0;
  waypoints.forEach((wp, i) => {
    const d = haversineM(lat, lon, wp.latitude, wp.longitude);
    if (d < minDist) { minDist = d; nearestIdx = i; }
  });
  if (nearestIdx !== _passedWpIdx) refreshWpProgress(nearestIdx);
}

function refreshWpProgress(currentIdx) {
  _passedWpIdx = currentIdx;

  // Trim route line: show only remaining waypoints
  if (routeLine && waypoints.length > 0 && mode !== 'edit') {
    const remaining = waypoints.slice(currentIdx).map(wp => [wp.latitude, wp.longitude]);
    if (remaining.length >= 2) routeLine.setLatLngs(remaining);
  }

  // Faded dashed overlay for the traveled path
  if (_passedLine) { _passedLine.remove(); _passedLine = null; }
  if (currentIdx > 0 && mode !== 'edit') {
    const passed = waypoints.slice(0, currentIdx + 1).map(wp => [wp.latitude, wp.longitude]);
    if (passed.length >= 2)
      _passedLine = L.polyline(passed, {color:'#aaa', weight:2, opacity:.45, dashArray:'5 5'}).addTo(map);
  }

  // Re-colour waypoint markers
  waypoints.forEach((wp, i) => {
    if (!wp.marker) return;
    let bg, border, sz, anchor;
    if (i < currentIdx)       { bg='#aaa';    border='#ccc'; sz=7;  anchor=3.5; }
    else if (i === currentIdx){ bg='#e67e22'; border='#fff'; sz=14; anchor=7;   }
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
  } catch {}
}, 500);

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
}

async function navStart() {
  await saveStopsConfig();   // persist latest stop config before launching follower
  try {
    const r = await fetch('/api/nav/start', {method:'POST'});
    const d = await r.json();
    if (!r.ok) { status('✗ ' + (d.error || 'Start failed'), 'err'); return; }
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

// Poll nav status every 2 s to catch process finishing naturally
setInterval(async () => {
  try {
    const r = await fetch('/api/nav/status');
    const d = await r.json();
    if (d.state !== _navState) _applyNavState(d.state);
  } catch {}
}, 2000);

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

    app.run(host='0.0.0.0', port=port, debug=False, use_reloader=False)


if __name__ == '__main__':
    main()
