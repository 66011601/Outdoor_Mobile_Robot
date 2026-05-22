"""
osm_route_app.py — Interactive web UI for route planning and waypoint editing.

Usage:
  ros2 run nav2_gps_waypoint_follower_demo osm_route_app
  Then open:  http://localhost:5000
"""

import os
import sys
import traceback
import yaml

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
        share_dir = get_package_share_directory('nav2_gps_waypoint_follower_demo')
        ws_root = os.path.normpath(os.path.join(share_dir, '..', '..', '..', '..'))
        src_dir  = os.path.join(ws_root, 'src',
                                'nav2_gps_waypoint_follower_demo', 'config')
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
        'network_type':     cfg_params.get('network_type', 'walk'),
        'custom_filter':    cfg_params.get('custom_filter') or None,
        'buffer_m':         float(cfg_params.get('buffer_m', 200.0)),
        'max_gap_m':        float(cfg_params.get('max_gap_m', 5.0)),
        'lateral_offset_m': float(cfg_params.get('lateral_offset_m', 0.0)),
        'offset_side':      cfg_params.get('offset_side', 'right'),
    }

    try:
        from nav2_gps_waypoint_follower_demo.osm_route_generator import haversine_distance
        if cfg['network_type'] == 'none':
            from nav2_gps_waypoint_follower_demo.osm_route_generator import generate_straight_route
            wps = generate_straight_route(stops, cfg)
        else:
            from nav2_gps_waypoint_follower_demo.osm_route_generator import generate_route_from_stops
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


@app.route('/api/reference')
def api_reference():
    path = _find_config('demo_waypoints copy.yaml')
    if not path:
        return jsonify({'error': 'Reference file not found', 'waypoints': []}), 404
    with open(path) as f:
        data = yaml.safe_load(f)
    wps = [w for w in (data.get('waypoints') or []) if w]
    return jsonify({'waypoints': wps, 'path': path})


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


@app.route('/api/config')
def api_config():
    path = _find_config('osm_route_config.yaml')
    if not path:
        return jsonify({'config': {}})
    with open(path) as f:
        data = yaml.safe_load(f)
    r = data.get('route', {})
    return jsonify({'config': {
        'network_type':     r.get('network_type', 'walk'),
        'custom_filter':    r.get('custom_filter') or '',
        'buffer_m':         r.get('buffer_m', 200.0),
        'max_gap_m':        r.get('max_gap_m', 5.0),
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

/* ── Status ── */
#status{padding:6px 8px;border-radius:4px;font-size:11px;min-height:22px;line-height:1.4}
.s-ok{background:#eafaf1;color:#1e8449}
.s-err{background:#fdedec;color:#c0392b}
.s-info{background:#eaf4fb;color:#1a5276}

/* ── Map ── */
#map{flex:1}

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
    <small>Click map to add stops &rarr; Generate &rarr; Edit (auto-saves)</small>
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
        <input type="number" id="cfg-gap" value="5" step="0.5" min="1"/>
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
    </div>

  </div><!-- sidebar-body -->

  <div id="sidebar-footer">
    <button class="btn btn-blue" id="btn-gen" onclick="generateRoute()">⚡ Generate Route</button>
    <button class="btn btn-gray" id="btn-ref" onclick="loadReference()">📍 Reference</button>
    <div id="status"></div>
  </div>
</div><!-- sidebar -->

<!-- ── Map ── -->
<div id="map">
  <div id="legend">
    <div class="leg"><div class="leg-line" style="background:#2980b9"></div> Generated route</div>
    <div class="leg"><div class="leg-line" style="background:#e74c3c;border-top:2px dashed #e74c3c;height:0"></div> Reference route</div>
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
let refLine   = null;
let refDots   = [];
let drawPts   = [];   // {lat, lon} raw free-draw points
let drawLine  = null;
let drawDots  = [];

// ═══════════════════════════════════════════════════════════════════
// Map
// ═══════════════════════════════════════════════════════════════════
const map = L.map('map', {zoomControl: true}).setView([13.75, 100.5], 13);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '© <a href="https://openstreetmap.org">OpenStreetMap</a>',
  maxZoom: 21
}).addTo(map);

map.on('click', e => {
  if (mode === 'define') addStop(e.latlng.lat, e.latlng.lng);
  else if (mode === 'draw') addDrawPoint(e.latlng.lat, e.latlng.lng);
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
  saveWaypoints();
  status(`✓ ${wps.length} drawn waypoints — drag to adjust`, 'ok');
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
    const saveNote = d.saved ? ' · saved ✓' : '';
    status(`✓ ${d.waypoints.length} waypoints · ${d.total_m} m${saveNote}`, 'ok');
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
  if (routeLine) routeLine.setLatLngs(waypoints.map(wp => [wp.latitude, wp.longitude]));
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
// Save
// ═══════════════════════════════════════════════════════════════════
async function saveWaypoints() {
  if (!waypoints.length) { status('Nothing to save.', 'err'); return; }
  try {
    const r = await fetch('/api/save', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({waypoints: waypoints.map(wp =>
        ({latitude: wp.latitude, longitude: wp.longitude, yaw: wp.yaw ?? 0.0}))})
    });
    const d = await r.json();
    if (!r.ok) throw new Error(d.error);
    status(`✓ Saved ${d.count} waypoints`, 'ok');
  } catch(e) {
    status('✗ ' + e.message, 'err');
  }
}

// ═══════════════════════════════════════════════════════════════════
// Reference route
// ═══════════════════════════════════════════════════════════════════
async function loadReference() {
  try {
    const r = await fetch('/api/reference');
    const d = await r.json();
    if (!r.ok) throw new Error(d.error || 'Not found');

    if (refLine)  { refLine.remove(); refLine = null; }
    refDots.forEach(m => m.remove()); refDots = [];

    const pts = d.waypoints.map(w => [w.latitude, w.longitude]);
    refLine = L.polyline(pts, {color:'#e74c3c', weight:3, opacity:.85,
                                dashArray:'6 5'}).addTo(map);
    pts.forEach(([lat, lon]) => {
      const m = L.circleMarker([lat, lon], {radius:5, color:'#e74c3c',
        fillColor:'#e74c3c', fillOpacity:.8, interactive:false}).addTo(map);
      refDots.push(m);
    });

    if (pts.length) map.fitBounds(refLine.getBounds(), {padding:[40,40]});
    status(`✓ Reference loaded (${pts.length} pts)`, 'ok');
  } catch(e) {
    status('✗ ' + e.message, 'err');
  }
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
// Boot: load config + center on reference
// ═══════════════════════════════════════════════════════════════════
(async () => {
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

  // Auto-load reference and center map on it
  try {
    const r = await fetch('/api/reference');
    if (r.ok) {
      const d = await r.json();
      const pts = d.waypoints.map(w => [w.latitude, w.longitude]);
      if (pts.length) {
        refLine = L.polyline(pts, {color:'#e74c3c', weight:3, opacity:.85,
                                    dashArray:'6 5'}).addTo(map);
        pts.forEach(([lat,lon]) => {
          const m = L.circleMarker([lat,lon],{radius:5,color:'#e74c3c',
            fillColor:'#e74c3c',fillOpacity:.8,interactive:false}).addTo(map);
          refDots.push(m);
        });
        map.fitBounds(refLine.getBounds(), {padding:[60,60]});
        status(`Reference route loaded (${pts.length} pts) — click map to add stops`, 'ok');
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
