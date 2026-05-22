"""
osm_route_generator.py

Queries OpenStreetMap for a pedestrian/sidewalk route between two GPS coordinates
and writes a dense waypoint YAML file compatible with route_waypoint_follower.py.

Usage (standalone):
  python osm_route_generator.py
  python osm_route_generator.py /path/to/osm_route_config.yaml
  python osm_route_generator.py /path/to/osm_route_config.yaml /path/to/output.yaml

Usage (ROS2):
  ros2 run nav2_gps_waypoint_follower_demo osm_route_generator
  ros2 run nav2_gps_waypoint_follower_demo osm_route_generator /path/to/osm_route_config.yaml

Visualization (compare generated route vs reference):
  ros2 run nav2_gps_waypoint_follower_demo osm_route_generator --visualize
  ros2 run nav2_gps_waypoint_follower_demo osm_route_generator --visualize /path/to/reference.yaml
"""

import math
import os
import sys

import yaml

try:
    import osmnx as ox
    import networkx as nx
except ImportError:
    print("ERROR: osmnx is not installed.")
    print("Install it with:  pip install osmnx scikit-learn")
    sys.exit(1)


# ---------------------------------------------------------------------------
# GPS math helpers
# ---------------------------------------------------------------------------

def haversine_distance(lat1, lon1, lat2, lon2):
    """Distance in metres between two GPS coordinates."""
    R = 6_371_000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(a))


def bearing_rad(lat1, lon1, lat2, lon2):
    """Forward bearing in radians (ROS yaw convention: 0=East... no, standard: 0=North)."""
    lat1, lat2 = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    # atan2 gives bearing from North, clockwise positive
    # Convert to ROS yaw: East=0, counter-clockwise positive
    bearing = math.atan2(x, y)          # [-pi, pi], 0=North, CW+
    ros_yaw = math.pi / 2.0 - bearing   # rotate so 0=East, CCW+
    # Normalise to [-pi, pi]
    while ros_yaw > math.pi:
        ros_yaw -= 2 * math.pi
    while ros_yaw < -math.pi:
        ros_yaw += 2 * math.pi
    return ros_yaw


def nav_bearing(lat1, lon1, lat2, lon2):
    """Navigation bearing in radians: 0=North, clockwise positive."""
    lat1, lat2 = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return math.atan2(x, y)


def offset_point(lat, lon, nav_bearing_rad, distance_m):
    """Move a GPS point by distance_m in direction nav_bearing_rad (0=North, CW+)."""
    R = 6_371_000
    d = distance_m / R
    lat_r = math.radians(lat)
    lon_r = math.radians(lon)
    new_lat = math.asin(math.sin(lat_r) * math.cos(d)
                        + math.cos(lat_r) * math.sin(d) * math.cos(nav_bearing_rad))
    new_lon = lon_r + math.atan2(math.sin(nav_bearing_rad) * math.sin(d) * math.cos(lat_r),
                                  math.cos(d) - math.sin(lat_r) * math.sin(new_lat))
    return math.degrees(new_lat), math.degrees(new_lon)


def apply_lateral_offset(waypoints, offset_m, side='right'):
    """
    Shift every waypoint perpendicular to the route by offset_m.
    side='right'  → offset to the right of the direction of travel
    side='left'   → offset to the left
    """
    if offset_m == 0.0:
        return waypoints
    result = []
    for i, wp in enumerate(waypoints):
        if i < len(waypoints) - 1:
            brg = nav_bearing(wp['latitude'], wp['longitude'],
                              waypoints[i + 1]['latitude'], waypoints[i + 1]['longitude'])
        else:
            brg = nav_bearing(waypoints[i - 1]['latitude'], waypoints[i - 1]['longitude'],
                              wp['latitude'], wp['longitude'])
        perp = brg + (math.pi / 2 if side == 'right' else -math.pi / 2)
        new_lat, new_lon = offset_point(wp['latitude'], wp['longitude'], perp, offset_m)
        result.append({'latitude': new_lat, 'longitude': new_lon, 'yaw': wp['yaw']})
    return result


def interpolate_wp(wp1, wp2, fraction):
    """Linearly interpolate between two waypoint dicts."""
    lat = wp1['latitude'] + fraction * (wp2['latitude'] - wp1['latitude'])
    lon = wp1['longitude'] + fraction * (wp2['longitude'] - wp1['longitude'])
    yaw = wp1.get('yaw', 0.0) + fraction * (wp2.get('yaw', 0.0) - wp1.get('yaw', 0.0))
    return {'latitude': lat, 'longitude': lon, 'yaw': yaw}


def densify(waypoints, max_gap_m):
    """Insert intermediate waypoints so no gap exceeds max_gap_m."""
    if len(waypoints) < 2:
        return waypoints
    dense = [waypoints[0]]
    for i in range(1, len(waypoints)):
        p1, p2 = waypoints[i - 1], waypoints[i]
        dist = haversine_distance(p1['latitude'], p1['longitude'],
                                  p2['latitude'], p2['longitude'])
        if dist > max_gap_m:
            n_seg = math.ceil(dist / max_gap_m)
            for j in range(1, n_seg):
                dense.append(interpolate_wp(p1, p2, j / n_seg))
        dense.append(p2)
    return dense


# ---------------------------------------------------------------------------
# OSM query
# ---------------------------------------------------------------------------

def build_graph(start_lat, start_lon, end_lat, end_lon, network_type, buffer_m,
                custom_filter=None):
    """
    Download the OSM graph covering start→end bbox plus buffer.
    osmnx 2.x bbox order: (west, south, east, north).

    custom_filter: Overpass QL filter string, e.g. '["highway"~"raceway|service"]'.
                   When set, network_type is ignored and only matching edges are returned.
    Falls back to 'all' network if requested network_type returns no routable edges.
    """
    north = max(start_lat, end_lat)
    south = min(start_lat, end_lat)
    east  = max(start_lon, end_lon)
    west  = min(start_lon, end_lon)

    deg_buf = buffer_m / 111_000.0
    north += deg_buf
    south -= deg_buf
    east  += deg_buf
    west  -= deg_buf

    bbox = (west, south, east, north)

    if custom_filter:
        print(f"Querying OSM (custom_filter: {custom_filter}) "
              f"bbox: W={west:.5f} S={south:.5f} E={east:.5f} N={north:.5f} ...")
    else:
        print(f"Querying OSM ({network_type} network) "
              f"bbox: W={west:.5f} S={south:.5f} E={east:.5f} N={north:.5f} ...")

    def _query(net_type, filt=None):
        try:
            return ox.graph_from_bbox(bbox, network_type=net_type,
                                      custom_filter=filt, simplify=True)
        except ValueError:
            return None

    if custom_filter:
        G = _query('all', custom_filter)
    else:
        G = _query(network_type)
        if (G is None or len(G.edges) == 0) and network_type != 'all':
            print(f"  No routable '{network_type}' network in this area — retrying with 'all' roads.")
            G = _query('all')

    if G is None or len(G.edges) == 0:
        raise ValueError(
            "OSM returned no routable graph for this bounding box. "
            "Check your coordinates, increase buffer_m, or set custom_filter."
        )

    print(f"Graph: {len(G.nodes)} nodes, {len(G.edges)} edges")
    return G


def _snap_node(G, lat, lon):
    """
    Snap a GPS point to the best OSM node.

    Uses nearest_edges() so we find the closest road *segment* first, then pick
    whichever of that segment's two endpoint nodes is closer to (lat, lon).
    This avoids snapping to a node on a different road that happens to be a few
    metres closer than any node on the intended road, and avoids overshooting the
    stop past the end of a road segment.

    Falls back to nearest_nodes() if nearest_edges() is unavailable.
    """
    try:
        u, v, _ = ox.nearest_edges(G, X=lon, Y=lat)
        du = haversine_distance(lat, lon, G.nodes[u]['y'], G.nodes[u]['x'])
        dv = haversine_distance(lat, lon, G.nodes[v]['y'], G.nodes[v]['x'])
        return u if du <= dv else v
    except Exception:
        return ox.nearest_nodes(G, X=lon, Y=lat)


def extract_route(G, start_lat, start_lon, end_lat, end_lon):
    """
    Find shortest path on the graph and return ordered (lat, lon) list.

    Snaps start/end to the nearest road *segment* endpoint (not just the nearest
    intersection) to avoid wrong-road detours and overrunning the stop.
    Falls back to undirected routing when the graph has one-way constraints
    (e.g. raceway circuits) that block directed routing.
    """
    orig = _snap_node(G, start_lat, start_lon)
    dest = _snap_node(G, end_lat,   end_lon)

    if orig == dest:
        raise ValueError(
            "Start and end resolve to the same OSM node — "
            "points may be too close together or in an area with sparse OSM data."
        )

    try:
        path_nodes = nx.shortest_path(G, orig, dest, weight='length')
    except nx.NetworkXNoPath:
        # One-way graph (e.g. raceway circuit): retry as undirected
        G_u = G.to_undirected()
        try:
            path_nodes = nx.shortest_path(G_u, orig, dest, weight='length')
            print("  Used undirected routing (one-way edges blocked directed path).")
        except nx.NetworkXNoPath:
            raise ValueError(
                f"No path found between OSM nodes {orig} and {dest}. "
                "Try a larger buffer_m or check that OSM has data in this area."
            )

    nodes_gdf, _ = ox.graph_to_gdfs(G)
    coords = []
    for node in path_nodes:
        row = nodes_gdf.loc[node]
        coords.append((float(row['y']), float(row['x'])))  # (lat, lon)

    # Replace the first and last node coords with the exact user-specified GPS
    # points so the route starts/ends precisely at the stop rather than at the
    # nearest OSM node (which can be past the stop or slightly off-road).
    coords[0]  = (start_lat, start_lon)
    coords[-1] = (end_lat,   end_lon)

    return coords


def coords_to_waypoints(coords):
    """
    Convert (lat, lon) list to waypoint dicts with yaw pointing toward next node.
    Last waypoint reuses the second-to-last yaw.
    """
    wps = []
    for i, (lat, lon) in enumerate(coords):
        if i < len(coords) - 1:
            yaw = bearing_rad(lat, lon, coords[i + 1][0], coords[i + 1][1])
        else:
            yaw = wps[-1]['yaw'] if wps else 0.0
        wps.append({'latitude': lat, 'longitude': lon, 'yaw': yaw})
    return wps


# ---------------------------------------------------------------------------
# Multi-stop route generation (used by web app)
# ---------------------------------------------------------------------------

def generate_route_from_stops(stops, cfg):
    """
    Generate a densified route passing through multiple stops via OSM.

    stops: list of {'latitude': float, 'longitude': float}
    cfg:   dict with keys: network_type, custom_filter, buffer_m,
                           max_gap_m, lateral_offset_m, offset_side
    Returns: list of {'latitude', 'longitude', 'yaw'} dicts
    """
    if len(stops) < 2:
        raise ValueError("Need at least 2 stops to generate a route.")

    lats = [float(s['latitude'])  for s in stops]
    lons = [float(s['longitude']) for s in stops]
    net  = cfg.get('network_type', 'walk')
    filt = cfg.get('custom_filter') or None

    G = build_graph(min(lats), min(lons), max(lats), max(lons),
                    net, cfg.get('buffer_m', 200.0), custom_filter=filt)

    all_coords = []
    for i in range(len(stops) - 1):
        s1, s2 = stops[i], stops[i + 1]
        try:
            coords = extract_route(G, s1['latitude'], s1['longitude'],
                                      s2['latitude'], s2['longitude'])
        except ValueError:
            if net != 'all' and not filt:
                print(f"  Segment {i+1} failed on '{net}' — retrying with 'all'.")
                G2 = build_graph(min(lats), min(lons), max(lats), max(lons),
                                 'all', cfg.get('buffer_m', 200.0))
                coords = extract_route(G2, s1['latitude'], s1['longitude'],
                                           s2['latitude'], s2['longitude'])
            else:
                raise

        if i > 0 and all_coords:
            coords = coords[1:]  # don't duplicate the shared via-point
        all_coords.extend(coords)

    raw_wps   = coords_to_waypoints(all_coords)
    dense_wps = densify(raw_wps, cfg.get('max_gap_m', 5.0))

    offset = float(cfg.get('lateral_offset_m', 0.0))
    if offset != 0.0:
        dense_wps = apply_lateral_offset(dense_wps, offset,
                                         cfg.get('offset_side', 'right'))
    return dense_wps


def generate_straight_route(stops, cfg):
    """
    Densified straight-line route between stops — no OSM query.
    For open fields, custom tracks, or environments not in OSM.
    """
    if len(stops) < 2:
        raise ValueError("Need at least 2 stops.")

    all_coords = []
    for i in range(len(stops) - 1):
        s1, s2 = stops[i], stops[i + 1]
        seg = [(float(s1['latitude']), float(s1['longitude'])),
               (float(s2['latitude']), float(s2['longitude']))]
        if i > 0 and all_coords:
            seg = seg[1:]
        all_coords.extend(seg)

    raw_wps   = coords_to_waypoints(all_coords)
    dense_wps = densify(raw_wps, cfg.get('max_gap_m', 5.0))

    offset = float(cfg.get('lateral_offset_m', 0.0))
    if offset != 0.0:
        dense_wps = apply_lateral_offset(dense_wps, offset,
                                         cfg.get('offset_side', 'right'))
    return dense_wps


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

def load_config(config_path):
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    route = cfg['route']
    start = route['start']
    end   = route['end']

    return {
        'start_lat':     float(start['latitude']),
        'start_lon':     float(start['longitude']),
        'end_lat':       float(end['latitude']),
        'end_lon':       float(end['longitude']),
        'network_type':      str(route.get('network_type', 'walk')),
        'custom_filter':     route.get('custom_filter', None),
        'buffer_m':          float(route.get('buffer_m', 200.0)),
        'max_gap_m':         float(route.get('max_gap_m', 5.0)),
        'lateral_offset_m':  float(route.get('lateral_offset_m', 0.0)),
        'offset_side':       str(route.get('offset_side', 'right')),
        'output_file':       str(route.get('output_file', '')),
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    # --- Resolve config path ---
    if len(sys.argv) > 1:
        config_path = sys.argv[1]
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            config_path = os.path.join(
                get_package_share_directory('nav2_gps_waypoint_follower_demo'),
                'config', 'osm_route_config.yaml')
        except Exception:
            config_path = os.path.normpath(
                os.path.join(os.path.dirname(__file__), '..', 'config', 'osm_route_config.yaml'))

    if not os.path.isfile(config_path):
        print(f"ERROR: Config file not found: {config_path}")
        sys.exit(1)

    cfg = load_config(config_path)

    # --- Resolve output path ---
    _src_output = None
    if len(sys.argv) > 2:
        output_path = sys.argv[2]
    elif cfg['output_file']:
        output_path = cfg['output_file']
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('nav2_gps_waypoint_follower_demo')
            output_path = os.path.join(share_dir, 'config', 'demo_waypoints.yaml')
            # colcon install layout: <ws>/install/<pkg>/share/<pkg>  →  4 levels up = <ws>
            ws_root = os.path.normpath(os.path.join(share_dir, '..', '..', '..', '..'))
            _src_output = os.path.join(
                ws_root, 'src', 'nav2_gps_waypoint_follower_demo', 'config', 'demo_waypoints.yaml')
        except Exception:
            output_path = os.path.normpath(
                os.path.join(os.path.dirname(__file__), '..', 'config', 'demo_waypoints.yaml'))

    print("=" * 60)
    print("OSM Route Generator")
    print("=" * 60)
    print(f"Start : ({cfg['start_lat']:.6f}, {cfg['start_lon']:.6f})")
    print(f"End   : ({cfg['end_lat']:.6f}, {cfg['end_lon']:.6f})")
    print(f"Network type : {cfg['network_type']}")
    print(f"Max gap      : {cfg['max_gap_m']} m")
    print(f"Lateral offset: {cfg['lateral_offset_m']} m ({cfg['offset_side']})")
    print(f"Output       : {output_path}")
    print("=" * 60)

    # --- Build graph and extract route ---
    G = build_graph(
        cfg['start_lat'], cfg['start_lon'],
        cfg['end_lat'],   cfg['end_lon'],
        cfg['network_type'], cfg['buffer_m'],
        custom_filter=cfg['custom_filter']
    )

    try:
        coords = extract_route(
            G,
            cfg['start_lat'], cfg['start_lon'],
            cfg['end_lat'],   cfg['end_lon']
        )
    except ValueError as e:
        if cfg['network_type'] != 'all' and not cfg['custom_filter']:
            print(f"  Routing on '{cfg['network_type']}' network failed ({e}) — retrying with 'all' roads.")
            G = build_graph(
                cfg['start_lat'], cfg['start_lon'],
                cfg['end_lat'],   cfg['end_lon'],
                'all', cfg['buffer_m']
            )
            coords = extract_route(
                G,
                cfg['start_lat'], cfg['start_lon'],
                cfg['end_lat'],   cfg['end_lon']
            )
        else:
            raise

    print(f"OSM path: {len(coords)} nodes")

    # --- Convert to waypoints, densify, then apply lateral offset ---
    raw_wps = coords_to_waypoints(coords)
    dense_wps = densify(raw_wps, cfg['max_gap_m'])

    if cfg['lateral_offset_m'] != 0.0:
        dense_wps = apply_lateral_offset(dense_wps, cfg['lateral_offset_m'], cfg['offset_side'])
        print(f"Lateral offset: {cfg['lateral_offset_m']} m to the {cfg['offset_side']}")

    total_dist = sum(
        haversine_distance(
            dense_wps[i]['latitude'],  dense_wps[i]['longitude'],
            dense_wps[i+1]['latitude'], dense_wps[i+1]['longitude']
        )
        for i in range(len(dense_wps) - 1)
    )

    print(f"Waypoints after densification: {len(dense_wps)}")
    print(f"Total route length: {total_dist:.0f} m")

    # --- Write YAML ---
    output = {'waypoints': [
        {
            'latitude':  round(wp['latitude'], 8),
            'longitude': round(wp['longitude'], 8),
            'yaw':       round(wp['yaw'], 6),
        }
        for wp in dense_wps
    ]}

    def _write_yaml(path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, 'w') as f:
            yaml.dump(output, f, default_flow_style=False, sort_keys=False)

    _write_yaml(output_path)
    print(f"\nWaypoints written to: {output_path}")

    # Mirror to source tree so the IDE shows the updated file
    if _src_output and _src_output != output_path:
        if os.path.isdir(os.path.dirname(_src_output)):
            _write_yaml(_src_output)
            print(f"Also mirrored to:     {_src_output}")

    print("Run the follower with:")
    print(f"  ros2 run nav2_gps_waypoint_follower_demo route_waypoint_follower")


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------

# def _load_waypoints_yaml(path):
#     """Load a waypoints YAML and return list of (lat, lon) tuples."""
#     with open(path, 'r') as f:
#         data = yaml.safe_load(f)
#     wps = data.get('waypoints', [])
#     return [(float(w['latitude']), float(w['longitude'])) for w in wps if w]


# def visualize(generated_yaml=None, reference_yaml=None, output_html=None):
#     """
#     Render an interactive HTML map comparing:
#       - Blue line  : generated OSM route (demo_waypoints.yaml)
#       - Red line   : reference / real route (demo_waypoints copy.yaml)

#     Opens the result in the default browser.
#     """
#     try:
#         import folium
#     except ImportError:
#         print("ERROR: folium is not installed.  pip install folium")
#         sys.exit(1)

#     # --- Resolve generated route path ---
#     if generated_yaml is None:
#         try:
#             from ament_index_python.packages import get_package_share_directory
#             share = get_package_share_directory('nav2_gps_waypoint_follower_demo')
#             generated_yaml = os.path.join(share, 'config', 'demo_waypoints.yaml')
#         except Exception:
#             generated_yaml = os.path.normpath(
#                 os.path.join(os.path.dirname(__file__), '..', 'config', 'demo_waypoints.yaml'))

#     # --- Resolve reference route path ---
#     if reference_yaml is None:
#         try:
#             from ament_index_python.packages import get_package_share_directory
#             share = get_package_share_directory('nav2_gps_waypoint_follower_demo')
#             candidate = os.path.join(share, 'config', 'demo_waypoints copy.yaml')
#             # Also check source tree
#             ws_root = os.path.normpath(os.path.join(share, '..', '..', '..', '..'))
#             src_candidate = os.path.join(
#                 ws_root, 'src', 'nav2_gps_waypoint_follower_demo',
#                 'config', 'demo_waypoints copy.yaml')
#             reference_yaml = src_candidate if os.path.isfile(src_candidate) else candidate
#         except Exception:
#             reference_yaml = os.path.normpath(
#                 os.path.join(os.path.dirname(__file__), '..', 'config', 'demo_waypoints copy.yaml'))

#     if output_html is None:
#         output_html = '/tmp/osm_route_comparison.html'

#     print("=" * 60)
#     print("Route Visualizer")
#     print("=" * 60)
#     print(f"Generated route : {generated_yaml}")
#     print(f"Reference route : {reference_yaml}")
#     print(f"Output HTML     : {output_html}")

#     # --- Load waypoints ---
#     generated_pts, reference_pts = [], []

#     if os.path.isfile(generated_yaml):
#         generated_pts = _load_waypoints_yaml(generated_yaml)
#         print(f"Generated route : {len(generated_pts)} waypoints loaded")
#     else:
#         print(f"WARNING: Generated route file not found — {generated_yaml}")
#         print("  Run osm_route_generator first to create it.")

#     if os.path.isfile(reference_yaml):
#         reference_pts = _load_waypoints_yaml(reference_yaml)
#         print(f"Reference route : {len(reference_pts)} waypoints loaded")
#     else:
#         print(f"WARNING: Reference route file not found — {reference_yaml}")

#     if not generated_pts and not reference_pts:
#         print("ERROR: Nothing to visualize.")
#         sys.exit(1)

#     # --- Compute map center ---
#     all_pts = generated_pts + reference_pts
#     center_lat = sum(p[0] for p in all_pts) / len(all_pts)
#     center_lon = sum(p[1] for p in all_pts) / len(all_pts)

#     # --- Build folium map ---
#     m = folium.Map(location=[center_lat, center_lon], zoom_start=17,
#                    tiles='OpenStreetMap')

#     if generated_pts:
#         # Route line
#         folium.PolyLine(
#             generated_pts, color='#2980b9', weight=3, opacity=0.9,
#             tooltip='Generated OSM route'
#         ).add_to(m)
#         # Waypoint dots
#         for i, (lat, lon) in enumerate(generated_pts):
#             folium.CircleMarker(
#                 location=[lat, lon], radius=3,
#                 color='#2980b9', fill=True, fill_color='#2980b9', fill_opacity=0.7,
#                 tooltip=f'Generated #{i+1} ({lat:.6f}, {lon:.6f})'
#             ).add_to(m)
#         # Start / end markers
#         folium.Marker(
#             generated_pts[0], tooltip='Generated START',
#             icon=folium.Icon(color='blue', icon='play', prefix='fa')
#         ).add_to(m)
#         folium.Marker(
#             generated_pts[-1], tooltip='Generated END',
#             icon=folium.Icon(color='blue', icon='stop', prefix='fa')
#         ).add_to(m)

#     if reference_pts:
#         folium.PolyLine(
#             reference_pts, color='#e74c3c', weight=3, opacity=0.9,
#             tooltip='Reference (real) route'
#         ).add_to(m)
#         for i, (lat, lon) in enumerate(reference_pts):
#             folium.CircleMarker(
#                 location=[lat, lon], radius=3,
#                 color='#e74c3c', fill=True, fill_color='#e74c3c', fill_opacity=0.7,
#                 tooltip=f'Reference #{i+1} ({lat:.6f}, {lon:.6f})'
#             ).add_to(m)
#         folium.Marker(
#             reference_pts[0], tooltip='Reference START',
#             icon=folium.Icon(color='red', icon='play', prefix='fa')
#         ).add_to(m)
#         folium.Marker(
#             reference_pts[-1], tooltip='Reference END',
#             icon=folium.Icon(color='red', icon='stop', prefix='fa')
#         ).add_to(m)

#     # --- Legend ---
#     legend_html = """
#     <div style="position:fixed;bottom:30px;left:30px;z-index:1000;
#                 background:white;padding:12px 16px;border-radius:8px;
#                 border:1px solid #ccc;font-size:13px;line-height:1.8;">
#       <b>Route Comparison</b><br>
#       <span style="color:#2980b9;">&#9644;</span> Generated OSM route<br>
#       <span style="color:#e74c3c;">&#9644;</span> Reference (real) route
#     </div>"""
#     m.get_root().html.add_child(folium.Element(legend_html))

#     # --- Auto-fit bounds ---
#     if all_pts:
#         lats = [p[0] for p in all_pts]
#         lons = [p[1] for p in all_pts]
#         m.fit_bounds([[min(lats), min(lons)], [max(lats), max(lons)]])

#     m.save(output_html)

#     # Also save next to the waypoints file so it's visible in VS Code explorer
#     try:
#         from ament_index_python.packages import get_package_share_directory
#         share = get_package_share_directory('nav2_gps_waypoint_follower_demo')
#         ws_root = os.path.normpath(os.path.join(share, '..', '..', '..', '..'))
#         ws_copy = os.path.join(ws_root, 'src', 'nav2_gps_waypoint_follower_demo',
#                                'config', 'osm_route_comparison.html')
#         m.save(ws_copy)
#     except Exception:
#         ws_copy = None

#     # Start a one-shot HTTP server in /tmp so VS Code Remote port-forward works
#     import subprocess, shutil
#     port = 8080
#     subprocess.Popen(
#         ['python3', '-m', 'http.server', str(port)],
#         cwd='/tmp', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
#     )

#     print(f"\nMap saved to: {output_html}")
#     if ws_copy:
#         print(f"Also copied to: {ws_copy}")
#     print()
#     print("Open in browser (VS Code Remote — use forwarded port):")
#     print(f"  http://localhost:{port}/osm_route_comparison.html")
#     print()
#     print("Or open directly if running locally:")
#     print(f"  firefox {output_html}")


# def visualize_main():
#     """Entry point for: ros2 run ... osm_route_generator --visualize [reference.yaml]"""
#     args = [a for a in sys.argv[1:] if a != '--visualize']
#     reference = args[0] if args else None
#     visualize(reference_yaml=reference)


if __name__ == '__main__':
    # if '--visualize' in sys.argv:
    #     visualize_main()
    # else:
    main()
