import math

import networkx as nx
import osmnx as ox

from dcvg_robot.common import haversine_distance

EARTH_RADIUS_M = 6_371_000


def nav_bearing(lat1, lon1, lat2, lon2):
    lat1, lat2 = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return math.atan2(x, y)


def ros_yaw(lat1, lon1, lat2, lon2):
    yaw = math.pi / 2.0 - nav_bearing(lat1, lon1, lat2, lon2)
    return math.atan2(math.sin(yaw), math.cos(yaw))


def offset_point(lat, lon, bearing, distance_m):
    d = distance_m / EARTH_RADIUS_M
    lat_r, lon_r = math.radians(lat), math.radians(lon)
    new_lat = math.asin(math.sin(lat_r) * math.cos(d)
                        + math.cos(lat_r) * math.sin(d) * math.cos(bearing))
    new_lon = lon_r + math.atan2(math.sin(bearing) * math.sin(d) * math.cos(lat_r),
                                 math.cos(d) - math.sin(lat_r) * math.sin(new_lat))
    return math.degrees(new_lat), math.degrees(new_lon)


def apply_lateral_offset(waypoints, offset_m, side):
    result = []
    for i, wp in enumerate(waypoints):
        a, b = (wp, waypoints[i + 1]) if i < len(waypoints) - 1 else (waypoints[i - 1], wp)
        brg = nav_bearing(a['latitude'], a['longitude'], b['latitude'], b['longitude'])
        perp = brg + (math.pi / 2 if side == 'right' else -math.pi / 2)
        lat, lon = offset_point(wp['latitude'], wp['longitude'], perp, offset_m)
        result.append({'latitude': lat, 'longitude': lon, 'yaw': wp['yaw']})
    return result


def densify(waypoints, max_gap_m):
    dense = waypoints[:1]
    for p1, p2 in zip(waypoints, waypoints[1:]):
        dist = haversine_distance(p1['latitude'], p1['longitude'], p2['latitude'], p2['longitude'])
        n_seg = math.ceil(dist / max_gap_m) if dist > max_gap_m else 1
        for j in range(1, n_seg):
            f = j / n_seg
            dense.append({key: p1[key] + f * (p2[key] - p1[key])
                          for key in ('latitude', 'longitude', 'yaw')})
        dense.append(p2)
    return dense


def coords_to_waypoints(coords):
    wps = []
    for i, (lat, lon) in enumerate(coords):
        if i < len(coords) - 1:
            yaw = ros_yaw(lat, lon, *coords[i + 1])
        else:
            yaw = wps[-1]['yaw'] if wps else 0.0
        wps.append({'latitude': lat, 'longitude': lon, 'yaw': yaw})
    return wps


def finish_route(coords, cfg):
    wps = densify(coords_to_waypoints(coords), cfg.get('max_gap_m', 5.0))
    offset = float(cfg.get('lateral_offset_m', 0.0))
    if offset != 0.0:
        wps = apply_lateral_offset(wps, offset, cfg.get('offset_side', 'right'))
    return wps


def build_graph(lats, lons, network_type, buffer_m, custom_filter=None):
    buf = buffer_m / 111_000.0
    bbox = (min(lons) - buf, min(lats) - buf, max(lons) + buf, max(lats) + buf)

    def query(net_type, filt=None):
        try:
            return ox.graph_from_bbox(bbox, network_type=net_type, custom_filter=filt,
                                      simplify=True)
        except ValueError:
            return None

    if custom_filter:
        graph = query('all', custom_filter)
    else:
        graph = query(network_type)
        if (graph is None or len(graph.edges) == 0) and network_type != 'all':
            graph = query('all')
    if graph is None or len(graph.edges) == 0:
        raise ValueError('OSM returned no routable graph for this bounding box. '
                         'Check your coordinates, increase buffer_m, or set custom_filter.')
    return graph


def snap_node(graph, lat, lon):
    try:
        u, v, _ = ox.nearest_edges(graph, X=lon, Y=lat)
        du = haversine_distance(lat, lon, graph.nodes[u]['y'], graph.nodes[u]['x'])
        dv = haversine_distance(lat, lon, graph.nodes[v]['y'], graph.nodes[v]['x'])
        return u if du <= dv else v
    except Exception:
        return ox.nearest_nodes(graph, X=lon, Y=lat)


def extract_route(graph, start_lat, start_lon, end_lat, end_lon):
    orig = snap_node(graph, start_lat, start_lon)
    dest = snap_node(graph, end_lat, end_lon)
    if orig == dest:
        raise ValueError('Start and end resolve to the same OSM node — points may be too '
                         'close together or in an area with sparse OSM data.')
    try:
        path = nx.shortest_path(graph, orig, dest, weight='length')
    except nx.NetworkXNoPath:
        try:
            path = nx.shortest_path(graph.to_undirected(), orig, dest, weight='length')
        except nx.NetworkXNoPath:
            raise ValueError(f'No path found between OSM nodes {orig} and {dest}. '
                             'Try a larger buffer_m or check that OSM has data in this area.')
    coords = [(float(graph.nodes[n]['y']), float(graph.nodes[n]['x'])) for n in path]
    coords[0] = (start_lat, start_lon)
    coords[-1] = (end_lat, end_lon)
    return coords


def generate_route_from_stops(stops, cfg):
    if len(stops) < 2:
        raise ValueError('Need at least 2 stops to generate a route.')
    lats = [float(s['latitude']) for s in stops]
    lons = [float(s['longitude']) for s in stops]
    net = cfg.get('network_type', 'walk')
    filt = cfg.get('custom_filter') or None
    buffer_m = cfg.get('buffer_m', 200.0)
    graph = build_graph(lats, lons, net, buffer_m, filt)

    coords = []
    for i in range(len(stops) - 1):
        leg = (lats[i], lons[i], lats[i + 1], lons[i + 1])
        try:
            seg = extract_route(graph, *leg)
        except ValueError:
            if net == 'all' or filt:
                raise
            seg = extract_route(build_graph(lats, lons, 'all', buffer_m), *leg)
        coords.extend(seg[1:] if coords else seg)
    return finish_route(coords, cfg)


def generate_straight_route(stops, cfg):
    if len(stops) < 2:
        raise ValueError('Need at least 2 stops.')
    coords = [(float(s['latitude']), float(s['longitude'])) for s in stops]
    return finish_route(coords, cfg)
