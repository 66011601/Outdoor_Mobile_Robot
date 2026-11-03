import array
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener

NEIGHBOURS = [(dx, dy, math.hypot(dx, dy))
              for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
XYZ_FIELDS = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate('xyz')]
PARAMETERS = {
    'enabled': True,
    'base_frame': 'base_footprint',
    'cell_size': 0.2,
    'min_x': -1.0,
    'max_x': 4.0,
    'half_width': 3.0,
    'min_ground_points': 3,
    'ground_percentile': 0.5,
    'max_slope_deg': 35.0,
    'relax_iterations': 5,
    'fill_cells': 3,
    'grass_height': 0.12,
    'step_height': 0.20,
    'max_ground_age': 0.5,
}


def read_xyz(msg):
    offsets = {field.name: field.offset for field in msg.fields}
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                      'offsets': [offsets['x'], offsets['y'], offsets['z']],
                      'itemsize': msg.point_step})
    points = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
    xyz = np.column_stack([points['x'], points['y'], points['z']])
    return xyz[np.isfinite(xyz).all(axis=1)]


def write_xyz(header, xyz):
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(xyz)
    msg.fields = XYZ_FIELDS
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(xyz)
    msg.is_dense = True
    msg.data = array.array('B', np.ascontiguousarray(xyz, dtype=np.float32).tobytes())
    return msg


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def neighbourhood_min(grid, rise):
    nx, ny = grid.shape
    padded = np.pad(grid, 1, constant_values=np.inf)
    result = grid.copy()
    for dx, dy, distance in NEIGHBOURS:
        np.minimum(result, padded[1 + dx:1 + dx + nx, 1 + dy:1 + dy + ny] + rise * distance,
                   out=result)
    return result


class TerrainFilter(Node):

    def __init__(self):
        super().__init__('terrain_filter')
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in PARAMETERS}
        self.enabled = p['enabled']
        self.base_frame = p['base_frame']
        self.cell_size = p['cell_size']
        self.min_x = p['min_x']
        self.half_width = p['half_width']
        self.nx = int(math.ceil((p['max_x'] - p['min_x']) / self.cell_size))
        self.ny = int(math.ceil(2.0 * p['half_width'] / self.cell_size))
        self.min_ground_points = p['min_ground_points']
        self.ground_percentile = p['ground_percentile']
        self.rise = math.tan(math.radians(p['max_slope_deg'])) * self.cell_size
        self.relax_iterations = p['relax_iterations']
        self.fill_cells = p['fill_cells']
        self.grass_height = p['grass_height']
        self.step_height = p['step_height']
        self.max_ground_age = p['max_ground_age']
        self.ground = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PointCloud2, 'obstacles', 5)
        self.create_subscription(PointCloud2, 'ground', self.on_ground, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'obstacles_raw', self.on_obstacles,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f'enabled={self.enabled} grass_height={self.grass_height} m '
            f'step_height={self.step_height} m max_slope={p["max_slope_deg"]}°')

    def on_ground(self, msg):
        self.ground = msg

    def matching_ground(self, msg):
        if self.ground is None:
            self.get_logger().warning('No ground cloud yet, passing obstacles through',
                                      throttle_duration_sec=5.0)
            return None
        if self.ground.header.frame_id != msg.header.frame_id:
            self.get_logger().warning(
                f'Ground frame {self.ground.header.frame_id} != obstacle frame '
                f'{msg.header.frame_id}, passing obstacles through', throttle_duration_sec=5.0)
            return None
        age = abs((Time.from_msg(msg.header.stamp)
                   - Time.from_msg(self.ground.header.stamp)).nanoseconds) * 1e-9
        if age > self.max_ground_age:
            self.get_logger().warning(f'Ground cloud {age:.2f} s off, passing obstacles through',
                                      throttle_duration_sec=5.0)
            return None
        return read_xyz(self.ground)

    def lookup_to_base(self, frame):
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
        except TransformException as exc:
            self.get_logger().warning(
                f'Waiting for {self.base_frame} → {frame}, passing obstacles through: {exc}',
                throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        return rotation_matrix(tf.transform.rotation), np.array([t.x, t.y, t.z])

    def cell_of(self, xyz):
        ix = np.floor((xyz[:, 0] - self.min_x) / self.cell_size).astype(np.int64)
        iy = np.floor((xyz[:, 1] + self.half_width) / self.cell_size).astype(np.int64)
        inside = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        return np.where(inside, ix * self.ny + iy, -1)

    def reference_heights(self, ground):
        cells = self.cell_of(ground)
        inside = cells >= 0
        cells, z = cells[inside], ground[inside, 2]
        order = np.lexsort((z, cells))
        cells, z = cells[order], z[order]
        counts = np.bincount(cells, minlength=self.nx * self.ny)
        starts = np.cumsum(counts) - counts
        seen = counts >= self.min_ground_points
        surface = np.full(self.nx * self.ny, np.inf)
        surface[seen] = z[starts[seen]
                          + (counts[seen] * self.ground_percentile).astype(np.int64)]
        surface = surface.reshape(self.nx, self.ny)
        seen = seen.reshape(self.nx, self.ny)
        for _ in range(self.relax_iterations):
            surface = neighbourhood_min(surface, self.rise)
        reference = np.where(seen, surface, np.inf)
        for _ in range(self.fill_cells):
            reference = np.where(seen, reference, neighbourhood_min(reference, 0.0))
        reference[np.isinf(reference)] = np.nan
        return reference

    def height_above(self, xyz, reference):
        fx = (xyz[:, 0] - self.min_x) / self.cell_size - 0.5
        fy = (xyz[:, 1] + self.half_width) / self.cell_size - 0.5
        x0 = np.floor(fx).astype(np.int64)
        y0 = np.floor(fy).astype(np.int64)
        wx = fx - x0
        wy = fy - y0
        x0 = np.clip(x0 + 1, 0, self.nx)
        y0 = np.clip(y0 + 1, 0, self.ny)
        padded = np.pad(reference, 1, constant_values=np.nan)
        ground_z = (padded[x0, y0] * (1 - wx) * (1 - wy) + padded[x0 + 1, y0] * wx * (1 - wy)
                    + padded[x0, y0 + 1] * (1 - wx) * wy + padded[x0 + 1, y0 + 1] * wx * wy)
        cells = self.cell_of(xyz)
        nearest = np.where(cells >= 0, reference.ravel()[np.maximum(cells, 0)], np.nan)
        return xyz[:, 2] - np.where(np.isnan(ground_z), nearest, ground_z)

    def on_obstacles(self, msg):
        ground = self.matching_ground(msg) if self.enabled else None
        to_base = self.lookup_to_base(msg.header.frame_id) if ground is not None else None
        if to_base is None:
            self.pub.publish(msg)
            return
        rotation, translation = to_base
        obstacles = read_xyz(msg) @ rotation.T + translation
        ground = ground @ rotation.T + translation
        reference = self.reference_heights(ground)
        obstacle_height = self.height_above(obstacles, reference)
        keep = np.isnan(obstacle_height) | (obstacle_height >= self.grass_height)
        ground_height = self.height_above(ground, reference)
        promote = ~np.isnan(ground_height) & (ground_height >= self.step_height)
        filtered = np.vstack([obstacles[keep], ground[promote]])
        self.pub.publish(write_xyz(msg.header, (filtered - translation) @ rotation))
        self.get_logger().info(
            f'obstacles kept {int(keep.sum())}/{len(obstacles)}, '
            f'ground promoted {int(promote.sum())}/{len(ground)}', throttle_duration_sec=10.0)


def main():
    rclpy.init()
    node = TerrainFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
