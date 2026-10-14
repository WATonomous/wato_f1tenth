#!/usr/bin/env python3
"""Porto static prior + live costmap; all output geometry uses simulator map metres."""
from collections import deque
from pathlib import Path
import math
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from global_planner.msg import ReferenceTrack, TrackWidth
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

# Repository Porto PGM is 0 occupied, 205 unknown, 254 free. Its YAML's .25
# free threshold would incorrectly declare 205 unknown pixels free.
MAP_RESOLUTION = .05
MAP_ORIGIN = np.array([-3.68, -8.47])
MAX_TILT = .04


def load_map(path):
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None or image.shape != (309, 107):
        raise ValueError('Expected the repository Porto map (309 x 107)')
    if not set(np.unique(image)).issubset({0, 205, 254, 255}):
        raise ValueError('Unexpected Porto map pixel encoding')
    cells = np.where(image >= 254, 0, np.where(image == 0, 100, -1)).astype(np.int8)
    contours, hierarchy = cv2.findContours((image >= 254).astype(np.uint8),
                                           cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    selected = sorted(range(len(contours)), key=lambda i: cv2.contourArea(contours[i]),
                      reverse=True)[:2]
    if len(selected) != 2 or hierarchy[0, selected[1], 3] != selected[0]:
        raise ValueError('Porto free space must contain one main annulus')
    loops = []
    for index in selected:
        pixels = contours[index][:, 0, :].astype(float)
        pixels[:, 1] = image.shape[0] - 1 - pixels[:, 1]
        loops.append(MAP_ORIGIN + (pixels + .5) * MAP_RESOLUTION)
    return np.flipud(cells).copy(), loops


def prior_for_grid(cells, width, height, resolution, origin):
    """Conservative supercover of each rotated destination cell in the static prior."""
    if not (0 < width <= 200 and 0 < height <= 200 and .01 <= resolution <= .025001):
        raise ValueError('Unexpected local grid dimensions/resolution')
    x, y = np.meshgrid(np.arange(width) * resolution, np.arange(height) * resolution)
    c, s = math.cos(origin[2]), math.sin(origin[2])
    corners = np.stack([np.stack((origin[0] + c*(x+dx) - s*(y+dy),
                                 origin[1] + s*(x+dx) + c*(y+dy)), axis=-1)
                        for dx, dy in ((0, 0), (resolution, 0),
                                       (0, resolution), (resolution, resolution))])
    low = np.floor((corners.min(axis=0) - MAP_ORIGIN) / MAP_RESOLUTION - 1e-9).astype(int)
    high = np.floor((corners.max(axis=0) - MAP_ORIGIN) / MAP_RESOLUTION + 1e-9).astype(int)
    # Include touching cells. With <= 2.5 cm output cells, the source index span is <= 2.
    if np.any(high - low > 2):
        raise ValueError('Static-prior sampling capacity exceeded')
    occupied = np.zeros((height, width), bool)
    unknown = np.zeros_like(occupied)
    for dy in range(3):
        for dx in range(3):
            ix, iy = low[..., 0]+dx, low[..., 1]+dy
            active = (ix <= high[..., 0]) & (iy <= high[..., 1])
            valid = (ix >= 0) & (iy >= 0) & (ix < cells.shape[1]) & (iy < cells.shape[0])
            values = cells[np.clip(iy, 0, cells.shape[0]-1),
                           np.clip(ix, 0, cells.shape[1]-1)]
            occupied |= active & valid & (values >= 50)
            unknown |= active & (~valid | (values < 0))
    return np.where(occupied, 100, np.where(unknown, -1, 0)).astype(np.int8)


def merge_prior(live, prior):
    result = live.copy()
    result[(result < 0) & (prior == 0)] = 0
    result[prior >= 50] = 100  # Live rays and self filtering cannot erase mapped walls.
    return result


def centered_reference(cells, points):
    """Shift the existing guide toward mapped clearance, then reference the rear axle."""
    p = points[:, :2]
    tangent = np.roll(p, -3, axis=0)-np.roll(p, 3, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    normal = np.c_[-tangent[:, 1], tangent[:, 0]]
    distance = cv2.distanceTransform((cells == 0).astype(np.uint8),
                                    cv2.DIST_L2, cv2.DIST_MASK_PRECISE)*MAP_RESOLUTION
    offsets = np.arange(-.5, .5001, .025)
    candidates = p[:, None, :] + normal[:, None, :]*offsets[None, :, None]
    index = np.floor((candidates-MAP_ORIGIN)/MAP_RESOLUTION).astype(int)
    valid = ((index[..., 0] >= 0) & (index[..., 0] < cells.shape[1]) &
             (index[..., 1] >= 0) & (index[..., 1] < cells.shape[0]))
    score = distance[np.clip(index[..., 1], 0, cells.shape[0]-1),
                     np.clip(index[..., 0], 0, cells.shape[1]-1)]-.04*np.abs(offsets)
    score[~valid] = -np.inf
    offset = offsets[score.argmax(axis=1)]
    offset = sum(np.roll(offset, k) for k in range(-4, 5))/9
    guide = p + normal*offset[:, None]
    # Smooth pixel-scale clearance offsets before taking tangents or fitting the Frenet spline.
    guide = sum(np.roll(guide, k, axis=0) for k in range(-8, 9))/17
    tangent = np.roll(guide, -3, axis=0)-np.roll(guide, 3, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    # shortcut: this map-specific 15 cm center/rear conversion matches autodrive.json;
    # upgrade to shared generated profile data before supporting another vehicle here.
    rear = guide-.15*tangent
    widths = points[:, 2:] + np.c_[offset, -offset]
    if not np.isfinite(rear).all() or np.any(widths <= 0):
        raise ValueError('Invalid centered Porto guidance')
    return np.c_[rear, widths][::4]


def planar_transform(transform):
    q = transform.rotation
    values = np.array([q.x, q.y, q.z, q.w, transform.translation.x,
                       transform.translation.y, transform.translation.z])
    if not np.isfinite(values).all() or abs(values[:4] @ values[:4] - 1) > 1e-3:
        raise ValueError('Invalid observation transform')
    if 1-2*(q.x*q.x+q.y*q.y) < math.cos(MAX_TILT):
        raise ValueError('Observation tilt exceeds planar approximation')
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return transform.translation.x, transform.translation.y, yaw


class PortoScene(Node):
    def __init__(self):
        super().__init__('porto_scene')
        share = Path(get_package_share_directory('mpc_controller'))
        self.cells, loops = load_map(share / 'maps' / 'porto.pgm')
        # The repository centerline stays farther from walls than the optimized line.
        points = np.loadtxt(share / 'maps' / 'my_map_clean.csv', delimiter=',')
        if points.shape[1] != 4 or not np.isfinite(points).all():
            raise ValueError('Invalid Porto reference')
        points = centered_reference(self.cells, points)
        reference = ReferenceTrack()
        reference.header.frame_id = reference.path.header.frame_id = 'map'
        reference.header.stamp = self.get_clock().now().to_msg()
        for x, y, right, left in points:
            reference.path.poses.append(self.pose(x, y, .4))
            # Widths are guidance only; measured loops and original occupied cells define legality.
            reference.widths.append(TrackWidth(right_m=float(right), left_m=float(left)))
        for path, loop in zip(reference.legal_boundaries, loops):
            path.header = reference.header
            path.poses = [self.pose(x, y) for x, y in loop]
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reference_pub = self.create_publisher(ReferenceTrack, '/global_planner/reference_track', qos)
        self.reference_pub.publish(reference)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/costmap', 1)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pending = deque(maxlen=4)
        self.last_stamp = 0
        self.last_warning = 0
        self.create_subscription(OccupancyGrid, '/costmap/lidar', self.receive, 1)
        self.create_timer(.005, self.retry)
        self.get_logger().info(f'Porto reference: {len(points)} points, legal loops '
                               f'{len(loops[0])}/{len(loops[1])}; prior unknown stays unknown')

    def pose(self, x, y, speed=0.):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), float(speed)
        pose.pose.orientation.w = 1.
        return pose

    def receive(self, grid):
        self.retry()
        self.pending.append((grid, time.monotonic()))
        self.retry()

    def retry(self):
        while self.pending:
            grid, receipt = self.pending[0]
            age = (self.get_clock().now()-Time.from_msg(grid.header.stamp)).nanoseconds/1e9
            stamp = Time.from_msg(grid.header.stamp).nanoseconds
            if stamp <= self.last_stamp or age > .15 or age < -.005 or time.monotonic()-receipt > .15:
                self.pending.popleft()
                continue
            try:
                # Wait for the observation-time TF; do not use latest TF or restamp the scan.
                msg = self.tf.lookup_transform('map', grid.header.frame_id, Time.from_msg(grid.header.stamp))
                tx, ty, yaw = planar_transform(msg.transform)
            except TransformException:
                return
            except ValueError as error:
                self.warn(error)
                self.pending.popleft()
                continue
            try:
                if grid.header.frame_id != 'roboracer_1' or len(grid.data) != grid.info.width*grid.info.height:
                    raise ValueError('Invalid live grid')
                o = grid.info.origin
                if not np.isfinite([o.position.x, o.position.y, o.position.z]).all() or o.position.z != 0:
                    raise ValueError('Invalid local grid origin')
                if o.orientation.x != 0 or o.orientation.y != 0 or o.orientation.z != 0 or o.orientation.w != 1:
                    raise ValueError('Expected axis-aligned local grid')
                ox = tx + math.cos(yaw)*o.position.x - math.sin(yaw)*o.position.y
                oy = ty + math.sin(yaw)*o.position.x + math.cos(yaw)*o.position.y
                prior = prior_for_grid(self.cells, grid.info.width, grid.info.height,
                                       grid.info.resolution, (ox, oy, yaw))
                live = np.asarray(grid.data, dtype=np.int8).reshape(prior.shape)
                if np.any((live < -1) | (live > 100)):
                    raise ValueError('Invalid occupancy values')
                grid.data = merge_prior(live, prior).ravel().tolist()
                grid.header.frame_id = 'map'
                o.position.x, o.position.y = ox, oy
                o.orientation.z, o.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                self.grid_pub.publish(grid)
                self.last_stamp = stamp
            except ValueError as error:
                self.warn(error)
            self.pending.popleft()

    def warn(self, error):
        if time.monotonic()-self.last_warning > 2:
            self.get_logger().warning(str(error))
            self.last_warning = time.monotonic()


def main():
    rclpy.init()
    node = PortoScene()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
