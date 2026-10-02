#!/usr/bin/python3
"""Virtual wall manager: owns per-map wall definitions and publishes the Nav2 keepout mask."""

import math
import os
import threading
import uuid
from datetime import datetime, timezone

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import MapPoint, VirtualWall, VirtualWallArray
from robot_interfaces.srv import (
    CheckMapPoint, ClearVirtualWalls, CreateVirtualWall, DeleteVirtualWall,
    ListVirtualWalls, SetVirtualWallEnabled, SwitchMap, UpdateVirtualWall)
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

FORMAT_VERSION = 1
KEEPOUT = 100
TYPE_NAMES = {VirtualWall.TYPE_LINE: 'line', VirtualWall.TYPE_POLYGON: 'polygon'}
TYPE_VALUES = {v: k for k, v in TYPE_NAMES.items()}


class WallError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code
        self.message = message


def now_iso():
    return datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')


def iso_to_time(text):
    msg = Time()
    try:
        dt = datetime.strptime(str(text), '%Y-%m-%dT%H:%M:%SZ').replace(tzinfo=timezone.utc)
        msg.sec = int(dt.timestamp())
    except ValueError:
        pass
    return msg


def time_to_iso(t):
    if t.sec == 0 and t.nanosec == 0:
        return now_iso()
    return datetime.fromtimestamp(t.sec, timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')


def segment_distance(px, py, a, b):
    """Distance from each (px, py) to the segment a-b."""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 < 1e-12:
        return np.hypot(px - ax, py - ay)
    t = np.clip(((px - ax) * dx + (py - ay) * dy) / length2, 0.0, 1.0)
    return np.hypot(px - (ax + t * dx), py - (ay + t * dy))


def wall_excess(wall, px, py):
    """Distance outside the wall's true geometry (0 inside a polygon / within a line's thickness)."""
    pts = [(p['x'], p['y']) for p in wall['points']]
    if wall['type'] == VirtualWall.TYPE_LINE:
        return np.maximum(segment_distance(px, py, pts[0], pts[1]) - wall['thickness'] / 2.0, 0.0)
    n = len(pts)
    inside = np.zeros(px.shape, dtype=bool)
    edge = np.full(px.shape, np.inf)
    for i in range(n):
        (x1, y1), (x2, y2) = pts[i], pts[(i + 1) % n]
        edge = np.minimum(edge, segment_distance(px, py, (x1, y1), (x2, y2)))
        if y1 != y2:
            crosses = ((y1 > py) != (y2 > py)) & (px < (x2 - x1) * (py - y1) / (y2 - y1) + x1)
            inside ^= crosses
    return np.where(inside, 0.0, edge)


class VirtualWallManager(Node):
    def __init__(self):
        super().__init__('virtual_wall_manager')
        map_yaml = self.declare_parameter('map_yaml', '').value
        self.walls_dir = self.declare_parameter('walls_dir', '').value
        self.default_thickness = self.declare_parameter('default_thickness', 0.15).value
        self.padding = self.declare_parameter('footprint_padding', 0.0).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.lock = threading.RLock()

        self.map_id = None
        self.map_yaml = None
        self.map_meta = None      # (resolution, origin_x, origin_y) from the active map yaml
        self.map_info = None      # nav_msgs/MapMetaData from the live /map
        self.walls = []
        self.status = 'NO_ACTIVE_MAP'

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.mask_pub = self.create_publisher(
            OccupancyGrid, '/robot/virtual_walls/keepout_mask', latched)
        self.info_pub = self.create_publisher(
            CostmapFilterInfo, '/robot/virtual_walls/filter_info', latched)
        self.state_pub = self.create_publisher(VirtualWallArray, '/robot/virtual_walls', latched)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/robot/virtual_walls/markers', latched)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)

        cb = ReentrantCallbackGroup()
        self.load_client = self.create_client(LoadMap, '/map_server/load_map', callback_group=cb)
        base = '/robot/virtual_walls/'
        for name, srv, handler in (
                ('create', CreateVirtualWall, self.srv_create),
                ('update', UpdateVirtualWall, self.srv_update),
                ('delete', DeleteVirtualWall, self.srv_delete),
                ('enable', SetVirtualWallEnabled, lambda q, r: self.srv_enable(q, r, True)),
                ('disable', SetVirtualWallEnabled, lambda q, r: self.srv_enable(q, r, False)),
                ('list', ListVirtualWalls, self.srv_list),
                ('clear', ClearVirtualWalls, self.srv_clear),
                ('check_point', CheckMapPoint, self.srv_check_point),
                ('switch_map', SwitchMap, self.srv_switch_map)):
            self.create_service(srv, base + name, handler, callback_group=cb)

        info = CostmapFilterInfo()
        info.type = 0
        info.filter_mask_topic = '/robot/virtual_walls/keepout_mask'
        info.base = 0.0
        info.multiplier = 1.0
        self.info_pub.publish(info)

        if map_yaml:
            try:
                self.activate_map(os.path.realpath(map_yaml))
            except WallError as e:
                self.get_logger().error(f'Cannot activate map {map_yaml}: {e.message}')
        else:
            self.get_logger().warn('No map_yaml given: no active map, walls disabled')
        self.publish_all()

    # ---------------------------------------------------------------- map / storage
    def walls_path(self, yaml_path, map_id):
        directory = self.walls_dir or os.path.dirname(yaml_path)
        return os.path.join(directory, f'{map_id}.virtual_walls.yaml')

    def activate_map(self, yaml_path):
        """Make yaml_path the active map and load its walls."""
        try:
            with open(yaml_path) as f:
                meta = yaml.safe_load(f)
            resolution = float(meta['resolution'])
            origin = [float(v) for v in meta['origin']]
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as e:
            raise WallError('MAP_LOAD_FAILED', f'bad map yaml {yaml_path}: {e}')
        with self.lock:
            self.map_yaml = yaml_path
            self.map_id = os.path.splitext(os.path.basename(yaml_path))[0]
            self.map_meta = (resolution, origin[0], origin[1])
            self.walls = []
            self.status = 'OK'
            self.load_walls()
        self.get_logger().info(
            f"Active map '{self.map_id}', {len(self.walls)} virtual wall(s) loaded")

    def load_walls(self):
        path = self.walls_path(self.map_yaml, self.map_id)
        if not os.path.exists(path):
            return
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            if data.get('map_id') != self.map_id:
                raise ValueError(f"file is for map '{data.get('map_id')}', active map is "
                                 f"'{self.map_id}'")
            walls = [self.wall_from_dict(d) for d in data.get('walls') or []]
        except (OSError, ValueError, KeyError, TypeError, WallError, yaml.YAMLError) as e:
            self.get_logger().error(f'Rejected wall file {path}: {e}')
            try:
                os.replace(path, path + '.rejected')
            except OSError:
                pass
            self.status = 'WALL_FILE_REJECTED'
            return
        self.walls = walls

    def save_walls(self):
        path = self.walls_path(self.map_yaml, self.map_id)
        data = {
            'format_version': FORMAT_VERSION,
            'map_id': self.map_id,
            'walls': [self.wall_to_dict(w) for w in self.walls],
        }
        tmp = path + '.tmp'
        try:
            with open(tmp, 'w') as f:
                yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)
            os.replace(tmp, path)
        except OSError as e:
            raise WallError('PERSIST_FAILED', f'cannot write {path}: {e}')

    def on_map(self, msg):
        with self.lock:
            self.map_info = msg.info
            self.publish_mask()

    # ---------------------------------------------------------------- wall conversion
    @staticmethod
    def wall_to_dict(w):
        d = {
            'id': w['id'], 'name': w['name'], 'type': TYPE_NAMES[w['type']],
            'enabled': w['enabled'],
        }
        if w['type'] == VirtualWall.TYPE_LINE:
            d['thickness'] = w['thickness']
        d['points'] = [[p['x'], p['y']] for p in w['points']]
        d['created_at'] = w['created_at']
        d['updated_at'] = w['updated_at']
        return d

    def wall_from_dict(self, d):
        if d['type'] not in TYPE_VALUES:
            raise WallError('INVALID_GEOMETRY', f"unknown wall type '{d['type']}'")
        wall = {
            'id': str(d['id']), 'name': str(d.get('name', '')), 'type': TYPE_VALUES[d['type']],
            'enabled': bool(d.get('enabled', True)),
            'thickness': float(d.get('thickness', self.default_thickness)),
            'points': [{'x': float(p[0]), 'y': float(p[1])} for p in d['points']],
            'created_at': str(d.get('created_at', now_iso())),
            'updated_at': str(d.get('updated_at', now_iso())),
        }
        self.validate(wall)
        return wall

    def wall_from_msg(self, m, wall_id, created_at=None):
        wall = {
            'id': wall_id, 'name': m.name, 'type': int(m.type), 'enabled': bool(m.enabled),
            'thickness': float(m.thickness) if m.thickness > 0 else self.default_thickness,
            'points': [{'x': float(p.x), 'y': float(p.y)} for p in m.points],
            'created_at': created_at or now_iso(), 'updated_at': now_iso(),
        }
        if m.map_id and m.map_id != self.map_id:
            raise WallError('MAP_MISMATCH',
                            f"wall map '{m.map_id}' is not the active map '{self.map_id}'")
        self.validate(wall)
        return wall

    def wall_to_msg(self, w):
        m = VirtualWall()
        m.id, m.name, m.map_id = w['id'], w['name'], self.map_id or ''
        m.type, m.enabled, m.thickness = w['type'], w['enabled'], w['thickness']
        m.points = [MapPoint(x=p['x'], y=p['y']) for p in w['points']]
        m.created_at = iso_to_time(w['created_at'])
        m.updated_at = iso_to_time(w['updated_at'])
        return m

    @staticmethod
    def validate(w):
        pts = w['points']
        if any(not (math.isfinite(p['x']) and math.isfinite(p['y'])) for p in pts):
            raise WallError('INVALID_GEOMETRY', 'points must be finite')
        if w['type'] == VirtualWall.TYPE_LINE:
            if len(pts) != 2:
                raise WallError('INVALID_GEOMETRY', 'a line wall needs exactly 2 points')
            if pts[0] == pts[1]:
                raise WallError('INVALID_GEOMETRY', 'line start and end are identical')
            if not (w['thickness'] > 0 and math.isfinite(w['thickness'])):
                raise WallError('INVALID_GEOMETRY', 'thickness must be > 0')
        elif w['type'] == VirtualWall.TYPE_POLYGON:
            if len(pts) < 3:
                raise WallError('INVALID_GEOMETRY', 'a polygon needs at least 3 points')
            area = sum(pts[i]['x'] * pts[(i + 1) % len(pts)]['y']
                       - pts[(i + 1) % len(pts)]['x'] * pts[i]['y'] for i in range(len(pts)))
            if abs(area) < 1e-9:
                raise WallError('INVALID_GEOMETRY', 'polygon has zero area')
        else:
            raise WallError('INVALID_GEOMETRY', f"unknown wall type {w['type']}")

    # ---------------------------------------------------------------- mask + publishing
    def grid_centres(self, info, x0, x1, y0, y1):
        xs = info.origin.position.x + (np.arange(x0, x1) + 0.5) * info.resolution
        ys = info.origin.position.y + (np.arange(y0, y1) + 0.5) * info.resolution
        return np.meshgrid(xs, ys)

    def build_mask(self, info):
        mask = np.zeros((info.height, info.width), dtype=np.int8)
        for w in self.walls:
            if not w['enabled']:
                continue
            xs = [p['x'] for p in w['points']]
            ys = [p['y'] for p in w['points']]
            reach = self.padding + (w['thickness'] / 2.0 if w['type'] == VirtualWall.TYPE_LINE
                                    else 0.0) + info.resolution
            ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
            x0 = max(int(math.floor((min(xs) - reach - ox) / res)), 0)
            x1 = min(int(math.ceil((max(xs) + reach - ox) / res)) + 1, info.width)
            y0 = max(int(math.floor((min(ys) - reach - oy) / res)), 0)
            y1 = min(int(math.ceil((max(ys) + reach - oy) / res)) + 1, info.height)
            if x0 >= x1 or y0 >= y1:
                self.get_logger().warn(f"Wall '{w['id']}' lies outside the map")
                continue
            px, py = self.grid_centres(info, x0, x1, y0, y1)
            hit = wall_excess(w, px, py) <= self.padding
            mask[y0:y1, x0:x1][hit] = KEEPOUT
        return mask

    def map_consistent(self):
        if self.map_info is None or self.map_meta is None:
            return False
        res, ox, oy = self.map_meta
        i = self.map_info
        return (abs(i.resolution - res) < 1e-6 and abs(i.origin.position.x - ox) < 1e-3
                and abs(i.origin.position.y - oy) < 1e-3)

    def publish_mask(self):
        if self.map_info is None:
            return
        grid = OccupancyGrid()
        grid.header.frame_id = self.map_frame
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info = self.map_info
        if self.map_id is None or not self.map_consistent():
            if self.map_id is not None:
                self.get_logger().warn(
                    f"Live /map does not match active map '{self.map_id}': "
                    'publishing an empty keepout mask')
            mask = np.zeros((self.map_info.height, self.map_info.width), dtype=np.int8)
        else:
            mask = self.build_mask(self.map_info)
        grid.data = mask.ravel().tolist()
        self.mask_pub.publish(grid)

    def publish_state(self):
        msg = VirtualWallArray()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.map_id = self.map_id or ''
        msg.walls = [self.wall_to_msg(w) for w in self.walls]
        self.state_pub.publish(msg)

    def publish_markers(self):
        out = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        out.markers.append(wipe)
        for n, w in enumerate(self.walls):
            on = w['enabled']
            color = ColorRGBA(r=1.0, g=0.0, b=0.8, a=1.0) if on else \
                ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.8)
            pts = [Point(x=p['x'], y=p['y'], z=0.05) for p in w['points']]
            if w['type'] == VirtualWall.TYPE_POLYGON:
                pts.append(pts[0])
            line = Marker()
            line.header.frame_id = self.map_frame
            line.ns, line.id = 'virtual_walls', n * 2
            line.action = Marker.ADD
            line.scale.x = max(w['thickness'], 0.08) if w['type'] == VirtualWall.TYPE_LINE \
                else 0.08
            line.color = color
            line.pose.orientation.w = 1.0
            if on:
                line.type = Marker.LINE_STRIP
                line.points = pts
            else:
                line.type = Marker.LINE_LIST
                line.scale.x = 0.04
                for a, b in zip(pts, pts[1:]):
                    seg = math.hypot(b.x - a.x, b.y - a.y)
                    steps = max(int(seg / 0.2), 1)
                    for k in range(0, steps, 2):
                        for t in (k / steps, min((k + 1) / steps, 1.0)):
                            line.points.append(Point(x=a.x + (b.x - a.x) * t,
                                                     y=a.y + (b.y - a.y) * t, z=0.05))
            out.markers.append(line)
            label = Marker()
            label.header.frame_id = self.map_frame
            label.ns, label.id = 'virtual_walls', n * 2 + 1
            label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x = sum(p['x'] for p in w['points']) / len(w['points'])
            label.pose.position.y = sum(p['y'] for p in w['points']) / len(w['points'])
            label.pose.position.z = 0.4
            label.pose.orientation.w = 1.0
            label.scale.z = 0.25
            label.color = color
            label.text = w['name'] or w['id']
            if not on:
                label.text += ' (disabled)'
            out.markers.append(label)
        self.marker_pub.publish(out)

    def publish_all(self):
        self.publish_mask()
        self.publish_state()
        self.publish_markers()

    # ---------------------------------------------------------------- services
    def run(self, resp, action):
        """Run a mutation under the lock, persist, publish; fill result fields on resp."""
        with self.lock:
            try:
                if self.map_id is None:
                    raise WallError('NO_ACTIVE_MAP', 'no active map')
                action()
                self.save_walls()
                self.publish_all()
                resp.success, resp.error_code, resp.message = True, '', 'ok'
            except WallError as e:
                resp.success, resp.error_code, resp.message = False, e.code, e.message
        return resp

    def find(self, wall_id):
        for i, w in enumerate(self.walls):
            if w['id'] == wall_id:
                return i
        raise WallError('WALL_NOT_FOUND', f"no wall with id '{wall_id}'")

    def srv_create(self, req, resp):
        def action():
            wid = req.wall.id
            if not wid:
                wid = ('zone_' if req.wall.type == VirtualWall.TYPE_POLYGON else 'wall_') \
                    + uuid.uuid4().hex[:8]
            elif any(w['id'] == wid for w in self.walls):
                raise WallError('DUPLICATE_ID', f"wall '{wid}' already exists")
            wall = self.wall_from_msg(req.wall, wid)
            self.walls.append(wall)
            resp.wall = self.wall_to_msg(wall)
        return self.run(resp, action)

    def srv_update(self, req, resp):
        def action():
            i = self.find(req.wall.id)
            wall = self.wall_from_msg(req.wall, req.wall.id, self.walls[i]['created_at'])
            self.walls[i] = wall
            resp.wall = self.wall_to_msg(wall)
        return self.run(resp, action)

    def srv_delete(self, req, resp):
        return self.run(resp, lambda: self.walls.pop(self.find(req.id)))

    def srv_enable(self, req, resp, enabled):
        def action():
            w = self.walls[self.find(req.id)]
            w['enabled'] = enabled
            w['updated_at'] = now_iso()
        return self.run(resp, action)

    def srv_clear(self, req, resp):
        return self.run(resp, self.walls.clear)

    def srv_list(self, req, resp):
        with self.lock:
            resp.success = self.map_id is not None
            resp.error_code = '' if resp.success else 'NO_ACTIVE_MAP'
            resp.message = 'ok' if resp.success else 'no active map'
            resp.map_id = self.map_id or ''
            resp.walls = [self.wall_to_msg(w) for w in self.walls]
        return resp

    def srv_check_point(self, req, resp):
        with self.lock:
            if self.map_id is None:
                resp.success, resp.error_code, resp.message = False, 'NO_ACTIVE_MAP', 'no active map'
                return resp
            px, py = np.array([req.point.x]), np.array([req.point.y])
            resp.wall_ids = [w['id'] for w in self.walls
                             if w['enabled'] and wall_excess(w, px, py)[0] <= self.padding]
            resp.restricted = bool(resp.wall_ids)
            resp.success, resp.error_code, resp.message = True, '', 'ok'
        return resp

    def srv_switch_map(self, req, resp):
        base = self.walls_dir or (os.path.dirname(self.map_yaml) if self.map_id else '')
        yaml_path = os.path.join(base, f'{req.map_id}.yaml')
        if not req.map_id or not os.path.exists(yaml_path):
            resp.success, resp.error_code = False, 'MAP_LOAD_FAILED'
            resp.message = f'map yaml not found: {yaml_path}'
            return resp
        if not self.load_client.wait_for_service(timeout_sec=2.0):
            resp.success, resp.error_code = False, 'MAP_LOAD_FAILED'
            resp.message = '/map_server/load_map unavailable'
            return resp
        done = threading.Event()
        holder = {}
        future = self.load_client.call_async(LoadMap.Request(map_url=yaml_path))
        future.add_done_callback(lambda f: (holder.update(r=f.result()), done.set()))
        if not done.wait(10.0) or holder['r'].result != LoadMap.Response.RESULT_SUCCESS:
            resp.success, resp.error_code = False, 'MAP_LOAD_FAILED'
            resp.message = f'map_server could not load {yaml_path}'
            return resp
        with self.lock:
            try:
                self.activate_map(os.path.realpath(yaml_path))
            except WallError as e:
                resp.success, resp.error_code, resp.message = False, e.code, e.message
                return resp
            self.map_info = holder['r'].map.info
            self.publish_all()
        resp.success, resp.error_code, resp.message = True, '', 'ok'
        return resp


def main():
    rclpy.init()
    node = VirtualWallManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
