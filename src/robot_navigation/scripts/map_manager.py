#!/usr/bin/python3
"""Authoritative writable-map lifecycle for localization and SLAM."""

import math
import os
import re
import shutil
import threading
import time
import uuid
import zlib
from datetime import datetime, timezone

import rclpy
import yaml
from PIL import Image
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import MapInfo, MapState, SafetyState
from robot_interfaces.srv import (ActivateMap, DeleteMap, ListMaps, RenameMap, SaveMap, StartMapping,
                                  StopMapping, SwitchMap)
from slam_toolbox.srv import DeserializePoseGraph, SerializePoseGraph
from std_msgs.msg import Bool
from std_srvs.srv import Empty, SetBool, Trigger
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage


MAP_ID = re.compile(r'^[a-z0-9][a-z0-9_-]{0,63}$')
FORMAT_VERSION = 1


class MapFailure(Exception):
    def __init__(self, code, detail):
        super().__init__(detail)
        self.code = code
        self.detail = detail


def iso_now():
    return datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')


def iso_time(text):
    out = rclpy.time.Time().to_msg()
    try:
        value = datetime.strptime(str(text), '%Y-%m-%dT%H:%M:%SZ').replace(tzinfo=timezone.utc)
        out.sec = int(value.timestamp())
    except (TypeError, ValueError):
        pass
    return out


def image_dimensions(path):
    # Decode the raster, rather than trusting a valid-looking header of a truncated file.
    with Image.open(path) as image:
        width, height = image.size
        if width <= 0 or height <= 0 or width * height > 4_000_000:
            raise ValueError('invalid or oversized map image dimensions')
        if image.format not in ('PNG', 'PPM'):
            raise ValueError('unsupported map image format')
        image.load()
        return width, height


class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')
        raw_maps_dir = self.declare_parameter('maps_dir', '~/.robot/maps').value
        self.maps_dir = os.path.realpath(os.path.expanduser(raw_maps_dir))
        self.bundled_yaml = os.path.realpath(
            os.path.expanduser(self.declare_parameter('bundled_map_yaml', '').value))
        self.default_map_id = self.declare_parameter('default_map_id', 'main_floor').value
        if not os.path.isabs(self.maps_dir):
            raise RuntimeError('maps_dir must resolve to an absolute path')

        self.lock = threading.RLock()
        self.transition_lock = threading.Lock()
        self.transition_thread = None
        self.shutting_down = False
        self.state = MapState.NO_MAP
        self.mode = MapState.MODE_NONE
        self.active_map_id = ''
        self.error_code = ''
        self.detail = 'starting'
        self.localization_ready = False
        self.navigation_ready = False
        self.last_amcl_rx = 0.0
        self.last_amcl_message = None
        self.last_localization_tf_rx = 0.0
        self.localization_generation = time.monotonic()
        self.map_generation = time.monotonic()
        self.map_ready = threading.Event()
        self.motion_ready = threading.Event()
        self.localization_ready_event = threading.Event()
        self.target_map_id = ''
        self.target_map_info = None
        self.localization_good_samples = 0
        self.localization_lost_since = time.monotonic()
        self.last_global_recovery = 0.0
        self.last_nomotion_update = 0.0
        self.map_wait_timeout = float(self.declare_parameter('map_wait_timeout', 20.0).value)
        self.localization_wait_timeout = float(
            self.declare_parameter('localization_wait_timeout', 45.0).value)
        self.max_position_stddev = float(
            self.declare_parameter('max_localization_position_stddev', 0.5).value)
        self.max_yaw_stddev = float(
            self.declare_parameter('max_localization_yaw_stddev', 0.387).value)
        self.mapping_unsaved = False
        self.mapping_saved_id = ''
        self.mapping_signature = None
        self.mapping_grid = None
        self.mapping_last_rx = 0.0
        self.saved_mapping_signature = None
        self.session_dir = os.path.join(self.maps_dir, '.mapping_session')
        self.session_prefix = os.path.join(self.session_dir, 'session')
        self.recoverable_session = False
        self.checkpoint_pending = False
        self.previous_map_id = ''
        self.restore_started = False
        self.bootstrap_pending = True

        cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub = self.create_publisher(MapState, '/robot/map/state', latched)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100, callback_group=cb)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched,
                                 callback_group=cb)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, latched,
                                 callback_group=cb)
        self.create_subscription(Bool, '/robot/navigation/runtime_ready', self.on_nav_ready,
                                 latched, callback_group=cb)
        self.create_subscription(SafetyState, '/robot/safety/state', self.on_safety,
                                 latched, callback_group=cb)

        self.map_server_params = self.create_client(
            SetParameters, '/map_server/set_parameters', callback_group=cb)
        self.amcl_params = self.create_client(SetParameters, '/amcl/set_parameters', callback_group=cb)
        self.map_server_state = self.create_client(GetState, '/map_server/get_state', callback_group=cb)
        self.map_server_change = self.create_client(ChangeState, '/map_server/change_state', callback_group=cb)
        self.nav_enable = self.create_client(
            SetBool, '/robot/navigation/set_enabled', callback_group=cb)
        self.safety_hold = self.create_client(
            SetBool, '/robot/safety/transition_hold', callback_group=cb)
        self.cancel_nav = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal', callback_group=cb)
        self.localization_manager = self.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_localization/manage_nodes',
            callback_group=cb)
        self.core_manager = self.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_navigation_core/manage_nodes', callback_group=cb)
        self.planner_state = self.create_client(GetState, '/planner_server/get_state', callback_group=cb)
        self.core_paused = False
        self.slam_state = self.create_client(
            GetState, '/slam_toolbox/get_state', callback_group=cb)
        self.slam_change = self.create_client(
            ChangeState, '/slam_toolbox/change_state', callback_group=cb)
        self.amcl_state = self.create_client(GetState, '/amcl/get_state', callback_group=cb)
        self.amcl_change = self.create_client(
            ChangeState, '/amcl/change_state', callback_group=cb)
        self.slam_serialize = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map', callback_group=cb)
        self.slam_deserialize = self.create_client(
            DeserializePoseGraph, '/slam_toolbox/deserialize_map', callback_group=cb)
        self.global_localization = self.create_client(
            Empty, '/reinitialize_global_localization', callback_group=cb)
        self.nomotion_update = self.create_client(
            Empty, '/request_nomotion_update', callback_group=cb)

        self.create_service(Trigger, '/robot/maps/bootstrap_ready', self.srv_bootstrap,
                            callback_group=cb)
        self.create_service(ListMaps, '/robot/maps/list', self.srv_list, callback_group=cb)
        self.create_service(ActivateMap, '/robot/maps/activate', self.srv_activate,
                            callback_group=cb)
        self.create_service(StartMapping, '/robot/maps/start_mapping', self.srv_start_mapping,
                            callback_group=cb)
        self.create_service(StopMapping, '/robot/maps/stop_mapping', self.srv_stop_mapping,
                            callback_group=cb)
        self.create_service(SaveMap, '/robot/maps/save', self.srv_save, callback_group=cb)
        self.create_service(RenameMap, '/robot/maps/rename', self.srv_rename, callback_group=cb)
        self.create_service(DeleteMap, '/robot/maps/delete', self.srv_delete, callback_group=cb)
        self.create_service(SwitchMap, '/robot/virtual_walls/switch_map', self.srv_legacy_switch,
                            callback_group=cb)

        try:
            self.seed_default()
            self.recoverable_session = self.detect_recoverable_session()
            available = self.scan_maps()
            selected = self.restore_active(available)
            if selected is None:
                raise MapFailure('MAP_UNAVAILABLE', 'no valid runtime maps')
            self.active_map_id = selected.id
            self.mode = MapState.MODE_LOCALIZATION
            self.state = MapState.LOADING
            self.detail = f'waiting for map_server to load {selected.id}'
            self.bootstrap_ok = True
        except MapFailure as exc:
            self.bootstrap_ok = False
            self.state = MapState.ERROR
            self.error_code, self.detail = exc.code, exc.detail
            self.get_logger().error(exc.detail)
        self.publish_state()
        self.create_timer(2.0, self.publish_state)
        self.create_timer(0.2, self.localization_tick, callback_group=cb)
        self.create_timer(10.0, self.checkpoint_mapping, callback_group=cb)

    # ---------- storage
    def map_dir(self, map_id):
        return os.path.join(self.maps_dir, map_id)

    def seed_default(self):
        os.makedirs(self.maps_dir, mode=0o755, exist_ok=True)
        target = self.map_dir(self.default_map_id)
        if os.path.exists(target):
            return
        if not self.bundled_yaml or not os.path.isfile(self.bundled_yaml):
            raise MapFailure('MAP_UNAVAILABLE', 'bundled default map is unavailable')
        tmp = os.path.join(self.maps_dir, f'.tmp-seed-{uuid.uuid4().hex}')
        try:
            os.mkdir(tmp)
            with open(self.bundled_yaml, encoding='utf-8') as stream:
                metadata = yaml.safe_load(stream) or {}
            image_value = metadata.get('image')
            if not image_value:
                raise ValueError('bundled map YAML has no image')
            source_image = image_value if os.path.isabs(image_value) else os.path.join(
                os.path.dirname(self.bundled_yaml), image_value)
            extension = os.path.splitext(source_image)[1].lower() or '.pgm'
            shutil.copy2(source_image, os.path.join(tmp, f'map{extension}'))
            metadata['image'] = f'map{extension}'
            with open(os.path.join(tmp, 'map.yaml'), 'w', encoding='utf-8') as stream:
                yaml.safe_dump(metadata, stream, sort_keys=False)
            self.write_default_walls(tmp, self.default_map_id)
            now = iso_now()
            self.write_metadata(tmp, self.default_map_id, 'Main Floor', now, now)
            self.validate_dir(tmp, expected_id=self.default_map_id)
            os.replace(tmp, target)
            self.get_logger().info(f"Seeded runtime map '{self.default_map_id}'")
        except Exception as exc:
            shutil.rmtree(tmp, ignore_errors=True)
            raise MapFailure('MAP_LOAD_FAILED', f'cannot seed default map: {exc}') from exc

    @staticmethod
    def write_default_walls(directory, map_id):
        path = os.path.join(directory, 'virtual_walls.yaml')
        if not os.path.exists(path):
            with open(path, 'w', encoding='utf-8') as stream:
                yaml.safe_dump({'format_version': 1, 'map_id': map_id, 'walls': []}, stream,
                               sort_keys=False)

    @staticmethod
    def write_metadata(directory, map_id, display_name, created, updated, alignment=None):
        metadata = {'format_version': FORMAT_VERSION, 'id': map_id,
                    'display_name': display_name or map_id,
                    'created_at': created, 'updated_at': updated}
        if alignment is not None:
            metadata['map_to_odom'] = alignment
        with open(os.path.join(directory, 'metadata.yaml'), 'w', encoding='utf-8') as stream:
            yaml.safe_dump(metadata, stream, sort_keys=False)

    def validate_id(self, map_id):
        if not isinstance(map_id, str) or not MAP_ID.fullmatch(map_id):
            raise MapFailure('MAP_ID_INVALID',
                             'map_id must match [a-z0-9][a-z0-9_-]{0,63}')

    def validate_dir(self, directory, expected_id=None):
        map_id = expected_id or os.path.basename(directory)
        self.validate_id(map_id)
        real_dir = os.path.realpath(directory)
        if os.path.commonpath((self.maps_dir, real_dir)) != self.maps_dir:
            raise MapFailure('MAP_METADATA_INVALID', 'map directory escapes maps_dir')
        yaml_path = os.path.join(real_dir, 'map.yaml')
        try:
            if os.path.islink(directory) or os.path.islink(yaml_path):
                raise ValueError('map directory and YAML may not be symlinks')
            with open(yaml_path, encoding='utf-8') as stream:
                nav = yaml.safe_load(stream) or {}
            if not isinstance(nav, dict):
                raise ValueError('map YAML must be an object')
            image_value = nav['image']
            if os.path.isabs(image_value):
                raise ValueError('absolute image paths are not allowed')
            image_path = os.path.realpath(os.path.join(real_dir, image_value))
            if os.path.islink(os.path.join(real_dir, image_value)):
                raise ValueError('image symlink escapes the supported map storage layout')
            if os.path.commonpath((real_dir, image_path)) != real_dir or not os.path.isfile(image_path):
                raise ValueError('referenced image is missing or escapes the map directory')
            resolution = float(nav['resolution'])
            origin_values = [float(value) for value in nav['origin']]
            if (not math.isfinite(resolution) or resolution <= 0.0 or len(origin_values) != 3
                    or not all(math.isfinite(value) for value in origin_values)):
                raise ValueError('invalid resolution or origin')
            if nav.get('mode', 'trinary') not in ('trinary', 'scale', 'raw'):
                raise ValueError('invalid map mode')
            for key in ('negate', 'occupied_thresh', 'free_thresh'):
                if key not in nav:
                    raise ValueError(f'missing {key}')
            if nav['negate'] not in (0, 1):
                raise ValueError('negate must be 0 or 1')
            occupied = float(nav['occupied_thresh'])
            free = float(nav['free_thresh'])
            if (not all(math.isfinite(value) for value in (occupied, free))
                    or not 0.0 <= free < occupied <= 1.0):
                raise ValueError('invalid occupancy thresholds')
            width, height = image_dimensions(image_path)
            if width <= 0 or height <= 0 or width * height > 4_000_000:
                raise ValueError('invalid or oversized map image dimensions')
            metadata_path = os.path.join(real_dir, 'metadata.yaml')
            metadata = {}
            if os.path.exists(metadata_path):
                if os.path.islink(metadata_path):
                    raise ValueError('metadata may not be a symlink')
                with open(metadata_path, encoding='utf-8') as stream:
                    metadata = yaml.safe_load(stream) or {}
                if not isinstance(metadata, dict):
                    raise ValueError('metadata YAML must be an object')
                if metadata.get('id', map_id) != map_id:
                    raise ValueError('metadata id does not match directory name')
                if int(metadata.get('format_version', FORMAT_VERSION)) != FORMAT_VERSION:
                    raise ValueError('unsupported metadata format_version')
                display_name = metadata.get('display_name', map_id)
                if not isinstance(display_name, str) or not display_name.strip() or len(display_name) > 128:
                    raise ValueError('invalid display_name')
                for key in ('created_at', 'updated_at'):
                    if key in metadata:
                        datetime.strptime(str(metadata[key]), '%Y-%m-%dT%H:%M:%SZ')
                if 'map_to_odom' in metadata:
                    transform = metadata['map_to_odom']
                    if (not isinstance(transform, list) or len(transform) != 3
                            or not all(math.isfinite(float(value)) for value in transform)):
                        raise ValueError('invalid map_to_odom transform')
            info = MapInfo()
            info.id = map_id
            info.name = str(metadata.get('display_name') or map_id)
            info.active = map_id == self.active_map_id
            info.resolution, info.width, info.height = resolution, width, height
            info.origin.position.x, info.origin.position.y = origin_values[0], origin_values[1]
            info.origin.orientation.z = math.sin(origin_values[2] / 2.0)
            info.origin.orientation.w = math.cos(origin_values[2] / 2.0)
            walls_path = os.path.join(real_dir, 'virtual_walls.yaml')
            info.has_virtual_walls = False
            if os.path.exists(walls_path):
                with open(walls_path, encoding='utf-8') as stream:
                    walls = yaml.safe_load(stream) or {}
                if not isinstance(walls, dict) or not isinstance(walls.get('walls', []), list):
                    raise ValueError('virtual walls YAML is invalid')
                info.has_virtual_walls = bool(walls.get('walls'))
            info.created_at = iso_time(metadata.get('created_at', ''))
            info.updated_at = iso_time(metadata.get('updated_at', ''))
            return info
        except MapFailure:
            raise
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            raise MapFailure('MAP_METADATA_INVALID', f"invalid map '{map_id}': {exc}") from exc

    def scan_maps(self):
        maps = []
        try:
            entries = sorted(os.scandir(self.maps_dir), key=lambda entry: entry.name)
        except OSError as exc:
            raise MapFailure('MAP_UNAVAILABLE', f'cannot scan maps_dir: {exc}') from exc
        for entry in entries:
            if entry.name.startswith('.') or not entry.is_dir(follow_symlinks=False):
                continue
            try:
                maps.append(self.validate_dir(entry.path))
            except MapFailure as exc:
                self.get_logger().warning(exc.detail)
        return maps

    def restore_active(self, available):
        ids = {item.id for item in available}
        active_file = os.path.join(self.maps_dir, '.active_map')
        try:
            with open(active_file, encoding='utf-8') as stream:
                saved = stream.read().strip()
        except OSError:
            saved = ''
        chosen = saved if saved in ids else self.default_map_id
        if chosen not in ids and ids:
            chosen = sorted(ids)[0]
        return next((item for item in available if item.id == chosen), None)

    def persist_active(self):
        tmp = os.path.join(self.maps_dir, '.active_map.tmp')
        with open(tmp, 'w', encoding='utf-8') as stream:
            stream.write(self.active_map_id + '\n')
        os.replace(tmp, os.path.join(self.maps_dir, '.active_map'))

    def detect_recoverable_session(self):
        """Detect complete legacy or generation checkpoints; SLAM validates their contents on resume."""
        marker = os.path.join(self.session_dir, 'session.yaml')
        try:
            with open(marker, encoding='utf-8') as stream:
                metadata = yaml.safe_load(stream)
            if metadata.get('format_version') != FORMAT_VERSION:
                return False
            checkpoint = metadata.get('checkpoint', 'session')
            if not re.fullmatch(r'(session|checkpoint_[a-f0-9]{32})', checkpoint):
                return False
            self.session_prefix = os.path.join(self.session_dir, checkpoint)
        except (OSError, TypeError, AttributeError, yaml.YAMLError):
            return False
        try:
            return all(os.path.isfile(self.session_prefix + suffix)
                       and os.path.getsize(self.session_prefix + suffix) > 0
                       for suffix in ('.posegraph', '.data'))
        except OSError:
            return False

    def clear_mapping_session(self):
        try:
            if os.path.exists(self.session_dir):
                shutil.rmtree(self.session_dir)
        except OSError as exc:
            raise MapFailure('MAPPING_SESSION_DELETE_FAILED', str(exc)) from exc
        self.recoverable_session = False

    def checkpoint_mapping(self):
        if not self.transition_lock.acquire(blocking=False):
            return
        with self.lock:
            if (self.shutting_down or self.state != MapState.MAPPING
                    or self.mode != MapState.MODE_MAPPING or not self.mapping_unsaved
                    or self.checkpoint_pending or not self.slam_serialize.service_is_ready()):
                self.transition_lock.release()
                return
            self.checkpoint_pending = True
        def completed():
            try:
                os.makedirs(self.session_dir, mode=0o700, exist_ok=True)
                checkpoint = 'checkpoint_' + uuid.uuid4().hex
                prefix = os.path.join(self.session_dir, checkpoint)
                request = SerializePoseGraph.Request(filename=prefix)
                result = self.call(self.slam_serialize, request,
                                   'MAPPING_CHECKPOINT_FAILED', timeout=30.0)
                if result.result != SerializePoseGraph.Response.RESULT_SUCCESS:
                    raise RuntimeError(f'serialize result {result.result}')
                marker = os.path.join(self.session_dir, '.session.yaml.tmp')
                with open(marker, 'w', encoding='utf-8') as stream:
                    yaml.safe_dump({'format_version': 1, 'updated_at': iso_now(),
                                    'checkpoint': checkpoint,
                                    'previous_map_id': self.previous_map_id}, stream)
                os.replace(marker, os.path.join(self.session_dir, 'session.yaml'))
                with self.lock:
                    self.recoverable_session = self.detect_recoverable_session()
                    self.publish_state()
                for entry in os.scandir(self.session_dir):
                    if (entry.name.startswith('checkpoint_')
                            and not entry.name.startswith(checkpoint)):
                        os.unlink(entry.path)
            except Exception as exc:
                self.get_logger().error(f'cannot checkpoint mapping session: {exc}')
            finally:
                self.checkpoint_pending = False
                self.transition_thread = None
                self.transition_lock.release()

        worker = threading.Thread(target=completed, name='map-checkpoint', daemon=True)
        self.transition_thread = worker
        worker.start()

    # ---------- state and async helpers
    def state_msg(self):
        msg = MapState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state, msg.mode = self.state, self.mode
        msg.active_map_id = self.active_map_id
        msg.map_available = bool((self.active_map_id or self.mapping_signature is not None)
                                 and self.state != MapState.ERROR)
        msg.localization_ready = self.localization_ready
        msg.navigation_runtime_ready = self.navigation_ready
        msg.unsaved_changes = self.mapping_unsaved
        msg.recoverable_session = self.recoverable_session
        msg.error_code, msg.detail = self.error_code, self.detail
        return msg

    def publish_state(self):
        """Publish authoritative state and log changes without logging every identical grid update."""
        with self.lock:
            self.state_pub.publish(self.state_msg())
            signature = (self.state, self.mode, self.active_map_id, self.recoverable_session,
                         self.mapping_unsaved, self.localization_ready, self.navigation_ready,
                         self.error_code, self.detail)
            if signature != getattr(self, '_logged_map_state', None):
                self._logged_map_state = signature
                self.get_logger().info(
                    f'[MAPPING] Map state published state={self.state} mode={self.mode} '
                    f'active_map_id={self.active_map_id} recoverable={self.recoverable_session} '
                    f'unsaved={self.mapping_unsaved} localization_ready={self.localization_ready} '
                    f'navigation_ready={self.navigation_ready} code={self.error_code} detail={self.detail}')

    def set_state(self, state, mode=None, detail='', error_code=''):
        with self.lock:
            self.state = state
            if mode is not None:
                self.mode = mode
            self.detail, self.error_code = detail, error_code
            self.publish_state()

    def admit_transition(self, response, prepare, state, mode, detail, operation,
                         failure_state, accepted_message):
        """Validate and reserve a transition, then run its slow work off the service callback."""
        if self.shutting_down:
            response.success, response.error_code = False, 'MAP_TRANSITION'
            response.message = 'map manager is shutting down'
            if hasattr(response, 'state'):
                response.state = self.state_msg()
            return response
        if not self.transition_lock.acquire(blocking=False):
            response.success, response.error_code = False, 'MAP_TRANSITION'
            response.message = 'another map transition is active'
            if hasattr(response, 'state'):
                response.state = self.state_msg()
            return response
        try:
            prepare()
            self.set_state(state, mode, detail)
            worker = threading.Thread(
                target=self.run_transition,
                args=(operation, failure_state),
                name='map-transition',
                daemon=True,
            )
            self.transition_thread = worker
            worker.start()
        except MapFailure as exc:
            self.transition_thread = None
            self.transition_lock.release()
            response.success, response.error_code, response.message = False, exc.code, exc.detail
            if hasattr(response, 'state'):
                response.state = self.state_msg()
            return response
        except Exception as exc:
            self.transition_thread = None
            self.transition_lock.release()
            response.success, response.error_code = False, 'MAP_TRANSITION_FAILED'
            response.message = str(exc)
            if hasattr(response, 'state'):
                response.state = self.state_msg()
            return response

        response.success, response.error_code, response.message = True, '', accepted_message
        if hasattr(response, 'state'):
            response.state = self.state_msg()
        return response

    def run_transition(self, operation, failure_state):
        try:
            operation()
        except MapFailure as exc:
            failure_state(exc)
        except Exception as exc:
            self.get_logger().error(f'unexpected map transition failure: {exc}')
            failure_state(MapFailure('MAP_TRANSITION_FAILED', str(exc)))
        finally:
            self.transition_thread = None
            self.transition_lock.release()

    def call(self, client, request, unavailable, timeout=30.0):
        """Refresh a stale discovery handle once before sending, never replaying a request."""
        temporary_client = None
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning(f'Retrying discovery with a fresh client: {client.srv_name}')
                temporary_client = self.create_client(
                    client.srv_type, client.srv_name, qos_profile=client.qos_profile,
                    callback_group=client.callback_group)
                client = temporary_client
                if not client.wait_for_service(timeout_sec=5.0):
                    raise MapFailure(unavailable, f'{client.srv_name} unavailable')
            done = threading.Event()
            holder = {}
            future = client.call_async(request)
            def complete(result):
                # Callback records the outcome; the transition worker reports failures.
                try:
                    holder['result'] = result.result()
                except Exception as exc:
                    holder['error'] = exc
                done.set()
            future.add_done_callback(complete)
            if not done.wait(timeout):
                future.cancel()
                raise MapFailure(unavailable, f'{client.srv_name} response timeout')
            if holder.get('result') is None:
                raise MapFailure(unavailable, f'{client.srv_name} call failed: {holder.get("error")}')
            return holder['result']
        finally:
            if temporary_client is not None:
                self.destroy_client(temporary_client)

    def fire_set_bool(self, client, value):
        if client.service_is_ready():
            client.call_async(SetBool.Request(data=value))

    def navigation_hold(self, hold):
        if hold:
            result = self.call(self.safety_hold, SetBool.Request(data=True), 'MAP_TRANSITION')
            if not result.success:
                raise MapFailure('MAP_TRANSITION', result.message)
            result = self.call(self.nav_enable, SetBool.Request(data=False), 'NAVIGATION_UNAVAILABLE')
            if not result.success:
                raise MapFailure('NAVIGATION_UNAVAILABLE', result.message)
            if self.cancel_nav.service_is_ready():
                self.cancel_nav.call_async(CancelGoal.Request())
        else:
            result = self.call(self.nav_enable, SetBool.Request(data=True), 'NAVIGATION_UNAVAILABLE')
            if not result.success:
                raise MapFailure('NAVIGATION_UNAVAILABLE', result.message)
            result = self.call(self.safety_hold, SetBool.Request(data=False), 'MAP_TRANSITION')
            if not result.success:
                raise MapFailure('MAP_TRANSITION', result.message)

    def manage_localization(self, command):
        response = self.call(
            self.localization_manager, ManageLifecycleNodes.Request(command=command),
            'LOCALIZATION_UNAVAILABLE', timeout=45.0)
        if not response.success:
            raise MapFailure('LOCALIZATION_UNAVAILABLE',
                             f'localization lifecycle command {command} rejected')

    def manage_core(self, command):
        result = self.call(self.core_manager, ManageLifecycleNodes.Request(command=command),
                           'NAVIGATION_UNAVAILABLE', timeout=45.0)
        if not result.success:
            raise MapFailure('NAVIGATION_UNAVAILABLE', f'core lifecycle command {command} rejected')

    def lifecycle_state(self, client, code):
        return self.call(client, GetState.Request(), code).current_state.label

    def change(self, client, transition, code):
        request = ChangeState.Request()
        request.transition.id = transition
        response = self.call(client, request, code)
        if not response.success:
            raise MapFailure(code, f'{client.srv_name} rejected transition {transition}')

    def reset_amcl(self):
        state = self.lifecycle_state(self.amcl_state, 'LOCALIZATION_UNAVAILABLE')
        if state == 'active':
            self.change(self.amcl_change, Transition.TRANSITION_DEACTIVATE,
                        'LOCALIZATION_UNAVAILABLE')
            state = 'inactive'
        if state == 'inactive':
            self.change(self.amcl_change, Transition.TRANSITION_CLEANUP,
                        'LOCALIZATION_UNAVAILABLE')
        self.change(self.amcl_change, Transition.TRANSITION_CONFIGURE,
                    'LOCALIZATION_UNAVAILABLE')
        self.change(self.amcl_change, Transition.TRANSITION_ACTIVATE,
                    'LOCALIZATION_UNAVAILABLE')

    def slam_start(self):
        state = self.lifecycle_state(self.slam_state, 'MAPPING_FAILED')
        if state == 'unconfigured':
            self.change(self.slam_change, Transition.TRANSITION_CONFIGURE, 'MAPPING_FAILED')
            state = 'inactive'
        if state == 'inactive':
            self.change(self.slam_change, Transition.TRANSITION_ACTIVATE, 'MAPPING_FAILED')
        elif state != 'active':
            raise MapFailure('MAPPING_FAILED', f'slam_toolbox is {state}')

    def slam_stop(self):
        state = self.lifecycle_state(self.slam_state, 'MAPPING_FAILED')
        if state == 'active':
            self.change(self.slam_change, Transition.TRANSITION_DEACTIVATE, 'MAPPING_FAILED')
            state = 'inactive'
        if state == 'inactive':
            self.change(self.slam_change, Transition.TRANSITION_CLEANUP, 'MAPPING_FAILED')

    # ---------- observed readiness
    def on_safety(self, msg):
        if msg.motion_inhibited or msg.state == SafetyState.FAULT:
            self.motion_ready.clear()
        else:
            self.motion_ready.set()

    def on_nav_ready(self, msg):
        with self.lock:
            changed = self.navigation_ready != bool(msg.data)
            self.navigation_ready = bool(msg.data)
            if changed:
                self.publish_state()

    @staticmethod
    def occupancy_signature(msg):
        payload = bytes((int(value) & 0xff) for value in msg.data)
        origin = msg.info.origin
        return (int(msg.info.width), int(msg.info.height), float(msg.info.resolution),
                origin.position.x, origin.position.y, origin.position.z,
                origin.orientation.x, origin.orientation.y, origin.orientation.z,
                origin.orientation.w, msg.header.frame_id, zlib.crc32(payload))

    def on_map(self, msg):
        restore_id = ''
        with self.lock:
            width, height = int(msg.info.width), int(msg.info.height)
            resolution = float(msg.info.resolution)
            if (width <= 0 or height <= 0 or width * height != len(msg.data)
                    or not math.isfinite(resolution) or resolution <= 0.0):
                self.get_logger().warning('ignoring malformed /map occupancy grid')
                return
            if self.mode == MapState.MODE_MAPPING:
                signature = self.occupancy_signature(msg)
                self.mapping_grid = msg
                self.mapping_last_rx = time.monotonic()
                self.mapping_signature = signature
                self.mapping_unsaved = signature != self.saved_mapping_signature
                self.map_ready.set()
                self.publish_state()
                return
            if self.mode != MapState.MODE_LOCALIZATION or self.state != MapState.LOADING:
                return
            if self.target_map_info is not None:
                expected = self.target_map_info
                if (width != expected.width or height != expected.height
                        or not math.isclose(resolution, expected.resolution, rel_tol=1e-6, abs_tol=1e-9)):
                    self.get_logger().warning('ignoring /map with unexpected geometry during switch')
                    return
                self.map_ready.set()
                return
            # Only the first map from launch drives bootstrap restoration. Explicit map
            # transitions are completed by their worker after lifecycle startup returns.
            if not self.bootstrap_pending:
                return
            self.bootstrap_pending = False
            if self.active_map_id != self.default_map_id and not self.restore_started:
                self.restore_started = True
                restore_id = self.active_map_id
            else:
                self.state = MapState.ACTIVE
                self.detail = f"map '{self.active_map_id}' active; waiting for localization"
                self.error_code = ''
                self.publish_state()
        if restore_id:
            threading.Thread(target=self.restore_selected_map, args=(restore_id,),
                             daemon=True).start()

    def restore_selected_map(self, map_id):
        try:
            with self.transition_lock:
                # map_server publishes before the launch manager finishes activating
                # AMCL. Wait for that initial transaction before restoring another map.
                deadline = time.monotonic() + self.map_wait_timeout
                while self.lifecycle_state(self.amcl_state, 'LOCALIZATION_UNAVAILABLE') != 'active':
                    if time.monotonic() >= deadline:
                        raise MapFailure('LOCALIZATION_UNAVAILABLE',
                                         'initial localization startup did not complete')
                    time.sleep(0.1)
                self.activate(map_id)
        except MapFailure as exc:
            self.set_state(MapState.ERROR, MapState.MODE_LOCALIZATION,
                           exc.detail, exc.code)

    def on_tf(self, msg):
        if any(tf.header.frame_id == 'map' and tf.child_frame_id == 'odom'
               for tf in msg.transforms):
            self.last_localization_tf_rx = time.monotonic()

    def on_amcl(self, msg, received=True):
        now = time.monotonic()
        with self.lock:
            if received:
                self.last_amcl_rx = now
                self.last_amcl_message = msg
            if self.mode != MapState.MODE_LOCALIZATION or self.state != MapState.ACTIVE:
                return
            covariance = msg.pose.covariance
            if (not self.localization_covariance_ok(
                    covariance, self.max_position_stddev, self.max_yaw_stddev)
                    or not self.localization_transforms_ready()):
                self.localization_good_samples = 0
                self.localization_ready_event.clear()
                if self.localization_ready:
                    self.localization_ready = False
                    self.navigation_ready = False
                    self.detail = f"map '{self.active_map_id}' active; localization lost"
                    self.publish_state()
                    self.fire_set_bool(self.nav_enable, False)
                return
            self.localization_good_samples += 1
            if self.localization_ready or self.localization_good_samples < 3:
                return
            self.localization_ready = True
            self.localization_lost_since = 0.0
            self.detail = f"map '{self.active_map_id}' and localization active"
            self.localization_ready_event.set()
            self.publish_state()
        if not self.transition_lock.locked():
            self.fire_set_bool(self.safety_hold, False)
            self.fire_set_bool(self.nav_enable, True)

    @staticmethod
    def localization_covariance_ok(covariance, max_position_stddev, max_yaw_stddev):
        if len(covariance) < 36:
            return False
        variances = (float(covariance[0]), float(covariance[7]), float(covariance[35]))
        # AMCL's stationary particle covariance can contain tiny negative roundoff.
        return (all(math.isfinite(value) and value >= -1e-12 for value in variances)
                and math.sqrt(max(0.0, variances[0], variances[1])) <= max_position_stddev
                and math.sqrt(max(0.0, variances[2])) <= max_yaw_stddev)

    def localization_transforms_ready(self):
        now = rclpy.time.Time()
        timeout = Duration(seconds=0.0)
        return (self.tf_buffer.can_transform('map', 'odom', now, timeout=timeout)
                and self.tf_buffer.can_transform('odom', 'base_footprint', now,
                                                 timeout=timeout))

    def localization_tick(self):
        now = time.monotonic()
        with self.lock:
            if self.mode != MapState.MODE_LOCALIZATION or self.state != MapState.ACTIVE:
                return
            # AMCL publishes covariance only after particle updates; while stationary
            # its continuously refreshed map->odom transform is the liveness signal.
            stale = self.last_amcl_message is None or now - self.last_localization_tf_rx > 1.0
            transforms_missing = not self.localization_transforms_ready()
            if self.localization_ready and (stale or transforms_missing):
                self.localization_ready = False
                self.localization_ready_event.clear()
                self.navigation_ready = False
                self.localization_good_samples = 0
                self.localization_lost_since = now
                self.detail = (f"map '{self.active_map_id}' active; "
                               f"localization {'stale' if stale else 'TF unavailable'}")
                self.publish_state()
                self.fire_set_bool(self.nav_enable, False)
            elif not self.localization_ready and self.localization_lost_since <= 0.0:
                self.localization_lost_since = now
            recover = (not self.transition_lock.locked()
                       and not self.localization_ready and self.localization_lost_since > 0.0
                       and now - self.localization_lost_since >= 10.0
                       and now - self.last_global_recovery >= 30.0
                       and self.global_localization.service_is_ready())
            if recover:
                self.last_global_recovery = now
            refine = (not self.localization_ready and now - self.last_nomotion_update >= 1.0
                      and self.nomotion_update.service_is_ready())
            if refine:
                self.last_nomotion_update = now
        if refine:
            self.nomotion_update.call_async(Empty.Request())
        if recover:
            self.get_logger().warning('requesting AMCL global localization recovery')
            self.global_localization.call_async(Empty.Request())
        if not stale and not transforms_missing and self.last_amcl_message is not None:
            self.on_amcl(self.last_amcl_message, received=False)

    # ---------- map transitions
    def wait_ready(self, event, timeout, code, detail):
        if not event.wait(timeout):
            raise MapFailure(code, detail)

    def activate_once(self, map_id, admitted=False):
        self.validate_id(map_id)
        info = self.validate_dir(self.map_dir(map_id), expected_id=map_id)
        if self.mode == MapState.MODE_MAPPING:
            raise MapFailure('MAPPING_ACTIVE', 'stop mapping before activating a saved map')
        self.navigation_hold(True)
        with self.lock:
            self.target_map_id = map_id
            self.target_map_info = info
            self.localization_ready = False
            self.navigation_ready = False
            self.localization_generation = time.monotonic()
            self.localization_good_samples = 0
            self.last_amcl_message = None
            self.last_localization_tf_rx = 0.0
            self.localization_lost_since = time.monotonic()
            self.map_ready.clear()
            self.localization_ready_event.clear()
        if not admitted:
            self.set_state(MapState.SWITCHING, MapState.MODE_LOCALIZATION,
                           f"switching to map '{map_id}'")
        try:
            if self.lifecycle_state(self.planner_state, 'NAVIGATION_UNAVAILABLE') == 'active':
                self.manage_core(ManageLifecycleNodes.Request.PAUSE)
                self.core_paused = True
            # Cycle map_server + amcl together through their lifecycle manager. Changing amcl
            # alone breaks its bond and the manager then takes map_server down mid-switch.
            amcl_label = self.lifecycle_state(self.amcl_state, 'LOCALIZATION_UNAVAILABLE')
            if amcl_label == 'inactive':
                # PAUSE already deactivated the nodes for mapping. RESET would attempt
                # invalid duplicate deactivations; clean the paused group before STARTUP.
                self.change(self.amcl_change, Transition.TRANSITION_CLEANUP,
                            'LOCALIZATION_UNAVAILABLE')
                if self.lifecycle_state(self.map_server_state, 'MAP_LOAD_FAILED') == 'inactive':
                    self.change(self.map_server_change, Transition.TRANSITION_CLEANUP,
                                'MAP_LOAD_FAILED')
            elif amcl_label == 'active':
                self.manage_localization(ManageLifecycleNodes.Request.RESET)
            elif amcl_label != 'unconfigured':
                raise MapFailure('LOCALIZATION_UNAVAILABLE', f'AMCL is {amcl_label}')
            self.set_map_server_yaml(os.path.join(self.map_dir(map_id), 'map.yaml'))
            self.set_initial_estimate(map_id)
            self.set_state(MapState.LOADING, MapState.MODE_LOCALIZATION,
                           f"loading map '{map_id}'")
            self.manage_localization(ManageLifecycleNodes.Request.STARTUP)
            # AMCL recreates these services during configure. Recreate clients too,
            # avoiding stale discovery handles across map lifecycle resets.
            self.destroy_client(self.nomotion_update)
            self.destroy_client(self.global_localization)
            self.nomotion_update = self.create_client(Empty, '/request_nomotion_update',
                                                       callback_group=self.amcl_state.callback_group)
            self.global_localization = self.create_client(Empty, '/reinitialize_global_localization',
                                                           callback_group=self.amcl_state.callback_group)
            self.wait_ready(self.map_ready, self.map_wait_timeout, 'MAP_LOAD_FAILED',
                            f"map_server did not publish map '{map_id}'")
            with self.lock:
                self.active_map_id = map_id
            self.set_state(MapState.ACTIVE, MapState.MODE_LOCALIZATION,
                           f"map '{map_id}' active; waiting for localization")
            self.wait_ready(self.localization_ready_event, self.localization_wait_timeout,
                            'LOCALIZATION_NOT_READY',
                            f"AMCL did not localize on map '{map_id}'")
            if self.core_paused:
                self.manage_core(ManageLifecycleNodes.Request.RESUME)
                self.core_paused = False
                self.wait_ready(self.localization_ready_event, self.localization_wait_timeout,
                                'LOCALIZATION_NOT_READY', 'localization lost during Nav2 resume')
            self.persist_active()
            self.navigation_hold(False)
            with self.lock:
                self.target_map_id = ''
                self.target_map_info = None
            return info
        except MapFailure:
            with self.lock:
                self.target_map_id = ''
                self.target_map_info = None
            raise

    def activate(self, map_id, admitted=False):
        previous = self.active_map_id
        try:
            return self.activate_once(map_id, admitted=admitted)
        except MapFailure as original:
            if previous and previous != map_id:
                self.get_logger().error(
                    f"activation of '{map_id}' failed; restoring '{previous}': {original.detail}")
                try:
                    info = self.activate_once(previous, admitted=True)
                    self.set_state(MapState.ACTIVE, MapState.MODE_LOCALIZATION,
                                   f"map switch failed; restored '{previous}'", original.code)
                    return info
                except MapFailure as rollback:
                    original = MapFailure(
                        'MAP_ROLLBACK_FAILED',
                        f"map switch failed ({original.detail}); rollback failed ({rollback.detail})")
            self.set_state(MapState.ERROR, MapState.MODE_LOCALIZATION,
                           original.detail, original.code)
            raise original

    def set_map_server_yaml(self, path):
        parameter = Parameter(name='yaml_filename', value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=path))
        response = self.call(self.map_server_params,
                             SetParameters.Request(parameters=[parameter]), 'MAP_LOAD_FAILED')
        if not response.results or not response.results[0].successful:
            raise MapFailure('MAP_LOAD_FAILED', 'map_server rejected yaml_filename update')

    @staticmethod
    def yaw_of(rotation):
        return math.atan2(2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                          1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))

    @staticmethod
    def compose_planar(map_to_odom, odom_pose):
        x, y, yaw = map_to_odom
        ox, oy, oyaw = odom_pose
        return (x + math.cos(yaw) * ox - math.sin(yaw) * oy,
                y + math.sin(yaw) * ox + math.cos(yaw) * oy,
                math.atan2(math.sin(yaw + oyaw), math.cos(yaw + oyaw)))

    def set_initial_estimate(self, map_id):
        try:
            with open(os.path.join(self.map_dir(map_id), 'metadata.yaml'), encoding='utf-8') as stream:
                metadata = yaml.safe_load(stream) or {}
        except FileNotFoundError:
            metadata = {}  # imported legacy maps use the configured odometry origin
        alignment = [float(value) for value in metadata.get('map_to_odom', [0.0, 0.0, 0.0])]
        try:
            odom = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        except Exception as exc:
            raise MapFailure('LOCALIZATION_UNAVAILABLE', f'cannot estimate robot pose from odometry: {exc}') from exc
        x, y, yaw = self.compose_planar(
            alignment, (odom.transform.translation.x, odom.transform.translation.y,
                        self.yaw_of(odom.transform.rotation)))
        parameters = [Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))
            for name, value in (('initial_pose.x', x), ('initial_pose.y', y),
                                ('initial_pose.z', 0.0), ('initial_pose.yaw', yaw))]
        response = self.call(self.amcl_params, SetParameters.Request(parameters=parameters),
                             'LOCALIZATION_UNAVAILABLE')
        if len(response.results) != len(parameters) or not all(item.successful for item in response.results):
            raise MapFailure('LOCALIZATION_UNAVAILABLE', 'AMCL rejected the map-specific initial pose')

    def start_mapping(self, resume_unsaved=False):
        """Initialize SLAM in the worker and commit MAPPING only after grid and safety readiness."""
        self.get_logger().info(f'[MAPPING] SLAM initialization started resume_unsaved={resume_unsaved}')
        self.navigation_hold(True)
        self.manage_localization(ManageLifecycleNodes.Request.PAUSE)
        if self.lifecycle_state(self.amcl_state, 'MAPPING_FAILED') == 'active':
            raise MapFailure('MAPPING_FAILED', 'AMCL remains active; refusing conflicting SLAM TF')
        # Inactive transient-local publishers retain the previous saved map. Destroy
        # that publisher before SLAM starts so the map saver cannot consume its cache.
        if self.lifecycle_state(self.amcl_state, 'MAPPING_FAILED') == 'inactive':
            self.change(self.amcl_change, Transition.TRANSITION_CLEANUP, 'MAPPING_FAILED')
        if self.lifecycle_state(self.map_server_state, 'MAPPING_FAILED') == 'inactive':
            self.change(self.map_server_change, Transition.TRANSITION_CLEANUP, 'MAPPING_FAILED')
        with self.lock:
            self.map_ready.clear()
            self.mapping_signature = None
            self.mapping_grid = None
            self.saved_mapping_signature = None
            self.set_state(MapState.SWITCHING, MapState.MODE_MAPPING, 'initializing SLAM mapping')
        self.slam_start()
        if resume_unsaved:
            request = DeserializePoseGraph.Request(
                filename=self.session_prefix,
                match_type=DeserializePoseGraph.Request.START_AT_FIRST_NODE)
            result = self.call(
                self.slam_deserialize, request, 'MAPPING_RECOVERY_FAILED', timeout=60.0)
            if result.result != DeserializePoseGraph.Response.RESULT_SUCCESS:
                raise MapFailure('MAPPING_RECOVERY_FAILED',
                                 f'slam_toolbox deserialize failed with result {result.result}')
        self.wait_ready(self.map_ready, self.map_wait_timeout, 'MAPPING_FAILED',
                        'slam_toolbox did not publish an occupancy grid')
        with self.lock:
            self.active_map_id = ''
            self.localization_ready = False
            self.mapping_unsaved = True
            self.mapping_saved_id = ''
        result = self.call(self.safety_hold, SetBool.Request(data=False), 'MAP_TRANSITION')
        if not result.success:
            raise MapFailure('MAP_TRANSITION', result.message)
        self.motion_ready.clear()
        self.wait_ready(self.motion_ready, self.map_wait_timeout, 'MAPPING_FAILED',
                        'manual velocity safety pipeline did not become ready')
        self.set_state(MapState.MAPPING, MapState.MODE_MAPPING, 'SLAM mapping active')
        self.get_logger().info('[MAPPING] SLAM initialization completed')

    def stop_mapping(self):
        self.navigation_hold(True)
        self.slam_stop()
        restore = self.previous_map_id or self.default_map_id
        with self.lock:
            self.mode = MapState.MODE_LOCALIZATION  # activate() refuses mapping mode
            self.publish_state()
        self.activate(restore, admitted=True)
        self.mapping_unsaved = False
        self.clear_mapping_session()

    def save_map(self, map_id, display_name, overwrite):
        temp = ''
        backup = ''
        target = self.map_dir(map_id)
        with self.lock:
            saving_signature = self.mapping_signature
            grid = self.mapping_grid
            if grid is None or time.monotonic() - self.mapping_last_rx > 5.0:
                raise MapFailure('MAP_SAVE_FAILED', 'no fresh SLAM occupancy grid is available')
        try:
            if os.path.exists(target) and not overwrite:
                raise MapFailure('MAP_ALREADY_EXISTS', f"map '{map_id}' already exists")
            temp = os.path.join(self.maps_dir, f'.tmp-{map_id}-{uuid.uuid4().hex}')
            os.mkdir(temp)
            self.write_grid_map(temp, grid)
            self.write_default_walls(temp, map_id)
            now = iso_now()
            created = now
            if os.path.exists(target):
                try:
                    with open(os.path.join(target, 'metadata.yaml'), encoding='utf-8') as stream:
                        created = (yaml.safe_load(stream) or {}).get('created_at', now)
                    shutil.copy2(os.path.join(target, 'virtual_walls.yaml'),
                                 os.path.join(temp, 'virtual_walls.yaml'))
                except OSError:
                    pass
                try:
                    shutil.copy2(os.path.join(target, 'locations.json'),
                                 os.path.join(temp, 'locations.json'))
                except OSError:
                    pass
            self.write_metadata(temp, map_id, display_name or map_id, created, now)
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            metadata_path = os.path.join(temp, 'metadata.yaml')
            with open(metadata_path, encoding='utf-8') as stream:
                metadata = yaml.safe_load(stream)
            metadata['map_to_odom'] = [transform.transform.translation.x,
                                       transform.transform.translation.y,
                                       self.yaw_of(transform.transform.rotation)]
            with open(metadata_path, 'w', encoding='utf-8') as stream:
                yaml.safe_dump(metadata, stream, sort_keys=False)
            self.validate_dir(temp, expected_id=map_id)
            if os.path.exists(target):
                backup = os.path.join(self.maps_dir, f'.old-{map_id}-{uuid.uuid4().hex}')
                os.replace(target, backup)
            os.replace(temp, target)
            temp = ''
            if backup:
                shutil.rmtree(backup)
                backup = ''
            with self.lock:
                self.mapping_saved_id = map_id
                self.saved_mapping_signature = saving_signature
                self.mapping_unsaved = self.mapping_signature != saving_signature
                if not self.mapping_unsaved:
                    self.clear_mapping_session()
            self.set_state(MapState.MAPPING, MapState.MODE_MAPPING,
                           f"mapping active; saved as '{map_id}'")
        except MapFailure:
            raise
        except Exception as exc:
            raise MapFailure('MAP_SAVE_FAILED', str(exc)) from exc
        finally:
            if temp:
                shutil.rmtree(temp, ignore_errors=True)
            if backup and not os.path.exists(target):
                os.replace(backup, target)

    @staticmethod
    def write_grid_map(directory, grid):
        """Persist the actual SLAM snapshot in Nav2's standard trinary raster format."""
        width, height = int(grid.info.width), int(grid.info.height)
        if width <= 0 or height <= 0 or width * height != len(grid.data):
            raise MapFailure('MAP_SAVE_FAILED', 'invalid occupancy grid dimensions')
        pixels = bytearray(width * height)
        for row in range(height):
            for column in range(width):
                value = int(grid.data[row * width + column])
                if value < -1 or value > 100:
                    raise MapFailure('MAP_SAVE_FAILED', 'occupancy value is outside -1..100')
                # Nav2's image coordinates start at the top; ROS grids start at the bottom.
                pixels[(height - row - 1) * width + column] = (
                    254 if 0 <= value <= 19 else 0 if value >= 65 else 205)
        Image.frombytes('L', (width, height), bytes(pixels)).save(os.path.join(directory, 'map.pgm'))
        origin = grid.info.origin
        metadata = {'image': 'map.pgm', 'mode': 'trinary',
                    'resolution': float(grid.info.resolution),
                    'origin': [float(origin.position.x), float(origin.position.y),
                               MapManager.yaw_of(origin.orientation)],
                    'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}
        with open(os.path.join(directory, 'map.yaml'), 'w', encoding='utf-8') as stream:
            yaml.safe_dump(metadata, stream, sort_keys=False)

    # ---------- services
    def srv_bootstrap(self, _request, response):
        response.success = self.bootstrap_ok
        response.message = self.detail
        return response

    def srv_list(self, _request, response):
        try:
            response.maps = self.scan_maps()
            response.success, response.error_code, response.message = True, '', 'ok'
        except MapFailure as exc:
            response.success, response.error_code, response.message = False, exc.code, exc.detail
        return response

    def srv_activate(self, request, response):
        def prepare():
            self.validate_id(request.map_id)
            self.validate_dir(self.map_dir(request.map_id), expected_id=request.map_id)
            if self.mode == MapState.MODE_MAPPING:
                raise MapFailure('MAPPING_ACTIVE', 'stop mapping before activating a saved map')
            with self.lock:
                self.localization_ready = False
                self.navigation_ready = False

        def failed(exc):
            self.set_state(MapState.ERROR, MapState.MODE_LOCALIZATION, exc.detail, exc.code)

        return self.admit_transition(
            response, prepare, MapState.SWITCHING, MapState.MODE_LOCALIZATION,
            f"switching to map '{request.map_id}'",
            lambda: self.activate(request.map_id, admitted=True), failed, 'map switch accepted')

    def srv_rename(self, request, response):
        if not self.transition_lock.acquire(blocking=False):
            response.success, response.error_code = False, 'MAP_TRANSITION'
            response.message = 'another map transition is active'
            return response
        try:
            self.validate_id(request.map_id)
            name = request.display_name.strip()
            if not name or len(name) > 128 or any(ord(char) < 32 for char in name):
                raise MapFailure('MAP_NAME_INVALID', 'display_name must be 1..128 printable characters')
            directory = self.map_dir(request.map_id)
            if os.path.islink(directory) or not os.path.isdir(directory):
                raise MapFailure('MAP_NOT_FOUND', f"map '{request.map_id}' does not exist")
            self.validate_dir(directory, expected_id=request.map_id)
            created = now = iso_now()
            alignment = None
            try:
                with open(os.path.join(directory, 'metadata.yaml'), encoding='utf-8') as stream:
                    metadata = yaml.safe_load(stream) or {}
                    created = metadata.get('created_at', now)
                    alignment = metadata.get('map_to_odom')
            except OSError:
                pass
            # Write beside the target and swap in, so a crash never leaves a torn metadata file.
            staging = os.path.join(directory, f'.metadata-{uuid.uuid4().hex}')
            os.mkdir(staging)
            try:
                self.write_metadata(staging, request.map_id, name, created, now, alignment)
                os.replace(os.path.join(staging, 'metadata.yaml'),
                           os.path.join(directory, 'metadata.yaml'))
            finally:
                shutil.rmtree(staging, ignore_errors=True)
            response.map = self.validate_dir(directory, expected_id=request.map_id)
            response.success, response.error_code, response.message = True, '', 'map renamed'
        except MapFailure as exc:
            response.success, response.error_code, response.message = False, exc.code, exc.detail
        except OSError as exc:
            response.success, response.error_code = False, 'MAP_RENAME_FAILED'
            response.message = str(exc)
        finally:
            self.transition_lock.release()
        return response

    def srv_delete(self, request, response):
        if not self.transition_lock.acquire(blocking=False):
            response.success, response.error_code = False, 'MAP_TRANSITION'
            response.message = 'another map transition is active'
            return response
        try:
            self.validate_id(request.map_id)
            if request.map_id == self.active_map_id:
                raise MapFailure('CANNOT_DELETE_ACTIVE_MAP', 'cannot delete the active map')
            if self.mode == MapState.MODE_MAPPING and request.map_id == self.previous_map_id:
                raise MapFailure('MAP_IN_USE', 'map is restored when mapping stops')
            directory = self.map_dir(request.map_id)
            if os.path.islink(directory) or not os.path.isdir(directory):
                raise MapFailure('MAP_NOT_FOUND', f"map '{request.map_id}' does not exist")
            if os.path.commonpath((self.maps_dir, os.path.realpath(directory))) != self.maps_dir:
                raise MapFailure('MAP_METADATA_INVALID', 'map directory escapes maps_dir')
            # Rename first (atomic, hidden from scan_maps), then remove.
            doomed = os.path.join(self.maps_dir, f'.old-{request.map_id}-{uuid.uuid4().hex}')
            os.replace(directory, doomed)
            shutil.rmtree(doomed, ignore_errors=True)
            response.success, response.error_code, response.message = True, '', 'map deleted'
        except MapFailure as exc:
            response.success, response.error_code, response.message = False, exc.code, exc.detail
        except OSError as exc:
            response.success, response.error_code = False, 'MAP_DELETE_FAILED'
            response.message = str(exc)
        finally:
            self.transition_lock.release()
        return response

    def srv_legacy_switch(self, request, response):
        wrapper_request = ActivateMap.Request(map_id=request.map_id)
        wrapper_response = self.srv_activate(wrapper_request, ActivateMap.Response())
        response.success = wrapper_response.success
        response.error_code = wrapper_response.error_code
        response.message = wrapper_response.message
        return response

    def srv_start_mapping(self, request, response):
        """Admit explicit recovery decisions promptly and preserve checkpoints on rejection."""
        if request.resume_unsaved and request.discard_unsaved:
            # Invalid input must not become a busy-transition error or acquire ownership.
            response.success, response.error_code = False, 'INVALID_ARGUMENT'
            response.message = 'resume_unsaved and discard_unsaved are mutually exclusive'
            response.state = self.state_msg()
            self.get_logger().info('[MAPPING] Transition rejected code=INVALID_ARGUMENT')
            return response

        def prepare():
            # Validate before starting the worker or deleting any recoverable data.
            if self.mode == MapState.MODE_MAPPING:
                raise MapFailure('MAPPING_ACTIVE', 'mapping is already active')
            self.recoverable_session = self.detect_recoverable_session()
            if (os.path.exists(self.session_dir) and not self.recoverable_session
                    and not request.discard_unsaved):
                raise MapFailure('MAPPING_RECOVERY_CORRUPT',
                                 'mapping checkpoint is invalid; explicitly discard it to start fresh')
            if request.resume_unsaved and not self.recoverable_session:
                raise MapFailure('NO_RECOVERABLE_SESSION', 'no recoverable mapping session exists')
            if self.recoverable_session and not (request.resume_unsaved or request.discard_unsaved):
                raise MapFailure('RECOVERABLE_SESSION_EXISTS',
                                 'Resume or discard the recoverable mapping session')
            self.previous_map_id = self.active_map_id

        def operation():
            # An explicit discard may involve slow filesystem work; keep it off the service callback.
            if request.discard_unsaved:
                self.clear_mapping_session()
            self.start_mapping(request.resume_unsaved)

        def failed(exc):
            # Preserve the checkpoint and expose startup failure through authoritative state.
            self.get_logger().error(f'[MAPPING] SLAM initialization failed code={exc.code} detail={exc.detail}')
            # Startup failures must leave SLAM inactive before attempting localization.
            try:
                self.slam_stop()
            except MapFailure as cleanup:
                self.get_logger().error(f'cannot stop failed SLAM session: {cleanup.detail}')
            self.set_state(MapState.ERROR, error_code=exc.code, detail=exc.detail)

        result = self.admit_transition(
            response, prepare, MapState.SWITCHING, None, 'leaving localization mode',
            operation, failed,
            'start mapping accepted')
        self.get_logger().info(
            f'[MAPPING] Transition accepted={result.success} code={result.error_code} '
            f'resume_unsaved={request.resume_unsaved} discard_unsaved={request.discard_unsaved}')
        return result

    def srv_stop_mapping(self, request, response):
        def prepare():
            if self.mode != MapState.MODE_MAPPING:
                raise MapFailure('NOT_MAPPING', 'mapping is not active')
            if self.mapping_unsaved and not request.discard_unsaved:
                raise MapFailure('UNSAVED_MAP',
                                 'mapping has unsaved changes; save or explicitly discard')

        def failed(exc):
            self.set_state(MapState.ERROR, error_code=exc.code, detail=exc.detail)

        return self.admit_transition(
            response, prepare, MapState.SWITCHING, None, 'stopping SLAM mapping',
            self.stop_mapping, failed, 'stop mapping accepted')

    def srv_save(self, request, response):
        def prepare():
            if self.mode != MapState.MODE_MAPPING or self.state != MapState.MAPPING:
                raise MapFailure('NOT_MAPPING', 'map saving requires active mapping')
            self.validate_id(request.map_id)
            target = self.map_dir(request.map_id)
            if os.path.exists(target) and not request.overwrite:
                raise MapFailure('MAP_ALREADY_EXISTS',
                                 f"map '{request.map_id}' already exists; overwrite is false")

        def failed(exc):
            self.set_state(MapState.MAPPING if self.mode == MapState.MODE_MAPPING else MapState.ERROR,
                           error_code=exc.code, detail=exc.detail)

        return self.admit_transition(
            response, prepare, MapState.SAVING, None, f"saving map '{request.map_id}'",
            lambda: self.save_map(request.map_id, request.display_name, request.overwrite),
            failed, 'save map accepted')


def main():
    rclpy.init()
    node = MapManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.2)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.shutting_down = True
        if node.transition_thread is not None and node.transition_thread.is_alive():
            node.get_logger().warning('waiting for active map transition before shutdown')
        while True:
            worker = node.transition_thread
            if worker is None or not worker.is_alive():
                break
            while worker.is_alive():
                if rclpy.ok():
                    executor.spin_once(timeout_sec=0.1)
                worker.join(timeout=0.1)
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
