#!/usr/bin/python3
"""Authoritative writable-map lifecycle for localization and SLAM."""

import math
import os
import re
import shutil
import struct
import threading
import time
import uuid
from datetime import datetime, timezone

import rclpy
import yaml
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import MapInfo, MapState
from robot_interfaces.srv import (ActivateMap, DeleteMap, ListMaps, RenameMap, SaveMap, StartMapping,
                                  StopMapping, SwitchMap)
from slam_toolbox.srv import SaveMap as SlamSaveMap
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


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
    with open(path, 'rb') as stream:
        head = stream.read(32)
        if head.startswith(b'\x89PNG\r\n\x1a\n') and len(head) >= 24:
            return struct.unpack('>II', head[16:24])
        if head[:2] in (b'P2', b'P5'):
            stream.seek(0)
            tokens = []
            while len(tokens) < 4:
                line = stream.readline()
                if not line:
                    break
                line = line.split(b'#', 1)[0]
                tokens.extend(line.split())
            if len(tokens) >= 3:
                return int(tokens[1]), int(tokens[2])
    raise ValueError('unsupported or malformed map image')


class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')
        raw_maps_dir = self.declare_parameter('maps_dir', '').value
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
        self.localization_generation = time.monotonic()
        self.mapping_unsaved = False
        self.mapping_saved_id = ''
        self.previous_map_id = ''
        self.restore_started = False
        self.bootstrap_pending = True

        cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub = self.create_publisher(MapState, '/robot/map/state', latched)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched,
                                 callback_group=cb)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, latched,
                                 callback_group=cb)
        self.create_subscription(Bool, '/robot/navigation/runtime_ready', self.on_nav_ready,
                                 latched, callback_group=cb)

        self.map_server_params = self.create_client(
            SetParameters, '/map_server/set_parameters', callback_group=cb)
        self.nav_enable = self.create_client(
            SetBool, '/robot/navigation/set_enabled', callback_group=cb)
        self.safety_hold = self.create_client(
            SetBool, '/robot/safety/transition_hold', callback_group=cb)
        self.cancel_nav = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal', callback_group=cb)
        self.localization_manager = self.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_localization/manage_nodes',
            callback_group=cb)
        self.slam_state = self.create_client(
            GetState, '/slam_toolbox/get_state', callback_group=cb)
        self.slam_change = self.create_client(
            ChangeState, '/slam_toolbox/change_state', callback_group=cb)
        self.amcl_state = self.create_client(GetState, '/amcl/get_state', callback_group=cb)
        self.amcl_change = self.create_client(
            ChangeState, '/amcl/change_state', callback_group=cb)
        self.slam_save = self.create_client(
            SlamSaveMap, '/slam_toolbox/save_map', callback_group=cb)

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
    def write_metadata(directory, map_id, display_name, created, updated):
        with open(os.path.join(directory, 'metadata.yaml'), 'w', encoding='utf-8') as stream:
            yaml.safe_dump({'format_version': FORMAT_VERSION, 'id': map_id,
                            'display_name': display_name or map_id,
                            'created_at': created, 'updated_at': updated}, stream,
                           sort_keys=False)

    def validate_id(self, map_id):
        if not MAP_ID.fullmatch(map_id or ''):
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
            with open(yaml_path, encoding='utf-8') as stream:
                nav = yaml.safe_load(stream) or {}
            image_value = nav['image']
            if os.path.isabs(image_value):
                raise ValueError('absolute image paths are not allowed')
            image_path = os.path.realpath(os.path.join(real_dir, image_value))
            if os.path.commonpath((real_dir, image_path)) != real_dir or not os.path.isfile(image_path):
                raise ValueError('referenced image is missing or escapes the map directory')
            resolution = float(nav['resolution'])
            origin_values = [float(value) for value in nav['origin']]
            if resolution <= 0.0 or len(origin_values) < 3:
                raise ValueError('invalid resolution or origin')
            width, height = image_dimensions(image_path)
            metadata_path = os.path.join(real_dir, 'metadata.yaml')
            metadata = {}
            if os.path.exists(metadata_path):
                with open(metadata_path, encoding='utf-8') as stream:
                    metadata = yaml.safe_load(stream) or {}
                if metadata.get('id', map_id) != map_id:
                    raise ValueError('metadata id does not match directory name')
                if int(metadata.get('format_version', FORMAT_VERSION)) != FORMAT_VERSION:
                    raise ValueError('unsupported metadata format_version')
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

    # ---------- state and async helpers
    def state_msg(self):
        msg = MapState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state, msg.mode = self.state, self.mode
        msg.active_map_id = self.active_map_id
        msg.map_available = bool(self.active_map_id and self.state != MapState.ERROR)
        msg.localization_ready = self.localization_ready
        msg.navigation_runtime_ready = self.navigation_ready
        msg.error_code, msg.detail = self.error_code, self.detail
        return msg

    def publish_state(self):
        with self.lock:
            self.state_pub.publish(self.state_msg())

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
            self.get_logger().exception('unexpected map transition failure')
            failure_state(MapFailure('MAP_TRANSITION_FAILED', str(exc)))
        finally:
            self.transition_thread = None
            self.transition_lock.release()

    def call(self, client, request, unavailable, timeout=30.0):
        if not client.wait_for_service(timeout_sec=5.0):
            raise MapFailure(unavailable, f'{client.srv_name} unavailable')
        done = threading.Event()
        holder = {}
        future = client.call_async(request)
        def complete(result):
            try:
                holder['result'] = result.result()
            except Exception as exc:  # callback records, service thread reports consistently
                holder['error'] = exc
            done.set()
        future.add_done_callback(complete)
        if not done.wait(timeout):
            future.cancel()
            raise MapFailure(unavailable, f'{client.srv_name} response timeout')
        if holder.get('result') is None:
            raise MapFailure(unavailable, f'{client.srv_name} call failed: {holder.get("error")}')
        return holder['result']

    def fire_set_bool(self, client, value):
        if client.service_is_ready():
            client.call_async(SetBool.Request(data=value))

    def navigation_hold(self, hold):
        self.call(self.nav_enable, SetBool.Request(data=not hold), 'NAVIGATION_UNAVAILABLE')
        if hold:
            self.call(self.safety_hold, SetBool.Request(data=True), 'MAP_TRANSITION')
            if self.cancel_nav.service_is_ready():
                self.cancel_nav.call_async(CancelGoal.Request())
        else:
            self.call(self.safety_hold, SetBool.Request(data=False), 'MAP_TRANSITION')

    def manage_localization(self, command):
        response = self.call(
            self.localization_manager, ManageLifecycleNodes.Request(command=command),
            'LOCALIZATION_UNAVAILABLE', timeout=45.0)
        if not response.success:
            raise MapFailure('LOCALIZATION_UNAVAILABLE',
                             f'localization lifecycle command {command} rejected')

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
    def on_nav_ready(self, msg):
        with self.lock:
            changed = self.navigation_ready != bool(msg.data)
            self.navigation_ready = bool(msg.data)
            if changed:
                self.publish_state()

    def on_map(self, _msg):
        restore_id = ''
        with self.lock:
            if self.mode != MapState.MODE_LOCALIZATION or self.state != MapState.LOADING:
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
                self.activate(map_id)
        except MapFailure as exc:
            self.set_state(MapState.ERROR, MapState.MODE_LOCALIZATION,
                           exc.detail, exc.code)

    def on_amcl(self, _msg):
        now = time.monotonic()
        with self.lock:
            self.last_amcl_rx = now
            if (self.mode != MapState.MODE_LOCALIZATION or self.state != MapState.ACTIVE
                    or now < self.localization_generation or self.localization_ready):
                return
            self.localization_ready = True
            self.detail = f"map '{self.active_map_id}' and localization active"
            self.publish_state()
        self.fire_set_bool(self.safety_hold, False)
        self.fire_set_bool(self.nav_enable, True)

    # ---------- map transitions
    def activate(self, map_id, admitted=False):
        self.validate_id(map_id)
        info = self.validate_dir(self.map_dir(map_id), expected_id=map_id)
        if self.mode == MapState.MODE_MAPPING:
            raise MapFailure('MAPPING_ACTIVE', 'stop mapping before activating a saved map')
        self.navigation_hold(True)
        with self.lock:
            self.active_map_id = map_id
            self.localization_ready = False
            self.navigation_ready = False
            self.localization_generation = time.monotonic()
        if not admitted:
            self.set_state(MapState.SWITCHING, MapState.MODE_LOCALIZATION,
                           f"switching to map '{map_id}'")
        try:
            # Cycle map_server + amcl together through their lifecycle manager. Changing amcl
            # alone breaks its bond and the manager then takes map_server down mid-switch.
            self.manage_localization(ManageLifecycleNodes.Request.RESET)
            self.set_map_server_yaml(os.path.join(self.map_dir(map_id), 'map.yaml'))
            self.set_state(MapState.LOADING, MapState.MODE_LOCALIZATION,
                           f"loading map '{map_id}'")
            self.manage_localization(ManageLifecycleNodes.Request.STARTUP)
            self.persist_active()
            self.set_state(MapState.ACTIVE, MapState.MODE_LOCALIZATION,
                           f"map '{map_id}' active; localization invalidated")
            self.fire_set_bool(self.safety_hold, False)
            return info
        except MapFailure as exc:
            self.set_state(MapState.ERROR, MapState.MODE_LOCALIZATION,
                           exc.detail, exc.code)
            raise

    def set_map_server_yaml(self, path):
        parameter = Parameter(name='yaml_filename', value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=path))
        response = self.call(self.map_server_params,
                             SetParameters.Request(parameters=[parameter]), 'MAP_LOAD_FAILED')
        if not response.results or not response.results[0].successful:
            raise MapFailure('MAP_LOAD_FAILED', 'map_server rejected yaml_filename update')

    def start_mapping(self):
        self.navigation_hold(True)
        self.manage_localization(ManageLifecycleNodes.Request.PAUSE)
        self.slam_start()
        with self.lock:
            self.active_map_id = ''
            self.localization_ready = False
            self.mapping_unsaved = True
            self.mapping_saved_id = ''
        self.set_state(MapState.MAPPING, MapState.MODE_MAPPING, 'SLAM mapping active')
        self.fire_set_bool(self.safety_hold, False)

    def stop_mapping(self):
        self.navigation_hold(True)
        self.slam_stop()
        self.manage_localization(ManageLifecycleNodes.Request.RESUME)
        restore = self.previous_map_id or self.default_map_id
        with self.lock:
            self.mode = MapState.MODE_LOCALIZATION  # activate() refuses mapping mode
        self.activate(restore, admitted=True)
        self.mapping_unsaved = False

    def save_map(self, map_id, display_name, overwrite):
        temp = ''
        backup = ''
        target = self.map_dir(map_id)
        try:
            temp = os.path.join(self.maps_dir, f'.tmp-{map_id}-{uuid.uuid4().hex}')
            os.mkdir(temp)
            save_prefix = os.path.join(temp, 'map')
            save_request = SlamSaveMap.Request(name=String(data=save_prefix))
            result = self.call(self.slam_save, save_request, 'MAP_SAVE_FAILED', timeout=60.0)
            if result.result != SlamSaveMap.Response.RESULT_SUCCESS:
                raise MapFailure('MAP_SAVE_FAILED',
                                 f'slam_toolbox save failed with result {result.result}')
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
            self.validate_dir(temp, expected_id=map_id)
            if os.path.exists(target):
                backup = os.path.join(self.maps_dir, f'.old-{map_id}-{uuid.uuid4().hex}')
                os.replace(target, backup)
            os.replace(temp, target)
            temp = ''
            if backup:
                shutil.rmtree(backup)
                backup = ''
            self.mapping_unsaved = False
            self.mapping_saved_id = map_id
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
                self.active_map_id = request.map_id
                self.localization_ready = False
                self.navigation_ready = False
                self.localization_generation = time.monotonic()

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
            if not name:
                raise MapFailure('MAP_NAME_INVALID', 'display_name cannot be empty')
            directory = self.map_dir(request.map_id)
            if os.path.islink(directory) or not os.path.isdir(directory):
                raise MapFailure('MAP_NOT_FOUND', f"map '{request.map_id}' does not exist")
            self.validate_dir(directory, expected_id=request.map_id)
            created = now = iso_now()
            try:
                with open(os.path.join(directory, 'metadata.yaml'), encoding='utf-8') as stream:
                    created = (yaml.safe_load(stream) or {}).get('created_at', now)
            except OSError:
                pass
            # Write beside the target and swap in, so a crash never leaves a torn metadata file.
            staging = os.path.join(directory, f'.metadata-{uuid.uuid4().hex}')
            os.mkdir(staging)
            try:
                self.write_metadata(staging, request.map_id, name, created, now)
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

    def srv_start_mapping(self, _request, response):
        def prepare():
            if self.mode == MapState.MODE_MAPPING:
                raise MapFailure('MAPPING_ACTIVE', 'mapping is already active')
            self.previous_map_id = self.active_map_id

        def failed(exc):
            self.set_state(MapState.ERROR, error_code=exc.code, detail=exc.detail)

        return self.admit_transition(
            response, prepare, MapState.SWITCHING, None, 'leaving localization mode',
            self.start_mapping, failed, 'start mapping accepted')

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
