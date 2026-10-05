#!/usr/bin/env python3
"""ROS2 Gateway Node interfacing with external Android WebSocket clients.

Bridges WebSocket Protocol Version 1 <-> ROS2 topics, services, and actions.
"""

from __future__ import annotations

import argparse
import asyncio
import ipaddress
import json
import logging
import math
import os
import re
import signal
import sys
import time
import uuid
from collections import deque
from typing import Any, Dict, List, Optional, Set

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_action_status_default
from rclpy.utilities import remove_ros_args
from robot_interfaces.msg import DockingState, MapPoint as RosMapPoint, MapState, RobotBatteryState, RobotHealth, RobotTelemetry, SafetyState, VirtualWall as RosVirtualWall
from robot_interfaces.srv import ActivateMap, CheckMapPoint, ClearVirtualWalls, CreateVirtualWall, DeleteMap, DeleteVirtualWall, ListMaps, ListVirtualWalls, RenameMap, SaveMap, SetVirtualWallEnabled, StartMapping, StopMapping, UpdateVirtualWall
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from unique_identifier_msgs.msg import UUID as UuidMsg
import websockets
from websockets.asyncio.server import ServerConnection, serve

LOGGER = logging.getLogger("robot_gateway")
PROTOCOL_VERSION = 1
RECENT_LOG_CAPACITY = 500
SPIN_BATCH = 25
LOG_LEVEL_NAMES = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "ERROR"}  # FATAL -> ERROR (SDK has no FATAL)
MAP_TRANSITION_TIMEOUT = 120.0  # lifecycle cycles (RESET/STARTUP, SLAM save) can take tens of seconds
MAX_MESSAGE_SIZE_BYTES = 5_000_000  # 5 MB payload limit
NAV_GOAL_RESPONSE_TIMEOUT_S = 10.0
NAV_CANCEL_WAIT_S = 5.0  # bounded wait for a cancelled navigator to reach a terminal state
LOOP_LAG_PERIOD_S = 0.1
SLOW_RESPONSE_WARN_S = 1.0  # pending goal response age that triggers a diagnostic warning
METRIC_WINDOW_S = 1.0
METRIC_WINDOWS_KEPT = 120
ACTIVE_GOAL_STATUSES = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)
NAV_NONE, NAV_TO, NAV_THROUGH, NAV_MISSION = "NONE", "NAVIGATE_TO", "NAVIGATE_THROUGH", "MISSION_NAVIGATE"
NAV_ACTION_NAMES = {NAV_TO: "navigate_to_pose", NAV_THROUGH: "navigate_through_poses", NAV_MISSION: "navigate_to_pose"}
GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN", GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING", GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}
FUTURE_OK, FUTURE_TIMEOUT, FUTURE_EXCEPTION = "OK", "TIMEOUT", "EXCEPTION"


def _finite(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _finite(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite(v) for v in value]
    return value


def safe_dumps(obj, **kwargs) -> str:
    """json.dumps that emits null for NaN/Inf so strict JSON parsers (Kotlin) accept it."""
    return json.dumps(_finite(obj), allow_nan=False, **kwargs)


class NavSendResult:
    """Outcome of one navigation goal send; `code` is None when Nav2 accepted the goal."""

    def __init__(self, code=None, message="", handle=None, result_future=None):
        self.code = code
        self.message = message
        self.handle = handle
        self.result_future = result_future


class MissionFailure(Exception):
    """A mission step failed; the message becomes the mission_progress detail."""


MISSION_STEP_TYPES = ("NAVIGATE", "NAVIGATE_TO_LOCATION", "WAIT", "DOCK", "UNDOCK")
MAX_MISSION_STEPS = 100
MAX_WAIT_MILLIS = 3_600_000
DOCKING_TIMEOUT_S = 300.0
MISSION_DISCONNECT_GRACE_S = 10.0


def ros_time_to_dict(stamp) -> Dict[str, int]:
    return {"sec": stamp.sec, "nanosec": stamp.nanosec}


def is_valid_map_id(map_id: str) -> bool:
    if not map_id or not isinstance(map_id, str):
        return False
    if ".." in map_id or "/" in map_id or "\\" in map_id or "\0" in map_id:
        return False
    return bool(re.match(r"^[a-zA-Z0-9_\-]+$", map_id))


MAP_STATE_NAMES = {
    MapState.NO_MAP: "NO_MAP", MapState.LOADING: "LOADING", MapState.ACTIVE: "ACTIVE",
    MapState.MAPPING: "MAPPING", MapState.SAVING: "SAVING", MapState.SWITCHING: "SWITCHING",
    MapState.ERROR: "ERROR",
}
MAP_MODE_NAMES = {
    MapState.MODE_NONE: "NONE", MapState.MODE_LOCALIZATION: "LOCALIZATION",
    MapState.MODE_MAPPING: "MAPPING",
}


def map_info_to_dict(m) -> Dict[str, Any]:
    return {
        "id": m.id,
        "name": m.name,
        "isActive": m.active,
        "resolution": m.resolution,
        "width": m.width,
        "height": m.height,
    }


def ros_wall_to_dict(wall) -> Dict[str, Any]:
    pts = []
    for p in wall.points:
        pts.append({"xMeters": p.x, "yMeters": p.y})
    wall_type = "LINE" if wall.type == RosVirtualWall.TYPE_LINE else "POLYGON"
    return {
        "id": wall.id,
        "mapId": wall.map_id,
        "name": wall.name,
        "type": wall_type,
        "points": pts,
        "thicknessMeters": wall.thickness,
        "enabled": wall.enabled
    }


def dict_to_ros_wall(wall_dict: Dict[str, Any]) -> RosVirtualWall:
    wall = RosVirtualWall()
    wall.id = str(wall_dict.get("id", ""))
    wall.map_id = str(wall_dict.get("mapId", ""))
    wall.name = str(wall_dict.get("name", ""))
    wall_type_str = str(wall_dict.get("type", "LINE")).upper()
    wall.type = RosVirtualWall.TYPE_POLYGON if wall_type_str == "POLYGON" else RosVirtualWall.TYPE_LINE
    wall.thickness = float(wall_dict.get("thicknessMeters", 0.15))
    wall.enabled = bool(wall_dict.get("enabled", True))
    pts = []
    for p in wall_dict.get("points", []):
        mp = RosMapPoint()
        mp.x = float(p.get("xMeters", 0.0))
        mp.y = float(p.get("yMeters", 0.0))
        pts.append(mp)
    wall.points = pts
    return wall


class RobotGatewayNode(Node):
    def __init__(self, host: str, port: int, auth_token: str = ""):
        super().__init__("robot_gateway")
        self.host = host
        self.port = port
        self.auth_token = auth_token
        self.maps_dir = os.path.realpath(os.path.expanduser(
            self.declare_parameter("maps_dir", "~/.robot/maps").value))

        self.ws_clients: Set[ServerConnection] = set()
        self.authenticated_clients: Set[ServerConnection] = set()
        self.session_ids: Dict[ServerConnection, str] = {}
        self.control_owner: Optional[ServerConnection] = None
        self.last_command_times: Dict[ServerConnection, Dict[str, float]] = {}
        self.motion_limits_lock = asyncio.Lock()

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "cmd_vel_manual", 10)

        # Service Clients
        self.srv_manual_stop = self.create_client(Trigger, "/robot/manual/stop")
        self.srv_estop = self.create_client(Trigger, "/robot/safety/emergency_stop")
        self.srv_release_estop = self.create_client(Trigger, "/robot/safety/release_emergency_stop")
        self.srv_dock = self.create_client(Trigger, "/robot/dock")
        self.srv_undock = self.create_client(Trigger, "/robot/undock")
        self.srv_cancel_dock = self.create_client(Trigger, "/robot/dock/cancel")
        self.srv_smoother_get = self.create_client(GetParameters, "/velocity_smoother/get_parameters")
        self.srv_smoother_set = self.create_client(SetParameters, "/velocity_smoother/set_parameters")
        self.srv_controller_get = self.create_client(GetParameters, "/controller_server/get_parameters")
        self.srv_controller_set = self.create_client(SetParameters, "/controller_server/set_parameters")
        self.srv_list_maps = self.create_client(ListMaps, "/robot/maps/list")
        self.srv_switch_map = self.create_client(ActivateMap, "/robot/maps/activate")
        self.srv_save_map = self.create_client(SaveMap, "/robot/maps/save")
        self.srv_rename_map = self.create_client(RenameMap, "/robot/maps/rename")
        self.srv_delete_map = self.create_client(DeleteMap, "/robot/maps/delete")
        self.srv_start_mapping = self.create_client(StartMapping, "/robot/maps/start_mapping")
        self.srv_stop_mapping = self.create_client(StopMapping, "/robot/maps/stop_mapping")
        self.srv_list_walls = self.create_client(ListVirtualWalls, "/robot/virtual_walls/list")
        self.srv_create_wall = self.create_client(CreateVirtualWall, "/robot/virtual_walls/create")
        self.srv_update_wall = self.create_client(UpdateVirtualWall, "/robot/virtual_walls/update")
        self.srv_delete_wall = self.create_client(DeleteVirtualWall, "/robot/virtual_walls/delete")
        self.srv_enable_wall = self.create_client(SetVirtualWallEnabled, "/robot/virtual_walls/enable")
        self.srv_disable_wall = self.create_client(SetVirtualWallEnabled, "/robot/virtual_walls/disable")
        self.srv_check_point = self.create_client(CheckMapPoint, "/robot/virtual_walls/check_point")
        self.srv_clear_walls = self.create_client(ClearVirtualWalls, "/robot/virtual_walls/clear")

        # Action Client
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.nav_through_client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")
        self.cancel_nav_clients = [
            self.create_client(CancelGoal, "navigate_to_pose/_action/cancel_goal"),
            self.create_client(CancelGoal, "navigate_through_poses/_action/cancel_goal"),
        ]
        self.srv_bt_state = self.create_client(GetState, "/bt_navigator/get_state")
        self.active_navigation_type = NAV_NONE
        self.active_goal_handle = None
        self.active_goal_uuid = b""
        # Goals reported ACCEPTED/EXECUTING/CANCELING on each action's status topic, by uuid.
        # Includes goals sent from RViz or the CLI, not just through this gateway.
        self.status_active: Dict[str, Dict[bytes, int]] = {a: {} for a in set(NAV_ACTION_NAMES.values())}
        self.pending_nav: Dict[str, Dict[str, Any]] = {}  # request id -> in-flight goal send
        self.timed_out_goals: Dict[bytes, str] = {}  # uuid -> request id, for late-accept evidence
        self.goal_status_seen: Dict[bytes, float] = {}  # uuid -> monotonic first seen on a status topic
        self.runtime_ready: Optional[bool] = None
        self.runtime_ready_at = 0.0
        self.metric_window: Dict[str, float] = self._new_metric_window()
        self.metric_windows: deque = deque(maxlen=METRIC_WINDOWS_KEPT)
        self.mission_task: Optional[asyncio.Task] = None
        self.mission_id = ""
        self.mission_owner = None
        self.mission_orphan_task: Optional[asyncio.Task] = None

        # Subscriptions
        self.create_subscription(RobotTelemetry, "/robot/telemetry", self.on_telemetry, 10)
        self.create_subscription(RobotBatteryState, "/robot/battery/state", self.on_battery, latched_qos)
        self.create_subscription(SafetyState, "/robot/safety/state", self.on_safety, latched_qos)
        self.create_subscription(DockingState, "/robot/docking/state", self.on_docking, latched_qos)
        self.create_subscription(RobotHealth, "/robot/health", self.on_health, latched_qos)
        self.create_subscription(MapState, "/robot/map/state", self.on_map_state, latched_qos)
        self.create_subscription(Log, "/rosout", self.on_rosout, 100)
        self.create_subscription(Bool, "/robot/navigation/runtime_ready", self.on_runtime_ready, latched_qos)
        for action in self.status_active:
            self.create_subscription(
                GoalStatusArray, f"{action}/_action/status",
                lambda msg, a=action: self.on_nav_status(a, msg), qos_profile_action_status_default)

        self.last_telemetry = None
        self.last_battery = None
        self.last_safety = None
        self.last_docking = None
        self.last_health = None
        self.last_map_state = None
        self.recent_logs: deque = deque(maxlen=RECENT_LOG_CAPACITY)
        self.design_limits: Optional[Dict[str, float]] = None  # speed ceiling, read once from the nav params
        self.current_limits: Optional[Dict[str, float]] = None

    # ---- ROS Subscriptions Callbacks ----
    def on_telemetry(self, msg: RobotTelemetry):
        payload = {
            "poseValid": msg.pose_valid,
            "frame": msg.frame,
            "xMeters": msg.x,
            "yMeters": msg.y,
            "yawRadians": msg.yaw,
            "linearVelocityMps": msg.linear_velocity,
            "angularVelocityRadPerSec": msg.angular_velocity,
            "isMoving": msg.is_moving,
            "navigationState": msg.navigation_state,
            "distanceRemainingMeters": msg.distance_remaining if math.isfinite(msg.distance_remaining) else None,
            "navigationElapsedSeconds": msg.navigation_elapsed_s,
            "hasGoal": msg.has_goal,
            "navigationErrorCode": msg.navigation_error_code,
            "navigationErrorMessage": msg.navigation_error_msg,
            "localizationState": msg.localization_state,
            "localizationPositionStddevMeters": msg.localization_position_stddev_m,
            "localizationYawStddevRadians": msg.localization_yaw_stddev_rad,
            "safetyState": msg.safety_state,
            "emergencyStopped": msg.emergency_stopped,
            "collisionStopped": msg.collision_stopped,
            "controlSource": msg.control_source,
            "nearestObstacleDistanceMeters": msg.nearest_obstacle_distance,
            "nearestObstacleBearingRadians": msg.nearest_obstacle_bearing,
            "dockingState": msg.docking_state,
            "isDocked": msg.is_docked,
            "dockingTimeSeconds": msg.docking_time_s,
            "batteryState": msg.battery_state,
            "batteryPercentage": msg.battery_percentage,
            "isCharging": msg.charging,
            "batteryVoltageVolts": msg.battery_voltage,
            "batteryTemperatureCelsius": msg.battery_temperature,
            "activeMap": msg.active_map,
        }
        self.last_telemetry = payload
        self.broadcast_event("telemetry", payload)

    def on_battery(self, msg: RobotBatteryState):
        payload = {
            "state": msg.state,
            "percentage": msg.percentage,
            "isCharging": msg.is_charging,
            "voltageVolts": msg.voltage,
            "currentAmps": msg.current,
            "temperatureCelsius": msg.temperature,
            "dataAgeSeconds": msg.data_age_s,
            "detail": msg.detail,
        }
        self.last_battery = payload
        self.broadcast_event("battery_state", payload)

    def on_safety(self, msg: SafetyState):
        payload = {
            "state": msg.state,
            "emergencyStopActive": msg.emergency_stop_active,
            "motionInhibited": msg.motion_inhibited,
            "controlSource": msg.control_source,
            "detail": msg.detail,
            "deadManStopCount": msg.dead_man_stop_count,
        }
        self.last_safety = payload
        self.broadcast_event("safety_state", payload)

    def on_docking(self, msg: DockingState):
        payload = {
            "state": msg.state,
            "dockId": msg.dock_id,
            "isDocked": msg.is_docked,
            "isCharging": msg.is_charging,
            "dockingTimeSeconds": msg.docking_time_s,
            "numRetries": msg.num_retries,
            "errorCode": msg.error_code,
            "errorName": msg.error_name,
            "errorMsg": msg.error_msg,
            "detail": msg.detail,
        }
        self.last_docking = payload
        self.broadcast_event("docking_state", payload)

    def on_health(self, msg: RobotHealth):
        active_errs = []
        for err in msg.active_errors:
            active_errs.append({
                "code": err.code,
                "subsystem": err.subsystem,
                "severity": ["INFO", "WARN", "ERROR", "FATAL"][min(err.severity, 3)],
                "message": err.message,
                "recoverable": err.recoverable,
                "sourceCode": err.source_code,
            })
        subsystems = []
        for name, lvl in zip(msg.subsystems, msg.subsystem_levels):
            subsystems.append({"name": name, "level": lvl})

        payload = {
            "overall": msg.overall,
            "subsystems": subsystems,
            "activeErrors": active_errs,
        }
        self.last_health = payload
        self.broadcast_event("robot_health", payload)

    def on_rosout(self, msg: Log):
        if msg.level < 20:
            return
        self.recent_logs.append({
            "timestampEpochMs": msg.stamp.sec * 1000 + msg.stamp.nanosec // 1_000_000,
            "level": LOG_LEVEL_NAMES.get(msg.level, "INFO"),
            "logger": msg.name,
            "message": msg.msg,
        })

    def on_map_state(self, msg: MapState):
        payload = {
            "state": MAP_STATE_NAMES.get(msg.state, "UNKNOWN"),
            "mode": MAP_MODE_NAMES.get(msg.mode, "UNKNOWN"),
            "activeMapId": msg.active_map_id,
            "mapAvailable": msg.map_available,
            "localizationReady": msg.localization_ready,
            "navigationRuntimeReady": msg.navigation_runtime_ready,
            "errorCode": msg.error_code,
            "detail": msg.detail,
        }
        if payload == self.last_map_state:
            return
        self.last_map_state = payload
        self.broadcast_event("map_state", payload)

    def on_runtime_ready(self, msg: Bool):
        self.runtime_ready = bool(msg.data)
        self.runtime_ready_at = time.monotonic()

    def on_nav_status(self, action: str, msg: GoalStatusArray):
        now = time.monotonic()
        active: Dict[bytes, int] = {}
        for entry in msg.status_list:
            uuid_bytes = bytes(entry.goal_info.goal_id.uuid)
            if entry.status in ACTIVE_GOAL_STATUSES:
                active[uuid_bytes] = entry.status
            if uuid_bytes in self.timed_out_goals and entry.status in ACTIVE_GOAL_STATUSES:
                LOGGER.warning(
                    "LATE_ACCEPT request=%s goal_uuid=%s action=%s status=%s: Nav2 accepted a goal whose "
                    "response the gateway had already given up on",
                    self.timed_out_goals[uuid_bytes], uuid_bytes.hex(), action, GOAL_STATUS_NAMES.get(entry.status))
            if uuid_bytes not in self.goal_status_seen and any(
                    p["uuid"] == uuid_bytes for p in self.pending_nav.values()):
                self.goal_status_seen[uuid_bytes] = now
        self.status_active[action] = active

    # ---- ROS loop instrumentation (measure only) ----
    @staticmethod
    def _new_metric_window() -> Dict[str, float]:
        return {"start": time.monotonic(), "max_spin_gap": 0.0, "max_spin_batch": 0.0,
                "max_spin_once": 0.0, "max_loop_lag": 0.0}

    def record_metric(self, name: str, value: float):
        if value > self.metric_window[name]:
            self.metric_window[name] = value

    def roll_metric_window(self) -> Optional[Dict[str, float]]:
        """Close the current window once it is old enough; returns it when closed."""
        now = time.monotonic()
        if now - self.metric_window["start"] < METRIC_WINDOW_S:
            return None
        closed = dict(self.metric_window, end=now)
        self.metric_windows.append(closed)
        self.metric_window = self._new_metric_window()
        return closed

    def metrics_between(self, t0: float, t1: float) -> Dict[str, float]:
        """Worst loop figures in 1 s windows overlapping [t0, t1] (including the open window)."""
        windows = [w for w in self.metric_windows if w["end"] >= t0 and w["start"] <= t1]
        windows.append(dict(self.metric_window))
        return {k: round(max(w[k] for w in windows), 4)
                for k in ("max_spin_gap", "max_spin_batch", "max_spin_once", "max_loop_lag")}

    def broadcast_event(self, event_type: str, payload: Any):
        if not self.authenticated_clients:
            return
        msg = safe_dumps({
            "type": event_type,
            "protocolVersion": PROTOCOL_VERSION,
            "payload": payload,
        })
        asyncio.create_task(self._async_broadcast(msg))

    async def _async_broadcast(self, msg: str):
        for ws in list(self.authenticated_clients):
            try:
                await ws.send(msg)
            except Exception:
                pass

    # ---- Control Session Ownership ----
    def check_control_ownership(self, ws: ServerConnection) -> bool:
        if self.control_owner is None:
            self.control_owner = ws
            LOGGER.info("Control session acquired by session %s", self.session_ids.get(ws))
            if self._mission_running() and self.mission_owner is None:
                # An orphaned mission is adopted by the new controller; its own disconnect
                # then starts a fresh grace period.
                self.mission_owner = ws
                if self.mission_orphan_task is not None:
                    self.mission_orphan_task.cancel()
                    self.mission_orphan_task = None
                LOGGER.info("Mission %s adopted by session %s", self.mission_id, self.session_ids.get(ws))
            return True
        return self.control_owner == ws

    def release_control_ownership(self, ws: ServerConnection):
        if self.control_owner == ws:
            LOGGER.info("Control session released by session %s", self.session_ids.get(ws))
            self.control_owner = None

    # ---- WebSocket Handler ----
    async def handle_client(self, websocket: ServerConnection):
        remote = websocket.remote_address
        session_id = uuid.uuid4().hex[:8]
        self.session_ids[websocket] = session_id
        LOGGER.info("Client connected: %s (session: %s)", remote, session_id)
        self.ws_clients.add(websocket)

        try:
            async for message_text in websocket:
                if len(message_text.encode("utf-8")) > MAX_MESSAGE_SIZE_BYTES:
                    await self.send_error(websocket, None, "PAYLOAD_TOO_LARGE", f"Message payload exceeds {MAX_MESSAGE_SIZE_BYTES} bytes")
                    continue
                try:
                    await self.process_message(websocket, message_text)
                except websockets.ConnectionClosed:
                    raise
                except Exception as exc:
                    LOGGER.exception("Unhandled error processing message (session: %s)", session_id)
                    await self.send_error(websocket, None, "INTERNAL_ERROR", f"Gateway failed to process the command: {exc}")
        except Exception as exc:
            LOGGER.info("Client connection closed: %s (session: %s, reason: %s)", remote, session_id, exc)
        finally:
            if self._mission_running() and self.mission_owner is websocket:
                LOGGER.warning("Mission %s owner disconnected; cancelling in %.0fs unless control is retaken",
                               self.mission_id, MISSION_DISCONNECT_GRACE_S)
                self.mission_owner = None
                self.mission_orphan_task = asyncio.create_task(self._cancel_orphaned_mission(self.mission_task))
            self.release_control_ownership(websocket)
            self.ws_clients.discard(websocket)
            self.authenticated_clients.discard(websocket)
            self.session_ids.pop(websocket, None)
            self.last_command_times.pop(websocket, None)
            LOGGER.info("Client disconnected: %s (session: %s)", remote, session_id)

    async def process_message(self, ws: ServerConnection, text: str):
        try:
            data = json.loads(text)
        except Exception:
            await self.send_error(ws, None, "INVALID_JSON", "Message is not valid JSON")
            return

        msg_type = data.get("type")
        msg_id = data.get("id")
        protocol_ver = data.get("protocolVersion", 1)

        if protocol_ver != PROTOCOL_VERSION:
            await self.send_error(ws, msg_id, "PROTOCOL_VERSION_MISMATCH", f"Protocol version {PROTOCOL_VERSION} required", recoverable=False)
            return

        if msg_type == "ping":
            await ws.send(safe_dumps({"type": "pong", "id": msg_id, "protocolVersion": PROTOCOL_VERSION}))
            return

        # Rate limiting per category
        category = "velocity" if msg_type == "manual_velocity" else ("auth" if msg_type == "hello" else "command")
        min_interval = 0.02 if category == "velocity" else (0.05 if category == "command" else 0.5)
        now_ts = time.time()
        client_times = self.last_command_times.setdefault(ws, {})
        last_ts = client_times.get(category, 0.0)
        if now_ts - last_ts < min_interval:
            await self.send_error(ws, msg_id, "RATE_LIMIT_EXCEEDED", f"Rate limit exceeded for '{category}' operations")
            return
        client_times[category] = now_ts

        if msg_type == "hello":
            await self.handle_hello(ws, msg_id, data.get("payload", {}))
            return

        if ws not in self.authenticated_clients:
            await self.send_error(ws, msg_id, "UNAUTHENTICATED", "Handshake hello required before commands", recoverable=False)
            return

        # E-stop may be engaged by any authenticated client, but releasing it requires an
        # already-established control owner. Do not use check_control_ownership here: that
        # helper auto-acquires an unowned session as a side effect.
        if msg_type == "release_emergency_stop":
            if self.control_owner is None:
                await self.send_error(
                    ws, msg_id, "CONTROL_SESSION_REQUIRED",
                    "An active control owner is required to release emergency stop")
                return
            if self.control_owner is not ws:
                await self.send_error(
                    ws, msg_id, "CONTROL_SESSION_IN_USE",
                    "Control session currently held by another client")
                return

        # ---- Commands Requiring Control Ownership ----
        control_commands = {
            "manual_velocity", "navigate_to", "navigate_through", "cancel_navigation", "stop",
            "dock", "undock", "cancel_docking", "switch_map", "start_mapping", "stop_mapping",
            "save_map", "rename_map", "delete_map",
            "create_virtual_wall", "update_virtual_wall", "delete_virtual_wall",
            "set_virtual_wall_enabled", "clear_virtual_walls",
            "save_location", "rename_location", "delete_location", "navigate_to_location",
            "submit_mission", "cancel_mission", "update_motion_limits"
        }
        if msg_type in control_commands:
            if not self.check_control_ownership(ws):
                await self.send_error(
                    ws,
                    msg_id,
                    "CONTROL_SESSION_IN_USE",
                    f"Control session currently held by another client",
                    recoverable=True
                )
                return

        # ---- Commands ----
        if msg_type == "manual_velocity":
            await self.handle_manual_velocity(ws, msg_id, data.get("payload", {}))
        elif msg_type == "stop":
            await self.handle_stop(ws, msg_id)
        elif msg_type == "emergency_stop":
            await self.handle_trigger_service(ws, msg_id, self.srv_estop, "Emergency stop")
        elif msg_type == "release_emergency_stop":
            await self.handle_trigger_service(ws, msg_id, self.srv_release_estop, "Release emergency stop")
        elif msg_type == "dock":
            await self.handle_trigger_service(ws, msg_id, self.srv_dock, "Docking")
        elif msg_type == "undock":
            await self.handle_trigger_service(ws, msg_id, self.srv_undock, "Undocking")
        elif msg_type == "cancel_docking":
            await self.handle_trigger_service(ws, msg_id, self.srv_cancel_dock, "Cancel docking")
        elif msg_type == "navigate_to":
            await self.handle_navigate_to(ws, msg_id, data.get("payload", {}))
        elif msg_type == "navigate_through":
            await self.handle_navigate_through(ws, msg_id, data.get("payload", {}))
        elif msg_type == "cancel_navigation":
            await self.handle_cancel_navigation(ws, msg_id)
        elif msg_type == "list_maps":
            await self.handle_list_maps(ws, msg_id)
        elif msg_type == "switch_map":
            await self.handle_switch_map(ws, msg_id, data.get("payload", {}))
        elif msg_type == "start_mapping":
            await self.handle_start_mapping(ws, msg_id)
        elif msg_type == "stop_mapping":
            await self.handle_stop_mapping(ws, msg_id, data.get("payload", {}))
        elif msg_type == "save_map":
            await self.handle_save_map(ws, msg_id, data.get("payload", {}))
        elif msg_type == "rename_map":
            await self.handle_rename_map(ws, msg_id, data.get("payload", {}))
        elif msg_type == "delete_map":
            await self.handle_delete_map(ws, msg_id, data.get("payload", {}))
        elif msg_type == "list_virtual_walls":
            await self.handle_list_virtual_walls(ws, msg_id, data.get("payload", {}))
        elif msg_type == "create_virtual_wall":
            await self.handle_create_virtual_wall(ws, msg_id, data.get("payload", {}))
        elif msg_type == "update_virtual_wall":
            await self.handle_update_virtual_wall(ws, msg_id, data.get("payload", {}))
        elif msg_type == "delete_virtual_wall":
            await self.handle_delete_virtual_wall(ws, msg_id, data.get("payload", {}))
        elif msg_type == "set_virtual_wall_enabled":
            await self.handle_set_virtual_wall_enabled(ws, msg_id, data.get("payload", {}))
        elif msg_type == "clear_virtual_walls":
            await self.handle_clear_virtual_walls(ws, msg_id, data.get("payload", {}))
        elif msg_type == "list_saved_locations":
            await self.handle_list_saved_locations(ws, msg_id, data.get("payload", {}))
        elif msg_type == "save_location":
            await self.handle_save_location(ws, msg_id, data.get("payload", {}))
        elif msg_type == "rename_location":
            await self.handle_rename_location(ws, msg_id, data.get("payload", {}))
        elif msg_type == "delete_location":
            await self.handle_delete_location(ws, msg_id, data.get("payload", {}))
        elif msg_type == "navigate_to_location":
            await self.handle_navigate_to_location(ws, msg_id, data.get("payload", {}))
        elif msg_type == "check_map_point":
            await self.handle_check_map_point(ws, msg_id, data.get("payload", {}))
        elif msg_type == "submit_mission":
            await self.handle_submit_mission(ws, msg_id, data.get("payload", {}))
        elif msg_type == "cancel_mission":
            await self.handle_cancel_mission(ws, msg_id)
        elif msg_type == "update_motion_limits":
            await self.handle_update_motion_limits(ws, msg_id, data.get("payload", {}))
        elif msg_type == "request_state_snapshot":
            await self.handle_request_state_snapshot(ws, msg_id)
        elif msg_type == "get_recent_logs":
            await self.handle_get_recent_logs(ws, msg_id, data.get("payload", {}))
        else:
            await self.send_error(ws, msg_id, "UNKNOWN_COMMAND", f"Command type '{msg_type}' is unknown")

    async def handle_hello(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        token = payload.get("token")
        # Mask credentials in logs
        LOGGER.info("Received hello handshake with auth token: %s", "******" if token else "none")

        if self.auth_token and token != self.auth_token:
            await self.send_error(ws, msg_id, "INVALID_AUTHENTICATION", "Invalid authentication token", recoverable=False)
            return

        self.authenticated_clients.add(ws)
        session_id = self.session_ids.get(ws, "")
        LOGGER.info("Client authenticated successfully (session: %s)", session_id)

        response = {
            "type": "hello_ack",
            "id": msg_id,
            "protocolVersion": PROTOCOL_VERSION,
            "payload": {
                "gatewayVersion": "0.2.0",
                "capabilities": [
                    "navigation",
                    "manual_control",
                    "docking",
                    "safety",
                    "telemetry",
                    "battery",
                    "health",
                    "map_switching",
                    "map_management",
                    "mapping",
                    "virtual_walls",
                    "saved_locations",
                    "waypoint_navigation",
                    "missions",
                    "robot_configuration",
                    "diagnostics",
                    "log_retrieval"
                ],
                "sessionId": session_id
            }
        }
        await ws.send(safe_dumps(response))

        # Catch up client on last known state
        if self.last_telemetry:
            await ws.send(safe_dumps({"type": "telemetry", "protocolVersion": PROTOCOL_VERSION, "payload": self.last_telemetry}))
        if self.last_battery:
            await ws.send(safe_dumps({"type": "battery_state", "protocolVersion": PROTOCOL_VERSION, "payload": self.last_battery}))
        if self.last_safety:
            await ws.send(safe_dumps({"type": "safety_state", "protocolVersion": PROTOCOL_VERSION, "payload": self.last_safety}))
        if self.last_docking:
            await ws.send(safe_dumps({"type": "docking_state", "protocolVersion": PROTOCOL_VERSION, "payload": self.last_docking}))
        if self.last_health:
            await ws.send(safe_dumps({"type": "robot_health", "protocolVersion": PROTOCOL_VERSION, "payload": self.last_health}))
        if self.last_map_state:
            await ws.send(safe_dumps({"type": "map_state", "protocolVersion": PROTOCOL_VERSION, "payload": self.last_map_state}))

    async def handle_manual_velocity(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        if not isinstance(payload, dict):
            await self.send_error(ws, msg_id, "INVALID_ARGUMENT", "Manual velocity payload must be an object")
            return

        linear_value = payload.get("linearMps")
        angular_value = payload.get("angularRadPerSec")
        values = (linear_value, angular_value)
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in values):
            await self.send_error(
                ws, msg_id, "INVALID_ARGUMENT",
                "linearMps and angularRadPerSec must be numbers")
            return

        linear = float(linear_value)
        angular = float(angular_value)
        if not (math.isfinite(linear) and math.isfinite(angular)):
            await self.send_error(
                ws, msg_id, "INVALID_ARGUMENT",
                "linearMps and angularRadPerSec must be finite numbers")
            return

        error = None
        async with self.motion_limits_lock:
            if not await self._read_design_limits():
                error = (
                    "SERVICE_UNAVAILABLE",
                    "Velocity limits are unavailable; manual motion is disabled",
                )
            else:
                max_linear = self.current_limits["linear"]
                max_angular = self.current_limits["angular"]
                if abs(linear) > max_linear or abs(angular) > max_angular:
                    error = (
                        "VELOCITY_LIMIT_EXCEEDED",
                        f"Manual velocity exceeds the active limit "
                        f"({max_linear:.2f} m/s, {max_angular:.2f} rad/s)",
                    )
                else:
                    twist = TwistStamped()
                    twist.header.stamp = self.get_clock().now().to_msg()
                    twist.header.frame_id = "base_footprint"
                    twist.twist.linear.x = linear
                    twist.twist.angular.z = angular
                    self.cmd_vel_pub.publish(twist)

        if error is not None:
            await self.send_error(ws, msg_id, error[0], error[1])
            return

        await self.send_ack(ws, msg_id)

    async def handle_stop(self, ws: ServerConnection, msg_id: str | None):
        self._cancel_mission_if_running()
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_footprint"
        self.cmd_vel_pub.publish(twist)

        if self.srv_manual_stop.service_is_ready():
            req = Trigger.Request()
            self.srv_manual_stop.call_async(req)

        await self.send_ack(ws, msg_id)

    async def _await_future(self, future, timeout: float = 10.0):
        """Await an rclpy future from asyncio; None on timeout or exception."""
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                future.cancel()
                return None
            await asyncio.sleep(0.01)
        try:
            return future.result()
        except Exception:
            return None

    async def _await_future_outcome(self, future, timeout: float):
        """Like _await_future, but keeps timeout, exception and result distinct.

        Returns (FUTURE_OK, result, None), (FUTURE_TIMEOUT, None, None) or (FUTURE_EXCEPTION, None, exc).
        """
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                future.cancel()
                return FUTURE_TIMEOUT, None, None
            await asyncio.sleep(0.01)
        try:
            return FUTURE_OK, future.result(), None
        except Exception as exc:
            return FUTURE_EXCEPTION, None, exc

    async def handle_trigger_service(self, ws: ServerConnection, msg_id: str | None, client, name: str):
        if not client.service_is_ready():
            await self.send_error(ws, msg_id, "SERVICE_UNAVAILABLE", f"{name} service is unavailable")
            return

        req = Trigger.Request()
        future = client.call_async(req)

        res = await self._await_future(future)
        if res and res.success:
            await self.send_ack(ws, msg_id)
        else:
            msg = res.message if res else "Service call failed"
            await self.send_error(ws, msg_id, "SERVICE_FAILED", msg)

    def _pose_stamped(self, point: Dict[str, Any]) -> PoseStamped:
        yaw = float(point.get("yawRadians", 0.0))
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = Time().to_msg()  # zero stamp: use the latest available transform
        pose.pose.position.x = float(point.get("xMeters", 0.0))
        pose.pose.position.y = float(point.get("yMeters", 0.0))
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _believed_active(self) -> str:
        if self.active_navigation_type == NAV_NONE:
            return "NONE"
        return f"{self.active_navigation_type}:{self.active_goal_uuid.hex()}"

    def _bt_state_label(self, future) -> str:
        if future is None:
            return "unavailable"
        if not future.done():
            return "pending"
        try:
            return future.result().current_state.label
        except Exception as exc:
            return f"error:{exc!r}"

    async def _clear_other_navigator(self, nav_type: str) -> Optional[str]:
        """Cancel and wait out a goal on the other navigator; None when clear, else a failure reason.

        Nav2 rejects a goal for one navigator while the other is running (navigator preemption is
        off), so a command that replaces it has to end it first.
        """
        mine = NAV_ACTION_NAMES[nav_type]
        other = next(a for a in self.status_active if a != mine)
        if not self.status_active[other]:
            return None
        LOGGER.info("Navigator switch %s -> %s: cancelling %d active %s goal(s): %s", other, mine,
                    len(self.status_active[other]), other,
                    [u.hex() for u in self.status_active[other]])
        client = self.cancel_nav_clients[0 if other == "navigate_to_pose" else 1]
        if client.service_is_ready():
            await self._await_future_outcome(client.call_async(CancelGoal.Request()), NAV_CANCEL_WAIT_S)
        deadline = time.monotonic() + NAV_CANCEL_WAIT_S
        while self.status_active[other]:
            if time.monotonic() > deadline:
                return f"{other} still has active goal(s) {NAV_CANCEL_WAIT_S:.0f}s after cancel"
            await asyncio.sleep(0.02)
        return None

    async def _send_nav_goal_core(self, nav_type: str, client, goal_msg, request_id: str,
                                  goals: List[Dict[str, Any]], frame: str) -> NavSendResult:
        """Send one goal, keeping every failure mode distinct, and log the whole request."""
        action = NAV_ACTION_NAMES[nav_type]
        info: Dict[str, Any] = {
            "request": request_id, "action": action, "nav_type": nav_type, "frame": frame,
            "goals": goals, "server_is_ready": client.server_is_ready(),
            "runtime_ready": self.runtime_ready,
            "runtime_ready_age_s": round(time.monotonic() - self.runtime_ready_at, 2) if self.runtime_ready_at else None,
            "believed_active_before": self._believed_active(),
            "status_active_before": {a: [u.hex() for u in d] for a, d in self.status_active.items() if d},
        }

        def finish(code, message, **extra) -> NavSendResult:
            info.update(extra)
            info["result"] = code or "ACCEPTED"
            (LOGGER.info if code is None else LOGGER.warning)("nav_request %s", json.dumps(_finite(info), default=str))
            return NavSendResult(code, message)

        if not info["server_is_ready"]:
            return finish("ACTION_SERVER_UNAVAILABLE", f"Nav2 {action} action server unavailable", accepted=False)

        failure = await self._clear_other_navigator(nav_type)
        if failure:
            return finish("NAVIGATOR_CANCEL_TIMEOUT", failure, accepted=False)

        bt_future = self.srv_bt_state.call_async(GetState.Request()) if self.srv_bt_state.service_is_ready() else None
        goal_uuid = UuidMsg(uuid=list(uuid.uuid4().bytes))
        uuid_bytes = bytes(goal_uuid.uuid)
        info["goal_uuid"] = uuid_bytes.hex()
        t_send = time.monotonic()
        info["send_wall_time"] = time.time()
        self.pending_nav[request_id] = {"t_send": t_send, "uuid": uuid_bytes, "warned": False}
        try:
            try:
                future = client.send_goal_async(goal_msg, goal_uuid=goal_uuid)
            except Exception as exc:
                info["response_latency_s"] = round(time.monotonic() - t_send, 4)
                return finish("ACTION_GOAL_SEND_EXCEPTION", f"Sending the goal raised {exc!r}",
                              accepted=False, exception=repr(exc), bt_navigator_state=self._bt_state_label(bt_future))
            state, handle, exc = await self._await_future_outcome(future, NAV_GOAL_RESPONSE_TIMEOUT_S)
        finally:
            self.pending_nav.pop(request_id, None)
        t_resp = time.monotonic()
        info["response_wall_time"] = time.time()
        info["response_latency_s"] = round(t_resp - t_send, 4)
        info["bt_navigator_state"] = self._bt_state_label(bt_future)
        info["goal_uuid_seen_on_status_topic"] = uuid_bytes in self.goal_status_seen
        self.goal_status_seen.pop(uuid_bytes, None)
        info["loop_metrics"] = self.metrics_between(t_send, t_resp)

        if state == FUTURE_TIMEOUT:
            self.timed_out_goals[uuid_bytes] = request_id
            return finish("ACTION_GOAL_RESPONSE_TIMEOUT",
                          f"No goal response from {action} within {NAV_GOAL_RESPONSE_TIMEOUT_S:.0f}s", accepted=None)
        if state == FUTURE_EXCEPTION:
            return finish("ACTION_GOAL_SEND_EXCEPTION", f"Goal send failed: {exc!r}", accepted=None, exception=repr(exc))
        if not handle.accepted:
            return finish("NAV2_GOAL_REJECTED", f"Nav2 {action} returned accepted=false", accepted=False)

        result_future = handle.get_result_async()
        self.active_navigation_type = nav_type
        self.active_goal_handle = handle
        self.active_goal_uuid = uuid_bytes

        def on_done(done_future, handle=handle, request=request_id, started=t_send):
            try:
                result = done_future.result()
                code = getattr(result.result, "error_code", None)
                msg = getattr(result.result, "error_msg", "")
                LOGGER.info("nav_result request=%s goal_uuid=%s status=%s error_code=%s error_msg=%r duration_s=%.2f",
                            request, uuid_bytes.hex(), GOAL_STATUS_NAMES.get(result.status, result.status),
                            code, msg, time.monotonic() - started)
            except Exception as exc:
                LOGGER.warning("nav_result request=%s goal_uuid=%s result future failed: %r", request, uuid_bytes.hex(), exc)
            if self.active_goal_handle is handle:
                self.active_navigation_type = NAV_NONE
                self.active_goal_handle = None
                self.active_goal_uuid = b""

        result_future.add_done_callback(on_done)
        info["accepted"] = True
        finish(None, "")
        return NavSendResult(None, "", handle, result_future)

    async def _send_nav_goal(self, ws: ServerConnection, msg_id: str | None, nav_type: str, client,
                             goal_msg, goals: List[Dict[str, Any]], frame: str) -> None:
        # A manual command replaces a running mission: end it, and its Nav2 goal, first.
        await self._cancel_mission_and_wait()
        request_id = msg_id or f"gw_{uuid.uuid4().hex[:8]}"
        outcome = await self._send_nav_goal_core(nav_type, client, goal_msg, request_id, goals, frame)
        if outcome.code:
            await self.send_error(ws, msg_id, outcome.code, outcome.message)
            return
        await self.send_ack(ws, msg_id)

    def _goal_summary(self, point: Dict[str, Any]) -> Dict[str, Any]:
        return {"x": point.get("xMeters"), "y": point.get("yMeters"), "yaw": point.get("yawRadians")}

    async def handle_navigate_to(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._pose_stamped(payload)
        await self._send_nav_goal(ws, msg_id, NAV_TO, self.nav_client, goal_msg,
                                  [self._goal_summary(payload)], goal_msg.pose.header.frame_id)

    async def handle_navigate_through(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        waypoints_data = payload.get("waypoints", [])
        if not waypoints_data:
            await self.send_error(ws, msg_id, "INVALID_ARGUMENT", "Waypoints list cannot be empty")
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses.header.frame_id = "map"
        goal_msg.poses.goals = [self._pose_stamped(wp) for wp in waypoints_data]
        await self._send_nav_goal(ws, msg_id, NAV_THROUGH, self.nav_through_client, goal_msg,
                                  [self._goal_summary(wp) for wp in waypoints_data], "map")

    async def handle_cancel_navigation(self, ws: ServerConnection, msg_id: str | None):
        await self._cancel_mission_and_wait()
        # Zero goal id + zero stamp cancels every goal on the action, including ones
        # that were not sent through this gateway (RViz, other clients).
        ready = [c for c in self.cancel_nav_clients if c.service_is_ready()]
        if not ready:
            await self.send_error(ws, msg_id, "ACTION_SERVER_UNAVAILABLE", "Nav2 navigation action servers unavailable")
            return

        for client in ready:
            await self._await_future(client.call_async(CancelGoal.Request()), timeout=5.0)
        await self.send_ack(ws, msg_id)

    async def handle_list_maps(self, ws: ServerConnection, msg_id: str | None):
        if not self.srv_list_maps.service_is_ready():
            await self.send_error(ws, msg_id, "SERVICE_UNAVAILABLE", "ListMaps service is unavailable")
            return

        req = ListMaps.Request()
        future = self.srv_list_maps.call_async(req)

        res = await self._await_future(future)
        if res and res.success:
            maps_list = [map_info_to_dict(m) for m in res.maps]
            resp = {
                "type": "list_maps_ack",
                "id": msg_id,
                "protocolVersion": PROTOCOL_VERSION,
                "payload": {
                    "maps": maps_list
                }
            }
            await ws.send(safe_dumps(resp))
        else:
            msg = res.message if res else "List maps failed"
            await self.send_error(ws, msg_id, "MAP_LIST_FAILED", msg)

    async def handle_switch_map(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        map_id = payload.get("mapId", "")
        if not is_valid_map_id(map_id):
            await self.send_error(ws, msg_id, "INVALID_MAP_ID", "Map ID contains invalid characters or path traversal attempt")
            return

        if not self.srv_switch_map.service_is_ready():
            await self.send_error(ws, msg_id, "SERVICE_UNAVAILABLE", "ActivateMap service is unavailable")
            return

        req = ActivateMap.Request()
        req.map_id = map_id
        res = await self._await_future(self.srv_switch_map.call_async(req), timeout=MAP_TRANSITION_TIMEOUT)
        if res and res.success:
            await self.send_ack(ws, msg_id)
        else:
            msg = res.message if res else "Switch map failed"
            await self.send_error(ws, msg_id, (res.error_code if res else "") or "MAP_SWITCH_FAILED", msg)

    async def handle_save_map(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        name = payload.get("name", "").strip()
        if not name:
            await self.send_error(ws, msg_id, "INVALID_MAP_NAME", "Map name cannot be empty")
            return

        map_id = re.sub(r'[^a-zA-Z0-9_\-]', '_', name.lower())
        if not is_valid_map_id(map_id):
            await self.send_error(ws, msg_id, "INVALID_MAP_ID", "Derived map ID contains invalid characters")
            return

        if not self.srv_save_map.service_is_ready():
            await self.send_error(ws, msg_id, "SERVICE_UNAVAILABLE", "SaveMap service is unavailable")
            return

        req = SaveMap.Request()
        req.map_id = map_id
        req.display_name = name
        req.overwrite = True
        res = await self._await_future(self.srv_save_map.call_async(req), timeout=MAP_TRANSITION_TIMEOUT)
        if res and res.success:
            resp = {
                "type": "save_map_ack",
                "id": msg_id,
                "protocolVersion": PROTOCOL_VERSION,
                "payload": {
                    "map": map_info_to_dict(res.map)
                }
            }
            await ws.send(safe_dumps(resp))
        else:
            msg = res.message if res else "Save map failed"
            await self.send_error(ws, msg_id, (res.error_code if res else "") or "MAP_SAVE_FAILED", msg)

    async def handle_rename_map(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        map_id = payload.get("mapId", "")
        new_name = payload.get("newName", "").strip()
        if not is_valid_map_id(map_id):
            await self.send_error(ws, msg_id, "INVALID_MAP_ID", "Map ID contains invalid characters or path traversal attempt")
            return
        if not new_name:
            await self.send_error(ws, msg_id, "INVALID_MAP_NAME", "New map name cannot be empty")
            return

        if not self.srv_rename_map.service_is_ready():
            await self.send_error(ws, msg_id, "SERVICE_UNAVAILABLE", "RenameMap service is unavailable")
            return

        req = RenameMap.Request()
        req.map_id = map_id
        req.display_name = new_name
        res = await self._await_future(self.srv_rename_map.call_async(req))
        if res and res.success:
            resp = {
                "type": "rename_map_ack",
                "id": msg_id,
                "protocolVersion": PROTOCOL_VERSION,
                "payload": {
                    "map": map_info_to_dict(res.map)
                }
            }
            await ws.send(safe_dumps(resp))
        else:
            msg = res.message if res else "Rename map failed"
            await self.send_error(ws, msg_id, (res.error_code if res else "") or "MAP_RENAME_FAILED", msg)

    async def handle_delete_map(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        map_id = payload.get("mapId", "")
        if not is_valid_map_id(map_id):
            await self.send_error(ws, msg_id, "INVALID_MAP_ID", "Map ID contains invalid characters or path traversal attempt")
            return

        if not self.srv_delete_map.service_is_ready():
            await self.send_error(ws, msg_id, "SERVICE_UNAVAILABLE", "DeleteMap service is unavailable")
            return

        req = DeleteMap.Request()
        req.map_id = map_id
        res = await self._await_future(self.srv_delete_map.call_async(req))
        if res and res.success:
            await self.send_ack(ws, msg_id)
        else:
            msg = res.message if res else "Delete map failed"
            await self.send_error(ws, msg_id, (res.error_code if res else "") or "MAP_DELETE_FAILED", msg)

    async def handle_start_mapping(self, ws: ServerConnection, msg_id: str | None):
        if not self.srv_start_mapping.service_is_ready():
            await self.send_error(ws, msg_id, "SERVICE_UNAVAILABLE", "StartMapping service is unavailable")
            return

        res = await self._await_future(
            self.srv_start_mapping.call_async(StartMapping.Request()), timeout=MAP_TRANSITION_TIMEOUT)
        if res and res.success:
            await self.send_ack(ws, msg_id)
        else:
            msg = res.message if res else "Start mapping failed"
            await self.send_error(ws, msg_id, (res.error_code if res else "") or "MAPPING_START_FAILED", msg)

    async def handle_stop_mapping(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        if not self.srv_stop_mapping.service_is_ready():
            await self.send_error(ws, msg_id, "SERVICE_UNAVAILABLE", "StopMapping service is unavailable")
            return

        req = StopMapping.Request()
        req.discard_unsaved = bool(payload.get("discardUnsaved", False))
        res = await self._await_future(
            self.srv_stop_mapping.call_async(req), timeout=MAP_TRANSITION_TIMEOUT)
        if res and res.success:
            await self.send_ack(ws, msg_id)
        else:
            msg = res.message if res else "Stop mapping failed"
            await self.send_error(ws, msg_id, (res.error_code if res else "") or "MAPPING_STOP_FAILED", msg)

    async def handle_list_virtual_walls(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        map_id = payload.get("mapId", "")
        if not is_valid_map_id(map_id):
            await self.send_error(ws, msg_id, "INVALID_MAP_ID", "Map ID is invalid or contains path traversal attempt")
            return

        if map_id != self._active_map_id():
            await self.send_error(ws, msg_id, "MAP_NOT_ACTIVE", "Virtual walls are only available for the active map")
            return

        if not self.srv_list_walls.service_is_ready():
            await self.send_error(ws, msg_id, "SERVICE_UNAVAILABLE", "ListVirtualWalls service is unavailable")
            return

        req = ListVirtualWalls.Request()
        future = self.srv_list_walls.call_async(req)
        res = await self._await_future(future)
        if res and res.success:
            walls_list = [ros_wall_to_dict(w) for w in res.walls]
            resp = {
                "type": "list_virtual_walls_ack",
                "id": msg_id,
                "protocolVersion": PROTOCOL_VERSION,
                "payload": {
                    "mapId": map_id,
                    "walls": walls_list
                }
            }
            await ws.send(safe_dumps(resp))
        else:
            msg = res.message if res else "List virtual walls failed"
            await self.send_error(ws, msg_id, "WALL_LIST_FAILED", msg)

    async def handle_create_virtual_wall(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        wall_dict = payload.get("wall", {})
        wall_ros = dict_to_ros_wall(wall_dict)

        if not self.srv_create_wall.service_is_ready():
            await self.send_error(ws, msg_id, "SERVICE_UNAVAILABLE", "CreateVirtualWall service is unavailable")
            return

        req = CreateVirtualWall.Request()
        req.wall = wall_ros
        future = self.srv_create_wall.call_async(req)
        res = await self._await_future(future)
        if res and res.success:
            resp = {
                "type": "create_virtual_wall_ack",
                "id": msg_id,
                "protocolVersion": PROTOCOL_VERSION,
                "payload": {
                    "wall": ros_wall_to_dict(res.wall)
                }
            }
            await ws.send(safe_dumps(resp))
        else:
            msg = res.message if res else "Create virtual wall failed"
            await self.send_error(ws, msg_id, "WALL_CREATE_FAILED", msg)

    async def handle_update_virtual_wall(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        wall_dict = payload.get("wall", {})
        wall_ros = dict_to_ros_wall(wall_dict)

        if not self.srv_update_wall.service_is_ready():
            await self.send_error(ws, msg_id, "SERVICE_UNAVAILABLE", "UpdateVirtualWall service is unavailable")
            return

        req = UpdateVirtualWall.Request()
        req.wall = wall_ros
        future = self.srv_update_wall.call_async(req)
        res = await self._await_future(future)
        if res and res.success:
            resp = {
                "type": "update_virtual_wall_ack",
                "id": msg_id,
                "protocolVersion": PROTOCOL_VERSION,
                "payload": {
                    "wall": ros_wall_to_dict(res.wall)
                }
            }
            await ws.send(safe_dumps(resp))
        else:
            msg = res.message if res else "Update virtual wall failed"
            await self.send_error(ws, msg_id, "WALL_UPDATE_FAILED", msg)

    async def handle_delete_virtual_wall(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        wall_id = payload.get("wallId", "")
        if not wall_id:
            await self.send_error(ws, msg_id, "INVALID_WALL_ID", "Virtual wall ID cannot be empty")
            return

        if not self.srv_delete_wall.service_is_ready():
            await self.send_error(ws, msg_id, "SERVICE_UNAVAILABLE", "DeleteVirtualWall service is unavailable")
            return

        req = DeleteVirtualWall.Request()
        req.id = wall_id
        future = self.srv_delete_wall.call_async(req)
        res = await self._await_future(future)
        if res and res.success:
            await self.send_ack(ws, msg_id)
        else:
            msg = res.message if res else "Delete virtual wall failed"
            await self.send_error(ws, msg_id, "WALL_DELETE_FAILED", msg)

    async def handle_set_virtual_wall_enabled(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        wall_id = payload.get("wallId", "")
        enabled = bool(payload.get("enabled", True))
        if not wall_id:
            await self.send_error(ws, msg_id, "INVALID_WALL_ID", "Virtual wall ID cannot be empty")
            return

        client = self.srv_enable_wall if enabled else self.srv_disable_wall
        if not client.service_is_ready():
            await self.send_error(ws, msg_id, "SERVICE_UNAVAILABLE", "SetVirtualWallEnabled service is unavailable")
            return

        req = SetVirtualWallEnabled.Request()
        req.id = wall_id
        res = await self._await_future(client.call_async(req))
        if res and res.success:
            await self.send_ack(ws, msg_id)
        else:
            msg = res.message if res else "Set virtual wall enabled failed"
            await self.send_error(ws, msg_id, (res.error_code if res else "") or "WALL_ENABLE_FAILED", msg)

    async def handle_clear_virtual_walls(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        map_id = payload.get("mapId", "")
        if not is_valid_map_id(map_id):
            await self.send_error(ws, msg_id, "INVALID_MAP_ID", "Map ID is invalid or contains path traversal attempt")
            return

        if map_id != self._active_map_id():
            await self.send_error(ws, msg_id, "MAP_NOT_ACTIVE", "Virtual walls are only available for the active map")
            return

        if not self.srv_clear_walls.service_is_ready():
            await self.send_error(ws, msg_id, "SERVICE_UNAVAILABLE", "ClearVirtualWalls service is unavailable")
            return

        req = ClearVirtualWalls.Request()
        future = self.srv_clear_walls.call_async(req)
        res = await self._await_future(future)
        if res and res.success:
            await self.send_ack(ws, msg_id)
        else:
            msg = res.message if res else "Clear virtual walls failed"
            await self.send_error(ws, msg_id, "WALL_CLEAR_FAILED", msg)

    def _active_map_id(self) -> str:
        if self.last_map_state:
            return self.last_map_state["activeMapId"]
        return (self.last_telemetry or {}).get("activeMap", "") or ""

    def _locations_file_path(self, map_id: str) -> Optional[str]:
        """Locations live inside their map's directory so they move and die with the map."""
        map_dir = os.path.join(self.maps_dir, map_id)
        if not os.path.isdir(map_dir) or os.path.islink(map_dir):
            return None
        return os.path.join(map_dir, "locations.json")

    def _load_saved_locations(self, map_id: str) -> List[Dict[str, Any]]:
        path = self._locations_file_path(map_id)
        if path is None or not os.path.exists(path):
            return []
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            LOGGER.warning("Unreadable locations file %s; treating as empty", path)
            return []

    def _save_saved_locations(self, map_id: str, locations: List[Dict[str, Any]]):
        path = self._locations_file_path(map_id)
        if path is None:
            raise FileNotFoundError(f"map '{map_id}' does not exist")
        tmp = f"{path}.{uuid.uuid4().hex[:8]}.tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(locations, f, indent=2)
        os.replace(tmp, path)

    async def _require_active_map(self, ws: ServerConnection, msg_id: str | None) -> Optional[str]:
        map_id = self._active_map_id()
        if not map_id:
            await self.send_error(ws, msg_id, "NO_ACTIVE_MAP", "No map is active")
            return None
        return map_id

    async def handle_list_saved_locations(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        map_id = payload.get("mapId", "")
        if not is_valid_map_id(map_id):
            await self.send_error(ws, msg_id, "INVALID_MAP_ID", "Map ID is invalid or contains path traversal attempt")
            return
        if self._locations_file_path(map_id) is None:
            await self.send_error(ws, msg_id, "MAP_NOT_FOUND", f"Map '{map_id}' does not exist")
            return

        locations = self._load_saved_locations(map_id)
        resp = {
            "type": "list_saved_locations_ack",
            "id": msg_id,
            "protocolVersion": PROTOCOL_VERSION,
            "payload": {
                "mapId": map_id,
                "locations": locations
            }
        }
        await ws.send(safe_dumps(resp))

    async def handle_save_location(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        name = payload.get("name", "").strip()
        pose = payload.get("pose", {})
        if not name:
            await self.send_error(ws, msg_id, "INVALID_LOCATION_NAME", "Location name cannot be empty")
            return

        map_id = payload.get("mapId") or await self._require_active_map(ws, msg_id)
        if not map_id:
            return
        if not is_valid_map_id(map_id):
            await self.send_error(ws, msg_id, "INVALID_MAP_ID", "Map ID is invalid or contains path traversal attempt")
            return

        try:
            x = float(pose.get("xMeters", 0.0))
            y = float(pose.get("yMeters", 0.0))
            yaw = float(pose.get("yawRadians", 0.0))
        except (TypeError, ValueError):
            await self.send_error(ws, msg_id, "INVALID_ARGUMENT", "Pose values must be numbers")
            return
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            await self.send_error(ws, msg_id, "INVALID_ARGUMENT", "Pose values must be finite")
            return

        loc = {
            "id": f"loc_{uuid.uuid4().hex[:8]}",
            "name": name,
            "mapId": map_id,
            "xMeters": x,
            "yMeters": y,
            "yawRadians": yaw
        }

        locations = self._load_saved_locations(map_id)
        locations.append(loc)
        try:
            self._save_saved_locations(map_id, locations)
        except OSError as exc:
            await self.send_error(ws, msg_id, "MAP_NOT_FOUND", f"Cannot store location: {exc}")
            return

        resp = {
            "type": "save_location_ack",
            "id": msg_id,
            "protocolVersion": PROTOCOL_VERSION,
            "payload": {
                "location": loc
            }
        }
        await ws.send(safe_dumps(resp))

    async def handle_rename_location(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        loc_id = payload.get("id", "")
        new_name = payload.get("name", "").strip()
        if not new_name:
            await self.send_error(ws, msg_id, "INVALID_LOCATION_NAME", "Location name cannot be empty")
            return
        map_id = await self._require_active_map(ws, msg_id)
        if not map_id:
            return

        locations = self._load_saved_locations(map_id)
        target_loc = None
        for loc in locations:
            if loc.get("id") == loc_id:
                loc["name"] = new_name
                target_loc = loc
                break

        if target_loc:
            self._save_saved_locations(map_id, locations)
            resp = {
                "type": "rename_location_ack",
                "id": msg_id,
                "protocolVersion": PROTOCOL_VERSION,
                "payload": {
                    "location": target_loc
                }
            }
            await ws.send(safe_dumps(resp))
        else:
            await self.send_error(ws, msg_id, "LOCATION_NOT_FOUND", f"Saved location '{loc_id}' not found")

    async def handle_delete_location(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        loc_id = payload.get("id", "")
        map_id = await self._require_active_map(ws, msg_id)
        if not map_id:
            return

        locations = self._load_saved_locations(map_id)
        new_locations = [loc for loc in locations if loc.get("id") != loc_id]
        if len(new_locations) < len(locations):
            self._save_saved_locations(map_id, new_locations)
            await self.send_ack(ws, msg_id)
        else:
            await self.send_error(ws, msg_id, "LOCATION_NOT_FOUND", f"Saved location '{loc_id}' not found")

    async def handle_navigate_to_location(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        loc_id = payload.get("id", "")
        map_id = await self._require_active_map(ws, msg_id)
        if not map_id:
            return
        locations = self._load_saved_locations(map_id)

        target_loc = next((loc for loc in locations if loc.get("id") == loc_id), None)
        if not target_loc:
            await self.send_error(ws, msg_id, "LOCATION_NOT_FOUND", f"Saved location '{loc_id}' not found")
            return

        nav_payload = {
            "xMeters": target_loc.get("xMeters", 0.0),
            "yMeters": target_loc.get("yMeters", 0.0),
            "yawRadians": target_loc.get("yawRadians", 0.0)
        }
        await self.handle_navigate_to(ws, msg_id, nav_payload)

    async def handle_check_map_point(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        try:
            x = float(payload.get("xMeters", 0.0))
            y = float(payload.get("yMeters", 0.0))
        except (TypeError, ValueError):
            await self.send_error(ws, msg_id, "INVALID_ARGUMENT", "Coordinates must be numbers")
            return

        if not (math.isfinite(x) and math.isfinite(y)):
            is_valid, reason = False, "Coordinates contain non-finite numbers"
        else:
            if not self.srv_check_point.service_is_ready():
                await self.send_error(ws, msg_id, "SERVICE_UNAVAILABLE", "CheckMapPoint service is unavailable")
                return
            req = CheckMapPoint.Request()
            req.point.x = x
            req.point.y = y
            res = await self._await_future(self.srv_check_point.call_async(req))
            if res is None or not res.success:
                msg = res.message if res else "Check map point failed"
                await self.send_error(ws, msg_id, (res.error_code if res else "") or "CHECK_POINT_FAILED", msg)
                return
            is_valid = not res.restricted
            reason = None if is_valid else "Point is inside virtual wall(s): " + ", ".join(res.wall_ids)

        resp = {
            "type": "check_map_point_ack",
            "id": msg_id,
            "protocolVersion": PROTOCOL_VERSION,
            "payload": {
                "isValid": is_valid,
                "reason": reason
            }
        }
        await ws.send(safe_dumps(resp))

    # ---- Missions ----
    def _mission_running(self) -> bool:
        return self.mission_task is not None and not self.mission_task.done()

    def _broadcast_mission(self, status: str, step: int, total: int, detail: str):
        self.broadcast_event("mission_progress", {
            "missionId": self.mission_id,
            "status": status,
            "currentStepIndex": step,
            "totalSteps": total,
            "detail": detail,
        })

    def _validate_mission_steps(self, steps: Any) -> Optional[str]:
        if not isinstance(steps, list) or not steps:
            return "Mission steps cannot be empty"
        if len(steps) > MAX_MISSION_STEPS:
            return f"Mission has more than {MAX_MISSION_STEPS} steps"
        for i, step in enumerate(steps):
            if not isinstance(step, dict):
                return f"Step {i} is not an object"
            kind = step.get("type")
            if kind not in MISSION_STEP_TYPES:
                return f"Step {i} has unknown type '{kind}'"
            try:
                if kind == "NAVIGATE":
                    values = [float(step[k]) for k in ("xMeters", "yMeters", "yawRadians")]
                    if not all(math.isfinite(v) for v in values):
                        return f"Step {i} has non-finite coordinates"
                elif kind == "NAVIGATE_TO_LOCATION":
                    if not step.get("locationId"):
                        return f"Step {i} is missing locationId"
                elif kind == "WAIT":
                    millis = int(step["durationMillis"])
                    if not 0 <= millis <= MAX_WAIT_MILLIS:
                        return f"Step {i} durationMillis must be 0..{MAX_WAIT_MILLIS}"
            except (KeyError, TypeError, ValueError):
                return f"Step {i} is missing or has invalid fields for {kind}"
        return None

    def _check_mission_abort(self):
        if (self.last_safety or {}).get("emergencyStopActive"):
            raise MissionFailure("Emergency stop active")

    async def handle_submit_mission(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        mission = payload.get("mission", {})
        steps = mission.get("steps", [])
        problem = self._validate_mission_steps(steps)
        if problem:
            await self.send_error(ws, msg_id, "INVALID_ARGUMENT", problem)
            return
        if self._mission_running():
            await self.send_error(ws, msg_id, "MISSION_IN_PROGRESS", "A mission is already running")
            return
        if (self.last_safety or {}).get("emergencyStopActive"):
            await self.send_error(ws, msg_id, "SAFETY_ACTIVE", "Emergency stop is active")
            return

        self.mission_id = mission.get("id") or f"mission_{uuid.uuid4().hex[:8]}"
        self.mission_owner = ws
        if self.mission_orphan_task is not None:
            self.mission_orphan_task.cancel()
        self.mission_task = asyncio.create_task(self._run_mission(steps))
        await self.send_ack(ws, msg_id)

    async def _cancel_orphaned_mission(self, task: asyncio.Task):
        await asyncio.sleep(MISSION_DISCONNECT_GRACE_S)
        if task is not self.mission_task or task.done():
            return
        if self.control_owner is None:
            LOGGER.warning("Mission %s orphaned by disconnect; cancelling", self.mission_id)
            task.cancel()
        else:
            self.mission_owner = self.control_owner

    def _cancel_mission_if_running(self):
        if self._mission_running():
            self.mission_task.cancel()

    async def _cancel_mission_and_wait(self):
        """Cancel the mission and wait until it has ended, which includes its Nav2 goal."""
        task = self.mission_task
        if task is None or task.done():
            return
        task.cancel()
        done, _ = await asyncio.wait({task}, timeout=NAV_CANCEL_WAIT_S + 2.0)
        if not done:
            LOGGER.warning("Mission %s did not finish cancelling within %.0fs", self.mission_id, NAV_CANCEL_WAIT_S + 2.0)

    async def handle_cancel_mission(self, ws: ServerConnection, msg_id: str | None):
        await self._cancel_mission_and_wait()
        await self.send_ack(ws, msg_id)

    async def _run_mission(self, steps: List[Dict[str, Any]]):
        total = len(steps)
        index = 0
        try:
            for index, step in enumerate(steps):
                self._check_mission_abort()
                self._broadcast_mission("RUNNING", index, total, f"Step {index + 1}/{total}: {step['type']}")
                await self._run_mission_step(step)
            self._broadcast_mission("SUCCEEDED", total, total, "Mission completed")
        except asyncio.CancelledError:
            self._broadcast_mission("CANCELLED", index, total, "Mission cancelled")
        except MissionFailure as exc:
            self._broadcast_mission("FAILED", index, total, str(exc))
        except Exception as exc:
            LOGGER.exception("Mission %s crashed", self.mission_id)
            self._broadcast_mission("FAILED", index, total, f"Internal error: {exc}")

    async def _run_mission_step(self, step: Dict[str, Any]):
        kind = step["type"]
        if kind == "NAVIGATE":
            await self._mission_navigate(step)
        elif kind == "NAVIGATE_TO_LOCATION":
            map_id = self._active_map_id()
            loc = next((l for l in self._load_saved_locations(map_id) if l.get("id") == step["locationId"]), None) if map_id else None
            if loc is None:
                raise MissionFailure(f"Saved location '{step['locationId']}' not found")
            await self._mission_navigate(loc)
        elif kind == "WAIT":
            remaining = int(step["durationMillis"]) / 1000.0
            deadline = time.monotonic() + remaining
            while time.monotonic() < deadline:
                self._check_mission_abort()
                await asyncio.sleep(min(0.2, max(0.0, deadline - time.monotonic())))
        elif kind == "DOCK":
            await self._mission_dock(self.srv_dock, DockingState.DOCKED, "Docking")
        elif kind == "UNDOCK":
            await self._mission_dock(self.srv_undock, DockingState.UNDOCKED, "Undocking")

    async def _mission_navigate(self, point: Dict[str, Any]):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._pose_stamped(point)
        request_id = f"{self.mission_id}_nav_{uuid.uuid4().hex[:6]}"
        outcome = await self._send_nav_goal_core(
            NAV_MISSION, self.nav_client, goal_msg, request_id,
            [self._goal_summary(point)], goal_msg.pose.header.frame_id)
        if outcome.code:
            raise MissionFailure(f"{outcome.code}: {outcome.message}")

        handle, result_future = outcome.handle, outcome.result_future
        try:
            while not result_future.done():
                self._check_mission_abort()
                await asyncio.sleep(0.1)
        except BaseException:
            await self._cancel_goal_and_wait(handle, result_future, request_id)
            raise

        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            detail = getattr(result.result, "error_msg", "") or f"goal status {result.status}"
            raise MissionFailure(f"Navigation failed: {detail}")

    async def _cancel_goal_and_wait(self, handle, result_future, request_id: str):
        """Cancel a Nav2 goal and wait for its terminal result, logging both."""
        outcome = await self._await_future_outcome(handle.cancel_goal_async(), NAV_CANCEL_WAIT_S)
        state, response, exc = outcome
        LOGGER.info("nav_cancel request=%s cancel_response=%s return_code=%s exception=%r", request_id, state,
                    getattr(response, "return_code", None), exc)
        state, _, exc = await self._await_future_outcome(result_future, NAV_CANCEL_WAIT_S)
        LOGGER.info("nav_cancel request=%s terminal_result=%s exception=%r", request_id, state, exc)

    async def _mission_dock(self, client, target_state: int, name: str):
        if not client.service_is_ready():
            raise MissionFailure(f"{name} service is unavailable")
        res = await self._await_future(client.call_async(Trigger.Request()), timeout=10.0)
        if res is None or not res.success:
            raise MissionFailure(f"{name} refused: {res.message if res else 'no response'}")

        # The Trigger only starts the operation; /robot/docking/state reports the outcome.
        deadline = time.monotonic() + DOCKING_TIMEOUT_S
        try:
            while True:
                self._check_mission_abort()
                state = (self.last_docking or {}).get("state")
                if state == target_state:
                    return
                if state in (DockingState.FAILED, DockingState.CANCELLED):
                    docking = self.last_docking or {}
                    outcome = "failed" if state == DockingState.FAILED else "cancelled"
                    raise MissionFailure(f"{name} {outcome}: {docking.get('errorMsg') or docking.get('detail', '')}")
                if time.monotonic() > deadline:
                    raise MissionFailure(f"{name} timed out")
                await asyncio.sleep(0.2)
        except BaseException:
            if self.srv_cancel_dock.service_is_ready():
                self.srv_cancel_dock.call_async(Trigger.Request())
            raise

    def _double_param(self, name: str, value) -> Parameter:
        pv = ParameterValue()
        if isinstance(value, (list, tuple)):
            pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            pv.double_array_value = [float(v) for v in value]
        else:
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        return Parameter(name=name, value=pv)

    async def _get_params(self, client, names: List[str]):
        if not client.service_is_ready():
            return None
        res = await self._await_future(client.call_async(GetParameters.Request(names=names)))
        if res is None or len(res.values) != len(names):
            return None
        return res.values

    async def _set_params(self, client, params: List[Parameter]) -> Optional[str]:
        """None on success, otherwise a failure reason."""
        if not client.service_is_ready():
            return "parameter service unavailable"
        res = await self._await_future(client.call_async(SetParameters.Request(parameters=params)))
        if res is None:
            return "no response"
        for param, result in zip(params, res.results):
            if not result.successful:
                return f"{param.name}: {result.reason or 'rejected'}"
        return None

    async def _read_design_limits(self) -> bool:
        """The speeds the nav stack was configured with are the ceiling clients can't exceed."""
        if self.design_limits is not None:
            return True
        smoother = await self._get_params(self.srv_smoother_get, ["max_velocity", "min_velocity"])
        controller = await self._get_params(self.srv_controller_get, ["FollowPath.vx_min"])
        if smoother is None or controller is None or len(controller) < 1:
            return False
        max_vel, min_vel = smoother[0].double_array_value, smoother[1].double_array_value
        if len(max_vel) < 3 or len(min_vel) < 3:
            return False
        configured = (max_vel[0], max_vel[2], min_vel[0], controller[0].double_value)
        if not all(math.isfinite(value) for value in configured):
            return False
        if max_vel[0] <= 0.0 or max_vel[2] <= 0.0:
            return False
        self.design_limits = {
            "linear": max_vel[0],
            "angular": max_vel[2],
            "reverse": abs(min_vel[0]),
            "controller_reverse": abs(controller[0].double_value),
        }
        self.current_limits = {"linear": max_vel[0], "angular": max_vel[2]}
        return True

    def _limit_params(self, linear: float, angular: float):
        d = self.design_limits
        smoother = [
            self._double_param("max_velocity", [linear, 0.0, angular]),
            self._double_param("min_velocity", [-min(linear, d["reverse"]), 0.0, -angular]),
        ]
        controller = [
            self._double_param("FollowPath.vx_max", linear),
            self._double_param("FollowPath.vx_min", -min(linear, d["controller_reverse"])),
            self._double_param("FollowPath.wz_max", angular),
        ]
        return smoother, controller

    async def handle_update_motion_limits(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        limits = payload.get("limits", {})
        try:
            linear = float(limits.get("maxLinearVelocityMps"))
            angular = float(limits.get("maxAngularVelocityRadPerSec"))
        except (TypeError, ValueError):
            await self.send_error(ws, msg_id, "INVALID_ARGUMENT", "Velocity limits must be numbers")
            return

        if not (math.isfinite(linear) and math.isfinite(angular)) or linear <= 0.0 or angular <= 0.0:
            await self.send_error(ws, msg_id, "INVALID_ARGUMENT", "Velocity limits must be positive numbers")
            return

        error = None
        async with self.motion_limits_lock:
            if not await self._read_design_limits():
                error = ("SERVICE_UNAVAILABLE", "Navigation parameter services are unavailable")
            elif linear > self.design_limits["linear"] or angular > self.design_limits["angular"]:
                error = (
                    "LIMITS_EXCEEDED",
                    f"Requested limits exceed the robot's configured maximum "
                    f"({self.design_limits['linear']:.2f} m/s, {self.design_limits['angular']:.2f} rad/s)",
                )
            else:
                smoother_params, controller_params = self._limit_params(linear, angular)
                failure = await self._set_params(self.srv_smoother_set, smoother_params)
                if failure is None:
                    failure = await self._set_params(self.srv_controller_set, controller_params)
                    if failure is not None:
                        # Keep the two nodes consistent: put the smoother back where it was.
                        previous, _ = self._limit_params(
                            self.current_limits["linear"], self.current_limits["angular"])
                        await self._set_params(self.srv_smoother_set, previous)
                if failure is not None:
                    error = ("LIMITS_REJECTED", f"Could not apply motion limits: {failure}")
                else:
                    self.current_limits = {"linear": linear, "angular": angular}

        if error is not None:
            await self.send_error(ws, msg_id, error[0], error[1])
            return

        LOGGER.info("Updated motion limits: linear=%.2f m/s, angular=%.2f rad/s", linear, angular)
        await self.send_ack(ws, msg_id)

    async def handle_request_state_snapshot(self, ws: ServerConnection, msg_id: str | None):
        snapshot_payload = {
            "telemetry": self.last_telemetry,
            "battery": self.last_battery,
            "safety": self.last_safety,
            "docking": self.last_docking,
            "health": self.last_health,
            "map": self.last_map_state
        }
        resp = {
            "type": "robot_state_snapshot",
            "id": msg_id,
            "protocolVersion": PROTOCOL_VERSION,
            "payload": snapshot_payload
        }
        await ws.send(safe_dumps(resp))

    async def handle_get_recent_logs(self, ws: ServerConnection, msg_id: str | None, payload: Dict[str, Any]):
        try:
            limit = int(payload.get("limit", 100))
        except (TypeError, ValueError):
            await self.send_error(ws, msg_id, "INVALID_ARGUMENT", "limit must be an integer")
            return
        limit = max(0, min(limit, RECENT_LOG_CAPACITY))

        resp = {
            "type": "get_recent_logs_ack",
            "id": msg_id,
            "protocolVersion": PROTOCOL_VERSION,
            "payload": {
                "logs": list(self.recent_logs)[-limit:] if limit else []
            }
        }
        await ws.send(safe_dumps(resp))

    async def send_ack(self, ws: ServerConnection, msg_id: str | None):
        if msg_id:
            await ws.send(safe_dumps({
                "type": "command_ack",
                "id": msg_id,
                "protocolVersion": PROTOCOL_VERSION
            }))

    async def send_error(self, ws: ServerConnection, msg_id: str | None, code: str, message: str, subsystem: str = "gateway", recoverable: bool = True):
        resp = {
            "type": "command_error",
            "protocolVersion": PROTOCOL_VERSION,
            "error": {
                "code": code,
                "subsystem": subsystem,
                "severity": "ERROR",
                "message": message,
                "recoverable": recoverable
            }
        }
        if msg_id:
            resp["id"] = msg_id
        await ws.send(safe_dumps(resp))


async def _monitor_loop(node: RobotGatewayNode):
    """Event-loop lag, pending goal response age and the 1 s metric windows."""
    while True:
        t0 = time.monotonic()
        await asyncio.sleep(LOOP_LAG_PERIOD_S)
        lag = time.monotonic() - t0 - LOOP_LAG_PERIOD_S
        node.record_metric("max_loop_lag", lag)

        for request_id, pending in node.pending_nav.items():
            age = time.monotonic() - pending["t_send"]
            if age > SLOW_RESPONSE_WARN_S and not pending["warned"]:
                pending["warned"] = True
                LOGGER.warning("nav_response_slow request=%s goal_uuid=%s pending_age_s=%.2f "
                               "uuid_seen_on_status_topic=%s loop_metrics=%s", request_id, pending["uuid"].hex(),
                               age, pending["uuid"] in node.goal_status_seen,
                               node.metrics_between(pending["t_send"], time.monotonic()))

        closed = node.roll_metric_window()
        if closed and (node.pending_nav or closed["max_spin_gap"] > 0.25 or closed["max_loop_lag"] > 0.25
                       or closed["max_spin_once"] > 0.1):
            LOGGER.info("ros_loop_metrics pending_nav=%d max_spin_gap=%.4f max_spin_batch=%.4f "
                        "max_spin_once=%.4f max_loop_lag=%.4f", len(node.pending_nav), closed["max_spin_gap"],
                        closed["max_spin_batch"], closed["max_spin_once"], closed["max_loop_lag"])


async def run_gateway(node: RobotGatewayNode):
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            pass

    monitor = asyncio.create_task(_monitor_loop(node))
    async with serve(node.handle_client, node.host, node.port):
        LOGGER.info("Robot Gateway listening on ws://%s:%d", node.host, node.port)
        last_batch_start = time.monotonic()
        while not stop_event.is_set():
            batch_start = time.monotonic()
            node.record_metric("max_spin_gap", batch_start - last_batch_start)
            last_batch_start = batch_start
            # One spin_once handles one ready callback; drain a batch so a busy topic
            # (e.g. /rosout) cannot delay service responses.
            for _ in range(SPIN_BATCH):
                once_start = time.monotonic()
                rclpy.spin_once(node, timeout_sec=0)
                node.record_metric("max_spin_once", time.monotonic() - once_start)
            node.record_metric("max_spin_batch", time.monotonic() - batch_start)
            await asyncio.sleep(0.01)
        LOGGER.info("Robot Gateway shutting down")
    monitor.cancel()


def parse_args():
    parser = argparse.ArgumentParser(description="Run the Robot Gateway WebSocket server.")
    parser.add_argument("--host", default="127.0.0.1", help="host/interface to bind (default: %(default)s)")
    parser.add_argument("--port", type=int, default=8080, help="port to listen on (default: %(default)s)")
    parser.add_argument("--auth-token", default="",
                        help="required authentication token (default: $ROBOT_GATEWAY_TOKEN, else none)")
    # ros2 launch appends --ros-args; keep them away from argparse.
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if not args.auth_token:
        args.auth_token = os.environ.get("ROBOT_GATEWAY_TOKEN", "")
    if not args.auth_token and not _is_loopback(args.host):
        parser.error("gateway host %r is not loopback but no auth token was provided "
                     "(--auth-token or ROBOT_GATEWAY_TOKEN)" % args.host)
    return args


def _is_loopback(host: str) -> bool:
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def main():
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    rclpy.init(args=sys.argv)
    node = RobotGatewayNode(host=args.host, port=args.port, auth_token=args.auth_token)

    try:
        asyncio.run(run_gateway(node))
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
