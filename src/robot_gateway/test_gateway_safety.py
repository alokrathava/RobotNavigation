"""Safety-critical gateway command validation tests."""

import asyncio
import json
import math
import time
import unittest
from types import SimpleNamespace

from robot_gateway import gateway_node
from robot_gateway.gateway_node import RobotGatewayNode
from robot_gateway.map_stream import MapStreamState
from robot_interfaces.msg import MapState
from nav_msgs.msg import OccupancyGrid


class _FakeTwistStamped:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.twist = SimpleNamespace(
            linear=SimpleNamespace(x=0.0),
            angular=SimpleNamespace(z=0.0),
        )


class _FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: "stamp")


class _ManualVelocityHarness:
    def __init__(self, limits=None, limits_available=True):
        self.current_limits = limits or {"linear": 0.5, "angular": 1.2}
        self.limits_available = limits_available
        self.motion_limits_lock = asyncio.Lock()
        self.published = []
        self.cmd_vel_pub = SimpleNamespace(publish=self.published.append)
        self.errors = []
        self.acks = []

    async def _read_design_limits(self):
        return self.limits_available

    def get_clock(self):
        return _FakeClock()

    async def send_error(self, _ws, msg_id, code, message, **_kwargs):
        self.errors.append((msg_id, code, message))

    async def send_ack(self, _ws, msg_id):
        self.acks.append(msg_id)


class _DispatchHarness:
    def __init__(self, authenticated, owner):
        self.authenticated_clients = set(authenticated)
        self.control_owner = owner
        self.last_command_times = {}
        self.errors = []
        self.service_calls = []
        self.srv_estop = object()
        self.srv_release_estop = object()

    async def send_error(self, _ws, msg_id, code, message, **_kwargs):
        self.errors.append((msg_id, code, message))

    async def handle_trigger_service(self, _ws, msg_id, service, label):
        self.service_calls.append((msg_id, service, label))


class _WebSocket:
    def __init__(self):
        self.messages = []

    async def send(self, text):
        self.messages.append(json.loads(text))


class _NavigateThroughHarness:
    def __init__(self):
        self.errors = []
        self.sent = []
        self.nav_through_client = object()

    async def send_error(self, _ws, msg_id, code, message, **_kwargs):
        self.errors.append((msg_id, code, message))

    @staticmethod
    def _pose_stamped(point):
        return gateway_node.PoseStamped()

    @staticmethod
    def _goal_summary(point):
        return point

    async def _send_nav_goal(self, _ws, msg_id, nav_type, client, goal, goals, frame,
                             timeout_seconds=300.0):
        self.sent.append((msg_id, nav_type, client, goal, goals, frame))


class _CompletedFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _ServiceClient:
    def __init__(self, response):
        self.response = response
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return _CompletedFuture(self.response)


class _MapCommandHarness:
    def __init__(self, response):
        self.srv_save_map = _ServiceClient(response)
        self.acks = []
        self.errors = []

    async def _await_future(self, future, timeout=10.0):
        return future.result()

    async def send_ack(self, _ws, msg_id):
        self.acks.append(msg_id)

    async def send_error(self, _ws, msg_id, code, message, **_kwargs):
        self.errors.append((msg_id, code, message))


class TestManualVelocityValidation(unittest.IsolatedAsyncioTestCase):
    async def _send(self, payload, limits=None, limits_available=True):
        harness = _ManualVelocityHarness(limits, limits_available)
        original_twist_stamped = gateway_node.TwistStamped
        gateway_node.TwistStamped = _FakeTwistStamped
        try:
            await RobotGatewayNode.handle_manual_velocity(harness, object(), "cmd-1", payload)
        finally:
            gateway_node.TwistStamped = original_twist_stamped
        return harness

    async def test_valid_boundary_velocity_is_published(self):
        harness = await self._send({"linearMps": -0.5, "angularRadPerSec": 1.2})

        self.assertEqual([], harness.errors)
        self.assertEqual(["cmd-1"], harness.acks)
        self.assertEqual(1, len(harness.published))
        self.assertEqual(-0.5, harness.published[0].twist.linear.x)
        self.assertEqual(1.2, harness.published[0].twist.angular.z)

    async def test_invalid_types_and_non_finite_values_are_rejected(self):
        invalid_payloads = [
            [],
            {},
            {"linearMps": "0.1", "angularRadPerSec": 0.0},
            {"linearMps": True, "angularRadPerSec": 0.0},
            {"linearMps": None, "angularRadPerSec": 0.0},
            {"linearMps": [], "angularRadPerSec": 0.0},
            {"linearMps": math.nan, "angularRadPerSec": 0.0},
            {"linearMps": math.inf, "angularRadPerSec": 0.0},
            {"linearMps": 0.0, "angularRadPerSec": -math.inf},
        ]
        for payload in invalid_payloads:
            with self.subTest(payload=payload):
                harness = await self._send(payload)
                self.assertEqual("INVALID_ARGUMENT", harness.errors[0][1])
                self.assertEqual([], harness.published)
                self.assertEqual([], harness.acks)

    async def test_runtime_lowered_limit_is_enforced(self):
        harness = await self._send(
            {"linearMps": 0.3, "angularRadPerSec": 0.0},
            limits={"linear": 0.2, "angular": 0.4},
        )

        self.assertEqual("VELOCITY_LIMIT_EXCEEDED", harness.errors[0][1])
        self.assertEqual([], harness.published)

    async def test_positive_and_negative_overspeed_are_rejected(self):
        overspeed_payloads = [
            {"linearMps": 0.5001, "angularRadPerSec": 0.0},
            {"linearMps": -0.5001, "angularRadPerSec": 0.0},
            {"linearMps": 0.0, "angularRadPerSec": 1.2001},
            {"linearMps": 0.0, "angularRadPerSec": -1.2001},
        ]
        for payload in overspeed_payloads:
            with self.subTest(payload=payload):
                harness = await self._send(payload)
                self.assertEqual("VELOCITY_LIMIT_EXCEEDED", harness.errors[0][1])
                self.assertEqual([], harness.published)
                self.assertEqual([], harness.acks)

    async def test_unavailable_limits_fail_closed(self):
        harness = await self._send(
            {"linearMps": 0.1, "angularRadPerSec": 0.1}, limits_available=False)

        self.assertEqual("SERVICE_UNAVAILABLE", harness.errors[0][1])
        self.assertEqual([], harness.published)
        self.assertEqual([], harness.acks)


class TestEmergencyStopOwnership(unittest.IsolatedAsyncioTestCase):
    @staticmethod
    def _message(command):
        return json.dumps({"type": command, "id": "cmd-1", "protocolVersion": 1})

    async def test_any_authenticated_client_can_engage_estop(self):
        owner, secondary = object(), object()
        harness = _DispatchHarness([owner, secondary], owner)

        await RobotGatewayNode.process_message(
            harness, secondary, self._message("emergency_stop"))

        self.assertEqual([], harness.errors)
        self.assertEqual(harness.srv_estop, harness.service_calls[0][1])

    async def test_owner_can_release_estop(self):
        owner = object()
        harness = _DispatchHarness([owner], owner)

        await RobotGatewayNode.process_message(
            harness, owner, self._message("release_emergency_stop"))

        self.assertEqual([], harness.errors)
        self.assertEqual(harness.srv_release_estop, harness.service_calls[0][1])

    async def test_secondary_client_cannot_release_estop(self):
        owner, secondary = object(), object()
        harness = _DispatchHarness([owner, secondary], owner)

        await RobotGatewayNode.process_message(
            harness, secondary, self._message("release_emergency_stop"))

        self.assertEqual("CONTROL_SESSION_IN_USE", harness.errors[0][1])
        self.assertEqual([], harness.service_calls)

    async def test_release_requires_an_existing_owner(self):
        client = object()
        harness = _DispatchHarness([client], None)

        await RobotGatewayNode.process_message(
            harness, client, self._message("release_emergency_stop"))

        self.assertEqual("CONTROL_SESSION_REQUIRED", harness.errors[0][1])
        self.assertEqual([], harness.service_calls)


class TestProtocolContract(unittest.IsolatedAsyncioTestCase):
    def test_map_activation_resynchronizes_cached_grid_identity(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 2
        grid.info.resolution = 0.05
        grid.info.origin.orientation.w = 1.0
        grid.data = [0, 0, 0, 0]
        stream = MapStreamState()
        stream.ingest_full(grid, 'old', 'LOCALIZATION')
        harness = SimpleNamespace(map_stream=stream, pending_map_event=None,
                                  last_map_state=None, broadcast_event=lambda *args: None)
        state = MapState()
        state.state, state.mode = MapState.ACTIVE, MapState.MODE_LOCALIZATION
        state.active_map_id = 'new'
        RobotGatewayNode.on_map_state(harness, state)
        self.assertEqual('map_snapshot', harness.pending_map_event[0])
        self.assertEqual('new', harness.pending_map_event[1]['mapId'])
        self.assertEqual(2, harness.pending_map_event[1]['sequence'])
    def test_autonomy_gate_requires_fresh_localized_safe_state(self):
        harness = SimpleNamespace(
            last_map_state={"state": "ACTIVE", "localizationReady": True},
            runtime_ready=True,
            last_telemetry={"poseValid": True, "localizationState": "LOCALIZED"},
            last_telemetry_at=time.monotonic(),
            last_safety={"emergencyStopActive": False, "motionInhibited": False},
        )
        self.assertIsNone(RobotGatewayNode._autonomy_error(harness))
        harness.last_map_state["localizationReady"] = False
        self.assertEqual("LOCALIZATION_NOT_READY",
                         RobotGatewayNode._autonomy_error(harness)[0])

    def test_navigation_timeout_is_bounded_and_finite(self):
        self.assertEqual(300.0, RobotGatewayNode._navigation_timeout({}))
        self.assertEqual(1.0, RobotGatewayNode._navigation_timeout({"timeoutSeconds": 1}))
        self.assertIsNone(RobotGatewayNode._navigation_timeout({"timeoutSeconds": 0}))
        self.assertIsNone(RobotGatewayNode._navigation_timeout(
            {"timeoutSeconds": float("nan")}))

    def test_discovery_auto_mode_only_enables_for_lan_gateway(self):
        self.assertFalse(gateway_node.discovery_is_enabled("127.0.0.1", "auto"))
        self.assertTrue(gateway_node.discovery_is_enabled("0.0.0.0", "auto"))
        self.assertFalse(gateway_node.discovery_is_enabled("0.0.0.0", "off"))
        self.assertTrue(gateway_node.discovery_is_enabled("127.0.0.1", "on"))

    def test_discovery_beacon_matches_android_wire_contract(self):
        beacon = gateway_node.discovery_beacon("robot-01", "Robot 01", 8080)

        self.assertEqual("robot_discovery_beacon", beacon["type"])
        self.assertEqual("robot-01", beacon["robotId"])
        self.assertEqual(8080, beacon["port"])
        self.assertEqual(beacon["port"], beacon["gatewayPort"])
        self.assertFalse(beacon["tls"])

    async def test_discovery_loop_broadcasts_and_closes_its_socket(self):
        class FakeSocket:
            def __init__(self):
                self.options = []
                self.blocking = None
                self.closed = False

            def setsockopt(self, *option):
                self.options.append(option)

            def setblocking(self, blocking):
                self.blocking = blocking

            def close(self):
                self.closed = True

        class FakeLoop:
            def __init__(self):
                self.sent = []

            async def sock_sendto(self, udp_socket, message, target):
                self.sent.append((udp_socket, json.loads(message), target))
                raise asyncio.CancelledError

        fake_socket = FakeSocket()
        fake_loop = FakeLoop()
        original_socket = gateway_node.socket.socket
        original_get_running_loop = gateway_node.asyncio.get_running_loop
        gateway_node.socket.socket = lambda *_args: fake_socket
        gateway_node.asyncio.get_running_loop = lambda: fake_loop
        harness = SimpleNamespace(
            robot_id="robot-01", robot_name="Robot 01", port=8080,
            discovery_port=8088, discovery_interval=1.0,
        )
        try:
            with self.assertRaises(asyncio.CancelledError):
                await gateway_node._discovery_loop(harness)
        finally:
            gateway_node.socket.socket = original_socket
            gateway_node.asyncio.get_running_loop = original_get_running_loop

        self.assertEqual(("255.255.255.255", 8088), fake_loop.sent[0][2])
        self.assertEqual("robot_discovery_beacon", fake_loop.sent[0][1]["type"])
        self.assertFalse(fake_socket.blocking)
        self.assertTrue(fake_socket.closed)

    async def test_snapshot_uses_map_state_schema(self):
        map_state = {
            "state": "ACTIVE", "mode": "LOCALIZATION", "activeMapId": "main_floor",
            "mapAvailable": True, "localizationReady": True,
            "navigationRuntimeReady": True, "errorCode": "", "detail": "ready",
        }
        harness = SimpleNamespace(
            last_telemetry=None, last_battery=None, last_safety=None,
            last_docking=None, last_health=None, last_map_state=map_state,
        )
        websocket = _WebSocket()

        await RobotGatewayNode.handle_request_state_snapshot(harness, websocket, "snapshot-1")

        payload = websocket.messages[0]["payload"]
        self.assertNotIn("map", payload)
        self.assertEqual(map_state, payload["mapState"])

    async def test_navigate_through_defaults_to_stop_on_failure(self):
        harness = _NavigateThroughHarness()

        await RobotGatewayNode.handle_navigate_through(
            harness, object(), "nav-1",
            {"waypoints": [{"xMeters": 1.0, "yMeters": 2.0, "yawRadians": 0.0}]})

        self.assertEqual([], harness.errors)
        self.assertEqual(1, len(harness.sent))

    async def test_navigate_through_rejects_false_stop_on_failure(self):
        harness = _NavigateThroughHarness()

        await RobotGatewayNode.handle_navigate_through(
            harness, object(), "nav-1",
            {"waypoints": [{"xMeters": 1.0}], "stopOnFailure": False})

        self.assertEqual("UNSUPPORTED_ARGUMENT", harness.errors[0][1])
        self.assertEqual([], harness.sent)

    async def test_navigate_through_rejects_non_boolean_stop_on_failure(self):
        harness = _NavigateThroughHarness()

        await RobotGatewayNode.handle_navigate_through(
            harness, object(), "nav-1",
            {"waypoints": [{"xMeters": 1.0}], "stopOnFailure": "true"})

        self.assertEqual("INVALID_ARGUMENT", harness.errors[0][1])
        self.assertEqual([], harness.sent)

    async def test_save_map_acknowledges_admission_without_a_typed_result(self):
        harness = _MapCommandHarness(SimpleNamespace(success=True, message="accepted"))
        websocket = _WebSocket()

        await RobotGatewayNode.handle_save_map(
            harness, websocket, "save-1", {"name": "Warehouse A"})

        self.assertEqual(["save-1"], harness.acks)
        self.assertEqual([], websocket.messages)
        self.assertEqual([], harness.errors)

    async def test_save_map_reports_admission_rejection(self):
        harness = _MapCommandHarness(SimpleNamespace(
            success=False, error_code="MAP_TRANSITION", message="busy"))

        await RobotGatewayNode.handle_save_map(
            harness, _WebSocket(), "save-1", {"name": "Warehouse A"})

        self.assertEqual([], harness.acks)
        self.assertEqual("MAP_TRANSITION", harness.errors[0][1])


if __name__ == "__main__":
    unittest.main()
