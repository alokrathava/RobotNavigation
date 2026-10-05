"""Safety-critical gateway command validation tests."""

import asyncio
import json
import math
import unittest
from types import SimpleNamespace

from robot_gateway import gateway_node
from robot_gateway.gateway_node import RobotGatewayNode


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


if __name__ == "__main__":
    unittest.main()
