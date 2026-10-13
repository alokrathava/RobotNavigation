"""Navigation admission races, planning rejection, and cancellation failure regressions."""

import asyncio
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from robot_gateway.gateway_node import NAV_MISSION, NAV_TO, NavSendResult, RobotGatewayNode
from geometry_msgs.msg import PoseStamped


class AdmissionHarness:
    _send_nav_goal_core = RobotGatewayNode._send_nav_goal_core

    def __init__(self):
        self.navigation_lock = asyncio.Lock()
        self.active_goal_handle = None
        self.error = None
        self.planning_error = None
        self.executed = 0
        self.in_flight = 0
        self.maximum_in_flight = 0

    def _autonomy_error(self):
        return self.error

    async def _preflight_path(self, nav_type, goal):
        await asyncio.sleep(0.01)
        return self.planning_error

    async def _execute_nav_goal(self, *args):
        self.in_flight += 1
        self.maximum_in_flight = max(self.maximum_in_flight, self.in_flight)
        await asyncio.sleep(0.01)
        self.executed += 1
        self.in_flight -= 1
        return NavSendResult()


class TestNavigationAdmission(unittest.IsolatedAsyncioTestCase):
    async def test_mission_navigation_uses_the_point_planner(self):
        pose = PoseStamped()
        pose.pose.orientation.w = 1.0
        harness = SimpleNamespace(path_to_client=SimpleNamespace(server_is_ready=lambda: False))
        result = await RobotGatewayNode._preflight_path(
            harness, NAV_MISSION, SimpleNamespace(pose=pose))
        self.assertEqual('PLANNER_UNAVAILABLE', result[0])
    async def test_same_socket_receives_estop_while_navigation_response_is_pending(self):
        handled = asyncio.Event()
        release = asyncio.Event()
        order = []

        class Socket:
            remote_address = ('127.0.0.1', 1)

            async def __aiter__(self):
                yield 'navigate'
                yield 'estop'
                await handled.wait()

        async def process(ws, message):
            if message == 'navigate':
                order.append('navigation_pending')
                await release.wait()
            else:
                order.append('estop')
                handled.set()

        harness = SimpleNamespace(
            session_ids={}, ws_clients=set(), authenticated_clients=set(),
            map_subscribers=set(), path_subscribers=set(), last_command_times={},
            _mission_running=lambda: False, release_control_ownership=lambda ws: None,
            _process_client_message=process)
        try:
            await asyncio.wait_for(RobotGatewayNode.handle_client(harness, Socket()), 0.5)
        finally:
            release.set()
        self.assertEqual(['navigation_pending', 'estop'], order)

    async def test_rejected_path_never_sends_motion_goal(self):
        harness = AdmissionHarness()
        harness.planning_error = ('DESTINATION_UNREACHABLE', 'blocked')
        result = await harness._send_nav_goal_core(NAV_TO, None, None, 'one', [], 'map')
        self.assertEqual('DESTINATION_UNREACHABLE', result.code)
        self.assertEqual(0, harness.executed)

    async def test_localization_loss_during_planning_blocks_execution(self):
        harness = AdmissionHarness()

        async def lose_localization(*args):
            harness.error = ('LOCALIZATION_NOT_READY', 'lost')
            return None

        harness._preflight_path = lose_localization
        result = await harness._send_nav_goal_core(NAV_TO, None, None, 'one', [], 'map')
        self.assertEqual('LOCALIZATION_NOT_READY', result.code)
        self.assertEqual(0, harness.executed)

    async def test_simultaneous_admissions_are_serialized(self):
        harness = AdmissionHarness()
        await asyncio.gather(*[
            harness._send_nav_goal_core(NAV_TO, None, None, str(i), [], 'map')
            for i in range(3)])
        self.assertEqual(3, harness.executed)
        self.assertEqual(1, harness.maximum_in_flight)

    async def test_timeout_with_no_terminal_result_engages_safety(self):
        stopped = []
        canceled = []

        async def await_future(*args, **kwargs):
            return None

        async def fail_safe():
            stopped.append(True)

        harness = SimpleNamespace(
            timed_out_results=set(), _await_future=await_future, _fail_safe_stop=fail_safe)
        handle = SimpleNamespace(cancel_goal_async=lambda: canceled.append(True))
        future = SimpleNamespace(done=lambda: False)
        with patch('robot_gateway.gateway_node.NAV_CANCEL_WAIT_S', 0.01):
            await RobotGatewayNode._enforce_navigation_timeout(
                harness, handle, future, b'goal', 'one', 0.01)
        self.assertEqual([True], canceled)
        self.assertEqual([True], stopped)


if __name__ == '__main__':
    unittest.main()
