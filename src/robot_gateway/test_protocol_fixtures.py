#!/usr/bin/env python3
"""Cross-language Protocol v1 fixture tests for robot_gateway."""

import json
import os
from pathlib import Path
import unittest
from unittest import mock

from robot_gateway.protocol_contract import (
    GATEWAY_CAPABILITIES,
    GATEWAY_VERSION,
    PROTOCOL_VERSION,
)


REQUIRED_FIXTURES = {
    "battery_state.json", "cancel_docking.json", "cancel_mission.json",
    "cancel_navigation.json", "check_map_point.json", "check_map_point_ack.json",
    "clear_virtual_walls.json", "command_ack.json", "command_error.json",
    "create_virtual_wall.json", "create_virtual_wall_ack.json", "delete_location.json",
    "delete_map.json", "delete_virtual_wall.json", "dock.json", "docking_state.json",
    "emergency_stop.json", "get_recent_logs.json", "get_recent_logs_ack.json",
    "hello.json", "hello_ack.json", "list_maps.json", "list_maps_ack.json",
    "list_saved_locations.json", "list_saved_locations_ack.json",
    "list_virtual_walls.json", "list_virtual_walls_ack.json", "manual_velocity.json",
    "map_state.json", "mission_progress.json", "navigate_through.json",
    "navigate_to.json", "navigate_to_location.json", "ping.json", "pong.json",
    "release_emergency_stop.json", "rename_location.json", "rename_location_ack.json",
    "rename_map.json", "rename_map_ack.json", "request_state_snapshot.json",
    "robot_discovery_beacon.json", "robot_health.json", "robot_state_snapshot.json",
    "safety_state.json", "save_location.json", "save_location_ack.json", "save_map.json",
    "set_virtual_wall_enabled.json", "start_mapping.json", "stop.json",
    "stop_mapping.json", "submit_mission.json", "switch_map.json", "telemetry.json",
    "undock.json", "update_motion_limits.json", "update_virtual_wall.json",
    "update_virtual_wall_ack.json",
}


def fixture_directory() -> Path:
    override = os.environ.get("PROTOCOL_FIXTURES_DIR")
    path = (Path(override).expanduser() if override else
            Path(__file__).resolve().parents[2] / "protocol-fixtures")
    if not path.is_dir():
        raise RuntimeError(f"Protocol fixture directory does not exist: {path}")
    return path


class TestProtocolFixtures(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixtures_dir = fixture_directory()

    def load(self, name):
        with (self.fixtures_dir / name).open("r", encoding="utf-8") as stream:
            return json.load(stream, parse_constant=lambda value: self.fail(
                f"{name} contains non-standard JSON constant {value}"))

    def test_required_fixture_set_is_present(self):
        present = {path.name for path in self.fixtures_dir.glob("*.json")}
        self.assertEqual(set(), REQUIRED_FIXTURES - present)

    def test_invalid_fixture_override_fails_instead_of_using_a_local_fallback(self):
        with mock.patch.dict(
                os.environ,
                {"PROTOCOL_FIXTURES_DIR": "/path/that/does/not/exist"}):
            with self.assertRaises(RuntimeError):
                fixture_directory()

    def test_every_fixture_is_a_protocol_v1_object(self):
        for path in sorted(self.fixtures_dir.glob("*.json")):
            with self.subTest(fixture=path.name):
                data = self.load(path.name)
                self.assertIsInstance(data, dict)
                self.assertIsInstance(data.get("type"), str)
                self.assertTrue(data["type"])
                self.assertEqual(PROTOCOL_VERSION, data.get("protocolVersion"))

    def test_hello_ack_uses_the_canonical_gateway_contract(self):
        data = self.load("hello_ack.json")
        self.assertEqual(GATEWAY_VERSION, data["payload"]["gatewayVersion"])
        self.assertEqual(list(GATEWAY_CAPABILITIES), data["payload"]["capabilities"])

    def test_map_state_schema_is_identical_in_snapshot(self):
        event = self.load("map_state.json")["payload"]
        snapshot = self.load("robot_state_snapshot.json")["payload"]
        self.assertNotIn("map", snapshot)
        self.assertEqual(event, snapshot["mapState"])

    def test_long_running_map_commands_use_generic_acceptance_ack(self):
        ack = self.load("command_ack.json")
        self.assertEqual("command_ack", ack["type"])
        self.assertFalse((self.fixtures_dir / "save_map_ack.json").exists())

    def test_waypoint_fixture_exposes_only_supported_semantics(self):
        data = self.load("navigate_through.json")
        self.assertIs(True, data["payload"]["stopOnFailure"])

    def test_discovery_fixture_matches_android_listener_fields(self):
        data = self.load("robot_discovery_beacon.json")
        self.assertEqual("robot_discovery_beacon", data["type"])
        self.assertEqual(data["port"], data["gatewayPort"])
        self.assertEqual(GATEWAY_VERSION, data["gatewayVersion"])
        self.assertFalse(data["tls"])


if __name__ == "__main__":
    unittest.main()
