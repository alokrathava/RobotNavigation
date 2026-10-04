#!/usr/bin/env python3
"""Cross-language protocol fixture tests for robot_gateway."""

import json
import os
import unittest


class TestProtocolFixtures(unittest.TestCase):

    def setUp(self):
        env_dir = os.environ.get("PROTOCOL_FIXTURES_DIR")
        if env_dir and os.path.exists(env_dir):
            self.fixtures_dir = env_dir
        else:
            self.fixtures_dir = "/home/ghostone/AndroidStudioProjects/Robot-Controller/protocol-fixtures"

    def test_hello_fixture(self):
        filepath = os.path.join(self.fixtures_dir, "hello.json")
        self.assertTrue(os.path.exists(filepath), f"Fixture path missing: {filepath}")
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)

        self.assertEqual(data["type"], "hello")
        self.assertEqual(data["protocolVersion"], 1)
        self.assertEqual(data["payload"]["token"], "secret-token")

    def test_hello_ack_fixture(self):
        filepath = os.path.join(self.fixtures_dir, "hello_ack.json")
        self.assertTrue(os.path.exists(filepath), f"Fixture path missing: {filepath}")
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)

        self.assertEqual(data["type"], "hello_ack")
        self.assertEqual(data["payload"]["gatewayVersion"], "0.2.0")
        self.assertIn("map_switching", data["payload"]["capabilities"])

    def test_telemetry_fixture(self):
        filepath = os.path.join(self.fixtures_dir, "telemetry.json")
        self.assertTrue(os.path.exists(filepath), f"Fixture path missing: {filepath}")
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)

        self.assertEqual(data["type"], "telemetry")
        self.assertEqual(data["payload"]["activeMap"], "main_floor")
        self.assertEqual(data["payload"]["xMeters"], 1.25)

    def test_save_map_fixture(self):
        filepath = os.path.join(self.fixtures_dir, "save_map.json")
        self.assertTrue(os.path.exists(filepath), f"Fixture path missing: {filepath}")
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)

        self.assertEqual(data["type"], "save_map")
        self.assertEqual(data["payload"]["name"], "Warehouse A")

    def test_create_virtual_wall_fixture(self):
        filepath = os.path.join(self.fixtures_dir, "create_virtual_wall.json")
        self.assertTrue(os.path.exists(filepath), f"Fixture path missing: {filepath}")
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)

        self.assertEqual(data["type"], "create_virtual_wall")
        self.assertEqual(data["payload"]["wall"]["name"], "Keepout")

    def test_save_location_fixture(self):
        filepath = os.path.join(self.fixtures_dir, "save_location.json")
        self.assertTrue(os.path.exists(filepath), f"Fixture path missing: {filepath}")
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)

        self.assertEqual(data["type"], "save_location")
        self.assertEqual(data["payload"]["name"], "Reception")

    def test_submit_mission_fixture(self):
        filepath = os.path.join(self.fixtures_dir, "submit_mission.json")
        self.assertTrue(os.path.exists(filepath), f"Fixture path missing: {filepath}")
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)

        self.assertEqual(data["type"], "submit_mission")
        self.assertEqual(data["payload"]["mission"]["name"], "Patrol Mission")


if __name__ == "__main__":
    unittest.main()
