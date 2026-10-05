"""Admission and serialization tests for long-running map transitions."""

import os
import sys
import threading
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from map_manager import MapFailure, MapManager  # noqa: E402


class _Logger:
    def exception(self, *_args, **_kwargs):
        pass


class _Harness:
    admit_transition = MapManager.admit_transition
    run_transition = MapManager.run_transition

    def __init__(self):
        self.transition_lock = threading.Lock()
        self.transition_thread = None
        self.shutting_down = False
        self.states = []
        self.current_state = "initial"

    def set_state(self, state, mode=None, detail="", error_code=""):
        self.current_state = state
        self.states.append((state, mode, detail, error_code))

    def state_msg(self):
        return self.current_state

    @staticmethod
    def get_logger():
        return _Logger()


class TestTransitionAdmission(unittest.TestCase):
    def test_service_returns_while_worker_is_still_running_and_rejects_overlap(self):
        harness = _Harness()
        started = threading.Event()
        release = threading.Event()

        def operation():
            started.set()
            release.wait(2.0)

        response = SimpleNamespace(success=None, error_code="", message="", state=None)
        result = harness.admit_transition(
            response, lambda: None, 5, 1, "switching", operation,
            lambda exc: self.fail(str(exc)), "accepted")

        self.assertTrue(result.success)
        self.assertTrue(started.wait(0.5))
        self.assertTrue(harness.transition_lock.locked())
        self.assertEqual((5, 1, "switching", ""), harness.states[0])

        overlapping = SimpleNamespace(success=None, error_code="", message="", state=None)
        result = harness.admit_transition(
            overlapping, lambda: None, 4, 2, "saving", lambda: None,
            lambda exc: None, "accepted")
        self.assertFalse(result.success)
        self.assertEqual("MAP_TRANSITION", result.error_code)

        worker = harness.transition_thread
        release.set()
        worker.join(1.0)
        self.assertFalse(harness.transition_lock.locked())

    def test_preflight_rejection_does_not_start_or_reserve_worker(self):
        harness = _Harness()
        response = SimpleNamespace(success=None, error_code="", message="", state=None)

        result = harness.admit_transition(
            response,
            lambda: (_ for _ in ()).throw(MapFailure("NOT_MAPPING", "not mapping")),
            5, None, "stopping", lambda: None, lambda exc: None, "accepted")

        self.assertFalse(result.success)
        self.assertEqual("NOT_MAPPING", result.error_code)
        self.assertIsNone(harness.transition_thread)
        self.assertFalse(harness.transition_lock.locked())
        self.assertEqual([], harness.states)

    def test_shutdown_rejects_new_transition_admission(self):
        harness = _Harness()
        harness.shutting_down = True
        response = SimpleNamespace(success=None, error_code="", message="", state=None)

        result = harness.admit_transition(
            response, lambda: self.fail("prepare must not run"), 5, 1, "switching",
            lambda: self.fail("operation must not run"), lambda exc: None, "accepted")

        self.assertFalse(result.success)
        self.assertEqual("MAP_TRANSITION", result.error_code)
        self.assertEqual("map manager is shutting down", result.message)
        self.assertFalse(harness.transition_lock.locked())

    def test_worker_failure_is_reported_and_releases_transition(self):
        harness = _Harness()
        failed = threading.Event()
        failure = {}

        def operation():
            raise MapFailure("MAP_LOAD_FAILED", "map server rejected map")

        def on_failure(exc):
            failure["code"] = exc.code
            failed.set()

        response = SimpleNamespace(success=None, error_code="", message="", state=None)
        harness.admit_transition(
            response, lambda: None, 5, 1, "switching", operation,
            on_failure, "accepted")

        self.assertTrue(failed.wait(0.5))
        self.assertEqual("MAP_LOAD_FAILED", failure["code"])
        if harness.transition_thread is not None:
            harness.transition_thread.join(1.0)
        self.assertFalse(harness.transition_lock.locked())


if __name__ == "__main__":
    unittest.main()
