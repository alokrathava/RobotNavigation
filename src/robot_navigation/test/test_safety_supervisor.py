import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from safety_supervisor import SafetySupervisor  # noqa: E402


def twist(linear=0.0, angular=0.0):
    return SimpleNamespace(twist=SimpleNamespace(
        linear=SimpleNamespace(x=linear, y=0.0), angular=SimpleNamespace(z=angular)))


def test_nonzero_manual_input_cancels_navigation_without_status_race():
    canceled = []
    harness = SimpleNamespace(
        synthetic_zeros=0, last_manual_rx=None, last_manual_zero=True,
        estop=False, manual_takeover_hold=False,
        active_goals={'navigate_to_pose': False},
        now_s=lambda: 10.0,
        get_logger=lambda: SimpleNamespace(info=lambda *_: None),
        cancel_navigation=lambda force=False: canceled.append(force),
    )
    SafetySupervisor.on_manual(harness, twist(linear=0.2))
    assert not harness.manual_takeover_hold  # ordinary manual driving need not pause
    assert canceled == [True]


def test_stale_scan_gates_nonzero_velocity_and_cancels_autonomy():
    published = []
    canceled = []
    harness = SimpleNamespace(
        last_scan_rx=0.0, scan_timeout=1.0, last_safe_rx=None, fault=False,
        estop=False, release_hold=False, transition_hold=False,
        transition_release_hold=False, manual_takeover_hold=False,
        now_s=lambda: 10.0,
        age=lambda stamp: 10.0 - stamp,
        zero_cmd=lambda: twist(),
        publish_cmd=lambda msg: published.append(msg),
        cancel_navigation=lambda force=False: canceled.append(force),
    )
    SafetySupervisor.on_safe(harness, twist(linear=0.2))
    assert published[0].twist.linear.x == 0.0
    assert harness.fault
    assert canceled == [True]


def test_localization_loss_cancels_navigation_and_preserves_manual_recovery():
    from robot_interfaces.msg import MapState

    canceled = []
    published = []
    harness = SimpleNamespace(
        autonomy_allowed=True, manual_active=lambda: True,
        cancel_navigation=lambda force=False: canceled.append(force),
        publish_cmd=lambda msg: published.append(msg), zero_cmd=lambda: twist())
    state = SimpleNamespace(state=MapState.ACTIVE, mode=MapState.MODE_LOCALIZATION,
                            localization_ready=False)
    SafetySupervisor.on_map_state(harness, state)
    assert not harness.autonomy_allowed
    assert canceled == [True]
    assert not published


def test_estop_release_remains_held_until_active_goal_ends():
    canceled = []
    locked = []
    harness = SimpleNamespace(
        estop=False, transition_hold=False, transition_release_hold=False,
        release_hold=True, active_goals={'navigate_to_pose': True},
        manual_active=lambda: False, last_nav_rx=None, nav_timeout=0.5,
        age=lambda stamp: float('inf'),
        cancel_navigation=lambda force=False: canceled.append(force),
        lock_pub=SimpleNamespace(publish=lambda msg: locked.append(msg.data)),
        zero_cmd=lambda: twist(), publish_cmd=lambda msg: None)
    SafetySupervisor.on_tick(harness)
    assert harness.release_hold
    assert canceled == [True]
    assert locked == [True]


def test_silent_velocity_pipeline_forces_final_output_zero():
    published = []
    harness = SimpleNamespace(
        transition_release_hold=False, estop=False, transition_hold=False,
        release_hold=False, last_manual_zero=True, manual_takeover_hold=False,
        last_out_zero=False, last_safe_rx=1.0, watchdog_timeout=0.5, fault=False,
        age=lambda stamp: 10.0 - stamp,
        lock_pub=SimpleNamespace(publish=lambda message: None),
        get_logger=lambda: SimpleNamespace(error=lambda *_: None),
        zero_cmd=lambda: twist(), publish_cmd=lambda message: published.append(message))
    SafetySupervisor.on_tick(harness)
    assert harness.fault
    assert len(published) == 1
    assert published[0].twist.linear.x == 0.0
    assert published[0].twist.angular.z == 0.0
