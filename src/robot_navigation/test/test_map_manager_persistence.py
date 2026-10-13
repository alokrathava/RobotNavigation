import os
import math
import threading
import sys
from types import SimpleNamespace

import pytest
import yaml
from PIL import Image
from nav_msgs.msg import OccupancyGrid

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from map_manager import MapFailure, MapManager  # noqa: E402


class Harness:
    validate_id = MapManager.validate_id
    validate_dir = MapManager.validate_dir

    def __init__(self, maps_dir):
        self.maps_dir = str(maps_dir)
        self.active_map_id = ""


def make_map(root, occupied=0.65, free=0.196):
    directory = root / "test_map"
    directory.mkdir()
    (directory / "map.pgm").write_bytes(b"P5\n2 2\n255\n\x00\xff\x00\xff")
    (directory / "map.yaml").write_text(
        "image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n"
        f"negate: 0\noccupied_thresh: {occupied}\nfree_thresh: {free}\nmode: trinary\n",
        encoding="utf-8")
    (directory / "metadata.yaml").write_text(
        "format_version: 1\nid: test_map\ndisplay_name: Test Map\n",
        encoding="utf-8")
    return directory


def test_valid_map_metadata_and_geometry_are_loaded(tmp_path):
    info = Harness(tmp_path).validate_dir(make_map(tmp_path), expected_id="test_map")
    assert (info.width, info.height, info.resolution) == (2, 2, 0.05)
    assert info.name == "Test Map"


def test_corrupted_thresholds_are_rejected(tmp_path):
    directory = make_map(tmp_path, occupied=0.2, free=0.7)
    with pytest.raises(MapFailure, match="occupancy thresholds"):
        Harness(tmp_path).validate_dir(directory, expected_id="test_map")


def test_truncated_raster_is_rejected(tmp_path):
    directory = make_map(tmp_path)
    (directory / 'map.pgm').write_bytes(b'P5\n2 2\n255\n\x00')
    with pytest.raises(MapFailure):
        Harness(tmp_path).validate_dir(directory, expected_id='test_map')


def test_image_symlink_escaping_map_directory_is_rejected(tmp_path):
    outside = tmp_path / "outside.pgm"
    outside.write_bytes(b"P5\n1 1\n255\n\x00")
    directory = make_map(tmp_path)
    (directory / "map.pgm").unlink()
    (directory / "map.pgm").symlink_to(outside)
    with pytest.raises(MapFailure, match="escapes"):
        Harness(tmp_path).validate_dir(directory, expected_id="test_map")


def test_localization_covariance_requires_bounded_finite_variance():
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = 0.04
    covariance[35] = 0.01
    assert MapManager.localization_covariance_ok(covariance, 0.5, 0.387)
    covariance[0] = float("nan")
    assert not MapManager.localization_covariance_ok(covariance, 0.5, 0.387)


def test_global_recovery_does_not_replace_estimate_during_activation():
    from robot_interfaces.msg import MapState
    requests = []
    transition = threading.Lock()
    transition.acquire()
    harness = SimpleNamespace(
        lock=threading.Lock(), transition_lock=transition,
        mode=MapState.MODE_LOCALIZATION, state=MapState.ACTIVE,
        last_amcl_message=None, last_localization_tf_rx=0.0,
        localization_ready=False, localization_lost_since=1.0,
        last_global_recovery=0.0, last_nomotion_update=0.0,
        localization_transforms_ready=lambda: False,
        global_localization=SimpleNamespace(service_is_ready=lambda: True,
            call_async=lambda request: requests.append(request)),
        nomotion_update=SimpleNamespace(service_is_ready=lambda: False))
    from unittest.mock import patch
    with patch('map_manager.time.monotonic', return_value=50.0):
        MapManager.localization_tick(harness)
    assert not requests
    assert harness.last_global_recovery == 0.0


def test_stationary_covariance_roundoff_is_clamped_but_invalid_values_rejected():
    from robot_state_manager import RobotStateManager
    covariance = [0.0] * 36
    covariance[0] = -3.7e-35
    covariance[7] = -2.6e-75
    covariance[35] = 1.1e-13
    message = SimpleNamespace(pose=SimpleNamespace(covariance=covariance))
    harness = SimpleNamespace(amcl_stddev=None)
    assert MapManager.localization_covariance_ok(covariance, 0.5, 0.387)
    RobotStateManager.on_amcl(harness, message)
    assert harness.amcl_stddev[0] == 0.0
    assert harness.amcl_stddev[1] > 0.0
    for invalid in (-0.01, float('nan'), float('inf')):
        covariance[0] = invalid
        assert not MapManager.localization_covariance_ok(covariance, 0.5, 0.387)
        RobotStateManager.on_amcl(harness, message)
        assert harness.amcl_stddev is None


def test_failed_activation_restores_previous_map():
    calls = []
    harness = SimpleNamespace(active_map_id="old")

    def activate_once(map_id, admitted=False):
        calls.append((map_id, admitted))
        if map_id == "new":
            raise MapFailure("MAP_LOAD_FAILED", "bad map")
        return "restored"

    harness.activate_once = activate_once
    harness.get_logger = lambda: SimpleNamespace(error=lambda *_: None)
    harness.set_state = lambda *args: calls.append(("state", args))
    result = MapManager.activate(harness, "new", admitted=True)
    assert result == "restored"
    assert calls[:2] == [("new", True), ("old", True)]


def test_active_map_marker_is_restored_from_disk(tmp_path):
    harness = SimpleNamespace(maps_dir=str(tmp_path), active_map_id='test_map',
                              default_map_id='main_floor')
    MapManager.persist_active(harness)
    choices = [SimpleNamespace(id='main_floor'), SimpleNamespace(id='test_map')]
    assert MapManager.restore_active(harness, choices).id == 'test_map'


def test_failed_activation_and_rollback_report_error_without_releasing_hold():
    states = []
    attempts = []
    def reject(map_id, admitted=False):
        attempts.append(map_id)
        raise MapFailure('MAP_LOAD_FAILED', 'unavailable ' + map_id)
    harness = SimpleNamespace(
        active_map_id='old', activate_once=reject,
        get_logger=lambda: SimpleNamespace(error=lambda *_: None),
        set_state=lambda *args: states.append(args))
    with pytest.raises(MapFailure) as error:
        MapManager.activate(harness, 'new', admitted=True)
    assert error.value.code == 'MAP_ROLLBACK_FAILED'
    assert attempts == ['new', 'old']
    from robot_interfaces.msg import MapState
    assert states[-1][0] == MapState.ERROR
    assert states[-1][-1] == 'MAP_ROLLBACK_FAILED'


def test_malformed_yaml_collection_is_rejected(tmp_path):
    directory = make_map(tmp_path)
    (directory / 'map.yaml').write_text('- not an object\n', encoding='utf-8')
    with pytest.raises(MapFailure):
        Harness(tmp_path).validate_dir(directory, expected_id='test_map')


def test_initial_pose_composes_saved_alignment_with_current_odometry():
    x, y, yaw = MapManager.compose_planar((1.0, 2.0, math.pi / 2), (2.0, 0.0, 0.0))
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(4.0)
    assert yaw == pytest.approx(math.pi / 2)


def test_saved_grid_preserves_origin_and_ros_bottom_up_cell_coordinates(tmp_path):
    grid = OccupancyGrid()
    grid.info.width, grid.info.height, grid.info.resolution = 3, 2, 0.05
    grid.info.origin.position.x = -1.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [-1, 0, 100, 100, 0, -1]
    MapManager.write_grid_map(str(tmp_path), grid)
    with Image.open(tmp_path / 'map.pgm') as image:
        assert list(image.tobytes()) == [0, 254, 205, 205, 254, 0]
    with (tmp_path / 'map.yaml').open() as stream:
        metadata = yaml.safe_load(stream)
    assert metadata['origin'] == [-1.0, 0.0, 0.0]
    assert metadata['resolution'] == pytest.approx(0.05)


def test_rename_preserves_saved_map_alignment(tmp_path):
    directory = make_map(tmp_path)
    metadata = yaml.safe_load((directory / 'metadata.yaml').read_text())
    metadata['map_to_odom'] = [1.0, 2.0, 0.3]
    (directory / 'metadata.yaml').write_text(yaml.safe_dump(metadata), encoding='utf-8')
    harness = Harness(tmp_path)
    harness.transition_lock = threading.Lock()
    harness.map_dir = lambda map_id: str(tmp_path / map_id)
    harness.write_metadata = MapManager.write_metadata
    response = SimpleNamespace(success=None, error_code='', message='', map=None)
    MapManager.srv_rename(harness, SimpleNamespace(map_id='test_map', display_name='Renamed'), response)
    assert response.success
    assert yaml.safe_load((directory / 'metadata.yaml').read_text())['map_to_odom'] == [1.0, 2.0, 0.3]
