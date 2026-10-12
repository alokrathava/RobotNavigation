import base64
import zlib
from types import SimpleNamespace
import unittest

from robot_gateway.map_stream import ENCODING, MapStreamError, MapStreamState


def ns(**kwargs):
    return SimpleNamespace(**kwargs)


def pose():
    return ns(position=ns(x=-1.0, y=-2.0, z=0.0),
              orientation=ns(x=0.0, y=0.0, z=0.0, w=1.0))


def grid(values, width=2, height=2):
    return ns(header=ns(frame_id="map", stamp=ns(sec=1, nanosec=2)),
              info=ns(width=width, height=height, resolution=0.05, origin=pose()),
              data=values)


def decode(payload):
    raw = zlib.decompress(base64.b64decode(payload["data"]))
    return [value if value < 128 else value - 256 for value in raw]


class TestMapStream(unittest.TestCase):
    def test_full_grid_snapshot_contains_geometry_and_lossless_signed_cells(self):
        state = MapStreamState()
        event, payload = state.ingest_full(
            grid([-1, 0, 42, 100]), "main_floor", "LOCALIZATION")
        self.assertEqual("map_snapshot", event)
        self.assertEqual(ENCODING, payload["encoding"])
        self.assertEqual("map", payload["frameId"])
        self.assertEqual(-1.0, payload["origin"]["position"]["x"])
        self.assertEqual([-1, 0, 42, 100], decode(payload))

    def test_small_full_grid_change_becomes_sequenced_update(self):
        state = MapStreamState()
        state.ingest_full(grid([0] * 100, 10, 10))
        event, payload = state.ingest_full(grid([0] * 99 + [100], 10, 10))
        self.assertEqual("map_update", event)
        self.assertEqual(1, payload["baseSequence"])
        self.assertEqual(2, payload["sequence"])
        self.assertEqual((9, 9, 1, 1),
                         (payload["x"], payload["y"], payload["width"], payload["height"]))
        self.assertEqual([100], decode(payload))

    def test_update_bounds_and_malformed_full_grids_are_rejected(self):
        state = MapStreamState()
        state.ingest_full(grid([0, 0, 0, 0]))
        update = ns(header=ns(frame_id='map', stamp=ns(sec=2, nanosec=0)), x=2, y=0,
                    width=1, height=1, data=[1])
        with self.assertRaises(MapStreamError):
            state.ingest_update(update)
        with self.assertRaises(MapStreamError):
            state.ingest_full(grid([0, 0, 0]))

    def test_unchanged_grid_does_not_advance_sequence(self):
        state = MapStreamState()
        state.ingest_full(grid([0, 0, 0, 0]))
        self.assertIsNone(state.ingest_full(grid([0, 0, 0, 0])))
        self.assertEqual(1, state.sequence)

    def test_identity_change_forces_complete_snapshot(self):
        state = MapStreamState()
        state.ingest_full(grid([0, 0, 0, 0]), 'old', 'LOCALIZATION')
        event, payload = state.ingest_full(grid([0, 0, 0, 0]), 'new', 'LOCALIZATION')
        self.assertEqual('map_snapshot', event)
        self.assertEqual('new', payload['mapId'])

    def test_update_changes_canonical_snapshot(self):
        state = MapStreamState()
        state.ingest_full(grid([0] * 100, 10, 10))
        update = ns(header=ns(frame_id='map', stamp=ns(sec=2, nanosec=0)),
                    x=9, y=9, width=1, height=1, data=[100])
        event, payload = state.ingest_update(update)
        self.assertEqual('map_update', event)
        self.assertEqual(1, payload['baseSequence'])
        self.assertEqual(100, decode(state.snapshot())[-1])


if __name__ == "__main__":
    unittest.main()
