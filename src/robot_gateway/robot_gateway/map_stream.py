"""Versioned, bounded JSON representation of ROS occupancy grids.

ROS message objects stay inside the gateway.  Clients receive compressed signed-int8
cell data together with all geometry needed to reconstruct the grid.
"""

from __future__ import annotations

import base64
import math
import zlib


ENCODING = "occupancy-int8-zlib-base64-v1"
STREAM_VERSION = 1
MAX_CELLS = 4_000_000
MAX_ENCODED_BYTES = 5_000_000


class MapStreamError(ValueError):
    pass


def _encode(values):
    raw = bytes((int(value) & 0xff) for value in values)
    encoded = base64.b64encode(zlib.compress(raw, level=6)).decode("ascii")
    if len(encoded.encode("ascii")) > MAX_ENCODED_BYTES:
        raise MapStreamError("encoded occupancy grid exceeds the 5 MB limit")
    return encoded


def _pose(pose):
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }


def _stamp(stamp):
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


class MapStreamState:
    """Canonical grid plus sequencing for snapshots and bounded rectangular updates."""

    def __init__(self):
        self.sequence = 0
        self.frame_id = ""
        self.stamp = None
        self.width = 0
        self.height = 0
        self.resolution = 0.0
        self.origin = None
        self.cells = []
        self.map_id = ""
        self.mode = "NONE"

    @property
    def available(self):
        return bool(self.width and self.height and self.origin is not None)

    @staticmethod
    def _validate(width, height, resolution, values):
        if width <= 0 or height <= 0 or width * height > MAX_CELLS:
            raise MapStreamError("occupancy grid dimensions are invalid or too large")
        if len(values) != width * height:
            raise MapStreamError("occupancy grid cell count does not match its dimensions")
        if not math.isfinite(resolution) or resolution <= 0.0:
            raise MapStreamError("occupancy grid resolution is invalid")
        if any(int(value) < -1 or int(value) > 100 for value in values):
            raise MapStreamError("occupancy grid contains a value outside -1..100")

    def ingest_full(self, msg, map_id="", mode="NONE"):
        width, height = int(msg.info.width), int(msg.info.height)
        resolution = float(msg.info.resolution)
        values = [int(value) for value in msg.data]
        self._validate(width, height, resolution, values)
        origin = _pose(msg.info.origin)
        components = list(origin['position'].values()) + list(origin['orientation'].values())
        if not all(math.isfinite(value) for value in components):
            raise MapStreamError('occupancy grid origin must be finite')
        norm = sum(value * value for value in origin['orientation'].values())
        if abs(norm - 1.0) > 0.01 or not str(msg.header.frame_id):
            raise MapStreamError('occupancy grid needs a frame and unit origin quaternion')
        geometry_changed = (
            width != self.width or height != self.height
            or abs(resolution - self.resolution) > 1e-12
            or self.origin is None or _pose(msg.info.origin) != self.origin
            or str(msg.header.frame_id) != self.frame_id
        )
        previous = self.sequence
        changed = [index for index, value in enumerate(values)
                   if geometry_changed or index >= len(self.cells) or value != self.cells[index]]
        if not geometry_changed and not changed and str(map_id) == self.map_id and str(mode) == self.mode:
            self.stamp = msg.header.stamp
            return None
        identity_changed = str(map_id) != self.map_id or str(mode) != self.mode
        self.sequence += 1
        self.frame_id = str(msg.header.frame_id)
        self.stamp = msg.header.stamp
        self.width, self.height, self.resolution = width, height, resolution
        self.origin = _pose(msg.info.origin)
        self.cells = values
        self.map_id, self.mode = str(map_id), str(mode)
        if geometry_changed or identity_changed or len(changed) > int(width * height * 0.35):
            return "map_snapshot", self.snapshot()
        left = min(index % width for index in changed)
        right = max(index % width for index in changed)
        top = min(index // width for index in changed)
        bottom = max(index // width for index in changed)
        if (right - left + 1) * (bottom - top + 1) > int(width * height * 0.35):
            return 'map_snapshot', self.snapshot()
        patch = [values[row * width + column]
                 for row in range(top, bottom + 1)
                 for column in range(left, right + 1)]
        return "map_update", self._update(previous, left, top,
                                          right - left + 1, bottom - top + 1, patch)

    def ingest_update(self, msg):
        if not self.available:
            raise MapStreamError("cannot apply a map update before a complete snapshot")
        if msg.header.frame_id != self.frame_id:
            raise MapStreamError('occupancy update frame does not match the current map')
        x, y = int(msg.x), int(msg.y)
        width, height = int(msg.width), int(msg.height)
        values = [int(value) for value in msg.data]
        if (x < 0 or y < 0 or width <= 0 or height <= 0
                or x + width > self.width or y + height > self.height):
            raise MapStreamError("occupancy update is outside the current map")
        if len(values) != width * height or any(value < -1 or value > 100 for value in values):
            raise MapStreamError("occupancy update payload is invalid")
        previous = self.sequence
        changed = False
        for row in range(height):
            start = (y + row) * self.width + x
            incoming = values[row * width:(row + 1) * width]
            if self.cells[start:start + width] != incoming:
                self.cells[start:start + width] = incoming
                changed = True
        if not changed:
            return None
        self.sequence += 1
        self.stamp = msg.header.stamp
        if width * height > int(self.width * self.height * 0.35):
            return "map_snapshot", self.snapshot()
        return "map_update", self._update(previous, x, y, width, height, values)

    def snapshot(self):
        if not self.available:
            raise MapStreamError("no occupancy grid is available")
        return {
            "streamVersion": STREAM_VERSION,
            "sequence": self.sequence,
            "mapId": self.map_id,
            "mode": self.mode,
            "frameId": self.frame_id,
            "stamp": _stamp(self.stamp),
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": self.origin,
            "encoding": ENCODING,
            "cellCount": len(self.cells),
            "data": _encode(self.cells),
        }

    def _update(self, base_sequence, x, y, width, height, values):
        return {
            "streamVersion": STREAM_VERSION,
            "baseSequence": base_sequence,
            "sequence": self.sequence,
            "mapId": self.map_id,
            "frameId": self.frame_id,
            "stamp": _stamp(self.stamp),
            "x": x,
            "y": y,
            "width": width,
            "height": height,
            "encoding": ENCODING,
            "cellCount": len(values),
            "data": _encode(values),
        }
