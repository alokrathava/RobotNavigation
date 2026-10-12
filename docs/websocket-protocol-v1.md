# RobotNavigation WebSocket Protocol v1

Every message is a JSON object with `type` and `protocolVersion: 1`. Commands may carry
an `id`; acknowledgements and errors repeat it. Existing Protocol v1 messages and fields
remain valid. A successful `command_ack` means a long running ROS operation was admitted.
Its actual progress and completion are reported by state or result events.

The `hello_ack.payload.capabilities` list advertises optional behavior. This release adds
`map_streaming`, `localization_recovery`, and `navigation_path`.

## Map lifecycle

`start_mapping` accepts optional Boolean `resumeUnsaved` and `discardUnsaved` fields. They
are mutually exclusive. If a recoverable SLAM session exists after a restart, the caller
must select one. `stop_mapping.payload.discardUnsaved` explicitly permits discarding an
unsaved session. `save_map.payload` contains `name` and optional `overwrite`; overwrite
defaults to `false`.

`map_state.payload` contains `state`, `mode`, `activeMapId`, `mapAvailable`,
`localizationReady`, `navigationRuntimeReady`, `unsavedChanges`, `recoverableSession`,
`errorCode`, and `detail`. Motion remains inhibited during map changes until the map,
AMCL covariance, required TF chain, and navigation runtime are ready. A failed switch
restores the previous map when possible.

## Occupancy-grid stream

Send `subscribe_map` to opt in. The gateway immediately sends a complete `map_snapshot`
when a map is available, then sends `map_update` events at no more than 2 Hz. Send
`request_map_snapshot` to resynchronize after a missing sequence and `unsubscribe_map`
to stop the stream.

A snapshot provides `streamVersion`, `sequence`, `mapId`, `mode`, `frameId`, ROS `stamp`,
`width`, `height`, `resolution`, the complete origin position and quaternion, `encoding`,
`cellCount`, and `data`. An update provides `baseSequence`, `sequence`, the changed
rectangle (`x`, `y`, `width`, `height`), and the same encoding metadata. Geometry changes
always produce a new snapshot.

Queued changes may be coalesced into a complete snapshot. Accept any newer snapshot as
a baseline; apply an update only when `baseSequence` matches the current sequence.
Sequences are scoped to a gateway process: reconnect and subscribe again after restart.
Map identity or mode changes also force a snapshot. Clients need no ROS SDK.

`occupancy-int8-zlib-base64-v1` means the cells are ROS signed int8 occupancy values in
row-major order (`-1` unknown, `0..100` known), represented as their two's-complement
bytes, compressed with zlib, then base64 encoded. Clients must verify `cellCount`, sequence,
and decoded dimensions. The gateway rejects maps above 4,000,000 cells or 5 MB encoded.

## Localization

`set_initial_pose.payload` contains finite `xMeters`, `yMeters`, and optional
`yawRadians`. The pose must be in a known free cell of the active map. The gateway publishes
it to AMCL on `/initialpose`. `reinitialize_localization` requests AMCL global
localization. RobotNavigation also requests global localization after sustained loss, with
a cooldown, once no map activation is in progress. Autonomous commands are admitted only
while localization state is `LOCALIZED`.

## Navigation

`navigate_to.payload` contains finite `xMeters`, `yMeters`, `yawRadians`, and optional
`timeoutSeconds`. `navigate_through.payload.waypoints` contains up to 100 poses and accepts
the same timeout. The timeout defaults to 300 seconds and must be from 1 through 3600.
Before execution, the gateway requires fresh localized pose and safety state, an active
map and runtime, and a successful Nav2 `ComputePath` result.
The server timeout default is configurable through `navigation_timeout_seconds`.

Accepted goals end with a `navigation_result` event containing `requestId`, `goalId`,
`action`, `status`, `errorCode`, `errorMessage`, and `durationSeconds`. Status is one of
`SUCCEEDED`, `CANCELED`, `ABORTED`, `TIMED_OUT`, or `FAILED`.

Send `subscribe_navigation_path` to receive `navigation_path` events at no more than 2 Hz.
Each event identifies `planned` or `travelled`, carries frame and stamp, and contains up to
500 poses; `truncated` reports omitted older poses. Send `unsubscribe_navigation_path` to
stop them.

The canonical examples are the JSON files in [`protocol-fixtures`](../protocol-fixtures).

## Motion safety

Nonzero manual input cancels autonomy. E-stop and stop remain responsive during planning
and goal admission. Map transitions invalidate pending navigation and hold final velocity
output until localization and runtime qualify. Cancellation without a confirmed terminal
result engages E-stop. E-stop release holds motion until goals and recent commands stop.

## Map persistence and recovery

Maps default to `~/.robot/maps` (override with `--maps-dir`). Saved maps contain validated
Nav2 trinary PGM/YAML and metadata. Saving captures the real SLAM grid, preserves its
resolution and origin, and atomically commits a staged directory. Metadata preserves
measured map-to-odometry alignment through rename and restart. The active-map marker is
committed only after successful activation.

Unsaved sessions are checkpointed every ten seconds using SLAM Toolbox native pose-graph
serialization. Immutable generations and an atomic session marker support restart recovery
through native deserialization. Interrupted sessions require explicit resume or discard.
Checkpoints share the map-operation lock and cannot start during shutdown or failed mapping.
SLAM and AMCL do not run as simultaneous map-to-odometry publishers.
