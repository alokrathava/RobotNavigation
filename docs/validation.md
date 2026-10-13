# SLAM and Navigation Validation

Build from the repository root after sourcing ROS 2 Lyrical:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select robot_gateway robot_navigation
colcon test-result --verbose
```

For the Gazebo scenario, run `./run.sh --headless --mode navigation`, connect a Protocol v1 client, subscribe to
the map, start mapping, and drive with `manual_velocity`. Confirm snapshot sequences advance,
save without overwrite, stop mapping, and activate the saved map. Restart in navigation mode
and confirm the saved active map returns with `localizationReady` and
`navigationRuntimeReady`. Send a reachable `navigate_to`, observe `navigation_result`, then
send another goal followed by `cancel_navigation`; `/cmd_vel` must settle at zero.

Also verify the negative paths: duplicate save without `overwrite`, malformed or occupied
initial pose, navigation while localization is lost, overlapping map operations, an invalid
or corrupted map directory, and map switching during active navigation. The map transition
hold must remain asserted until qualified localization returns or rollback completes.

## Automated results (2026-10-12)

ROS 2 Lyrical, Gazebo simulation, project Python 3.14.4; no hardware or Android build.
All five packages built successfully. The direct regression command below passed
**69 tests and 86 subtests**. Package tests also passed; their aggregate reporting count
is different from individual pytest cases.

```bash
.venv/bin/python -m pytest -q \
  src/robot_gateway/test_map_stream.py \
  src/robot_gateway/test_navigation_actions.py \
  src/robot_gateway/test_gateway_safety.py \
  src/robot_gateway/test_protocol_fixtures.py src/robot_navigation/test
git diff --check
```

Coverage includes lifecycle failure/admission, checkpoint shutdown guards, corrupted maps
and metadata, active-map persistence, saved-grid geometry/alignment, activation rollback
and rollback failure, covariance, stale localization admission, planner rejection,
simultaneous goals, timeout cancellation failure, protocol fixtures, occupancy encoding,
map identity resynchronization, stale scans, manual takeover, and E-stop release with
an active goal.
The final velocity watchdog also has a regression proving silent input forces zero output.
Stationary covariance roundoff is clamped only within 1e-12; materially negative,
nonfinite or excessive covariance remains invalid. Automatic global recovery cannot
replace the initial pose estimate while a map activation is still running.

## Reproduce the actual WebSocket scenario

Use a new isolated map root. Keep the launcher running in one terminal and run the client
in another sourced terminal. The launcher seeds the bundled map automatically.

```bash
./run.sh --headless --mode navigation --gateway-port 18080 \
  --maps-dir /tmp/robotnavigation-validation
.venv/bin/python ci/gazebo_websocket_validation.py --phase mapping
```

Stop the launcher with Ctrl-C and restart with the same arguments/storage root. Then:

```bash
.venv/bin/python ci/gazebo_websocket_validation.py --phase navigation --switch-before-navigation
.venv/bin/python ci/gazebo_websocket_validation.py --phase safety
.venv/bin/python ci/gazebo_websocket_validation.py --phase transitions
.venv/bin/python ci/gazebo_websocket_validation.py --phase localization
```

The client asserts real grid decoding, manual displacement, map completion, localization,
destination arrival, native action results, stopped telemetry and path bounds. The
transitions phase switches during a goal, observes cancellation/hold, and reconnects for
a complete snapshot. The localization phase temporarily deactivates AMCL and always
attempts reactivation.

For unsaved recovery, run `--phase checkpoint`, stop/restart with the same storage root,
then run `--phase mapping --resume-unsaved`.

## Executed live evidence

- Final-source P0: mapping, 0.376 m manual displacement, occupancy decoding, save,
  rename/delete, duplicate-save and unsaved-stop rejection, stop and activation passed.
  `e2e_validation` became active with localization and navigation runtime ready.
- Earlier live runs: persisted-map restart, both Nav2 actions, arrival, cancellation,
  stopped motion, manual takeover and dead-man passed. E-stop during pending navigation
  acknowledged in 0.114 s.
- Earlier checkpoint restart restored the native serialized SLAM graph and continued
  manual mapping. These tests used actual Gazebo/ROS data, with no mocked motion.
- Final restart: generated map restored, localization qualified, then a live same-map
  switch completed. `NavigateToPose` succeeded in 1.872 s and `NavigateThroughPoses`
  in 1.984 s; arrival, cancellation, stopped telemetry and path streaming passed.
- Final safety: manual takeover and dead-man passed; E-stop acknowledged in 0.114 s.
  Releasing E-stop immediately after stopping an active goal resulted in native
  cancellation and stopped telemetry before motion was released.
- Live map switch during navigation canceled the goal and asserted motion hold.
  A second connection received a complete decoded snapshot of the active saved map.
- Live AMCL pause via the localization manager revoked readiness and rejected autonomy
  with `LOCALIZATION_NOT_READY`; resume restored qualified localization and runtime.

Isolated artifacts: `/tmp/robotnavigation-e2e-maps-20261012-final`. Functional logs:
`~/.robot/logs/robot-navigation-20261012-141856.log` (P0) and
`robot-navigation-20261012-142429.log` (restart, actions, safety, transition and loss).
An intermediate restart failed localization safely; this exposed and led to the
global-recovery-during-activation fix. Failed runs are not counted as passes.
After the final covariance correction, restart and map switching qualified successfully;
both actions again succeeded (2.008 s and 2.053 s), with cancellation and stopped motion.
Log: `~/.robot/logs/robot-navigation-20261012-142737.log`.
The final-source safety rerun also passed all assertions; pending-navigation E-stop
acknowledged in 0.111 s, including immediate release with an active goal.

## Task checklist

| Area | Implementation | Evidence |
| --- | --- | --- |
| SLAM lifecycle/manual mapping/unsaved recovery | DONE | Lifecycle tests; live mapping and checkpoint restart |
| Map CRUD, activation and persistence | DONE | Storage/rollback tests; live CRUD and activation |
| Versioned grid streaming and size bounds | DONE | Encoding/sequence/fixture tests; decoded live grid |
| Transition hold and transform ownership | DONE | Admission/failure tests; live SLAM/AMCL transitions |
| AMCL readiness/recovery/autonomy gate | DONE | Covariance/admission tests; live localization |
| Nav2 actions/results/path/cancel/timeout | DONE | Action tests; live actions and cancellation |
| Manual takeover/E-stop/collision/watchdog/dead-man | DONE | Safety regressions; live takeover/E-stop/dead-man |
| Exhaustive live storage/process failure injection | PARTIAL | Automatic rollback/timeout failure tests; further live injection remains |

## Known issues and remaining checks

- Installed native `ros_gz_bridge` and occasionally the Nav2 component container crash
  during SIGINT teardown in this environment. The launcher escalates shutdown only for
  its owned process group. Functional tests pass; native clean shutdown remains an
  upstream/environment issue. Installed dependencies were not rebuilt.
- Exhaustive live disk-full, sudden process death during commit, sensor dropout, and
  simultaneous localization/rollback failure remain additional fault-injection coverage.
  Automated failure tests are not represented as live runs.
- Collision monitor lifecycle and its velocity pipeline were active in Gazebo. A dedicated
  obstacle-contact/polygon-boundary live test was not run; configured zones and stale-scan
  gating were reviewed, and the latter has an automated regression.
- Android SDK/client execution was not performed. Passing Protocol v1 fixtures and
  gateway tests does not certify every external client implementation.
- One final navigation attempt was canceled safely after transient localization readiness
  loss; readiness recovered and the subsequent complete scenario passed. This remains a
  simulation stability observation requiring longer-duration characterization. The gates
  were not weakened to keep that goal running.
