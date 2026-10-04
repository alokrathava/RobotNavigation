# Testing & Validation

## 1. Validation Philosophy

Robot software often appears to work during one interactive session and fails after a restart, network disconnect, lifecycle race, or CPU spike.

RobotNavigation therefore treats repeatability as a separate engineering requirement from feature implementation.

## 2. Current Test Assets

The repository includes cross-language protocol fixture tests under <code>robot_gateway</code>.

They validate fixture concepts such as:

- hello;
- hello_ack;
- telemetry;
- save map;
- virtual wall creation;
- saved location;
- mission submission.

The fixture tests are intended to use the same protocol examples as the companion Android RobotSDK project.

Set:

~~~bash
export PROTOCOL_FIXTURES_DIR=/path/to/Robot-Controller/protocol-fixtures
~~~

before running them on a machine where the companion repository is stored elsewhere.

## 3. Setup-Time Validation

<code>setup.sh</code> performs practical checks rather than only running colcon.

It verifies:

- ROS package discovery;
- installed launch files;
- executable Python scripts;
- Python compilation;
- gateway import;
- launch parsing;
- map directory writability.

## 4. Runtime Validation

<code>run.sh</code> uses an embedded rclpy probe to wait for mode-appropriate readiness.

It validates more than process existence.

Navigation-mode expectations include real robot graph/runtime signals.

If readiness fails, the launcher prints the tail of the runtime log and tears down the partially started process group.

## 5. Navigation Reliability Tests

Recommended repeated sequence:

~~~text
cold start
 ↓
runtime_ready=true
 ↓
NavigateToPose
 ↓
cancel
 ↓
NavigateThroughPoses
 ↓
cancel
 ↓
NavigateToPose
 ↓
mission
 ↓
disconnect/reconnect
 ↓
clean shutdown
~~~

Run the sequence repeatedly, including under representative CPU load.

## 6. Gateway Failure Diagnostics

The gateway differentiates:

~~~text
server unavailable
goal response timeout
goal send exception
Nav2 rejected goal
navigator cancellation timeout
~~~

When a navigation failure occurs, capture:

~~~bash
ros2 lifecycle get /bt_navigator
ros2 topic echo --once /robot/navigation/runtime_ready
ros2 action list -t
ros2 topic echo /navigate_to_pose/_action/status
ros2 topic echo /navigate_through_poses/_action/status
~~~

alongside gateway navigation diagnostics.

## 7. Map Acceptance Tests

### Persistence

- create/save map;
- restart;
- active map still exists;
- metadata valid.

### Switching

~~~text
Map A → Map B → Map A
~~~

Confirm:

- navigation is held during transition;
- correct map is active;
- AMCL resumes;
- virtual walls follow the map;
- navigation runtime returns ready.

### Mapping

- start mapping;
- drive/teleop;
- save;
- stop mapping;
- activate saved map;
- localize;
- navigate.

### Corrupt input

Verify malformed map metadata/image/path is rejected safely.

## 8. Safety Acceptance Tests

### Dead-man

Send non-zero manual velocity, stop the command stream, confirm zero velocity is injected within the configured timeout.

### E-stop

While moving:

1. trigger emergency stop;
2. confirm output zero;
3. confirm active navigation cancelled;
4. release E-stop;
5. confirm stale input does not restart motion;
6. send fresh command.

### Collision monitor

Move toward an obstacle and verify slowdown/stop state is reflected through safety and health.

### Pipeline watchdog

Simulate command-pipeline silence while last output is non-zero; confirm final zero output and fault.

## 9. Mission Ownership Tests

Required scenario:

~~~text
Client A starts long mission
Client A disconnects
Client B takes control within 10 s
Mission continues
Client B disconnects while mission still active
No client reconnects
Grace expires
Mission cancels
Underlying navigation cancels
Robot stops safely
~~~

This validates mission ownership rather than only mission sequencing.

## 10. Deployment Acceptance

Before using a physical SBC as an unattended robot controller, validate:

- 20+ clean cold starts;
- clean Ctrl+C shutdown;
- restart after forced process termination;
- stale PID recovery;
- occupied gateway port;
- missing/invalid ROS environment;
- network gateway without token;
- network disconnect/reconnect;
- map persistence;
- low disk conditions;
- thermal load;
- system reboot.

## 11. What Is Not Yet Proven

Simulation cannot prove:

- final motor safety;
- real encoder quality;
- LiDAR mounting/calibration;
- wheel slip behavior;
- real BMS integration;
- physical docking repeatability;
- EMI/power integrity;
- physical E-stop compliance.

Those should become a separate hardware acceptance phase.
