# Runtime & Deployment

## 1. Deployment Goal

A robot project should not require the operator to remember a long sequence of ROS commands.

The repository uses two top-level scripts:

~~~text
setup.sh   machine/workspace preparation
run.sh     runtime supervision
~~~

The intended experience is:

~~~bash
git clone <repo>
cd RobotNavigation
./setup.sh
./run.sh
~~~

## 2. setup.sh Responsibilities

### Environment discovery

It reports:

- Ubuntu version;
- architecture;
- Python version;
- ROS distribution;
- available memory;
- free disk;
- repository location.

### Dependency installation

It verifies/installs packages such as:

- git;
- curl;
- Python development tooling;
- build-essential;
- cmake;
- rosdep;
- colcon;
- iproute2.

ROS dependencies are resolved from package manifests using rosdep rather than duplicated manually.

### Python environment

The WebSocket gateway requires a modern WebSocket Python package while ROS Python packages usually live in the system environment.

The setup script therefore creates:

~~~text
.venv
~~~

with:

~~~bash
python3 -m venv --system-site-packages .venv
~~~

This allows the virtual environment to see system-installed <code>rclpy</code> while still controlling the project's Python dependencies.

### Build

~~~bash
colcon build --symlink-install
~~~

The script tracks a build stamp containing ROS distribution, architecture, Python path, and repository location. It cleans stale build state when those assumptions change.

### Validation

After build, it checks:

- robot_bringup;
- robot_description;
- robot_interfaces;
- robot_navigation;
- robot_gateway;
- installed launch files;
- Python script executability;
- Python compilation;
- gateway importability;
- top-level launch parsing.

## 3. run.sh Responsibilities

<code>run.sh</code> is a supervisor around:

~~~bash
ros2 launch robot_bringup robot.launch.py
~~~

It deliberately does **not** launch every ROS node manually.

The launch system and lifecycle coordinator remain authoritative.

## 4. Process Ownership

A common anti-pattern is:

~~~bash
pkill ros2
pkill python3
killall gz
~~~

RobotNavigation does not do this.

Instead it starts the robot stack with its own:

- session;
- process group;
- unique run ID.

Runtime metadata lives under:

~~~text
~/.robot/run/robot_navigation.pid
~~~

The metadata records:

- launch PID;
- process group ID;
- process start time;
- supervisor PID;
- run ID;
- backend;
- mode;
- maps directory;
- gateway settings;
- ROS distribution;
- log path.

The start-time check protects against PID reuse.

## 5. Stale / Orphan Detection

Before a new run, the launcher distinguishes:

- a valid running RobotNavigation instance;
- a stale PID file;
- a dead leader with RobotNavigation-owned orphan processes;
- unrelated processes.

<code>--clean</code> stops only the previous RobotNavigation-owned process group.

## 6. Readiness Probe

The supervisor does not declare success when <code>ros2 launch</code> merely starts.

Its embedded ROS probe observes real runtime state.

Navigation mode checks signals such as:

- map state;
- navigation runtime readiness;
- map topic;
- odometry;
- LiDAR;
- TF;
- navigation actions;
- gateway TCP listening state.

Mapping mode uses mapping-appropriate readiness instead of requiring navigation to be active.

## 7. Gateway Preflight

Before launch:

- port range is validated;
- port conflicts are detected with <code>ss</code>;
- loopback vs non-loopback bind is checked;
- non-loopback bind requires authentication.

Default:

~~~text
127.0.0.1:8080
~~~

Network exposure:

~~~bash
export ROBOT_GATEWAY_TOKEN="..."
./run.sh --gateway-host 0.0.0.0
~~~

## 8. Logging

Logs are timestamped under:

~~~text
~/.robot/logs/
~~~

Example:

~~~text
robot-navigation-20261004-173000.log
~~~

The full ROS launch output stays in the log.

The terminal mirrors important lines such as:

- ERROR;
- FATAL;
- process death;
- tracebacks;
- gateway listening.

## 9. Shutdown

Normal shutdown path:

~~~text
Ctrl+C
 ↓
SIGINT process group
 ↓
wait
 ↓
SIGTERM if necessary
 ↓
wait
 ↓
SIGKILL only as last resort
~~~

Maps are left untouched.

## 10. SBC Deployment Path

Once the project is stable on the selected SBC, <code>run.sh</code> is a suitable boundary to place under a service manager such as systemd.

Recommended future deployment model:

~~~text
Power on
 ↓
Ubuntu boots
 ↓
systemd service
 ↓
run.sh --backend hardware
 ↓
robot.launch.py
 ↓
hardware drivers + RobotNavigation
~~~

Systemd should be added only after hardware startup, restart behavior, and shutdown are validated.

## 11. Runtime Files

~~~text
~/.robot/
├── maps/    persistent maps and per-map data
├── logs/    timestamped runtime logs
└── run/     PID/run metadata
~~~
