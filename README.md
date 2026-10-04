# RobotNavigation

<p align="center">
  <strong>ROS 2 autonomous mobile-robot runtime with Nav2, SLAM, persistent maps, safety supervision, docking, telemetry, health monitoring, and an application-facing WebSocket gateway.</strong>
</p>

<p align="center">
  <img alt="ROS 2" src="https://img.shields.io/badge/ROS%202-Robot%20Runtime-111111?style=flat-square">
  <img alt="Nav2" src="https://img.shields.io/badge/Nav2-Autonomous%20Navigation-111111?style=flat-square">
  <img alt="Python" src="https://img.shields.io/badge/Python-rclpy-111111?style=flat-square&logo=python">
  <img alt="Gazebo" src="https://img.shields.io/badge/Gazebo-Simulation-111111?style=flat-square">
  <img alt="Protocol" src="https://img.shields.io/badge/WebSocket%20Protocol-v1-111111?style=flat-square">
</p>

---

## Overview

**RobotNavigation** is the robot-side autonomy platform for a small mobile robot. It combines standard ROS 2 navigation components with custom robot-level orchestration needed to turn a Nav2 stack into an application-controlled product:

- autonomous pose and waypoint navigation;
- AMCL localization and SLAM Toolbox mapping;
- persistent multi-map lifecycle management;
- map-scoped virtual walls rendered into Nav2 keepout filters;
- manual/autonomous command arbitration;
- velocity smoothing and collision monitoring;
- software E-stop, dead-man timeout, stale-command watchdogs, and transition holds;
- docking / undocking orchestration;
- battery, telemetry, and health aggregation;
- custom robot-level ROS messages and services;
- a WebSocket-to-ROS gateway for Android or other external clients;
- setup and runtime scripts for repeatable deployment on Ubuntu/SBC systems;
- a clean simulation-to-hardware boundary.

This repository is intentionally **simulation-first**. The complete robot software stack can be exercised in Gazebo before the final physical robot and hardware drivers are introduced.

> **Engineering boundary:** Nav2, localization, safety, mapping, and physical execution remain on the robot. External applications request behavior and observe state; they do not implement the robot's autonomy loop.

---

## 60-Second Recruiter View

If you are reviewing this project for a software, robotics, embedded-Linux, or platform-engineering role, the main engineering work is **not simply installing Nav2**.

The custom platform layer includes:

| Area | Custom engineering |
|---|---|
| Runtime orchestration | Lifecycle-aware startup coordinator and robot-level readiness signal |
| Deployment | Idempotent <code>setup.sh</code>, supervised <code>run.sh</code>, process-group ownership, readiness probing |
| Map platform | Persistent writable map store, validation, activation, mapping transitions, restart persistence |
| Safety | Final velocity gate, E-stop, dead-man, stale pipeline watchdog, map-transition hold |
| Spatial policy | Per-map virtual walls converted to Nav2 keepout masks |
| Application integration | Versioned WebSocket protocol translating commands to ROS topics/services/actions |
| Gateway reliability | Action-response diagnostics, cross-navigator arbitration, mission ownership/grace handling |
| Observability | Aggregated telemetry, health evaluation, stable robot error model, ROS diagnostics |
| Simulation/hardware split | Same high-level stack with simulated or physical backend interfaces |

The result is a robot-side platform with a clear path from:

~~~text
Android / External App
        ↓
     RobotSDK
        ↓
 WebSocket Protocol
        ↓
   robot_gateway
        ↓
 ROS 2 Robot Runtime
        ↓
 Nav2 + SLAM + Safety
        ↓
 Gazebo / Physical Robot
~~~

---

## System Architecture

~~~mermaid
flowchart LR
    APP[Android / Operator App]
    SDK[RobotSDK]
    GW[robot_gateway<br/>Protocol v1]

    subgraph RUNTIME[RobotNavigation]
        MAP[Map Manager]
        WALL[Virtual Wall Manager]
        COORD[Navigation Runtime Coordinator]
        NAV[Nav2]
        SLAM[SLAM Toolbox]
        SAFE[Safety Pipeline]
        DOCK[Docking Runtime]
        STATE[Telemetry + Health]
    end

    BACKEND[Gazebo / Hardware Drivers]

    APP --> SDK
    SDK <-->|WebSocket| GW

    GW <--> MAP
    GW <--> WALL
    GW <--> NAV
    GW <--> DOCK
    GW <--> STATE

    MAP <--> SLAM
    MAP --> COORD
    COORD --> NAV
    WALL --> NAV
    NAV --> SAFE
    SAFE --> BACKEND
    BACKEND --> NAV
    BACKEND --> STATE
~~~

### Responsibility split

**External application**
- operator UX;
- robot selection and connection;
- requests actions;
- visualizes state.

**robot_gateway**
- authenticates clients;
- negotiates protocol capabilities;
- translates application commands into ROS actions, services, and topics;
- broadcasts robot state;
- manages command/control session ownership.

**RobotNavigation**
- owns robot-side autonomy and runtime state;
- manages maps and localization;
- enforces software motion safety;
- aggregates telemetry and health.

**Gazebo / hardware drivers**
- provide sensors, odometry, drivetrain, battery, and docking signals.

---

## Core ROS Packages

The workspace is split into five ROS packages:

~~~text
src/
├── robot_bringup/       Top-level launch and simulation composition
├── robot_description/   Robot model and robot_state_publisher
├── robot_gateway/       WebSocket ↔ ROS 2 application gateway
├── robot_interfaces/    Robot-level messages and services
└── robot_navigation/    Nav2, SLAM, maps, safety, docking, telemetry, health
~~~

### robot_bringup

Provides the preferred single entry point:

~~~bash
ros2 launch robot_bringup robot.launch.py
~~~

The launch supports:

~~~text
backend:=sim | hardware
mode:=navigation | mapping
gateway:=true | false
gateway_host:=127.0.0.1
gateway_port:=8080
maps_dir:=~/.robot/maps
rviz:=true | false
~~~

### robot_navigation

Integrates and configures:

- Nav2 planner, MPPI controller, behaviors, and BT navigator;
- AMCL;
- SLAM Toolbox;
- velocity smoother;
- collision monitor;
- Open Navigation docking;
- custom safety supervisor;
- custom map manager;
- custom virtual-wall manager;
- navigation monitoring;
- battery and docking state managers;
- robot telemetry and health monitors;
- runtime readiness coordinator.

### robot_interfaces

Defines stable robot-level abstractions such as:

- <code>RobotTelemetry</code>
- <code>RobotHealth</code>
- <code>RobotError</code>
- <code>RobotBatteryState</code>
- <code>SafetyState</code>
- <code>DockingState</code>
- <code>MapInfo</code>
- <code>MapState</code>
- <code>VirtualWall</code>

and services for map and virtual-wall lifecycle operations.

### robot_gateway

Bridges Protocol v1 WebSocket messages to ROS 2 and currently advertises capabilities for:

- navigation;
- waypoint navigation;
- manual control;
- docking;
- safety;
- telemetry;
- battery;
- health;
- map switching and map management;
- mapping;
- virtual walls;
- saved locations;
- missions;
- runtime robot configuration;
- diagnostics;
- log retrieval.

---

## Navigation

The runtime supports both standard Nav2 navigation frontends:

~~~text
/navigate_to_pose
/navigate_through_poses
~~~

The custom navigation monitor follows both action types and publishes operator-facing state such as the travelled path and active goal.

The gateway also explicitly handles cross-navigator transitions. Because Nav2 can reject a goal for one navigator while the other navigator is active, the gateway cancels and waits out the conflicting navigator before sending the replacement goal.

For intermittent action-response failures, the gateway distinguishes:

- action server unavailable;
- goal send exception;
- goal-response timeout;
- actual Nav2 <code>accepted=false</code> rejection.

It also records response latency, runtime readiness, BT navigator lifecycle state, action status visibility, and ROS/asyncio loop timing around navigation requests.

See [Navigation & Safety](docs/navigation-and-safety.md).

---

## Lifecycle-Aware Runtime Readiness

RobotNavigation does not use a fixed sleep to assume that Nav2 is ready.

A custom <code>navigation_runtime_coordinator</code> checks:

1. map bootstrap;
2. localization lifecycle state;
3. navigation-core lifecycle state;
4. planner/controller/behavior action availability;
5. required costmap services;
6. BT navigator and docking lifecycle state;
7. final frontend action availability.

It then publishes:

~~~text
/robot/navigation/runtime_ready
~~~

The external application and deployment tooling can consume one robot-level readiness signal instead of guessing from multiple ROS components.

---

## Safety Architecture

The velocity path is deliberately layered:

~~~mermaid
flowchart LR
    MAN[Manual Velocity]
    AUTO[Nav2 Velocity]
    MUX[twist_mux]
    SMOOTH[Velocity Smoother]
    COLL[Collision Monitor]
    SAFE[Safety Supervisor]
    CMD[/cmd_vel]
    DRIVE[Drivetrain]

    MAN --> MUX
    AUTO --> MUX
    MUX --> SMOOTH --> COLL --> SAFE --> CMD --> DRIVE
~~~

The custom safety supervisor is the final software publisher of <code>/cmd_vel</code>.

It implements:

- software emergency-stop gating;
- cancellation of active navigation/docking during E-stop;
- manual-control dead-man timeout;
- velocity-pipeline watchdog;
- scan freshness monitoring;
- map-transition motion hold;
- navigation cancellation when manual control takes over;
- safety-state publication.

> The software E-stop is a software control feature. A real robot still requires the appropriate physical E-stop, motor safety, electrical protection, and any application-specific certified safety mechanisms.

See [Navigation & Safety](docs/navigation-and-safety.md).

---

## Persistent Map Platform

Runtime maps live outside the source tree:

~~~text
~/.robot/maps/
└── <map_id>/
    ├── map.yaml
    ├── map.pgm or map.png
    ├── metadata.yaml
    ├── virtual_walls.yaml
    └── locations.json
~~~

The custom map manager supports:

- seed/default map bootstrap;
- map validation;
- list;
- activate;
- rename;
- delete;
- start mapping;
- save;
- stop mapping;
- persisted active-map selection;
- localization lifecycle transitions;
- navigation/safety holds during map transitions.

Keeping maps in runtime storage prevents software updates from overwriting robot-generated spatial data.

See [Maps & Spatial Controls](docs/maps-and-spatial-controls.md).

---

## Virtual Walls

Virtual walls are stored per map and converted to Nav2 keepout masks.

Supported concepts include:

- line walls;
- polygon restricted regions;
- create/update/delete;
- enable/disable;
- clear;
- point restriction checks.

This keeps two separate concepts clean:

~~~text
Occupancy map = what physically exists
Virtual walls = where the robot is allowed to go
~~~

---

## Docking and Battery

The runtime integrates Open Navigation docking with robot-level battery and docking state.

In simulation, helper nodes provide:

- battery state;
- dock contact;
- external dock pose detection.

In hardware mode, real BMS/contact/dock-detector drivers are expected to publish equivalent interfaces.

The configured main dock is anchored at:

~~~text
map pose: (0.0, 0.0, 0.0)
~~~

The same higher-level docking manager and application API can therefore be reused when simulation producers are replaced by real sensors.

---

## Telemetry and Health

RobotNavigation aggregates many ROS components into application-friendly robot state.

Important topics include:

~~~text
/robot/telemetry
/robot/health
/robot/safety/state
/robot/battery/state
/robot/docking/state
/robot/map/state
/robot/navigation/runtime_ready
~~~

Telemetry includes information such as:

- pose and yaw;
- linear/angular velocity;
- movement state;
- navigation state;
- distance remaining;
- navigation error;
- localization quality;
- safety state;
- obstacle information;
- docking state;
- battery / charging state;
- active map.

The health monitor evaluates subsystem freshness, lifecycle state, TF availability, localization, navigation, safety, docking, battery, and telemetry, then maps lower-level failures into stable robot-level error codes.

See [Telemetry & Health](docs/telemetry-and-health.md).

---

## WebSocket Gateway and RobotSDK

The gateway lets Android or other clients control the robot without embedding ROS 2 middleware in the application.

~~~mermaid
sequenceDiagram
    participant App as Android App
    participant SDK as RobotSDK
    participant GW as robot_gateway
    participant Nav as ROS 2 / Nav2
    participant Robot as Robot / Gazebo

    App->>SDK: navigateTo(x, y, yaw)
    SDK->>GW: Protocol v1 command
    GW->>Nav: NavigateToPose
    Nav-->>GW: goal accepted
    GW-->>SDK: command_ack
    Nav->>Robot: velocity commands
    Robot-->>Nav: sensor / odometry / TF
    Nav-->>GW: navigation + robot state
    GW-->>SDK: telemetry events
    SDK-->>App: reactive state
~~~

### Gateway safety and reliability controls

Current implementation includes:

- hello/authentication handshake;
- protocol-version validation;
- per-category rate limiting;
- maximum message size;
- control-session ownership;
- localhost-only unauthenticated default;
- mandatory token when binding to a non-loopback address;
- token redaction in logs;
- mission disconnect grace period;
- adoption of an orphaned mission by a new controller;
- explicit navigation failure classification and diagnostics.

See [Gateway & Protocol](docs/gateway-and-protocol.md).

---

## Setup

The repository includes an idempotent setup script:

~~~bash
./setup.sh
~~~

It:

- requires Ubuntu;
- detects the installed ROS 2 distribution rather than hardcoding one;
- verifies/installs base build tooling;
- initializes and resolves rosdep;
- creates a Python virtual environment with ROS system packages visible;
- installs Python requirements;
- verifies <code>rclpy</code> and the required WebSocket version;
- builds the workspace with colcon;
- verifies all five ROS packages;
- validates installed launch files and Python executables;
- creates runtime directories without touching existing maps.

See [Getting Started](docs/getting-started.md).

---

## Run

Normal simulation:

~~~bash
./run.sh --backend sim --mode navigation --rviz
~~~

Headless SBC/server-style simulation:

~~~bash
./run.sh --backend sim --mode navigation --headless
~~~

Hardware mode:

~~~bash
./run.sh --backend hardware --mode navigation
~~~

Status:

~~~bash
./run.sh --status
~~~

Restart a previous RobotNavigation-owned stack:

~~~bash
./run.sh --clean
~~~

### Runtime supervisor behavior

<code>run.sh</code> does not broadly kill ROS or Python processes.

It:

- owns the launched stack in a dedicated process group;
- stores guarded PID/run metadata under <code>~/.robot/run</code>;
- detects stale PID files and orphaned owned processes;
- checks gateway-port conflicts;
- enforces gateway authentication for non-loopback binds;
- streams important errors while keeping a complete timestamped log;
- performs real ROS readiness probing;
- gracefully shuts down its own process group with SIGINT → SIGTERM → SIGKILL escalation only when needed;
- leaves maps untouched.

See [Runtime & Deployment](docs/runtime-and-deployment.md).

---

## Network Gateway Example

Local development is loopback-only by default:

~~~bash
./run.sh --backend sim
~~~

To expose the gateway to another device on the LAN:

~~~bash
export ROBOT_GATEWAY_TOKEN="replace-with-a-strong-token"

./run.sh \
  --backend sim \
  --gateway-host 0.0.0.0
~~~

Both <code>run.sh</code> and the gateway reject non-loopback startup without a token.

---

## What Is Custom vs. Third-Party

This project intentionally builds on proven ROS components.

### Upstream components used

- ROS 2
- Nav2
- AMCL
- SLAM Toolbox
- MPPI controller
- NavFn planner
- Nav2 velocity smoother
- Nav2 collision monitor
- Open Navigation docking
- twist_mux
- Gazebo
- RViz

### RobotNavigation custom platform layer

- system bringup composition;
- lifecycle-aware navigation runtime coordinator;
- writable multi-map manager;
- map transition state machine;
- virtual-wall persistence and keepout-mask generation;
- final velocity safety supervisor;
- docking state manager;
- battery monitor and simulation backends;
- navigation monitor;
- robot telemetry aggregator;
- robot health/error model;
- custom robot messages/services;
- WebSocket gateway;
- mission execution / ownership behavior;
- gateway action diagnostics;
- deployment and runtime supervision scripts.

This distinction is important: the project demonstrates **systems integration, runtime design, API design, reliability, safety-oriented software, and productization around ROS**, rather than claiming to reimplement Nav2.

---

## Current Engineering Status

### Implemented in simulation

- autonomous navigation stack;
- pose and waypoint navigation support;
- AMCL localization;
- SLAM mapping runtime;
- persistent map lifecycle;
- virtual walls / keepout filters;
- manual/autonomous command pipeline;
- software E-stop;
- dead-man timeout;
- collision monitoring;
- docking software stack and simulation producers;
- battery state simulation/monitoring;
- telemetry;
- health monitoring;
- WebSocket command gateway;
- saved locations;
- multi-step missions;
- setup/deployment scripts.

### Still a hardware-stage responsibility

The final physical robot still needs real implementations for the hardware-facing interfaces, including:

- drivetrain / motor controller;
- wheel odometry / encoders;
- LiDAR;
- IMU;
- BMS / battery data;
- dock contact;
- dock detection;
- physical safety hardware.

The repository should therefore be described as a **simulation-first autonomous mobile robot software platform with a defined hardware integration boundary**, not as a commercially certified physical robot.

See [Simulation → Hardware](docs/simulation-and-hardware.md).

---

## Validation Philosophy

A feature is not considered reliable only because a ROS node launched.

The project uses or is designed around acceptance checks for:

- cold startup;
- readiness;
- action availability;
- pose navigation;
- waypoint navigation;
- navigation cancellation;
- map A → B → A switching;
- mapping → save → localization;
- persistent maps / walls / locations;
- E-stop;
- dead-man timeout;
- disconnect/reconnect;
- mission ownership;
- clean shutdown;
- repeated startup under load.

The gateway also has cross-language protocol fixture tests intended to validate compatibility with the companion Android RobotSDK fixtures.

See [Testing & Validation](docs/testing-and-validation.md).

---

## Documentation

Start with [Documentation Home](docs/README.md).

| Document | Focus |
|---|---|
| [Architecture](docs/architecture.md) | system decomposition, state ownership, lifecycle and data flow |
| [Getting Started](docs/getting-started.md) | install, build, first simulation run |
| [Runtime & Deployment](docs/runtime-and-deployment.md) | setup.sh, run.sh, process ownership, logs, SBC deployment |
| [Navigation & Safety](docs/navigation-and-safety.md) | Nav2, readiness, control pipeline, E-stop, dead-man |
| [Maps & Spatial Controls](docs/maps-and-spatial-controls.md) | maps, SLAM lifecycle, virtual walls, saved locations |
| [Gateway & Protocol](docs/gateway-and-protocol.md) | WebSocket API boundary, sessions, missions, navigation diagnostics |
| [Telemetry & Health](docs/telemetry-and-health.md) | robot state aggregation, subsystem health, errors |
| [Simulation → Hardware](docs/simulation-and-hardware.md) | backend abstraction and physical robot interfaces |
| [Testing & Validation](docs/testing-and-validation.md) | acceptance strategy, protocol fixtures, validation boundary |
| [Recruiter & Portfolio Guide](docs/recruiter-and-portfolio.md) | project story, resume bullets, demo sequence |

---

## Recommended Recruiter Demo

A concise demo should show the system, not only a moving Gazebo model:

1. run the project with <code>./run.sh --backend sim --mode navigation --rviz</code>;
2. show Gazebo + RViz localization;
3. send a navigation goal;
4. show the path and telemetry;
5. demonstrate a virtual wall changing the allowed navigation region;
6. demonstrate E-stop / motion inhibition;
7. show map state or mapping flow;
8. show Android RobotSDK / Controller communication through the gateway;
9. explain how <code>backend:=hardware</code> replaces simulation producers without redesigning the upper stack.

That sequence demonstrates robotics, distributed systems, Linux/runtime engineering, safety-oriented control flow, API design, and software architecture.

---

## Tech Stack

**ROS 2 · Nav2 · Python · rclpy · Gazebo · SLAM Toolbox · AMCL · MPPI · NavFn · TF2 · RViz · Open Navigation Docking · WebSockets · Linux · Bash · colcon · rosdep**

---

## Project Stage

**Active engineering project — simulation-first, hardware integration planned.**

The repository is designed to move from a fully exercised software stack to a small SBC-based physical robot without changing the application-facing architecture.
