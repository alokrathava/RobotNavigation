# Recruiter & Portfolio Guide

## 1. One-Sentence Project Description

**RobotNavigation is a simulation-first ROS 2 autonomous mobile-robot platform that combines Nav2 navigation, SLAM/localization, persistent map management, layered motion safety, docking, health/telemetry, deployment tooling, and a WebSocket API for external applications.**

## 2. 30-Second Interview Explanation

"I built the robot-side software platform around ROS 2 and Nav2 for a small autonomous mobile robot. I use Nav2 for planning and control, but I built the surrounding product layer: lifecycle-aware startup, persistent multi-map management, map-scoped virtual walls, a final velocity safety supervisor, docking/battery/health abstractions, telemetry aggregation, a WebSocket gateway for my Android RobotSDK, and setup/run tooling that can deploy the same upper stack to simulation or an SBC-based physical robot."

## 3. What Makes the Project Senior-Level

The strongest parts to discuss are architectural decisions:

### Clear boundaries

- Android is not coupled to ROS/DDS.
- Nav2 remains authoritative for autonomy.
- runtime maps are outside source control.
- safety remains robot-side.
- simulation/hardware backends are replaceable.

### Reliability

- lifecycle-aware readiness;
- real startup probing;
- action-response diagnostics;
- process ownership and graceful shutdown;
- reconnect/mission ownership behavior.

### Safety-oriented systems thinking

- command arbitration;
- smoothing;
- collision monitoring;
- final velocity gate;
- dead-man;
- E-stop;
- stale-command watchdog.

### Productization

- robot-level telemetry/errors;
- map lifecycle API;
- WebSocket protocol;
- deployment scripts;
- external SDK integration.

## 4. Resume Version — Senior Software / Robotics

**RobotNavigation — ROS 2 Autonomous Mobile Robot Platform**

- Architected a ROS 2 mobile-robot runtime integrating **Nav2, AMCL, SLAM Toolbox, Gazebo, docking, telemetry and health monitoring** behind a unified simulation/hardware launch architecture.
- Built **lifecycle-aware runtime orchestration and readiness checks** for localization and Nav2 services, replacing fragile fixed-delay startup assumptions with dependency and action/service discovery.
- Designed a persistent **multi-map lifecycle** with SLAM map creation, validated runtime storage, map switching, restart persistence, and map-scoped virtual-wall/keepout configuration.
- Implemented a layered motion-safety path with **manual/autonomous arbitration, velocity smoothing, collision monitoring, E-stop gating, dead-man timeout, map-transition hold and stale-command watchdogs**.
- Developed a **versioned WebSocket-to-ROS 2 gateway** exposing navigation, waypoint routing, mapping, docking, missions, saved locations, telemetry, diagnostics and configuration to external Android clients.
- Built robot-level **telemetry, health and stable error abstractions** across localization, navigation, sensors, battery, docking, safety and lifecycle components.
- Created idempotent setup and supervised runtime tooling for Ubuntu/SBC deployment, including dependency resolution, build validation, process-group ownership, readiness probing, logging and graceful shutdown.

**Technologies:** ROS 2, Nav2, Python, rclpy, Gazebo, SLAM Toolbox, AMCL, MPPI, TF2, WebSockets, Linux, Bash, colcon, rosdep

## 5. Short Resume Version

**RobotNavigation | ROS 2 / Nav2 Autonomous Mobile Robot Runtime**

- Built a simulation-first ROS 2 robot platform integrating Nav2 navigation, SLAM/AMCL, persistent maps, virtual walls, docking, telemetry and subsystem health.
- Designed a layered safety pipeline with command arbitration, smoothing, collision monitoring, E-stop, dead-man timeout and velocity watchdogs.
- Developed a WebSocket gateway connecting Android RobotSDK clients to ROS actions/services plus repeatable Ubuntu/SBC setup and supervised runtime tooling.

## 6. Portfolio Description

### Title

**RobotNavigation — Autonomous Mobile Robot Software Platform**

### Description

RobotNavigation is the robot-side autonomy platform I built for a small mobile robot. It combines ROS 2 and Nav2 with the missing product-level pieces needed for an application-controlled robot: persistent maps, runtime lifecycle management, virtual walls, layered motion safety, docking, battery/health monitoring, telemetry, and a WebSocket API consumed by my Android RobotSDK.

I developed the system simulation-first in Gazebo and separated simulation from hardware-facing interfaces so the same navigation, safety, map, gateway and monitoring architecture can later run on an SBC-based physical robot.

### Architecture caption

~~~text
Android / RobotSDK
        ↓
WebSocket Gateway
        ↓
ROS 2 Robot Runtime
        ↓
Nav2 + SLAM + Safety
        ↓
Gazebo / Physical Robot
~~~

## 7. Portfolio Feature Cards

### Autonomous Navigation

Nav2-based pose and waypoint navigation with lifecycle-aware readiness, MPPI control, NavFn planning, AMCL localization and SLAM Toolbox mapping.

### Safety Pipeline

Manual/autonomous velocity arbitration followed by smoothing, collision monitoring, a software E-stop gate, dead-man timeout and command watchdog.

### Persistent Mapping

Runtime multi-map storage, map switching, SLAM save lifecycle, map validation and per-map virtual walls / saved locations.

### Application Gateway

Versioned WebSocket API that translates application commands into ROS actions/services and broadcasts robot-level telemetry and health.

### Observability

Aggregated telemetry and stable robot errors across navigation, localization, sensors, docking, battery, map state and safety.

### Deployment

Idempotent machine setup plus a process-owning runtime supervisor with real readiness probes, status reporting, logs and graceful shutdown.

## 8. Recommended Screenshots

Add:

~~~text
docs/images/01-gazebo-overview.png
docs/images/02-rviz-navigation.png
docs/images/03-virtual-walls.png
docs/images/04-mapping.png
docs/images/05-android-controller.png
docs/images/06-runtime-terminal.png
~~~

## 9. Recommended Demo Video

Aim for 45–90 seconds.

Suggested sequence:

1. launch with <code>run.sh</code>;
2. show READY;
3. Android sends navigation target;
4. robot moves in Gazebo;
5. RViz shows plan/travelled path;
6. add/show virtual wall;
7. trigger software E-stop;
8. show safety state;
9. show map/telemetry on Android.

## 10. Interview Topics to Be Ready For

Expect deeper questions about:

- why WebSocket instead of direct ROS on Android;
- lifecycle race conditions;
- DDS discovery issues;
- why maps are runtime data;
- how map switching interacts with AMCL;
- how manual control cancels autonomy;
- what happens when the client disconnects;
- software vs hardware E-stop;
- how simulation interfaces will map to hardware;
- why use Nav2 rather than write a planner;
- what remains to be hardware validated.

## 11. Public Portfolio Checklist

Before putting the GitHub URL on a resume:

- make the repository accessible to recruiters;
- remove local IDE files that should not be public;
- confirm there are no credentials or private paths;
- add a root LICENSE if the repository is intended to be open source;
- add screenshots;
- add a demo video link;
- confirm setup/run instructions on a clean machine;
- keep "simulation validated" separate from "hardware validated."

The current repository is private, so a recruiter following a resume link will not be able to inspect it unless access is changed.
