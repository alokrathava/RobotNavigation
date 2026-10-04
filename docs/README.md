# RobotNavigation Documentation

This documentation is organized for two audiences:

1. **Recruiters / hiring managers** who want to understand the engineering scope quickly.
2. **Engineers** who want enough detail to build, run, extend, or integrate the robot runtime.

## Start Here

If you have only a few minutes:

1. [Root README](../README.md)
2. [Architecture](architecture.md)
3. [Recruiter & Portfolio Guide](recruiter-and-portfolio.md)

If you want to run the project:

1. [Getting Started](getting-started.md)
2. [Runtime & Deployment](runtime-and-deployment.md)
3. [Simulation → Hardware](simulation-and-hardware.md)

If you want a technical deep dive:

- [Navigation & Safety](navigation-and-safety.md)
- [Maps & Spatial Controls](maps-and-spatial-controls.md)
- [Gateway & Protocol](gateway-and-protocol.md)
- [Telemetry & Health](telemetry-and-health.md)
- [Testing & Validation](testing-and-validation.md)

## System at a Glance

~~~mermaid
flowchart LR
    A[Android / External Client]
    B[RobotSDK]
    C[robot_gateway]
    D[RobotNavigation Runtime]
    E[Nav2 / SLAM]
    F[Safety]
    G[Gazebo / Hardware]

    A --> B --> C --> D
    D --> E --> F --> G
    G --> D
    D --> C --> B --> A
~~~

## Core Design Ideas

- Robot autonomy remains on the robot.
- External clients consume robot-level APIs, not raw Nav2 internals.
- Runtime maps are persistent data, not source-code assets.
- Safety does not depend on an Android connection.
- Navigation readiness is explicit and lifecycle-aware.
- Simulation and hardware share the same upper architecture.
- The gateway separates protocol/networking concerns from ROS logic.
- Deployment tooling owns only RobotNavigation processes, not every ROS process on the machine.

## Documentation Baseline

These pages describe the repository state after:

~~~text
main: fdf2d7466306f3088bcfbc173eac58ec969eaf9e
PR #2: Gateway resolved
setup.sh / run.sh deployment tooling added
~~~

The documentation intentionally distinguishes implemented software from future physical-hardware validation.
