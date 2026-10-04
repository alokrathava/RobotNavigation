# Architecture

## 1. Architectural Goal

RobotNavigation is not intended to be a monolithic "robot node."

It is a collection of ROS 2 packages and product-level runtime services that turn navigation, mapping, safety, monitoring, and application connectivity into one coherent robot platform.

The architecture optimizes for:

- clear ownership of state;
- replaceable simulation/hardware backends;
- deterministic startup;
- safe motion control;
- persistent robot data;
- application-facing abstractions;
- diagnosability.

## 2. Full System

~~~mermaid
flowchart TB
    subgraph CLIENT[External Application]
        UI[Operator UI]
        SDK[RobotSDK]
    end

    subgraph GATEWAY[Application Boundary]
        GW[robot_gateway]
    end

    subgraph PRODUCT[Robot-Level Services]
        MAP[map_manager]
        VW[virtual_wall_manager]
        COORD[navigation_runtime_coordinator]
        NAVMON[nav_monitor]
        DOCKM[docking_manager]
        BAT[battery_monitor]
        STATE[robot_state_manager]
        HEALTH[robot_health_monitor]
    end

    subgraph NAV2[Autonomy]
        MAPSERVER[map_server]
        AMCL[AMCL]
        SLAM[SLAM Toolbox]
        PLAN[Planner Server]
        CTRL[Controller Server]
        BEH[Behavior Server]
        BT[BT Navigator]
        DOCK[Docking Server]
    end

    subgraph MOTION[Motion Safety]
        MUX[twist_mux]
        SMOOTH[Velocity Smoother]
        COLL[Collision Monitor]
        SAFE[safety_supervisor]
    end

    subgraph BACKEND[Backend]
        SIM[Gazebo + Sim Producers]
        HW[Hardware Drivers]
    end

    UI --> SDK
    SDK <-->|Protocol v1| GW

    GW <--> MAP
    GW <--> VW
    GW <--> BT
    GW <--> DOCKM
    GW <--> STATE
    GW <--> HEALTH

    MAP <--> MAPSERVER
    MAP <--> AMCL
    MAP <--> SLAM
    MAP --> COORD
    COORD --> MAPSERVER
    COORD --> AMCL
    COORD --> PLAN
    COORD --> CTRL
    COORD --> BEH
    COORD --> BT
    COORD --> DOCK

    VW --> PLAN
    NAVMON --> STATE
    DOCK --> DOCKM
    BAT --> STATE
    DOCKM --> STATE
    STATE --> HEALTH

    CTRL --> MUX
    BEH --> MUX
    GW --> MUX
    MUX --> SMOOTH --> COLL --> SAFE
    SAFE --> SIM
    SAFE --> HW

    SIM --> AMCL
    SIM --> STATE
    HW --> AMCL
    HW --> STATE
~~~

## 3. Package Boundaries

### robot_bringup

**Responsibility:** compose the full system.

The top-level launch chooses:

- simulation vs hardware;
- navigation vs mapping;
- map directory;
- RViz;
- gateway;
- gateway networking settings.

It deliberately delegates robot autonomy to <code>robot_navigation</code> rather than duplicating launch logic.

### robot_description

**Responsibility:** robot model / TF description.

The physical and simulated platform share the same conceptual robot frame model.

### robot_interfaces

**Responsibility:** product-level ROS contract.

Nav2 messages are useful inside Nav2, but external applications need robot concepts.

The custom interface package provides stable domain types for:

- robot telemetry;
- health/errors;
- safety;
- maps;
- virtual walls;
- battery;
- docking.

### robot_navigation

**Responsibility:** robot autonomy runtime.

This package owns the glue between standard ROS components and product behavior:

- navigation configuration;
- mapping/localization coordination;
- motion safety;
- virtual walls;
- robot state;
- health;
- docking;
- simulation-side power/dock sources.

### robot_gateway

**Responsibility:** network/API boundary.

The gateway converts Protocol v1 application commands to ROS topics, services, and actions, and converts robot state back to application events.

## 4. State Ownership

| State | Authority |
|---|---|
| Robot pose | TF / localization |
| Navigation execution | Nav2 |
| Current map | map_manager |
| Runtime map files | ~/.robot/maps |
| Virtual walls | virtual_wall_manager |
| Final software velocity | safety_supervisor |
| Battery interpretation | battery_monitor |
| Docking state | docking runtime |
| Aggregated operator state | robot_state_manager |
| System health | robot_health_monitor |
| Control session | robot_gateway |
| Process ownership | run.sh |

## 5. Startup Model

The system avoids:

~~~text
launch nodes
sleep 10
assume ready
~~~

Instead, readiness is built from lifecycle and endpoint state.

~~~mermaid
stateDiagram-v2
    [*] --> MapBootstrap
    MapBootstrap --> Localization
    Localization --> NavigationCore
    NavigationCore --> Frontend
    Frontend --> Ready

    Ready --> Held: map transition / navigation disabled
    Held --> Localization

    Localization --> Waiting: dependencies unavailable
    NavigationCore --> Waiting: endpoints unavailable
    Frontend --> Waiting: action discovery unavailable
    Waiting --> Localization
~~~

The runtime coordinator verifies localization nodes, core Nav2 nodes, required actions/services, and final frontend actions before publishing <code>/robot/navigation/runtime_ready</code>.

## 6. ROS Time vs Wall Time

Navigation algorithms, costmaps, localization, and simulation use ROS time where appropriate.

Safety/runtime orchestration nodes that must remain responsive even when simulation time pauses use wall time.

Examples include:

- safety supervisor;
- map lifecycle management;
- virtual wall manager;
- navigation readiness coordinator;
- deployment readiness probe.

## 7. Simulation / Hardware Boundary

~~~mermaid
flowchart LR
    COMMON[Common Robot Runtime]
    SIM[Simulation Backend]
    HW[Hardware Backend]

    COMMON -->|backend=sim| SIM
    COMMON -->|backend=hardware| HW

    SIM --> SENSORS1[/scan /odom TF battery dock]
    HW --> SENSORS2[/scan /odom TF battery dock]
~~~

Above the backend boundary, RobotNavigation should not care whether the signal came from Gazebo or real electronics.

## 8. Application Boundary

~~~text
App
 ↓
RobotSDK
 ↓
WebSocket protocol
 ↓
robot_gateway
 ↓
ROS 2
~~~

Benefits:

- Android does not need ROS middleware;
- protocol compatibility can be versioned;
- authentication can be enforced at one boundary;
- one gateway can translate application concepts into multiple ROS primitives;
- the robot remains autonomous if the UI disappears.

## 9. Custom vs Upstream Responsibility

### Upstream navigation algorithms

RobotNavigation uses Nav2, AMCL, SLAM Toolbox, MPPI, NavFn, collision monitor, velocity smoother, Open Navigation docking, and twist_mux.

### Custom systems engineering

The repository adds:

- lifecycle-aware startup;
- application-oriented state;
- persistent map lifecycle;
- map transitions;
- keepout policy;
- software safety gate;
- battery/docking abstraction;
- WebSocket protocol;
- mission ownership;
- navigation action diagnostics;
- process supervision.

The architecture is therefore an integration/platform project, not a claim to have recreated Nav2.
