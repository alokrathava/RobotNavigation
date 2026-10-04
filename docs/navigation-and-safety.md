# Navigation & Safety

## 1. Nav2 Integration

The project uses Nav2 for autonomous navigation rather than implementing a planner/controller from scratch.

Configured components include:

- Map Server;
- AMCL;
- Planner Server with NavFn;
- Controller Server with MPPI;
- Behavior Server;
- BT Navigator;
- Velocity Smoother;
- Collision Monitor;
- Open Navigation Docking.

The robot-specific value is in how these components are coordinated, protected, observed, and exposed to external clients.

## 2. Navigation Frontends

RobotNavigation supports:

~~~text
NavigateToPose
NavigateThroughPoses
~~~

The WebSocket gateway exposes both.

The navigation monitor also tracks both action status streams, using the action type plus goal UUID to identify the active navigation operation.

## 3. Lifecycle Startup

Nav2 lifecycle components are split into groups.

### Localization

~~~text
map_server
amcl
~~~

### Navigation core

~~~text
controller_server
planner_server
behavior_server
velocity_smoother
collision_monitor
~~~

### Navigation frontend

~~~text
bt_navigator
docking_server
~~~

The custom runtime coordinator activates each layer only after required dependencies are discoverable.

## 4. Runtime Readiness

A robot-level readiness topic is published:

~~~text
/robot/navigation/runtime_ready
~~~

It combines:

- lifecycle state;
- map state;
- planner/controller/behavior action availability;
- costmap services;
- final navigation/docking action availability.

This is more useful to an application than checking whether one action name happens to appear in the ROS graph.

## 5. Cross-Navigator Arbitration

Nav2 may reject a goal for one navigator while the other navigator is actively processing a goal.

The gateway therefore treats a navigator switch explicitly:

~~~mermaid
flowchart LR
    NEW[New navigation request]
    CHECK{Other navigator active?}
    CANCEL[Cancel conflicting navigator]
    WAIT[Wait for terminal state]
    SEND[Send new goal]
    ERROR[Return cancel-timeout error]

    NEW --> CHECK
    CHECK -- no --> SEND
    CHECK -- yes --> CANCEL --> WAIT
    WAIT -- cleared --> SEND
    WAIT -- timeout --> ERROR
~~~

This avoids treating cross-navigator behavior as a random goal rejection.

## 6. Navigation Diagnostics

Navigation goal dispatch records information such as:

- request ID;
- action type;
- requested goals;
- frame;
- server readiness;
- robot runtime readiness;
- believed active navigation;
- BT navigator lifecycle state;
- response latency;
- action status visibility;
- event-loop / ROS spin timing.

Failure modes are separated:

~~~text
ACTION_SERVER_UNAVAILABLE
ACTION_GOAL_RESPONSE_TIMEOUT
ACTION_GOAL_SEND_EXCEPTION
NAV2_GOAL_REJECTED
NAVIGATOR_CANCEL_TIMEOUT
~~~

This distinction was introduced specifically to avoid mislabeling every failed goal response as "Nav2 rejected the goal."

## 7. Motion Pipeline

~~~mermaid
flowchart LR
    MAN[cmd_vel_manual]
    NAV[cmd_vel_nav]
    MUX[twist_mux]
    SMOOTH[velocity_smoother]
    COLL[collision_monitor]
    SAFE[safety_supervisor]
    OUT[/cmd_vel]

    MAN --> MUX
    NAV --> MUX
    MUX --> SMOOTH --> COLL --> SAFE --> OUT
~~~

### Why this order?

**twist_mux** selects the current command source.

**velocity smoother** limits abrupt changes.

**collision monitor** modifies/stops commands based on nearby obstacles.

**safety supervisor** is the final software gate before the drivetrain.

## 8. Safety Supervisor

The safety supervisor publishes the final <code>/cmd_vel</code> and manages robot software safety state.

### Emergency stop

When engaged:

- publishes zero velocity;
- locks the motion path;
- cancels active navigation and docking actions;
- reports emergency-stop state.

### Release behavior

Releasing E-stop does not immediately replay an old manual command.

The system holds motion until the manual stream becomes quiet, then requires fresh input.

### Manual dead-man

If a non-zero manual command stops arriving for longer than the configured manual timeout, a zero command is injected.

This protects against:

- dropped Wi-Fi;
- application crash;
- joystick stream loss;
- disconnected operator.

### Velocity watchdog

If the safe velocity pipeline becomes silent while the last output was non-zero, the supervisor forces zero velocity and marks a fault.

### Map-transition hold

Map activation/mapping transitions can request a motion hold.

This prevents movement while map/localization authority is changing.

### Manual takeover

Fresh manual input can cancel active autonomous navigation.

## 9. Collision Monitoring

Nav2 Collision Monitor provides protective behavior based on LiDAR geometry.

The safety state exposes conditions such as:

- NORMAL;
- CAUTION / slowdown;
- COLLISION_STOP;
- EMERGENCY_STOP;
- FAULT.

The health monitor distinguishes a protective collision stop from a broken safety component.

## 10. Physical Safety Boundary

The software safety architecture improves robot behavior, but it is not a certified safety system.

A physical robot must still design:

- hardware E-stop;
- motor enable / power isolation;
- current limiting;
- battery protection;
- physical bumpers where appropriate;
- application-specific safety standards.
