# Telemetry & Health

## 1. Why Aggregate State?

Raw ROS topics are useful for robotics engineers but inconvenient for an operator application.

An application should not need to subscribe independently to TF, odometry, AMCL, action status, battery, docking, safety, map lifecycle, and collision state.

RobotNavigation therefore aggregates these into product-level messages.

## 2. Robot Telemetry

Primary topic:

~~~text
/robot/telemetry
~~~

The telemetry model includes concepts such as:

### Pose
- pose valid;
- frame;
- x/y;
- yaw.

### Motion
- linear velocity;
- angular velocity;
- moving/not moving;
- current control source.

### Navigation
- navigation state;
- goal active;
- distance remaining;
- elapsed time;
- navigation error code/message.

### Localization
- localization state;
- position standard deviation;
- yaw standard deviation.

### Safety
- safety state;
- emergency-stop state;
- collision-stop state;
- nearest obstacle information.

### Docking
- docking state;
- docked state;
- docking duration.

### Battery
- battery state;
- percentage;
- charging;
- voltage;
- temperature.

### Map
- active map.

## 3. Navigation Monitor

The navigation monitor:

- tracks NavigateToPose;
- tracks NavigateThroughPoses;
- tracks goal UUID/action type;
- resets the travelled path on a new navigation goal;
- publishes the destination based on the plan;
- records the travelled path from TF.

## 4. Robot Health

Primary topic:

~~~text
/robot/health
~~~

and standard ROS diagnostics:

~~~text
/diagnostics
~~~

The health monitor evaluates:

- LiDAR freshness;
- odometry freshness;
- IMU freshness;
- camera freshness where present;
- drivetrain/joint-state freshness;
- TF chain;
- map manager;
- map availability;
- AMCL/localization;
- BT navigator/planner/controller/behavior lifecycle state;
- velocity smoother;
- collision monitor;
- safety supervisor;
- E-stop;
- docking;
- battery;
- telemetry freshness.

## 5. Stable Error Model

Examples include:

~~~text
LOCALIZATION_UNAVAILABLE
LOCALIZATION_LOST
MAP_UNAVAILABLE
MAP_LOAD_FAILED
MAPPING_FAILED
NAVIGATION_UNAVAILABLE
NAVIGATION_FAILED
NAVIGATION_TIMEOUT
GOAL_UNREACHABLE
PATH_BLOCKED
ODOMETRY_UNAVAILABLE
LIDAR_UNAVAILABLE
TF_UNAVAILABLE
MANUAL_CONTROL_TIMEOUT
COLLISION_STOP
EMERGENCY_STOP
SAFETY_COMPONENT_FAULT
DOCKING_FAILED
LOW_BATTERY
CRITICAL_BATTERY
BATTERY_DATA_STALE
INTERNAL_ERROR
~~~

This gives the UI a stable robot error instead of requiring it to understand every Nav2/internal package-specific error number.

## 6. Criticality

Motion-critical areas include concepts such as:

- LiDAR;
- odometry;
- TF;
- localization;
- map;
- navigation;
- velocity control;
- collision monitoring;
- emergency stop.

A degraded optional subsystem should not be confused with a broken motion-critical subsystem.

## 7. Freshness Matters

A topic existing in the graph is not proof that the sensor is healthy.

The health monitor tracks the age of the latest data and can distinguish:

~~~text
topic exists but stale
~~~

from:

~~~text
fresh data arriving
~~~

## 8. Application Use

The WebSocket gateway subscribes to telemetry, battery, safety, docking, health, and map state, then broadcasts application-ready JSON.

That gives RobotSDK a clean reactive state model without ROS dependencies.
