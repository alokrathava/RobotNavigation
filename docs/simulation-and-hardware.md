# Simulation → Hardware

## 1. Development Strategy

RobotNavigation is built software-first.

The intended sequence is:

~~~text
software architecture
 ↓
Gazebo validation
 ↓
operator / SDK integration
 ↓
SBC deployment
 ↓
real sensors + drivetrain
 ↓
hardware tuning and acceptance
~~~

This reduces the amount of hardware debugging mixed with software architecture work.

## 2. Backend Selection

Top-level launch:

~~~text
backend:=sim
backend:=hardware
~~~

The upper stack remains largely the same.

## 3. Simulation Backend

Simulation provides the physical interfaces required by the robot runtime.

Gazebo provides robot motion/sensors, while helper nodes simulate:

- battery;
- dock contact;
- dock detection pose.

Navigation, mapping, gateway, health, safety, maps, and application behavior run as the same robot-level software.

## 4. Hardware Backend

Hardware mode does not invent fake physical state.

It expects real drivers to publish the required interfaces.

A typical small differential-drive robot will need:

### Drivetrain

- motor-controller interface;
- wheel encoder input;
- odometry;
- final velocity command consumption.

### Perception

- 2D LiDAR for Nav2/collision monitoring;
- IMU where used;
- optional camera.

### TF

Expected conceptual chain:

~~~text
map
 ↓
odom
 ↓
base_footprint / base_link
 ↓
sensor frames
~~~

### Power

- battery percentage;
- voltage/current;
- charging state;
- temperature/faults where available.

### Docking

- contact detection;
- external dock pose detector, e.g. vision/tag-based detection.

## 5. Software Boundary

~~~mermaid
flowchart TB
    APP[Application / RobotSDK]
    GATEWAY[Gateway]
    ROBOT[RobotNavigation]
    DRIVER[Hardware Drivers]
    MCU[MCU / Motor Controller]
    SENSORS[LiDAR / IMU / BMS / Dock]

    APP --> GATEWAY --> ROBOT
    ROBOT --> DRIVER
    DRIVER --> MCU
    DRIVER --> SENSORS
    SENSORS --> DRIVER --> ROBOT
~~~

The gateway does not need to know whether a LaserScan came from Gazebo or a real LiDAR.

Nav2 does not need to know whether odometry came from simulated wheel joints or encoder firmware.

## 6. SBC Deployment

The project is designed for an Ubuntu-capable SBC.

<code>setup.sh</code> detects CPU architecture rather than assuming x86.

<code>run.sh</code> uses standard Linux process/session primitives and can run headless.

Before selecting final hardware, profile:

- idle CPU;
- navigation CPU;
- SLAM CPU;
- RAM;
- DDS overhead;
- thermal throttling;
- storage write behavior.

## 7. What Must Be Re-Tuned on Hardware

Expect to revisit:

- robot footprint;
- wheel radius/separation;
- odometry covariance;
- AMCL noise parameters;
- velocity/acceleration limits;
- MPPI constraints;
- collision zones;
- LiDAR filtering;
- docking offsets;
- battery thresholds;
- watchdog timeouts.

The architecture should remain; parameters and drivers will change.

## 8. Physical Safety

Hardware introduces failure modes simulation cannot prove:

- motor-controller runaway;
- relay/contact faults;
- electrical brownout;
- encoder inversion;
- wheel slip;
- LiDAR obstruction;
- network loss;
- SBC freeze;
- battery faults.

The physical robot needs independent electrical and mechanical safety appropriate to its use.

## 9. Recommended Hardware Bringup Order

1. motor controller with wheels off ground;
2. command zero/non-zero velocity safely;
3. encoder direction and odometry;
4. TF chain;
5. LiDAR;
6. teleoperation with software dead-man;
7. collision monitor;
8. map localization;
9. autonomous short path;
10. docking;
11. battery integration;
12. full acceptance tests.

Avoid introducing every sensor and autonomy feature simultaneously.
