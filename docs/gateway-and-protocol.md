# Gateway & Protocol

## 1. Purpose

The gateway provides a network boundary between applications and ROS 2.

~~~text
Android / External Client
        ↓
   WebSocket JSON
        ↓
    robot_gateway
        ↓
topics / services / actions
        ↓
       ROS 2
~~~

The application does not need DDS, rclpy/rclcpp, or Nav2 message definitions.

## 2. Protocol

Current protocol:

~~~text
Protocol Version: 1
Gateway Version advertised: 0.2.0
~~~

Messages use JSON envelopes containing concepts such as:

- type;
- ID;
- protocolVersion;
- payload;
- structured error.

## 3. Authentication

A client first sends a hello handshake.

If the gateway is configured with a token, the client must provide the correct token.

The gateway:

- masks token presence in logs;
- refuses commands before hello/authentication;
- rejects protocol-version mismatch.

## 4. Network Exposure

Default gateway bind:

~~~text
127.0.0.1:8080
~~~

Both the gateway CLI and <code>run.sh</code> reject a non-loopback bind without authentication.

Example LAN launch:

~~~bash
export ROBOT_GATEWAY_TOKEN="strong-token"
./run.sh --gateway-host 0.0.0.0
~~~

## 5. Capabilities

The hello acknowledgement currently advertises:

~~~text
navigation
manual_control
docking
safety
telemetry
battery
health
map_switching
map_management
mapping
virtual_walls
saved_locations
waypoint_navigation
missions
robot_configuration
diagnostics
log_retrieval
~~~

Clients should use capability negotiation instead of assuming every future gateway exposes every feature.

## 6. Control Session Ownership

Multiple clients may be connected, but state-changing control commands use an ownership model.

The first controller issuing a control command acquires the active control session.

A different client attempting a control command receives a control-session-in-use error.

This avoids two operator devices simultaneously commanding motion or map changes.

## 7. Emergency Stop Semantics

Emergency stop is intentionally treated differently from normal control ownership: a safety stop should not be blocked simply because another client owns the control session.

Release behavior is security-sensitive because it re-enables motion. The robot-side safety supervisor still requires fresh control after release.

## 8. Navigation Commands

Supported command concepts include:

~~~text
navigate_to
navigate_through
cancel_navigation
navigate_to_location
~~~

The gateway creates the corresponding Nav2 action goals.

## 9. Navigation Failure Classification

The gateway keeps goal-send outcomes distinct.

~~~text
ACTION_SERVER_UNAVAILABLE
ACTION_GOAL_RESPONSE_TIMEOUT
ACTION_GOAL_SEND_EXCEPTION
NAV2_GOAL_REJECTED
NAVIGATOR_CANCEL_TIMEOUT
~~~

A timeout waiting for a ROS action response is not the same as Nav2 returning <code>accepted=false</code>.

## 10. Cross-Navigator Switching

Before sending a goal to one navigation frontend, the gateway checks whether the other frontend still has an active goal.

If so:

1. cancel conflicting action;
2. wait for terminal status;
3. send replacement goal.

This makes NavigateToPose ↔ NavigateThroughPoses transitions deterministic.

## 11. Action Diagnostics

Each navigation request can log:

- request ID;
- goal UUID;
- action;
- frame;
- coordinates;
- server readiness;
- runtime readiness;
- active navigation state;
- BT navigator lifecycle state;
- response latency;
- whether the UUID appeared on the action status topic;
- ROS spin-loop timing;
- asyncio loop lag.

The gateway also detects a late acceptance when Nav2 accepts a goal after the gateway already timed out waiting for the response.

## 12. Manual Control

Manual velocity is published to:

~~~text
cmd_vel_manual
~~~

It does not bypass the robot safety pipeline.

## 13. Maps and Spatial Commands

Gateway commands include:

- list/switch/save/rename/delete map;
- start/stop mapping;
- list/create/update/delete walls;
- enable/disable/clear walls;
- check point restriction;
- saved-location CRUD.

## 14. Missions

Mission step types currently include:

~~~text
NAVIGATE
NAVIGATE_TO_LOCATION
WAIT
DOCK
UNDOCK
~~~

The mission executor runs steps sequentially and reports progress.

### Mission ownership

When a mission owner disconnects:

~~~text
owner disconnects
 ↓
10 s grace period
 ↓
new control owner?
 ├─ yes → mission is adopted
 └─ no  → mission is cancelled
~~~

## 15. State Broadcasts

The gateway broadcasts application-ready state for:

- telemetry;
- battery;
- safety;
- docking;
- health;
- map state.

A newly authenticated client receives the last known state snapshot where available.

## 16. Diagnostics and Logs

The gateway maintains a bounded recent ROS log buffer and exposes log retrieval through the protocol.

It also provides state snapshots for clients that need to resynchronize after reconnect.

## 17. RobotSDK Boundary

~~~text
RobotController UI
 ↓
RobotSDK / RobotClient
 ↓
Protocol v1
 ↓
robot_gateway
 ↓
RobotNavigation
~~~

This keeps mobile application code independent from ROS implementation details.
