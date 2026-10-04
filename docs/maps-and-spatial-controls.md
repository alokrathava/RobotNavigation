# Maps & Spatial Controls

## 1. Runtime Map Store

Generated maps are robot data, so they do not live permanently inside the source tree.

Default runtime location:

~~~text
~/.robot/maps
~~~

Each map is isolated:

~~~text
~/.robot/maps/
└── main_floor/
    ├── map.yaml
    ├── map.pgm
    ├── metadata.yaml
    ├── virtual_walls.yaml
    └── locations.json
~~~

## 2. Why This Matters

Keeping maps outside the repository means:

- a software update does not overwrite robot-generated maps;
- multiple maps can exist;
- a map has its own metadata/policy;
- saved locations remain associated with the correct frame;
- the same source build can run on different robots/sites.

## 3. Map IDs and Validation

The map manager validates concepts such as:

- map ID format;
- absolute runtime map root;
- path containment;
- map YAML structure;
- referenced image existence;
- supported image format;
- map dimensions;
- resolution;
- origin;
- metadata consistency.

These checks reduce the chance that a malformed map leaves the robot runtime in an undefined state.

## 4. Default Map Bootstrap

The repository can ship with a bundled default map.

At runtime the map manager seeds it into the writable map store only when required.

After that, the runtime copy becomes robot data.

## 5. Map State Machine

Robot-level map state includes states such as:

~~~text
NO_MAP
LOADING
ACTIVE
MAPPING
SAVING
SWITCHING
ERROR
~~~

and modes such as:

~~~text
NONE
LOCALIZATION
MAPPING
~~~

The map manager publishes state on:

~~~text
/robot/map/state
~~~

## 6. Map Activation

A map transition is coordinated rather than changing a parameter while the robot continues driving.

Conceptually:

~~~mermaid
flowchart TD
    REQUEST[Activate map]
    HOLD[Enable safety transition hold]
    NAVOFF[Disable navigation frontend]
    CANCEL[Cancel navigation]
    LOCSTOP[Transition localization]
    CONFIG[Point map_server at new YAML]
    LOCSTART[Restart localization]
    WAIT[Wait for AMCL / map readiness]
    NAVON[Re-enable navigation]
    RELEASE[Release motion hold]

    REQUEST --> HOLD --> NAVOFF --> CANCEL --> LOCSTOP --> CONFIG --> LOCSTART --> WAIT --> NAVON --> RELEASE
~~~

## 7. Mapping Lifecycle

Typical mapping flow:

~~~text
localization mode
 ↓
hold / disable navigation
 ↓
pause localization
 ↓
activate SLAM
 ↓
mapping
 ↓
save map
 ↓
deactivate SLAM
 ↓
activate saved map
 ↓
restart localization
 ↓
navigation ready
~~~

The manager supports start, save, stop, and restore behavior.

## 8. Virtual Walls

Virtual walls are map-specific operational policy.

Supported geometry:

- LINE;
- POLYGON.

Supported operations:

- list;
- create;
- update;
- delete;
- enable;
- disable;
- clear;
- point restriction check.

## 9. Keepout Filter Flow

~~~mermaid
flowchart LR
    FILE[virtual_walls.yaml]
    MANAGER[virtual_wall_manager]
    MASK[Occupancy keepout mask]
    INFO[CostmapFilterInfo]
    NAV2[Nav2 costmap KeepoutFilter]
    PLAN[Planner]

    FILE --> MANAGER
    MANAGER --> MASK
    MANAGER --> INFO
    MASK --> NAV2
    INFO --> NAV2
    NAV2 --> PLAN
~~~

## 10. Saved Locations

Saved locations are application-level named poses.

Examples:

~~~text
Reception
Charging Dock
Inspection Point
Kitchen
Storage
~~~

They are stored per map so coordinates from one floor/site cannot accidentally be reused on another map.

The gateway supports:

- list;
- save;
- rename;
- delete;
- navigate to saved location.

## 11. Dock Relationship

The current configured main dock is:

~~~text
map frame
x = 0.0
y = 0.0
yaw = 0.0
~~~

This gives the system a stable home reference while the rest of the map can contain arbitrary saved locations and virtual walls.
