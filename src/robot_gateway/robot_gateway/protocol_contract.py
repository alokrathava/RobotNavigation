"""Single source of truth for the application-facing gateway contract."""

PROTOCOL_VERSION = 1
GATEWAY_VERSION = "0.3.0"

GATEWAY_CAPABILITIES = (
    "navigation",
    "manual_control",
    "docking",
    "safety",
    "telemetry",
    "battery",
    "health",
    "map_switching",
    "map_management",
    "mapping",
    "map_streaming",
    "localization_recovery",
    "navigation_path",
    "virtual_walls",
    "saved_locations",
    "waypoint_navigation",
    "missions",
    "robot_configuration",
    "robot_discovery",
    "diagnostics",
    "log_retrieval",
)
