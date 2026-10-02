builtin_interfaces/Time stamp

# POSE (map frame, from TF)
bool pose_valid
string frame
float64 x
float64 y
float64 yaw

# MOTION (from /odom)
float32 linear_velocity
float32 angular_velocity
bool is_moving

# NAVIGATION
string navigation_state      # IDLE PLANNING NAVIGATING REPLANNING SUCCEEDED CANCELLED FAILED
float32 distance_remaining   # NaN when no active goal
float32 navigation_elapsed_s
bool has_goal
geometry_msgs/PoseStamped active_goal
uint16 navigation_error_code
string navigation_error_msg

# LOCALIZATION
string localization_state    # UNINITIALIZED LOCALIZED DEGRADED LOST

# SAFETY
uint8 safety_state           # robot_interfaces/SafetyState constants
bool emergency_stopped
bool collision_stopped
string control_source

# OBSTACLE
float32 nearest_obstacle_distance   # m from robot origin, NaN if no valid return
float32 nearest_obstacle_bearing    # rad in base frame

# DOCKING
uint8 docking_state          # robot_interfaces/DockingState constants
bool is_docked
float32 docking_time_s
uint16 docking_retries
uint16 docking_error_code
string docking_error_name

# BATTERY
uint8 battery_state          # robot_interfaces/RobotBatteryState constants
float32 battery_percentage
bool charging
float32 battery_voltage
float32 battery_temperature

# MAP
string active_map
