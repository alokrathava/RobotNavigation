uint8 UNDOCKED=0
uint8 NAVIGATING_TO_DOCK=1
uint8 PRESTAGING=2
uint8 ALIGNING=3
uint8 DOCKING=4
uint8 DOCKED=5
uint8 UNDOCKING=6
uint8 FAILED=7
uint8 CANCELLED=8

builtin_interfaces/Time stamp
uint8 state
string dock_id
geometry_msgs/PoseStamped dock_pose
bool is_docked
bool is_charging
float32 docking_time_s       # elapsed time of the active dock/undock operation
uint16 num_retries
uint16 error_code            # nav2 DockRobot/UndockRobot result error code, 0 = none
string error_name            # stable id, e.g. FAILED_TO_CHARGE
string error_msg             # server-provided text
string detail
