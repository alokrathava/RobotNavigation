uint8 OK=0
uint8 DEGRADED=1
uint8 ERROR=2
uint8 EMERGENCY=3

builtin_interfaces/Time stamp
uint8 overall
string[] subsystems          # parallel arrays: name / diagnostic level (0 OK,1 WARN,2 ERROR,3 STALE)
uint8[] subsystem_levels
RobotError[] active_errors
