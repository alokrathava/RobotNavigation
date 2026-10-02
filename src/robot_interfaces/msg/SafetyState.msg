uint8 NORMAL=0
uint8 CAUTION=1          # collision monitor slowdown active
uint8 COLLISION_STOP=2   # collision monitor stop active
uint8 EMERGENCY_STOP=3
uint8 FAULT=4            # /scan stale or drivetrain output lost while moving

builtin_interfaces/Time stamp
uint8 state
bool emergency_stop_active
bool motion_inhibited    # true while E-stopped or waiting for manual input to go quiet after release
string control_source    # "none" | "nav" | "manual"
string detail
