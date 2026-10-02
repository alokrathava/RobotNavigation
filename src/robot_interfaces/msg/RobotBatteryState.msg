uint8 UNKNOWN=0
uint8 DISCHARGING=1
uint8 LOW=2
uint8 CRITICAL=3
uint8 CHARGING=4
uint8 FULL=5
uint8 FAULT=6

builtin_interfaces/Time stamp
uint8 state
float32 percentage           # 0..100
bool is_charging             # dock contact AND charge current
float32 voltage
float32 current              # A, positive while charging
float32 temperature          # deg C
float32 data_age_s           # age of the underlying BatteryState
string detail
