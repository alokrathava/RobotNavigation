uint8 INFO=0
uint8 WARN=1
uint8 ERROR=2
uint8 FATAL=3

string code                  # stable identifier, e.g. LOCALIZATION_LOST
string subsystem
uint8 severity
string message               # human readable only; never parse
bool recoverable
builtin_interfaces/Time stamp
uint16 source_code           # original nav2 error code if any, else 0
