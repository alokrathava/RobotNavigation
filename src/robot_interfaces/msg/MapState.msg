uint8 NO_MAP=0
uint8 LOADING=1
uint8 ACTIVE=2
uint8 MAPPING=3
uint8 SAVING=4
uint8 SWITCHING=5
uint8 ERROR=6

uint8 MODE_NONE=0
uint8 MODE_LOCALIZATION=1
uint8 MODE_MAPPING=2

builtin_interfaces/Time stamp
uint8 state
uint8 mode
string active_map_id
bool map_available
bool localization_ready
bool navigation_runtime_ready
bool unsaved_changes
bool recoverable_session
string error_code
string detail
