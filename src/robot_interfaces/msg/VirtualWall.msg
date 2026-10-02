uint8 TYPE_LINE=0
uint8 TYPE_POLYGON=1

string id
string name
string map_id
uint8 type
# Map-frame points: exactly 2 for LINE, 3 or more for POLYGON.
MapPoint[] points
# Wall thickness in metres (LINE only; ignored for POLYGON).
float64 thickness
bool enabled
builtin_interfaces/Time created_at
builtin_interfaces/Time updated_at
