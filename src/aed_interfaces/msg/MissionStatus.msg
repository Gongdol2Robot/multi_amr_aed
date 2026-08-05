uint8 ASSIGNED=0
uint8 DISPATCHING=1
uint8 EN_ROUTE=2
uint8 ARRIVED=3
uint8 COMPLETED=4
uint8 CANCELED=5
uint8 BLOCKED=6
uint8 NETWORK_LOST=7
uint8 NAVIGATION_ERROR=8
uint8 MISSION_FAILED=9

string mission_id
string event_id
string robot_id
uint32 assignment_version
uint8 status
builtin_interfaces/Time stamp
string reason
