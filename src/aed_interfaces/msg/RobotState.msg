uint8 AVAILABLE=0
uint8 BUSY=1
uint8 UNAVAILABLE=2
uint8 ERROR=3

uint8 ROLE_NONE=0
uint8 ROLE_PATROL=1
uint8 ROLE_CLEANING=2
uint8 ROLE_AED_DELIVERY=3
uint8 ROLE_GUIDE=4
uint8 ROLE_RETURN=5

string robot_id
builtin_interfaces/Time stamp
geometry_msgs/PoseStamped pose
float32 battery_percentage
uint8 availability
uint8 role
string mission_id
string detail

