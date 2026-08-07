uint8 AVAILABLE=0
uint8 BUSY=1
uint8 BLOCKED=2
uint8 NETWORK_LOST=3
uint8 NAVIGATION_ERROR=4
uint8 LOCALIZATION_ERROR=5
uint8 LOW_BATTERY=6
uint8 EMERGENCY_STOP=7
uint8 UNAVAILABLE=8

uint8 ROLE_NONE=0
uint8 ROLE_AED_DELIVERY=1
uint8 ROLE_HELPER_REQUEST=2
uint8 ROLE_GUIDE=3
uint8 ROLE_RETURN=4

string robot_id
builtin_interfaces/Time stamp
geometry_msgs/PoseStamped pose
# 관제와 ETA 계산이 모두 쓴다. 받는 쪽마다 pose 로 다시 계산하면
# 같은 값이 화면마다 달라진다.
float32 speed_mps
float32 battery_percentage
uint8 availability
uint8 role
string mission_id
bool is_docked
bool network_ok
bool localization_ok
bool nav2_ok
bool emergency_stop
bool path_valid
float32 estimated_path_cost
string path_event_id
builtin_interfaces/Time last_heartbeat
string detail
