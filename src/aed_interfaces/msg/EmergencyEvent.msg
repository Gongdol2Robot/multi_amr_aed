uint8 DETECTED=0
uint8 CONFIRMED=1
uint8 DISPATCHED=2
uint8 RESOLVED=3
uint8 CANCELED=4

string event_id
builtin_interfaces/Time detected_at
geometry_msgs/PointStamped location
float32 confidence
uint32 consecutive_detections
uint8 status
string source_id
string camera_id
string zone_id
# 이 이벤트를 판단한 시점의 혼잡도. CrowdLevel 의 등급과 같은 값이다.
# 사후에 "왜 2대가 갔나"를 되짚으려면 이 값이 남아 있어야 한다.
uint8 crowd_level
