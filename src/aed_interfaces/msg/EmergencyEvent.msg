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

