# GuideHelper 결과처럼 위치·거리가 필요한 구조화된 조력자 관측용 메시지다.
# vision_detector의 즉시 정지 신호(Bool helper_confirmed)와는 목적이 다르다.
string event_id
string robot_id
uint16 helper_count
uint16 evidence_count
float32 distance_m
geometry_msgs/PoseStamped helper_pose
builtin_interfaces/Time stamp
