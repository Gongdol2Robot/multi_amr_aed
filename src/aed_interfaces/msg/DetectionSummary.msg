# 한 프레임의 검출 결과 요약.
#
# 고정 웹캠과 AMR OAK-D가 같은 메시지를 사용하며, 활성 backend와 모드에
# 따라 쓰러진 사람·실제 사람·조력자 후보 항목을 채운다.
#
# 상자 좌표까지 전부 실으면 초당 수십 개가 오가는데, 화면은 개수와 최고
# 신뢰도만 쓴다. 상세 상자가 필요하면 debug 영상에 이미 그려져 있다.

string camera_id
builtin_interfaces/Time stamp

uint32 person_count          # 서 있는 사람 포함 전체
uint32 fallen_count          # 쓰러진 것으로 판단된 사람
uint32 helper_count          # 환자 근처의 조력자 또는 helper 검출 수

float32 top_fallen_confidence
float32 top_helper_confidence

# 가장 신뢰도 높은 쓰러진 사람의 지도 좌표. 호모그래피로 변환한 값이며
# 변환이 불가능하면 frame_id 가 빈 문자열이다.
geometry_msgs/PointStamped fallen_location
