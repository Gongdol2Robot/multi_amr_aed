# 한 프레임의 검출 결과 요약.
#
# 고정 웹캠은 쓰러진 사람만 본다. AMR 의 OAK-D 는 쓰러진 사람과 helper(빨간
# RC카)를 함께 본다. 같은 메시지를 쓰되 채우는 항목이 다르다.
#
# 상자 좌표까지 전부 실으면 초당 수십 개가 오가는데, 화면은 개수와 최고
# 신뢰도만 쓴다. 상세 상자가 필요하면 debug 영상에 이미 그려져 있다.

string camera_id
builtin_interfaces/Time stamp

uint32 person_count          # 서 있는 사람 포함 전체
uint32 fallen_count          # 쓰러진 것으로 판단된 사람
uint32 helper_count          # helper 로봇(빨간 RC카). 웹캠은 항상 0

float32 top_fallen_confidence
float32 top_helper_confidence

# 가장 신뢰도 높은 쓰러진 사람의 지도 좌표. 호모그래피로 변환한 값이며
# 변환이 불가능하면 frame_id 가 빈 문자열이다.
geometry_msgs/PointStamped fallen_location
