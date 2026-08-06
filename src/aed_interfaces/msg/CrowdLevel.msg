# 특정 구역의 혼잡도. 출동 로봇 대수를 정하는 근거다.
#
# 등급만 보내면 "왜 그 등급인가"를 사후에 알 수 없다. 판단에 쓴 사람 수와
# 기준값을 함께 실어, 기준을 조정할 때 근거가 남게 한다.

uint8 CLEAR=0       # 사람 없음 또는 통행 지장 없음
uint8 MODERATE=1    # 사람이 있으나 통과 가능
uint8 CONGESTED=2   # 통행이 막힘. 로봇 2대를 보낸다

string camera_id
string zone_id
builtin_interfaces/Time stamp

uint8 level
uint32 person_count          # 관심 영역 안에서 검출된 사람 수
uint32 threshold             # 이 값 이상이면 CONGESTED 로 본다
float32 roi_occupancy        # 관심 영역 중 사람이 차지한 비율 0.0~1.0
