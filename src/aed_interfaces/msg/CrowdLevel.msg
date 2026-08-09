# 특정 구역의 혼잡도. 출동 로봇 대수를 정하는 근거다.
#
# 등급만 보내면 "왜 그 등급인가"를 사후에 알 수 없다. 판단에 쓴 사람 수와
# 기준값을 함께 실어, 기준을 조정할 때 근거가 남게 한다.

uint8 CLEAR=0       # 사람 없음
uint8 BUSY=1        # 1명, 통과 가능
uint8 CROWDED=2     # 2명, 감속 통과
uint8 BLOCKED=3     # 3명 이상, 통행 불가
uint8 NOT_APPLICABLE=255  # 혼잡도를 사용하지 않는 카메라

string camera_id
string zone_id
builtin_interfaces/Time stamp

uint8 level
uint32 person_count          # 관심 영역 안에서 검출된 사람 수
float32 time_multiplier      # 1.0/1.1/1.2, 통행 불가는 NaN
bool traversable
