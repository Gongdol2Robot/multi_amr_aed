# 로봇 센서 상태와 대체 주행 여부.
#
# 라이다가 죽으면 Nav2 를 쓸 수 없다. 그때 sensor_recovery 가 cmd_vel 로
# 직접 몬다. 두 주체가 같은 cmd_vel 에 동시에 쓰면 로봇이 요동치므로,
# degraded 가 참인 동안 emergency_mission_manager는 Nav2 goal을 내지 않는다.

uint8 OK=0
uint8 LIDAR_STALE=1          # 스캔이 끊김
uint8 LIDAR_INVALID=2        # 스캔은 오는데 값이 비정상
uint8 LIDAR_DEAD=3           # 노드 자체가 죽음

string robot_id
builtin_interfaces/Time stamp

uint8 lidar_status
float32 scan_age_s           # 마지막 스캔 이후 경과 시간
float32 valid_ratio          # 유효 거리값 비율 0.0~1.0

bool degraded                # 참이면 Nav2 사용 불가
bool manual_drive_active     # sensor_recovery 가 cmd_vel 을 잡고 있는지
string detail
