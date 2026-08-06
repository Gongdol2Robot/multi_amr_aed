"""AED 도착 예상 시간을 낸다. 순수 계산이라 ROS 도 저장소도 모른다.

관제실에서 "언제 도착하나"는 운영자가 가장 자주 묻는 것이다. 그런데 이 값은
본질적으로 추정이라, 어떻게 구했는지 모르면 믿을 수도 없고 틀렸을 때 원인도
못 찾는다. 그래서 규칙을 이 파일 하나에 모으고 근거를 함께 내보낸다.

거리를 구하는 방법이 둘이다.

  1) RobotState.estimated_path_cost — mission_manager 가 로봇을 고를 때 쓰는
     값이고 Nav2 경로 길이에서 온다. 벽을 돌아가는 실제 거리라 이쪽이 맞다.
  2) 직선 거리 — 1) 이 없을 때의 대비책. 벽을 뚫고 가는 거리라 항상 짧게
     나온다. 그래서 우회 계수를 곱해 보정한다.

속도도 둘이다. 지금 달리는 속도를 쓰되, 멈춰 있거나 이제 막 출발했으면
순항 속도로 대신한다. 0 으로 나누면 무한대가 나오고 화면이 깨진다.
"""

import math
from dataclasses import dataclass
from typing import Optional

# nav2_aed.yaml 의 max_vel_x 와 맞춘다. 실제로는 가감속과 회전 때문에
# 이보다 느리므로, 아래 EFFICIENCY 로 한 번 더 깎는다.
CRUISE_SPEED_MPS = 0.20

# 순항 속도로 계속 달리는 로봇은 없다. 회전, 감속, 장애물 회피가 섞인다.
# 실측 전까지 쓰는 값이고, 도착 기록이 쌓이면 조정해야 한다.
CRUISE_EFFICIENCY = 0.65

# 직선 거리를 실제 주행 거리로 바꾸는 계수. 벽과 장애물을 돌아가는 몫이다.
# 경로 비용을 받을 수 있으면 이 계수는 쓰이지 않는다.
DETOUR_FACTOR = 1.35

# 이보다 느리면 "지금 속도"를 신뢰하지 않는다. 정지 중이거나 회전 중이다.
MIN_TRUSTED_SPEED_MPS = 0.05

# 도착 판정 반경. Nav2 의 xy_goal_tolerance(0.15) 보다 조금 넉넉하게 둔다.
ARRIVAL_RADIUS_M = 0.25


@dataclass(frozen=True)
class EtaEstimate:
    """도착 예상과 그 근거. 근거를 같이 줘야 화면에서 신뢰도를 표시할 수 있다."""

    seconds: Optional[float]
    distance_m: float
    speed_mps: float
    # "path_cost" 또는 "straight_line". 어느 쪽으로 쟀는지.
    distance_source: str
    # "measured" 또는 "cruise". 실측 속도인지 가정값인지.
    speed_source: str

    @property
    def confident(self) -> bool:
        """둘 다 실측일 때만 믿을 만하다. 화면에서 이 값으로 표시를 달리한다."""
        return (
            self.distance_source == "path_cost"
            and self.speed_source == "measured"
        )


def estimate(
    robot_x: float, robot_y: float,
    target_x: float, target_y: float,
    speed_mps: float,
    path_cost: float = -1.0,
    path_valid: bool = True,
) -> EtaEstimate:
    """남은 거리와 속도로 도착까지 걸릴 시간을 낸다.

    path_cost 가 음수면 아직 경로를 못 구한 것이다(RobotState 의 규약).
    그 경우 직선 거리로 떨어진다.
    """
    straight = math.hypot(target_x - robot_x, target_y - robot_y)

    if path_cost >= 0.0 and path_valid:
        distance = path_cost
        distance_source = "path_cost"
    else:
        distance = straight * DETOUR_FACTOR
        distance_source = "straight_line"

    if speed_mps >= MIN_TRUSTED_SPEED_MPS:
        effective_speed = speed_mps
        speed_source = "measured"
    else:
        effective_speed = CRUISE_SPEED_MPS * CRUISE_EFFICIENCY
        speed_source = "cruise"

    # 이미 도착 반경 안이면 남은 시간은 0 이다. 작은 거리를 느린 속도로
    # 나눠 몇 초가 더 남았다고 표시하면 운영자가 헷갈린다.
    if straight <= ARRIVAL_RADIUS_M:
        seconds: Optional[float] = 0.0
    elif effective_speed <= 0.0:
        seconds = None
    else:
        seconds = distance / effective_speed

    return EtaEstimate(
        seconds=seconds,
        distance_m=distance,
        speed_mps=effective_speed,
        distance_source=distance_source,
        speed_source=speed_source,
    )
