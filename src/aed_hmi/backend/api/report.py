"""운영자가 지도에서 찍은 자리를 받는 창구.

관제는 지금까지 받기만 했다. 여기가 유일하게 밖으로 나가는 통로다.

왜 POST 인가
------------
좌표를 넘기고 **접수됐는지 즉시 답을 받아야** 한다. 화면은 그 답의
event_id 로 "내가 찍은 그 신고"를 이력에서 찾는다. WebSocket 으로 보내면
받았는지 알 수 없고, 어느 응답이 내 요청에 대한 것인지도 못 가린다.

왜 검출과 같은 토픽으로 내보내나
--------------------------------
`/aed/emergency_event` 에 `EmergencyEvent` 로 낸다. 카메라가 본 것과 같은
토픽·같은 타입이다. emergency_mission_manager는 출처를 가리지 않고 같은 규칙으로
배정하고, 무엇이 신고했는지는 `source_id` 에만 남는다. 사람이 찍었다고
다른 경로를 만들면 배정 규칙이 두 벌이 된다.

"""

import logging

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel, Field

LOGGER = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["report"])


class OperatorReport(BaseModel):
    """지도에서 찍은 한 점."""

    x: float = Field(..., description="지도 좌표 x (m)")
    y: float = Field(..., description="지도 좌표 y (m)")
    zone_id: str = Field("operator", max_length=64)


@router.post("/report")
async def report(request: Request, body: OperatorReport):
    context = request.app.state.context

    bridge = getattr(context, "bridge", None)
    if bridge is not None and bridge.connected:
        try:
            event_id = bridge.publish_operator_report(
                body.x, body.y, body.zone_id
            )
        except Exception as error:
            LOGGER.exception("운영자 신고 발행 실패")
            raise HTTPException(status_code=503, detail=str(error))
        return {"accepted": True, "event_id": event_id, "via": "ros"}

    raise HTTPException(status_code=503, detail="ROS bridge가 연결되지 않았다")
