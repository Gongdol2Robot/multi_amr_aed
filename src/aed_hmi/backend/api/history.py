"""이력 조회. 실시간 상태는 WebSocket 으로 가고, 여기는 지나간 것만 다룬다.

둘을 섞지 않는 이유는 성격이 달라서다. 실시간은 항상 최신 하나면 되고,
이력은 조건을 걸어 여러 건을 본다. 한 통로로 만들면 양쪽 다 어색해진다.
"""

from fastapi import APIRouter, HTTPException, Query, Request

from ..domain import serialize

router = APIRouter(prefix="/api", tags=["history"])


@router.get("/missions")
async def list_missions(
    request: Request,
    limit: int = Query(50, ge=1, le=500),
):
    """최근 임무 요약. 관제실 하단 표에 뿌린다."""
    repository = request.app.state.context.repository
    return {
        "items": [
            serialize.mission_summary(item)
            for item in repository.recent_missions(limit)
        ]
    }


@router.get("/missions/{mission_id}")
async def mission_detail(mission_id: str, request: Request):
    """한 임무의 상태 전이 전부. 왜 늦었는지 되짚을 때 본다."""
    repository = request.app.state.context.repository
    timeline = repository.mission_timeline(mission_id)
    if not timeline:
        raise HTTPException(status_code=404, detail=f"없는 임무: {mission_id}")
    return {
        "mission_id": mission_id,
        "timeline": [serialize.mission_event(item) for item in timeline],
    }


@router.get("/stats/response-time")
async def response_time(request: Request):
    """신고에서 도착까지 걸린 시간 통계."""
    return request.app.state.context.repository.response_time_stats()


@router.get("/stats/travel-time")
async def travel_time(request: Request):
    """출동 지시에서 도착까지. ETA 추정이 맞는지 견주는 기준이 된다."""
    from ..domain import eta

    stats = request.app.state.context.repository.travel_time_stats()
    # 지금 ETA 가 어떤 가정 위에 서 있는지 같이 내보낸다. 실측 평균과
    # 크게 어긋나면 이 값들을 조정해야 한다는 뜻이다.
    stats["assumptions"] = {
        "cruise_speed_mps": eta.CRUISE_SPEED_MPS,
        "cruise_efficiency": eta.CRUISE_EFFICIENCY,
        "detour_factor": eta.DETOUR_FACTOR,
    }
    return stats


@router.get("/stats/eta-accuracy")
async def eta_accuracy(request: Request, limit: int = 20):
    """예상이 실제와 얼마나 달랐나.

    /stats/travel-time 은 우리가 잰 주행 시간이고, 이쪽은 주행 전에 낸
    예상과 실제를 한 쌍으로 비교한 것이다. 예상 계수를 고칠 근거는
    이쪽에서만 나온다.
    """
    repository = request.app.state.context.repository
    stats = repository.eta_accuracy_stats()
    stats["recent"] = repository.recent_eta_records(limit)
    return stats


@router.get("/robots/{robot_id}/track")
async def robot_track(
    robot_id: str,
    request: Request,
    limit: int = Query(300, ge=1, le=5000),
):
    """최근 이동 궤적. 지도 위에 선으로 그린다."""
    return {
        "robot_id": robot_id,
        "points": request.app.state.context.repository.robot_track(
            robot_id, limit
        ),
    }


@router.get("/health")
async def health(request: Request):
    """감시용. 프로세스가 살아 있는지와 무엇에 붙어 있는지."""
    context = request.app.state.context
    snapshot = context.build_snapshot()
    return {
        "ok": True,
        "ros_connected": snapshot.ros_connected,
        "mock": context.settings.mock,
        "robots": len(snapshot.robots),
        "streams_online": sum(1 for item in snapshot.streams if item.online),
        "websocket_clients": context.hub.client_count,
    }
