"""실시간 상태 통로. WebSocket 하나로 화면 전체를 먹인다.

토픽별로 통로를 나누지 않는다. 화면은 어차피 한 번에 전부 다시 그리고,
통로가 여러 개면 어느 것이 최신인지 맞추는 일이 프론트엔드로 넘어간다.
"""

from fastapi import APIRouter, Request, WebSocket, WebSocketDisconnect

from ..domain import serialize

router = APIRouter(tags=["live"])


@router.websocket("/ws/live")
async def live(websocket: WebSocket):
    hub = websocket.app.state.context.hub
    await hub.connect(websocket)
    try:
        while True:
            # 화면에서 올라오는 말은 아직 없다. 받기를 계속해야 연결이
            # 끊긴 것을 알 수 있어서 대기만 한다.
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        await hub.disconnect(websocket)


@router.get("/api/live/snapshot")
async def snapshot(request: Request):
    """WebSocket 을 못 쓸 때의 대체 통로. 시험과 디버깅에도 쓴다."""
    context = request.app.state.context
    return serialize.system_snapshot(context.build_snapshot())
