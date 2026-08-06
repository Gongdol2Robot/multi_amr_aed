"""영상 중계. CompressedImage 로 받은 JPEG 를 MJPEG 로 흘린다.

브라우저는 <img src="/api/video/robot1"> 한 줄로 받는다. 별도 플레이어도,
코덱 협상도 필요 없다.

한 화면에 4갈래가 동시에 붙으므로, 프레임이 없을 때 바쁜 대기를 돌면
CPU 를 그대로 태운다. FrameBuffer 가 이벤트로 깨워 준다.
"""

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import StreamingResponse

router = APIRouter(prefix="/api/video", tags=["video"])

BOUNDARY = "aedframe"
# 프레임이 안 올 때 연결을 붙잡아 두는 한도. 이보다 길면 브라우저가 먼저 끊는다.
IDLE_TIMEOUT_S = 1.0


@router.get("/{stream_id}")
async def video_stream(stream_id: str, request: Request):
    registry = request.app.state.context.frames
    buffer = registry.get(stream_id)
    if buffer is None:
        raise HTTPException(
            status_code=404,
            detail=f"알 수 없는 영상 갈래: {stream_id} "
                   f"(가능: {', '.join(registry.stream_ids())})",
        )

    async def frames():
        while True:
            if await request.is_disconnected():
                break
            jpeg = buffer.wait_for_next(timeout=IDLE_TIMEOUT_S)
            if jpeg is None:
                continue
            yield (
                b"--" + BOUNDARY.encode() + b"\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                + jpeg + b"\r\n"
            )

    return StreamingResponse(
        frames(),
        media_type=f"multipart/x-mixed-replace; boundary={BOUNDARY}",
        headers={"Cache-Control": "no-store", "Connection": "close"},
    )


@router.get("/{stream_id}/snapshot")
async def video_snapshot(stream_id: str, request: Request):
    """정지 화면 한 장. 이력 화면의 썸네일이나 디버깅에 쓴다."""
    from fastapi.responses import Response

    buffer = request.app.state.context.frames.get(stream_id)
    if buffer is None:
        raise HTTPException(status_code=404, detail=f"없는 갈래: {stream_id}")
    jpeg = buffer.latest()
    if jpeg is None:
        raise HTTPException(status_code=503, detail="아직 프레임이 없습니다")
    return Response(content=jpeg, media_type="image/jpeg")
