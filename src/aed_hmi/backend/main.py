"""FastAPI 앱 조립. 여기서는 붙이기만 하고 로직을 두지 않는다.

실행:
  python3 -m backend.main                 ROS 에 붙는다
"""

import argparse
import contextlib
import logging
import os

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .api import history, live, map_view, report, video
from .context import Context, Settings

LOGGER = logging.getLogger(__name__)


def _start(context: Context, settings: Settings) -> None:
    context.hub.start()
    from .ros.bridge import RosBridge

    context.bridge = RosBridge(
        on_robot=context.on_robot,
        on_event=context.on_event,
        on_mission=context.on_mission,
        on_frame=context.on_frame,
        on_person_count=context.on_person_count,
        on_eta_record=context.on_eta_record,
        on_assignment=context.on_assignment,
    )
    context.bridge.start()
    if not context.bridge.connected:
        LOGGER.error(
            "ROS 노드가 안 떴다. ROS_SUPER_CLIENT=True 와 "
            "discovery server 설정을 확인하라."
        )


async def _stop(context: Context) -> None:
    await context.hub.stop()
    context.shutdown()


def create_app(settings: Settings) -> FastAPI:
    context = Context(settings)

    @contextlib.asynccontextmanager
    async def lifespan(app: FastAPI):
        _start(context, settings)
        yield
        await _stop(context)

    app = FastAPI(
        title="Multi-AMR AED 관제",
        description="AED 전달 AMR 의 상태·영상·이력을 한 화면에서 본다.",
        version="0.1.0",
        lifespan=lifespan,
    )
    app.state.context = context

    # 개발 중에는 Vite(5173)가 따로 뜨고 API 는 8000 이라 출처가 다르다.
    app.add_middleware(
        CORSMiddleware,
        allow_origins=list(settings.cors_origins),
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    app.include_router(live.router)
    app.include_router(history.router)
    app.include_router(video.router)
    app.include_router(map_view.router)
    app.include_router(report.router)
    return app


def parse_args() -> Settings:
    parser = argparse.ArgumentParser(description="Multi-AMR AED 관제 서버")
    parser.add_argument(
        "--db", default=os.environ.get("AED_HMI_DB", "var/aed_hmi.sqlite3"),
        help="SQLite 파일 경로",
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    return Settings(database_path=args.db, host=args.host, port=args.port)


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )
    import uvicorn

    settings = parse_args()
    uvicorn.run(
        create_app(settings),
        host=settings.host, port=settings.port, log_level="info",
    )


if __name__ == "__main__":
    main()
