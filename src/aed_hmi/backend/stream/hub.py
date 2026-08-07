"""WebSocket 연결을 모아 두고 상태를 주기적으로 밀어 넣는다.

밀어 넣는 방식(push)을 쓰는 이유는, 관제 화면이 "묻지 않아도 항상 최신"
이어야 하기 때문이다. 브라우저가 주기적으로 물으면 그 주기만큼 늦고,
연결이 끊긴 것도 늦게 안다.

전송 주기는 고정이다. ROS 메시지가 올 때마다 보내면 로봇 두 대가 각자
10Hz 로 상태를 뿌릴 때 초당 20번을 보내게 되고, 화면은 그만큼 못 그린다.
"""

import asyncio
import json
import logging
from typing import Callable

from fastapi import WebSocket

from ..domain import serialize

LOGGER = logging.getLogger(__name__)

# 관제 화면이 사람 눈에 충분히 부드럽고, 브라우저가 감당하는 선.
BROADCAST_INTERVAL_S = 0.25


class Hub:
    """접속한 화면들에 같은 상태를 뿌린다."""

    def __init__(self, snapshot_factory: Callable[[], object]) -> None:
        self._snapshot_factory = snapshot_factory
        self._clients: set[WebSocket] = set()
        self._lock = asyncio.Lock()
        self._task: asyncio.Task | None = None

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        async with self._lock:
            self._clients.add(websocket)
        # 새로 붙은 화면이 다음 주기까지 빈 화면을 보지 않게 즉시 한 장 보낸다.
        await self._send(websocket, self._payload())

    async def disconnect(self, websocket: WebSocket) -> None:
        async with self._lock:
            self._clients.discard(websocket)

    def start(self) -> None:
        self._task = asyncio.create_task(self._broadcast_loop())

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    @property
    def client_count(self) -> int:
        return len(self._clients)

    def _payload(self) -> dict:
        return {
            "type": "snapshot",
            "data": serialize.system_snapshot(self._snapshot_factory()),
        }

    async def _broadcast_loop(self) -> None:
        while True:
            await asyncio.sleep(BROADCAST_INTERVAL_S)
            if not self._clients:
                continue
            payload = self._payload()
            async with self._lock:
                targets = list(self._clients)
            dead = []
            for client in targets:
                if not await self._send(client, payload):
                    dead.append(client)
            if dead:
                async with self._lock:
                    for client in dead:
                        self._clients.discard(client)

    @staticmethod
    async def _send(websocket: WebSocket, payload: dict) -> bool:
        """보내기 실패는 정상적인 종료다. 예외를 위로 올리지 않는다."""
        try:
            await websocket.send_text(json.dumps(payload, ensure_ascii=False))
            return True
        except Exception:
            return False
