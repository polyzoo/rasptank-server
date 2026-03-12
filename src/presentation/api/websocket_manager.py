from __future__ import annotations

import asyncio
import json
import logging
from queue import Empty, Queue
from typing import Any

from fastapi import WebSocket

logger: logging.Logger = logging.getLogger(__name__)


class ConnectionManager:
    """Управление WebSocket-подключениями и рассылка сообщений.

    Поддерживает 'broadcast' из синхронных потоков через очередь.
    """

    def __init__(self) -> None:
        """Инициализация менеджера."""
        self._connections: set[WebSocket] = set()
        self._outgoing: Queue[dict[str, Any]] = Queue()
        self._broadcast_task: asyncio.Task[None] | None = None

    async def connect(self, websocket: WebSocket) -> None:
        """Принять новое подключение."""
        await websocket.accept()
        self._connections.add(websocket)
        logger.info("WebSocket подключён, всего: %d", len(self._connections))

    def disconnect(self, websocket: WebSocket) -> None:
        """Отключить клиента."""
        self._connections.discard(websocket)
        logger.info("WebSocket отключён, осталось: %d", len(self._connections))

    def broadcast(self, message: dict[str, Any]) -> None:
        """Добавить сообщение в очередь для рассылки."""
        self._outgoing.put_nowait(message)

    def start_broadcast_task(self) -> None:
        """Запустить фоновую задачу рассылки."""
        if self._broadcast_task is None or self._broadcast_task.done():
            self._broadcast_task = asyncio.create_task(self._broadcast_loop())

    async def __aenter__(self) -> ConnectionManager:
        """Запускает фоновый цикл при входе в async-контекст."""
        self.start_broadcast_task()
        return self

    async def __aexit__(self, *args: object) -> None:
        """Останавливает фоновый цикл при выходе из async-контекста."""
        if self._broadcast_task and not self._broadcast_task.done():
            self._broadcast_task.cancel()
            try:
                await self._broadcast_task
            except asyncio.CancelledError:
                pass

    async def _broadcast_loop(self) -> None:
        """Фоновый цикл рассылки сообщений из очереди."""
        while True:
            try:
                message: Any = self._outgoing.get_nowait()
            except Empty:
                await asyncio.sleep(0.1)
                continue

            data: str = json.dumps(message, ensure_ascii=False)
            dead: set[WebSocket] = set()

            for conn in self._connections:
                try:
                    await conn.send_text(data)
                except Exception as exc:
                    logger.exception("WebSocket не удалось отправить сообщение: %s", exc)
                    dead.add(conn)

            for conn in dead:
                self.disconnect(conn)
