from __future__ import annotations

from queue import Empty
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

import anyio
from fastapi import WebSocketDisconnect

from src.application.services.motion_events import MotionEvent, MotionEventHub
from src.presentation.api.v1.controllers.motion import motion_events_ws, motion_state


class FakeWebSocket:
    """Минимальная WebSocket-заглушка для прямого вызова endpoint handler."""

    def __init__(self, hub: MotionEventHub, *, disconnect_after_sends: int = 1) -> None:
        """Инициализировать WebSocket state и лимит отправок до disconnect."""
        self.accepted: bool = False
        self.sent: list[dict[str, Any]] = []
        app: SimpleNamespace = SimpleNamespace(state=SimpleNamespace(motion_events=hub))
        self.scope: dict[str, Any] = {"app": app}
        self.scope["app"].state.motion_events = hub
        self._disconnect_after_sends: int = disconnect_after_sends

    async def accept(self) -> None:
        """Зафиксировать принятие WebSocket-соединения."""
        self.accepted = True

    async def send_json(self, payload: dict[str, Any]) -> None:
        """Сохранить payload и имитировать отключение после N отправок."""
        self.sent.append(payload)
        if len(self.sent) >= self._disconnect_after_sends:
            raise WebSocketDisconnect()


def test_motion_state_returns_last_event() -> None:
    """motion_state возвращает последнее опубликованное событие."""

    async def run() -> None:
        """Выполнить async handler внутри anyio loop."""
        hub: MotionEventHub = MotionEventHub()
        hub.publish(
            MotionEvent(
                type="position",
                status="moving",
                x_cm=1.234,
                y_cm=5.678,
                heading_deg=90,
                obstacle_cm=11.111,
            )
        )

        response: dict[str, Any] = await motion_state(motion_events=hub)

        assert response["type"] == "position"
        assert response["status"] == "moving"
        assert response["x_cm"] == 1.23
        assert response["y_cm"] == 5.68
        assert response["obstacle_cm"] == 11.11

    anyio.run(run)


def test_motion_events_ws_accepts_sends_event_and_unsubscribes() -> None:
    """WebSocket handler отправляет стартовое событие и отписывает клиента."""

    async def run() -> None:
        """Выполнить WebSocket handler внутри anyio loop."""
        hub: MotionEventHub = MotionEventHub()
        websocket: FakeWebSocket = FakeWebSocket(hub)

        await motion_events_ws(websocket)

        assert websocket.accepted is True
        assert websocket.sent[0]["status"] == "idle"

        subscribers: set[Any] = getattr(hub, "_subscribers")
        assert subscribers == set()

    anyio.run(run)


def test_motion_events_ws_sends_last_event_when_queue_times_out() -> None:
    """WebSocket handler отправляет last_event при timeout очереди и продолжает цикл."""

    async def run() -> None:
        """Выполнить WebSocket handler с подмененной очередью событий."""
        hub: MotionEventHub = MotionEventHub()
        hub.publish(
            MotionEvent(
                type="status",
                status="stopped",
                x_cm=2.0,
                y_cm=3.0,
                heading_deg=4.0,
            )
        )
        websocket: FakeWebSocket = FakeWebSocket(hub, disconnect_after_sends=2)
        next_event: MotionEvent = MotionEvent(
            type="position",
            status="moving",
            x_cm=5.0,
            y_cm=6.0,
            heading_deg=7.0,
        )

        with patch(
            "src.presentation.api.v1.controllers.motion.asyncio.to_thread",
            side_effect=[Empty, next_event],
        ):
            await motion_events_ws(websocket)

        assert websocket.accepted is True
        assert websocket.sent[0]["status"] == "stopped"
        assert websocket.sent[0]["x_cm"] == 2.0
        assert websocket.sent[1]["status"] == "moving"
        assert getattr(hub, "_subscribers") == set()

    anyio.run(run)
