from __future__ import annotations

from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

import anyio
from fastapi import WebSocketDisconnect

from src.application.services.l1_models import L1SensorState
from src.presentation.api.v1.controllers.l1 import l1_state, l1_state_ws, l1_stop, l1_tracks
from src.presentation.api.v1.schemas.l1 import (
    L1ActionResponseSchema,
    L1SensorStateResponseSchema,
    L1TrackCommandRequestSchema,
)


class FakeIsolatedMotion:
    """Заглушка нового контура для тестов контроллера L1."""

    def __init__(self) -> None:
        """Подготовить состояние вызовов."""
        self.update_interval_sec: float = 0.01
        self.track_commands: list[tuple[int, int]] = []
        self.stop_calls: int = 0
        self.sensor_state: L1SensorState = L1SensorState(
            angular_speed_z_deg_per_sec=1.0,
            accel_x_m_s2=2.0,
            accel_y_m_s2=3.0,
            accel_z_m_s2=4.0,
            distance_cm=5.0,
        )

    def read_l1_state(self) -> L1SensorState:
        """Вернуть снимок датчиков."""
        return self.sensor_state

    def apply_l1_track_command(self, left_percent: int, right_percent: int) -> None:
        """Сохранить сырую команду бортам."""
        self.track_commands.append((left_percent, right_percent))

    def stop_l1(self) -> None:
        """Сохранить остановку бортов."""
        self.stop_calls += 1


class FakeWebSocket:
    """Минимальная заглушка WebSocket для прямого вызова handler."""

    def __init__(
        self, isolated_motion: FakeIsolatedMotion, *, disconnect_after_sends: int = 1
    ) -> None:
        """Сохранить ссылку на состояние приложения."""
        self.accepted: bool = False
        self.sent: list[dict[str, Any]] = []
        app: SimpleNamespace = SimpleNamespace(
            state=SimpleNamespace(isolated_motion=isolated_motion)
        )
        self.scope: dict[str, Any] = {"app": app}
        self._disconnect_after_sends: int = disconnect_after_sends

    async def accept(self) -> None:
        """Зафиксировать принятие соединения."""
        self.accepted = True

    async def send_json(self, payload: dict[str, Any]) -> None:
        """Сохранить отправленные данные и затем имитировать отключение."""
        self.sent.append(payload)
        if len(self.sent) >= self._disconnect_after_sends:
            raise WebSocketDisconnect()


def test_l1_state_returns_sensor_snapshot() -> None:
    """REST-обработчик возвращает снимок датчиков L1."""

    async def run() -> None:
        """Выполнить async handler внутри цикла событий."""
        isolated_motion: FakeIsolatedMotion = FakeIsolatedMotion()
        response: L1SensorStateResponseSchema = await l1_state(isolated_motion=isolated_motion)  # type: ignore[arg-type]

        assert response.distance_cm == 5.0
        assert response.accel_z_m_s2 == 4.0

    anyio.run(run)


def test_l1_tracks_and_stop_forward_commands() -> None:
    """REST-обработчики L1 передают команду борту и остановку в новый контур."""

    async def run() -> None:
        """Выполнить async handlers внутри цикла событий."""
        isolated_motion: FakeIsolatedMotion = FakeIsolatedMotion()

        accepted: L1ActionResponseSchema = await l1_tracks(
            body=L1TrackCommandRequestSchema(left_percent=10, right_percent=-10),
            isolated_motion=isolated_motion,  # type: ignore[arg-type]
        )
        stopped: L1ActionResponseSchema = await l1_stop(
            isolated_motion=isolated_motion  # type: ignore[arg-type]
        )

        assert accepted.status == "accepted"
        assert stopped.status == "stopped"
        assert isolated_motion.track_commands == [(10, -10)]
        assert isolated_motion.stop_calls == 1

    anyio.run(run)


def test_l1_ws_sends_sensor_snapshot() -> None:
    """WebSocket L1 отправляет снимок датчиков и завершает цикл после disconnect."""

    async def run() -> None:
        """Выполнить WebSocket handler внутри цикла событий."""
        websocket: FakeWebSocket = FakeWebSocket(FakeIsolatedMotion(), disconnect_after_sends=2)

        with patch(
            "src.presentation.api.v1.controllers.l1.asyncio.sleep",
            side_effect=WebSocketDisconnect(),
        ):
            await l1_state_ws(websocket)

        assert websocket.accepted is True
        assert websocket.sent[0]["distance_cm"] == 5.0

    anyio.run(run)
