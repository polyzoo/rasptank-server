from __future__ import annotations

from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

import anyio
from fastapi import WebSocketDisconnect

from src.application.services.l2_models import L2State
from src.presentation.api.v1.controllers.l2 import (
    l2_cmd_vel,
    l2_reset_state,
    l2_state,
    l2_state_ws,
    l2_stop,
)
from src.presentation.api.v1.schemas.l2 import (
    L2BodyVelocityRequestSchema,
    L2ResetStateRequestSchema,
    L2StateResponseSchema,
)


class FakeIsolatedMotion:
    """Заглушка нового контура для тестов контроллера L2."""

    def __init__(self) -> None:
        """Подготовить состояние вызовов."""
        self.update_interval_sec: float = 0.01
        self.cmd_vel_calls: list[tuple[float, float]] = []
        self.stop_calls: int = 0
        self.reset_calls: list[dict[str, float]] = []
        self.state: L2State = L2State(
            x_cm=1.0,
            y_cm=2.0,
            heading_deg=3.0,
            linear_speed_cm_per_sec=4.0,
            angular_speed_deg_per_sec=5.0,
            left_percent=6.0,
            right_percent=7.0,
            distance_cm=8.0,
        )

    def get_l2_state(self) -> L2State:
        """Вернуть состояние L2."""
        return self.state

    def apply_l2_body_velocity(
        self,
        linear_speed_cm_per_sec: float,
        angular_speed_deg_per_sec: float,
    ) -> L2State:
        """Сохранить команду корпуса."""
        self.cmd_vel_calls.append((linear_speed_cm_per_sec, angular_speed_deg_per_sec))
        return self.state

    def stop_l2(self) -> L2State:
        """Сохранить остановку L2."""
        self.stop_calls += 1
        return self.state

    def reset_l2_state(self, **kwargs: float) -> L2State:
        """Сохранить сброс состояния L2."""
        self.reset_calls.append(kwargs)
        return self.state


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


def test_l2_state_returns_current_snapshot() -> None:
    """REST-обработчик возвращает текущее состояние L2."""

    async def run() -> None:
        """Выполнить async handler внутри цикла событий."""
        response: L2StateResponseSchema = await l2_state(
            isolated_motion=FakeIsolatedMotion()  # type: ignore[arg-type]
        )

        assert response.x_cm == 1.0
        assert response.distance_cm == 8.0

    anyio.run(run)


def test_l2_cmd_vel_stop_and_reset_forward_commands() -> None:
    """REST-обработчики L2 передают команду, остановку и сброс состояния."""

    async def run() -> None:
        """Выполнить async handlers внутри цикла событий."""
        isolated_motion: FakeIsolatedMotion = FakeIsolatedMotion()

        cmd_response = await l2_cmd_vel(
            body=L2BodyVelocityRequestSchema(
                linear_speed_cm_per_sec=10.0,
                angular_speed_deg_per_sec=20.0,
            ),
            isolated_motion=isolated_motion,  # type: ignore[arg-type]
        )
        stop_response = await l2_stop(isolated_motion=isolated_motion)  # type: ignore[arg-type]
        reset_response = await l2_reset_state(
            body=L2ResetStateRequestSchema(x_cm=9.0, y_cm=8.0, heading_deg=7.0),
            isolated_motion=isolated_motion,  # type: ignore[arg-type]
        )

        assert cmd_response.left_percent == 6.0
        assert stop_response.right_percent == 7.0
        assert reset_response.heading_deg == 3.0
        assert isolated_motion.cmd_vel_calls == [(10.0, 20.0)]
        assert isolated_motion.stop_calls == 1
        assert isolated_motion.reset_calls[0]["heading_deg"] == 7.0

    anyio.run(run)


def test_l2_ws_sends_state_snapshot() -> None:
    """WebSocket L2 отправляет состояние и завершает цикл после disconnect."""

    async def run() -> None:
        """Выполнить WebSocket handler внутри цикла событий."""
        websocket: FakeWebSocket = FakeWebSocket(FakeIsolatedMotion(), disconnect_after_sends=2)

        with patch(
            "src.presentation.api.v1.controllers.l2.asyncio.sleep",
            side_effect=WebSocketDisconnect(),
        ):
            await l2_state_ws(websocket)

        assert websocket.accepted is True
        assert websocket.sent[0]["heading_deg"] == 3.0

    anyio.run(run)
