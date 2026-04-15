from __future__ import annotations

from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

import anyio
from fastapi import WebSocketDisconnect

from src.application.services.l3_models import KnownObstacle, L3State, TargetPoint, TargetRoute
from src.presentation.api.v1.controllers.l3 import (
    l3_cancel,
    l3_goal,
    l3_route,
    l3_state,
    l3_state_ws,
    l3_step,
)
from src.presentation.api.v1.schemas.l3 import (
    L3GoalRequestSchema,
    L3RouteRequestSchema,
    L3StateResponseSchema,
)


class FakeIsolatedMotion:
    """Заглушка нового контура для тестов контроллера L3."""

    def __init__(self) -> None:
        """Подготовить состояние вызовов."""
        self.update_interval_sec: float = 0.01
        self.goal_calls: list[tuple[TargetPoint, tuple[KnownObstacle, ...]]] = []
        self.route_calls: list[tuple[TargetRoute, tuple[KnownObstacle, ...]]] = []
        self.step_calls: int = 0
        self.cancel_calls: int = 0
        self.state: L3State = L3State(
            status="tracking",
            mode="point",
            planner_status="planned",
            target_x_cm=50.0,
            target_y_cm=10.0,
            active_point_index=0,
            total_points=1,
            distance_error_cm=12.0,
            heading_error_deg=5.0,
            target_heading_deg=15.0,
            linear_speed_cm_per_sec=20.0,
            angular_speed_deg_per_sec=30.0,
            detected_obstacle_x_cm=11.0,
            detected_obstacle_y_cm=22.0,
            detected_obstacle_radius_cm=8.0,
            detected_obstacle_kind="dynamic",
        )

    def get_l3_state(self) -> L3State:
        """Вернуть состояние L3."""
        return self.state

    def set_l3_goal(
        self,
        target: TargetPoint,
        obstacles: tuple[KnownObstacle, ...] = (),
    ) -> L3State:
        """Сохранить цель L3."""
        self.goal_calls.append((target, obstacles))
        return self.state

    def set_l3_route(
        self,
        route: TargetRoute,
        obstacles: tuple[KnownObstacle, ...] = (),
    ) -> L3State:
        """Сохранить маршрут L3."""
        self.route_calls.append((route, obstacles))
        return self.state

    def step_l3(self) -> L3State:
        """Сохранить ручной шаг L3."""
        self.step_calls += 1
        return self.state

    def cancel_l3(self) -> L3State:
        """Сохранить отмену L3."""
        self.cancel_calls += 1
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


def test_l3_state_returns_current_snapshot() -> None:
    """REST-обработчик возвращает текущее состояние L3."""

    async def run() -> None:
        """Выполнить async handler внутри цикла событий."""
        response: L3StateResponseSchema = await l3_state(
            isolated_motion=FakeIsolatedMotion()  # type: ignore[arg-type]
        )

        assert response.status == "tracking"
        assert response.target_x_cm == 50.0
        assert response.detected_obstacle_x_cm == 11.0

    anyio.run(run)


def test_l3_goal_route_step_and_cancel_forward_commands() -> None:
    """REST-обработчики L3 передают цель, маршрут, шаг и отмену в новый контур."""

    async def run() -> None:
        """Выполнить async handlers внутри цикла событий."""
        isolated_motion: FakeIsolatedMotion = FakeIsolatedMotion()

        goal_response = await l3_goal(
            body=L3GoalRequestSchema.model_validate(
                {
                    "target": {"x_cm": 10.0, "y_cm": 20.0},
                    "obstacles": [{"x_cm": 5.0, "y_cm": 6.0, "radius_cm": 3.0}],
                }
            ),
            isolated_motion=isolated_motion,  # type: ignore[arg-type]
        )
        route_response = await l3_route(
            body=L3RouteRequestSchema.model_validate(
                {
                    "points": [{"x_cm": 1.0, "y_cm": 2.0}, {"x_cm": 3.0, "y_cm": 4.0}],
                    "obstacles": [],
                }
            ),
            isolated_motion=isolated_motion,  # type: ignore[arg-type]
        )
        step_response = await l3_step(isolated_motion=isolated_motion)  # type: ignore[arg-type]
        cancel_response = await l3_cancel(isolated_motion=isolated_motion)  # type: ignore[arg-type]

        assert goal_response.planner_status == "planned"
        assert route_response.total_points == 1
        assert step_response.linear_speed_cm_per_sec == 20.0
        assert cancel_response.angular_speed_deg_per_sec == 30.0
        assert goal_response.detected_obstacle_kind == "dynamic"
        assert isolated_motion.goal_calls[0][0] == TargetPoint(x_cm=10.0, y_cm=20.0)
        assert isolated_motion.goal_calls[0][1] == (
            KnownObstacle(x_cm=5.0, y_cm=6.0, radius_cm=3.0),
        )
        assert isolated_motion.route_calls[0][0] == TargetRoute(
            points=(TargetPoint(x_cm=1.0, y_cm=2.0), TargetPoint(x_cm=3.0, y_cm=4.0))
        )
        assert isolated_motion.step_calls == 1
        assert isolated_motion.cancel_calls == 1

    anyio.run(run)


def test_l3_ws_sends_state_snapshot() -> None:
    """WebSocket L3 отправляет состояние и завершает цикл после disconnect."""

    async def run() -> None:
        """Выполнить WebSocket handler внутри цикла событий."""
        websocket: FakeWebSocket = FakeWebSocket(FakeIsolatedMotion(), disconnect_after_sends=2)

        with patch(
            "src.presentation.api.v1.controllers.l3.asyncio.sleep",
            side_effect=WebSocketDisconnect(),
        ):
            await l3_state_ws(websocket)

        assert websocket.accepted is True
        assert websocket.sent[0]["planner_status"] == "planned"
        assert websocket.sent[0]["detected_obstacle_x_cm"] == 11.0

    anyio.run(run)
