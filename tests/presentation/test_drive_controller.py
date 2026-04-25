from __future__ import annotations

from typing import Any

import anyio
import pytest

from src.application.models.route import (
    BackwardSegment,
    ForwardSegment,
    Route,
    TurnLeftSegment,
    TurnRightSegment,
)
from src.presentation.api.v1.controllers.drive import _schema_to_route, drive_route, drive_stop
from src.presentation.api.v1.schemas.drive import (
    DriveRouteResponseSchema,
    DriveStopResponseSchema,
    RouteRequestSchema,
)


class SpyDriveController:
    """Spy-контроллер для проверки вызовов эндпоинтов в слое представления."""

    def __init__(self) -> None:
        """Инициализировать хранилище вызовов."""
        self.routes: list[Route] = []
        self.stop_called: bool = False

    def execute_route(self, route: Route) -> None:
        """Запомнить переданный маршрут."""
        self.routes.append(route)

    def stop(self) -> None:
        """Зафиксировать вызов остановки."""
        self.stop_called = True


def test_schema_to_route_maps_all_supported_segments() -> None:
    """API-схема маршрута преобразуется во все поддерживаемые доменные сегменты."""
    data: dict[str, Any] = {
        "segments": [
            {"action": "forward", "distance_cm": 10},
            {"action": "backward", "distance_cm": 5},
            {"action": "turn_left", "angle_deg": 45},
            {"action": "turn_right", "angle_deg": 90},
        ]
    }
    body: RouteRequestSchema = RouteRequestSchema.model_validate(data)

    route: Route = _schema_to_route(body)

    assert [type(segment) for segment in route.segments] == [
        ForwardSegment,
        BackwardSegment,
        TurnLeftSegment,
        TurnRightSegment,
    ]
    assert route.segments[0].distance_cm == 10
    assert route.segments[1].distance_cm == 5
    assert route.segments[2].angle_deg == 45
    assert route.segments[3].angle_deg == 90


def test_route_request_rejects_empty_segments() -> None:
    """Пустой маршрут отклоняется на уровне Pydantic-схемы."""
    with pytest.raises(ValueError):
        RouteRequestSchema.model_validate({"segments": []})


def test_schema_to_route_rejects_unsupported_constructed_segment() -> None:
    """Защитная ветка отклоняет неподдерживаемый вручную собранный сегмент."""
    body: RouteRequestSchema = RouteRequestSchema.model_construct(segments=[object()])
    with pytest.raises(ValueError, match="Неподдерживаемый сегмент маршрута"):
        _schema_to_route(body)


def test_drive_route_executes_route_and_returns_accepted() -> None:
    """POST-запрос запускает маршрут и возвращает, что он принят."""

    async def run() -> None:
        """Выполнить асинхронный обработчик внутри цикла событий."""
        drive: SpyDriveController = SpyDriveController()
        body: RouteRequestSchema = RouteRequestSchema.model_validate(
            {
                "segments": [
                    {"action": "forward", "distance_cm": 12.5},
                ]
            }
        )

        response: DriveRouteResponseSchema = await drive_route(
            body=body,
            drive=drive,
            isolated_motion=None,
        )

        assert response.status == "accepted"
        assert len(drive.routes) == 1
        assert isinstance(drive.routes[0].segments[0], ForwardSegment)
        assert drive.routes[0].segments[0].distance_cm == 12.5

    anyio.run(run)


def test_drive_stop_stops_drive_and_returns_stopped() -> None:
    """POST-запрос вызывает остановку и возвращает, что он остановлен."""

    async def run() -> None:
        """Выполнить асинхронный обработчик внутри цикла событий."""
        drive: SpyDriveController = SpyDriveController()

        response: DriveStopResponseSchema = await drive_stop(
            drive=drive,
            isolated_motion=None,
        )

        assert response.status == "stopped"
        assert drive.stop_called is True

    anyio.run(run)
