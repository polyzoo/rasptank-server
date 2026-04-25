from __future__ import annotations

from types import SimpleNamespace
from typing import Any

import pytest
from fastapi import HTTPException, status

from src.application.services.motion_events import MotionEventHub
from src.presentation.api.dependencies import (
    get_drive_controller,
    get_isolated_motion_optional,
    get_isolated_motion_service,
    get_motion_events,
)


class DummyDriveController:
    """Минимальная заглушка контроллера движения для тестов зависимостей."""


def _request(
    drive_controller: Any = None,
    motion_events: MotionEventHub | None = None,
    isolated_motion: Any = None,
) -> SimpleNamespace:
    """Собрать минимальный объект похожий на запрос для приложения."""
    state: SimpleNamespace = SimpleNamespace(
        drive_controller=drive_controller,
        motion_events=motion_events,
        isolated_motion=isolated_motion,
    )
    return SimpleNamespace(app=SimpleNamespace(state=state))


def test_get_drive_controller_returns_configured_controller() -> None:
    """Зависимость возвращает контроллер из внутреннего состояния приложения."""
    drive: DummyDriveController = DummyDriveController()
    assert get_drive_controller(_request(drive_controller=drive)) is drive


def test_get_drive_controller_raises_when_missing() -> None:
    """Зависимость отдает 500-ый статус, если контроллер не инициализирован."""
    with pytest.raises(HTTPException) as exc_info:
        get_drive_controller(_request(drive_controller=None))
    assert exc_info.value.status_code == status.HTTP_500_INTERNAL_SERVER_ERROR


def test_get_motion_events_returns_configured_hub() -> None:
    """Зависимость возвращает поток событий движения из внутреннего состояния приложения."""
    hub: MotionEventHub = MotionEventHub()
    assert get_motion_events(_request(motion_events=hub)) is hub


def test_get_motion_events_raises_when_missing() -> None:
    """Зависимость отдает 500-ый статус, если поток событий не инициализирован."""
    with pytest.raises(HTTPException) as exc_info:
        get_motion_events(_request(motion_events=None))
    assert exc_info.value.status_code == status.HTTP_500_INTERNAL_SERVER_ERROR


def test_get_isolated_motion_service_returns_configured_service() -> None:
    """Зависимость возвращает изолированный контур из состояния приложения."""
    isolated_motion: object = object()
    assert get_isolated_motion_service(_request(isolated_motion=isolated_motion)) is isolated_motion


def test_get_isolated_motion_service_raises_when_missing() -> None:
    """Зависимость отдает 500-ый статус, если новый контур не инициализирован."""
    with pytest.raises(HTTPException) as exc_info:
        get_isolated_motion_service(_request(isolated_motion=None))
    assert exc_info.value.status_code == status.HTTP_500_INTERNAL_SERVER_ERROR


def test_get_isolated_motion_optional_returns_none_without_attribute() -> None:
    """Опциональная зависимость возвращает None, если атрибута нет в state."""
    request: SimpleNamespace = SimpleNamespace(app=SimpleNamespace(state=SimpleNamespace()))
    assert get_isolated_motion_optional(request) is None
