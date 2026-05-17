from __future__ import annotations

from types import SimpleNamespace
from typing import Any

import pytest
from fastapi import HTTPException, status

from src.presentation.api.dependencies import (
    get_isolated_motion_service,
)


def _request(
    isolated_motion: Any = None,
) -> SimpleNamespace:
    """Собрать минимальный объект похожий на запрос для приложения."""
    state: SimpleNamespace = SimpleNamespace(
        isolated_motion=isolated_motion,
    )
    return SimpleNamespace(app=SimpleNamespace(state=state))


def test_get_isolated_motion_service_returns_configured_service() -> None:
    """Зависимость возвращает изолированный контур из состояния приложения."""
    isolated_motion: object = object()
    assert get_isolated_motion_service(_request(isolated_motion=isolated_motion)) is isolated_motion


def test_get_isolated_motion_service_raises_when_missing() -> None:
    """Зависимость отдаёт HTTP 500 без координатора L1-L3."""
    with pytest.raises(HTTPException) as exc_info:
        get_isolated_motion_service(_request(isolated_motion=None))
    assert exc_info.value.status_code == status.HTTP_500_INTERNAL_SERVER_ERROR
