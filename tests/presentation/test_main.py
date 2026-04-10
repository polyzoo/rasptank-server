from __future__ import annotations

from unittest.mock import Mock, patch

import anyio
from fastapi import FastAPI

from src.config.settings import Settings
from src.main import create_app, lifespan


class FakeDriveController:
    """Минимальный контроллер с destroy hook для lifespan-тестов."""

    def __init__(self) -> None:
        """Инициализировать флаг вызова destroy."""
        self.destroy_called: bool = False

    def destroy(self) -> None:
        """Зафиксировать освобождение ресурсов."""
        self.destroy_called = True


def test_create_app_sets_state_and_mounts_routes() -> None:
    """create_app собирает состояние приложения и подключает основные маршруты."""
    settings: Settings = Settings()
    drive: FakeDriveController = FakeDriveController()

    with patch("src.main.create_drive_controller", return_value=drive) as factory:
        app: FastAPI = create_app(settings)

    factory.assert_called_once_with(settings, app.state.motion_events)
    assert app.title == "Server RaspTank"
    assert app.state.settings is settings
    assert app.state.drive_controller is drive

    paths: set[str | None] = {getattr(route, "path", None) for route in app.routes}
    assert "/v1/health" in paths
    assert "/demo" in paths


def test_lifespan_destroys_drive_controller_on_shutdown() -> None:
    """lifespan вызывает destroy у drive controller при остановке приложения."""

    async def run() -> None:
        """Пройти async context manager lifespan."""
        app: Mock = Mock()
        drive: FakeDriveController = FakeDriveController()
        app.state.drive_controller = drive

        async with lifespan(app):
            assert drive.destroy_called is False

        assert drive.destroy_called is True

    anyio.run(run)


def test_lifespan_ignores_missing_drive_controller() -> None:
    """lifespan корректно завершается, если drive controller отсутствует."""

    async def run() -> None:
        """Пройти async context manager lifespan без контроллера."""
        app: Mock = Mock()
        app.state.drive_controller = None

        async with lifespan(app):
            pass

    anyio.run(run)
