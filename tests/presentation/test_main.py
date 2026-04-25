from __future__ import annotations

from unittest.mock import Mock, patch

import anyio
from fastapi import FastAPI

from src.config.settings import Settings
from src.main import _parse_cors_origins, create_app, lifespan


class FakeDriveController:
    """Минимальный контроллер с destroy hook для lifespan-тестов."""

    def __init__(self) -> None:
        """Инициализировать флаг вызова destroy."""
        self.destroy_called: bool = False

    def destroy(self, *, release_devices: bool = True) -> None:
        """Зафиксировать освобождение ресурсов."""
        self.destroy_called = True


class FakeIsolatedMotion:
    """Минимальный новый контур с хуками start и destroy."""

    def __init__(self) -> None:
        """Подготовить флаги вызовов."""
        self.start_called: bool = False
        self.destroy_called: bool = False

    def start(self) -> None:
        """Зафиксировать запуск нового контура."""
        self.start_called = True

    def destroy(self, *, release_hardware: bool = True) -> None:
        """Зафиксировать освобождение ресурсов нового контура."""
        self.destroy_called = True


def test_parse_cors_origins_splits_comma_separated_list() -> None:
    """CORS_ORIGINS со списком через запятую превращается в список origin."""
    assert _parse_cors_origins("http://a, http://b") == ["http://a", "http://b"]
    assert _parse_cors_origins("*") == ["*"]


def test_create_app_sets_state_and_mounts_routes() -> None:
    """create_app собирает состояние приложения и подключает основные маршруты."""
    settings: Settings = Settings()
    drive: FakeDriveController = FakeDriveController()
    isolated_motion: FakeIsolatedMotion = FakeIsolatedMotion()

    motion_hardware: Mock = Mock()

    with (
        patch(
            "src.main.create_shared_motion_hardware", return_value=motion_hardware
        ) as hardware_factory,
        patch("src.main.create_drive_controller", return_value=drive) as drive_factory,
        patch(
            "src.main.create_isolated_motion_service", return_value=isolated_motion
        ) as motion_factory,
    ):
        app: FastAPI = create_app(settings)

    hardware_factory.assert_called_once_with(settings)
    motion_factory.assert_called_once_with(settings, motion_hardware)
    drive_factory.assert_called_once_with(
        settings,
        app.state.motion_events,
        motion_hardware,
        isolated_motion,
    )
    assert app.title == "Server RaspTank"
    assert app.state.settings is settings
    assert app.state.motion_hardware is motion_hardware
    assert app.state.drive_controller is drive
    assert app.state.isolated_motion is isolated_motion

    paths: set[str | None] = {getattr(route, "path", None) for route in app.routes}
    assert "/v1/health" in paths
    assert "/v1/l1/state" in paths
    assert "/v1/l2/state" in paths
    assert "/v1/l3/state" in paths


def test_lifespan_destroys_drive_controller_on_shutdown() -> None:
    """lifespan вызывает destroy у drive controller при остановке приложения."""

    async def run() -> None:
        """Пройти async context manager lifespan."""
        app: Mock = Mock()
        drive: FakeDriveController = FakeDriveController()
        isolated_motion: FakeIsolatedMotion = FakeIsolatedMotion()
        motion_hardware: Mock = Mock()
        app.state.drive_controller = drive
        app.state.isolated_motion = isolated_motion
        app.state.motion_hardware = motion_hardware

        async with lifespan(app):
            assert drive.destroy_called is False
            assert isolated_motion.start_called is True

        assert drive.destroy_called is True
        assert isolated_motion.destroy_called is True
        motion_hardware.destroy.assert_called_once_with()

    anyio.run(run)


def test_lifespan_ignores_missing_drive_controller() -> None:
    """lifespan корректно завершается, если drive controller отсутствует."""

    async def run() -> None:
        """Пройти async context manager lifespan без контроллера."""
        app: Mock = Mock()
        app.state.drive_controller = None
        app.state.isolated_motion = None

        async with lifespan(app):
            pass

    anyio.run(run)
