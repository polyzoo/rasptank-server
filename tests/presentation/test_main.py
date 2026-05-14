from __future__ import annotations

import logging
from unittest.mock import Mock, patch

import anyio
from fastapi import FastAPI

from src.config.settings import Settings
from src.main import (
    _configure_l123_debug_logging,
    _configure_motion_diag_logging,
    _parse_cors_origins,
    create_app,
    lifespan,
)


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


def test_docs_and_dashboard_endpoints() -> None:
    """Проверка доступности кастомного /docs и /dashboard."""
    settings: Settings = Settings()
    with (
        patch("src.main.create_shared_motion_hardware"),
        patch("src.main.create_drive_controller"),
        patch("src.main.create_isolated_motion_service"),
    ):
        app: FastAPI = create_app(settings)

        async def run() -> None:
            docs_endpoint = next(
                route.endpoint for route in app.routes if getattr(route, "path", None) == "/docs"
            )
            dashboard_endpoint = next(
                route.endpoint
                for route in app.routes
                if getattr(route, "path", None) == "/dashboard"
            )

            res_docs = await docs_endpoint()
            assert "swagger-ui-bundle.js" in res_docs.body.decode()

            res_dashboard = await dashboard_endpoint()
            assert res_dashboard.path == "src/static/dashboard.html"

        anyio.run(run)


def test_lifespan_destroys_drive_controller_on_shutdown() -> None:
    """lifespan вызывает destroy у drive controller при остановке приложения."""

    async def run() -> None:
        """Пройти async context manager lifespan."""
        app: Mock = Mock()
        drive: FakeDriveController = FakeDriveController()
        isolated_motion: FakeIsolatedMotion = FakeIsolatedMotion()
        motion_hardware: Mock = Mock()
        app.state.settings = Settings(motion_diag_logging_enabled=False)
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
        app.state.settings = Settings(motion_diag_logging_enabled=False)
        app.state.drive_controller = None
        app.state.isolated_motion = None

        async with lifespan(app):
            pass

    anyio.run(run)


def test_configure_l123_debug_logging_returns_early_when_disabled() -> None:
    """При выключенном флаге debug trace логгер не настраивается."""
    settings: Settings = Settings(l123_debug_trace_enabled=False)
    logger: logging.Logger = logging.getLogger("src.application.services.isolated_motion_service")
    previous_level: int = logger.level
    previous_handlers: list[logging.Handler] = list(logger.handlers)
    previous_propagate: bool = logger.propagate

    try:
        _configure_l123_debug_logging(settings)
        assert logger.level == previous_level
        assert logger.handlers == previous_handlers
        assert logger.propagate == previous_propagate
    finally:
        logger.setLevel(previous_level)
        logger.handlers = previous_handlers
        logger.propagate = previous_propagate


def test_configure_l123_debug_logging_attaches_stream_handler_when_enabled() -> None:
    """При включённом L123-трейсе логгер isolated_motion получает handler на stderr."""
    settings: Settings = Settings(l123_debug_trace_enabled=True)
    logger: logging.Logger = logging.getLogger("src.application.services.isolated_motion_service")
    previous_level: int = logger.level
    previous_handlers: list[logging.Handler] = list(logger.handlers)
    previous_propagate: bool = logger.propagate

    try:
        logger.handlers.clear()
        _configure_l123_debug_logging(settings)
        assert logger.level == logging.INFO
        assert len(logger.handlers) == 1
        assert isinstance(logger.handlers[0], logging.StreamHandler)
        assert logger.propagate is False
    finally:
        logger.handlers.clear()
        logger.handlers.extend(previous_handlers)
        logger.setLevel(previous_level)
        logger.propagate = previous_propagate


def test_configure_l123_debug_logging_skips_if_handler_already_present() -> None:
    """Повторная настройка L123 не добавляет второй handler."""
    settings: Settings = Settings(l123_debug_trace_enabled=True)
    logger: logging.Logger = logging.getLogger("src.application.services.isolated_motion_service")
    previous_level: int = logger.level
    previous_handlers: list[logging.Handler] = list(logger.handlers)
    previous_propagate: bool = logger.propagate

    try:
        logger.handlers.clear()
        _configure_l123_debug_logging(settings)
        first_count: int = len(logger.handlers)
        _configure_l123_debug_logging(settings)
        assert len(logger.handlers) == first_count == 1
    finally:
        logger.handlers.clear()
        logger.handlers.extend(previous_handlers)
        logger.setLevel(previous_level)
        logger.propagate = previous_propagate


def test_configure_motion_diag_logging_attaches_handlers_when_enabled() -> None:
    """Диагностика движения по умолчанию вешает INFO-handler только на L2 sync."""
    settings: Settings = Settings(motion_diag_logging_enabled=True)
    names: tuple[str, ...] = ("src.application.services.isolated_motion_service",)
    snapshots: dict[str, tuple[int, list[logging.Handler], bool]] = {
        name: (
            logging.getLogger(name).level,
            list(logging.getLogger(name).handlers),
            logging.getLogger(name).propagate,
        )
        for name in names
    }

    try:
        for name in names:
            log: logging.Logger = logging.getLogger(name)
            log.handlers.clear()

        _configure_motion_diag_logging(settings)

        for name in names:
            log = logging.getLogger(name)
            assert log.level == logging.INFO
            assert len(log.handlers) == 1
            assert isinstance(log.handlers[0], logging.StreamHandler)
            assert log.propagate is False
    finally:
        for name in names:
            log = logging.getLogger(name)
            prev_level, prev_handlers, prev_propagate = snapshots[name]
            log.handlers.clear()
            log.handlers.extend(prev_handlers)
            log.setLevel(prev_level)
            log.propagate = prev_propagate


def test_configure_motion_diag_logging_continue_when_handlers_already_present() -> None:
    """Повторный вызов не дублирует diag-handlers."""
    settings: Settings = Settings(motion_diag_logging_enabled=True)
    names: tuple[str, ...] = ("src.application.services.isolated_motion_service",)
    snapshots: dict[str, tuple[int, list[logging.Handler], bool]] = {
        name: (
            logging.getLogger(name).level,
            list(logging.getLogger(name).handlers),
            logging.getLogger(name).propagate,
        )
        for name in names
    }

    try:
        for name in names:
            logging.getLogger(name).handlers.clear()

        _configure_motion_diag_logging(settings)
        counts: dict[str, int] = {name: len(logging.getLogger(name).handlers) for name in names}
        _configure_motion_diag_logging(settings)
        for name in names:
            assert len(logging.getLogger(name).handlers) == counts[name]
    finally:
        for name in names:
            log = logging.getLogger(name)
            prev_level, prev_handlers, prev_propagate = snapshots[name]
            log.handlers.clear()
            log.handlers.extend(prev_handlers)
            log.setLevel(prev_level)
            log.propagate = prev_propagate


def test_configure_motion_diag_logging_returns_early_when_disabled() -> None:
    """При выключенной диагностике движения логгер sync не трогаем."""
    settings: Settings = Settings(motion_diag_logging_enabled=False)
    names: tuple[str, ...] = ("src.application.services.isolated_motion_service",)
    snapshots: dict[str, tuple[int, list[logging.Handler], bool]] = {}
    for name in names:
        log: logging.Logger = logging.getLogger(name)
        snapshots[name] = (log.level, list(log.handlers), log.propagate)

    try:
        _configure_motion_diag_logging(settings)
        for name in names:
            log = logging.getLogger(name)
            prev_level, prev_handlers, prev_propagate = snapshots[name]
            assert log.level == prev_level
            assert log.handlers == prev_handlers
            assert log.propagate == prev_propagate
    finally:
        for name in names:
            log = logging.getLogger(name)
            prev_level, prev_handlers, prev_propagate = snapshots[name]
            log.setLevel(prev_level)
            log.handlers = prev_handlers
            log.propagate = prev_propagate
