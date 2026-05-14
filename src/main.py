from __future__ import annotations

import logging
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.openapi.docs import get_swagger_ui_html
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from starlette.middleware.cors import CORSMiddleware

from src.application.factories import (
    create_drive_controller,
    create_isolated_motion_service,
    create_shared_motion_hardware,
)
from src.application.services.motion_events import MotionEventHub
from src.config.settings import Settings
from src.presentation.api.exception_handlers import setup_exception_handlers
from src.presentation.api.v1.routers import router as v1_router


def _parse_cors_origins(raw: str) -> list[str]:
    text = raw.strip()
    if text == "*":
        return ["*"]
    return [part.strip() for part in text.split(",") if part.strip()]


def _configure_l123_debug_logging(settings: Settings) -> None:
    """Вывести трейс L1->L2->L3 в stderr, даже если корневой logging не настроен."""
    if not settings.l123_debug_trace_enabled:
        return
    log = logging.getLogger("src.application.services.isolated_motion_service")
    log.setLevel(logging.INFO)
    if log.handlers:
        return
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("%(levelname)s %(name)s: %(message)s"))
    log.addHandler(handler)
    log.propagate = False


def _configure_motion_diag_logging(settings: Settings) -> None:
    """Компактная диагностика движения в stderr без шумного DEBUG по умолчанию."""
    if not settings.motion_diag_logging_enabled:
        return

    formatter: logging.Formatter = logging.Formatter("%(levelname)s %(name)s: %(message)s")
    module_logger: logging.Logger = logging.getLogger(
        "src.application.services.isolated_motion_service",
    )
    module_logger.setLevel(logging.INFO)
    if module_logger.handlers:
        return
    diag_handler: logging.StreamHandler = logging.StreamHandler()
    diag_handler.setFormatter(formatter)
    module_logger.addHandler(diag_handler)
    module_logger.propagate = False


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    """Управление жизненным циклом приложения."""
    _configure_l123_debug_logging(app.state.settings)
    _configure_motion_diag_logging(app.state.settings)
    if app.state.isolated_motion is not None:
        app.state.isolated_motion.start()

    yield

    if app.state.isolated_motion is not None:
        app.state.isolated_motion.destroy(release_hardware=False)

    if app.state.drive_controller is not None:
        app.state.drive_controller.destroy(release_devices=False)

    if app.state.motion_hardware is not None:
        app.state.motion_hardware.destroy()


def create_app(settings: Settings) -> FastAPI:
    """Создание FastAPI-приложения."""
    app: FastAPI = FastAPI(
        title="Server RaspTank",
        version="1.0.0",
        description="Сервис для работы с RaspTank.",
        lifespan=lifespan,
        docs_url=None,
    )

    app.mount("/static", StaticFiles(directory="src/static"), name="static")

    @app.get("/docs", include_in_schema=False)
    async def custom_swagger_ui_html():
        return get_swagger_ui_html(
            openapi_url=app.openapi_url,
            title=app.title + " - Swagger UI",
            oauth2_redirect_url=app.swagger_ui_oauth2_redirect_url,
            swagger_js_url="/static/swagger-ui-bundle.js",
            swagger_css_url="/static/swagger-ui.css",
        )

    @app.get("/dashboard", include_in_schema=False)
    async def get_dashboard():
        return FileResponse("src/static/dashboard.html")

    app.state.settings = settings
    app.state.motion_events = MotionEventHub()
    app.state.motion_hardware = create_shared_motion_hardware(settings)
    app.state.isolated_motion = create_isolated_motion_service(settings, app.state.motion_hardware)
    app.state.drive_controller = create_drive_controller(
        settings,
        app.state.motion_events,
        app.state.motion_hardware,
        app.state.isolated_motion,
    )

    setup_exception_handlers(app)

    cors_origins = _parse_cors_origins(settings.cors_origins)
    app.add_middleware(
        CORSMiddleware,
        allow_origins=cors_origins,
        allow_credentials=False,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    app.include_router(v1_router)

    return app


settings: Settings = Settings()
app: FastAPI = create_app(settings=settings)

if __name__ == "__main__":  # pragma: no cover
    import uvicorn

    uvicorn.run("src.main:app", host=settings.app_host, port=settings.app_port, reload=True)
