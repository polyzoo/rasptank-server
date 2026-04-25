from __future__ import annotations

from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

from fastapi import FastAPI
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


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    """Управление жизненным циклом приложения."""
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
        docs_url="/docs",
    )

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
