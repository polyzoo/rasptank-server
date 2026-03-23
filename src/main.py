from __future__ import annotations

from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI

from src.application.factories import create_drive_controller
from src.config.settings import Settings
from src.presentation.api.exception_handlers import setup_exception_handlers
from src.presentation.api.v1.routers import router as v1_router


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    """Управление жизненным циклом приложения."""
    yield
    if app.state.drive_controller is not None:
        app.state.drive_controller.destroy()


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
    app.state.drive_controller = create_drive_controller(settings)

    setup_exception_handlers(app)

    app.include_router(v1_router)

    return app


settings: Settings = Settings()
app: FastAPI = create_app(settings=settings)

if __name__ == "__main__":
    uvicorn.run("src.main:app", host=settings.app_host, port=settings.app_port, reload=True)
