from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

from fastapi import FastAPI
import uvicorn

from src.config.settings import Settings
from src.presentation.api.exception_handlers import setup_exception_handlers
from src.presentation.api.v1.routers import router as v1_router


@asynccontextmanager
async def lifespan(_: FastAPI) -> AsyncIterator[None]:
    """Контекстный менеджер для управления жизненным циклом приложения."""
    yield


def create_app(settings: Settings) -> FastAPI:
    """Создание и настройка приложения."""
    app: FastAPI = FastAPI(
        title="Server RaspTank",
        version="1.0.0",
        description="Сервер для машины RaspTank.",
        lifespan=lifespan,
        docs_url="/docs",
    )

    # TODO: внедрить контроллеры, реализующие протоколы из src.application.protocols
    app.state.health_controller = None
    app.state.drive_controller = None
    app.state.settings = settings

    setup_exception_handlers(app)

    app.include_router(v1_router)

    return app


settings: Settings = Settings()
app: FastAPI = create_app(settings=settings)

if __name__ == "__main__":
    uvicorn.run("src.main:app", host=settings.app_host, port=settings.app_port, reload=True)
