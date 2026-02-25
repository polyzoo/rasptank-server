from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

from fastapi import FastAPI
import uvicorn

from src.application.services.drive_controller import DriveController
from src.config.settings import Settings
from src.infrastructures.motor import MotorController
from src.infrastructures.ultrasonic import UltrasonicSensor
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

    motor_controller: MotorController = MotorController()
    ultrasonic_sensor: UltrasonicSensor = UltrasonicSensor()
    drive_controller: DriveController = DriveController(
        motor_controller=motor_controller,
        ultrasonic_sensor=ultrasonic_sensor,
        min_obstacle_distance_cm=20.0,
        deceleration_distance_cm=50.0,
        base_speed_percent=60,
        update_interval_sec=0.1,
    )

    app.state.settings = settings
    app.state.drive_controller = drive_controller

    setup_exception_handlers(app)

    app.include_router(v1_router)

    return app


settings: Settings = Settings()
app: FastAPI = create_app(settings=settings)

if __name__ == "__main__":
    uvicorn.run("src.main:app", host=settings.app_host, port=settings.app_port, reload=True)
