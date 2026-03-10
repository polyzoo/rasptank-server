from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI

from src.application.services.drive_controller import DriveController
from src.config.settings import Settings
from src.infrastructures.motor import MotorController
from src.infrastructures.ultrasonic import UltrasonicSensor
from src.presentation.api.exception_handlers import setup_exception_handlers
from src.presentation.api.v1.routers import router as v1_router


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    """Управление жизненным циклом приложения."""
    yield
    drive_controller: DriveController = app.state.drive_controller
    drive_controller.destroy()


def create_app(settings: Settings) -> FastAPI:
    """Создание FastAPI-приложения."""
    app: FastAPI = FastAPI(
        title="Server RaspTank",
        version="1.0.0",
        description="Сервис для работы с RaspTank.",
        lifespan=lifespan,
        docs_url="/docs",
    )

    ultrasonic_sensor: UltrasonicSensor = UltrasonicSensor()
    motor_controller: MotorController = MotorController(
        tl_left_offset=settings.tl_left_offset,
        tl_right_offset=settings.tl_right_offset,
    )

    drive_controller: DriveController = DriveController(
        motor_controller=motor_controller,
        ultrasonic_sensor=ultrasonic_sensor,
        min_obstacle_distance_cm=settings.min_obstacle_distance_cm,
        deceleration_distance_cm=settings.deceleration_distance_cm,
        base_speed_percent=settings.base_speed_percent,
        turn_speed_percent=settings.turn_speed_percent,
        max_speed_cm_per_sec=settings.max_speed_cm_per_sec,
        update_interval_sec=settings.update_interval_sec,
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
