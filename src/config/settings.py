from typing import final

from pydantic import BaseSettings, Field


@final
class Settings(BaseSettings):
    """Основные настройки приложения."""

    app_host: str = Field(default="0.0.0.0", alias="APP_HOST", description="Хост веб-сервера.")
    app_port: int = Field(default=8010, alias="APP_PORT", description="Порт веб-сервера.")

    # Параметры контроллера движения
    min_obstacle_distance_cm: float = Field(
        default=20.0,
        ge=0,
        alias="MIN_OBSTACLE_DISTANCE_CM",
        description="Минимальная дистанция до препятствия (см), при которой остановка.",
    )
    deceleration_distance_cm: float = Field(
        default=50.0,
        ge=0,
        alias="DECELERATION_DISTANCE_CM",
        description="Зона торможения по препятствию (см).",
    )
    base_speed_percent: int = Field(
        default=60,
        ge=0,
        le=100,
        alias="BASE_SPEED_PERCENT",
        description="Базовая скорость движения (%).",
    )
    update_interval_sec: float = Field(
        default=0.1,
        gt=0,
        le=1.0,
        alias="UPDATE_INTERVAL_SEC",
        description="Интервал обновления датчика (с).",
    )

    class Config:
        env_file: str = ".env"
        env_file_encoding: str = "utf-8"
        extra: str = "ignore"
