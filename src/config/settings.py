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
    turn_speed_percent: int = Field(
        default=50,
        ge=0,
        le=100,
        alias="TURN_SPEED_PERCENT",
        description="Скорость поворотов на месте (%).",
    )
    update_interval_sec: float = Field(
        default=0.1,
        gt=0,
        le=1.0,
        alias="UPDATE_INTERVAL_SEC",
        description="Интервал обновления датчика (с).",
    )

    # Прямолинейность (отклонение ≤ 5 см)
    tl_left_offset: int = Field(
        default=0,
        ge=-50,
        le=50,
        alias="TL_LEFT_OFFSET",
        description="Смещение левого мотора (M2).",
    )
    tl_right_offset: int = Field(
        default=0,
        ge=-50,
        le=50,
        alias="TL_RIGHT_OFFSET",
        description="Смещение правого мотора (M1).",
    )

    # Оценка пройденного пути
    max_speed_cm_per_sec: float = Field(
        default=30.0,
        gt=0,
        le=100.0,
        alias="MAX_SPEED_CM_PER_SEC",
        description="Линейная скорость при 100%.",
    )

    # Поворот на 90° (с)
    turn_duration_90_deg_sec: float = Field(
        default=0.5,
        gt=0,
        le=2.0,
        alias="TURN_DURATION_90_DEG_SEC",
        description="Длительность поворота на 90°.",
    )

    class Config:
        env_file: str = ".env"
        env_file_encoding: str = "utf-8"
        extra: str = "ignore"
