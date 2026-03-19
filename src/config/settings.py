from __future__ import annotations

from typing import final

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict


@final
class Settings(BaseSettings):
    """Основные настройки приложения."""

    app_host: str = Field(
        default="0.0.0.0",
        validation_alias="APP_HOST",
        description="Хост веб-сервера.",
    )
    app_port: int = Field(
        default=8010,
        validation_alias="APP_PORT",
        description="Порт веб-сервера.",
    )

    # Параметры контроллера движения
    min_obstacle_distance_cm: float = Field(
        default=20.0,
        ge=0,
        validation_alias="MIN_OBSTACLE_DISTANCE_CM",
        description="Минимальная дистанция до препятствия (см), при которой остановка.",
    )
    deceleration_distance_cm: float = Field(
        default=50.0,
        ge=0,
        validation_alias="DECELERATION_DISTANCE_CM",
        description="Зона торможения по препятствию (см).",
    )
    base_speed_percent: int = Field(
        default=60,
        ge=0,
        le=100,
        validation_alias="BASE_SPEED_PERCENT",
        description="Базовая скорость движения (%).",
    )
    turn_speed_percent: int = Field(
        default=50,
        ge=0,
        le=100,
        validation_alias="TURN_SPEED_PERCENT",
        description="Скорость поворотов на месте (%).",
    )
    update_interval_sec: float = Field(
        default=0.1,
        gt=0,
        le=1.0,
        validation_alias="UPDATE_INTERVAL_SEC",
        description="Интервал обновления датчика (с).",
    )
    avoidance_scan_angle_deg: float = Field(
        default=45.0,
        ge=0,
        le=90.0,
        validation_alias="AVOIDANCE_SCAN_ANGLE_DEG",
        description="Угол короткого сканирования при выборе стороны обхода.",
    )
    avoidance_side_step_cm: float = Field(
        default=12.0,
        gt=0,
        le=100.0,
        validation_alias="AVOIDANCE_SIDE_STEP_CM",
        description="Длина одного бокового шага при обходе препятствия.",
    )
    avoidance_forward_step_cm: float = Field(
        default=15.0,
        gt=0,
        le=100.0,
        validation_alias="AVOIDANCE_FORWARD_STEP_CM",
        description="Длина одного шага вдоль препятствия.",
    )
    avoidance_rejoin_step_cm: float = Field(
        default=12.0,
        gt=0,
        le=100.0,
        validation_alias="AVOIDANCE_REJOIN_STEP_CM",
        description="Длина одной попытки возврата на исходную траекторию.",
    )
    avoidance_max_attempts: int = Field(
        default=24,
        ge=1,
        le=200,
        validation_alias="AVOIDANCE_MAX_ATTEMPTS",
        description="Максимальное число шагов автомата обхода.",
    )
    avoidance_confirm_readings: int = Field(
        default=3,
        ge=1,
        le=9,
        validation_alias="AVOIDANCE_CONFIRM_READINGS",
        description="Количество чтений датчика для подтверждения свободного пути.",
    )
    avoidance_min_side_clearance_cm: float = Field(
        default=25.0,
        gt=0,
        le=200.0,
        validation_alias="AVOIDANCE_MIN_SIDE_CLEARANCE_CM",
        description="Минимально допустимое расстояние для выбора стороны обхода.",
    )
    avoidance_max_lateral_offset_cm: float = Field(
        default=60.0,
        gt=0,
        le=500.0,
        validation_alias="AVOIDANCE_MAX_LATERAL_OFFSET_CM",
        description="Максимально допустимое боковое смещение при обходе.",
    )
    avoidance_max_bypass_distance_cm: float = Field(
        default=200.0,
        gt=0,
        le=1000.0,
        validation_alias="AVOIDANCE_MAX_BYPASS_DISTANCE_CM",
        description="Максимальная суммарная длина обходного манёвра.",
    )

    # Прямолинейность (отклонение ≤ 5 см)
    tl_left_offset: int = Field(
        default=0,
        ge=-50,
        le=50,
        validation_alias="TL_LEFT_OFFSET",
        description="Смещение левого мотора (M2).",
    )
    tl_right_offset: int = Field(
        default=0,
        ge=-50,
        le=50,
        validation_alias="TL_RIGHT_OFFSET",
        description="Смещение правого мотора (M1).",
    )

    # Оценка пройденного пути
    max_speed_cm_per_sec: float = Field(
        default=30.0,
        gt=0,
        le=100.0,
        validation_alias="MAX_SPEED_CM_PER_SEC",
        description="Линейная скорость при 100%.",
    )

    model_config: SettingsConfigDict = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
    )
