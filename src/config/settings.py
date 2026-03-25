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
        description="Хост сервера.",
    )
    app_port: int = Field(
        default=8010,
        validation_alias="APP_PORT",
        description="Порт сервера.",
    )
    min_obstacle_distance_cm: float = Field(
        default=20.0,
        ge=0,
        validation_alias="MIN_OBSTACLE_DISTANCE_CM",
        description="Минимальная дистанция до препятствия (см).",
    )
    deceleration_distance_cm: float = Field(
        default=10.0,
        ge=0,
        validation_alias="DECELERATION_DISTANCE_CM",
        description="Зона торможения (см).",
    )
    base_speed_percent: int = Field(
        default=55,
        ge=0,
        le=100,
        validation_alias="BASE_SPEED_PERCENT",
        description="Базовая скорость (%).",
    )
    turn_speed_percent: int = Field(
        default=72,
        ge=0,
        le=100,
        validation_alias="TURN_SPEED_PERCENT",
        description="Скорость поворота (%).",
    )
    turn_slowdown_remaining_deg: float = Field(
        default=8.0,
        ge=0.0,
        le=60.0,
        validation_alias="TURN_SLOWDOWN_REMAINING_DEG",
        description="Угол начала замедления поворота (°).",
    )
    turn_creep_speed_percent: int = Field(
        default=42,
        ge=10,
        le=100,
        validation_alias="TURN_CREEP_SPEED_PERCENT",
        description="Медленная скорость поворота (%).",
    )
    turn_angle_trim_deg: float = Field(
        default=-2.0,
        ge=-15.0,
        le=15.0,
        validation_alias="TURN_ANGLE_TRIM_DEG",
        description="Поправка угла поворота (°).",
    )
    last_turn_angle_trim_deg: float = Field(
        default=2.0,
        ge=-15.0,
        le=15.0,
        validation_alias="LAST_TURN_ANGLE_TRIM_DEG",
        description="Поправка последнего поворота (°).",
    )
    update_interval_sec: float = Field(
        default=0.1,
        gt=0,
        le=1.0,
        validation_alias="UPDATE_INTERVAL_SEC",
        description="Интервал обновления (с).",
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
        ge=3,
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
    tl_left_offset: int = Field(
        default=0,
        ge=-50,
        le=50,
        validation_alias="TL_LEFT_OFFSET",
        description="Смещение левого мотора.",
    )
    tl_right_offset: int = Field(
        default=0,
        ge=-50,
        le=50,
        validation_alias="TL_RIGHT_OFFSET",
        description="Смещение правого мотора.",
    )
    heading_hold_enabled: bool = Field(
        default=True,
        validation_alias="HEADING_HOLD_ENABLED",
        description="Удержание курса.",
    )
    heading_hold_kp: float = Field(
        default=6.0,
        ge=0.0,
        le=10.0,
        validation_alias="HEADING_HOLD_KP",
        description="Коэффициент P.",
    )
    heading_hold_steer_max: int = Field(
        default=85,
        ge=0,
        le=100,
        validation_alias="HEADING_HOLD_STEER_MAX",
        description="Максимум руления (%).",
    )
    heading_hold_deadband_deg: float = Field(
        default=0.15,
        ge=0.0,
        le=5.0,
        validation_alias="HEADING_HOLD_DEADBAND_DEG",
        description="Зона нечувствительности (°).",
    )
    heading_hold_steer_speed_ratio: float = Field(
        default=0.58,
        ge=0.05,
        le=0.58,
        validation_alias="HEADING_HOLD_STEER_SPEED_RATIO",
        description="Ограничение руления по скорости.",
    )
    heading_hold_min_speed_percent: float = Field(
        default=0.0,
        ge=0.0,
        le=30.0,
        validation_alias="HEADING_HOLD_MIN_SPEED_PERCENT",
        description="Минимальная скорость для руления (%).",
    )
    heading_hold_steer_cap_min_speed_percent: float = Field(
        default=45.0,
        ge=0.0,
        le=60.0,
        validation_alias="HEADING_HOLD_STEER_CAP_MIN_SPEED_PERCENT",
        description="Минимум скорости для лимита руления (%).",
    )
    heading_hold_steer_trim: int = Field(
        default=0,
        ge=-20,
        le=20,
        validation_alias="HEADING_HOLD_STEER_TRIM",
        description="Постоянная поправка руления (%).",
    )
    heading_hold_invert_steer: bool = Field(
        default=True,
        validation_alias="HEADING_HOLD_INVERT_STEER",
        description="Инверсия знака руления.",
    )
    forward_soft_start_sec: float = Field(
        default=0.35,
        ge=0.0,
        le=2.0,
        validation_alias="FORWARD_SOFT_START_SEC",
        description="Мягкий старт прямого сегмента (с).",
    )
    turn_check_interval_sec: float = Field(
        default=0.01,
        gt=0.0,
        le=0.2,
        validation_alias="TURN_CHECK_INTERVAL_SEC",
        description="Пауза цикла поворота (с).",
    )
    turn_obstacle_check_interval_sec: float = Field(
        default=0.20,
        gt=0.0,
        le=1.0,
        validation_alias="TURN_OBSTACLE_CHECK_INTERVAL_SEC",
        description="Пауза проверки препятствия (с).",
    )
    turn_timeout_per_deg: float = Field(
        default=0.05,
        gt=0.0,
        le=1.0,
        validation_alias="TURN_TIMEOUT_PER_DEG",
        description="Секунд на градус поворота.",
    )
    turn_timeout_min: float = Field(
        default=1.0,
        gt=0.0,
        le=10.0,
        validation_alias="TURN_TIMEOUT_MIN",
        description="Минимальный таймаут поворота (с).",
    )
    max_speed_cm_per_sec: float = Field(
        default=28.0,
        gt=0,
        le=100.0,
        validation_alias="MAX_SPEED_CM_PER_SEC",
        description="Скорость при 100%.",
    )

    model_config: SettingsConfigDict = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
    )
