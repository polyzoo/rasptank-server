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
    turn_angle_trim_deg: float = Field(
        default=0.0,
        ge=-15.0,
        le=15.0,
        validation_alias="TURN_ANGLE_TRIM_DEG",
        description=(
            "Добавляется к углу поворота из маршрута: отрицательное — остановиться раньше "
            "(при переразвороте ~92° вместо 90° попробуйте -2…-4 на угол)."
        ),
    )
    last_turn_angle_trim_deg: float = Field(
        default=0.0,
        ge=-15.0,
        le=15.0,
        validation_alias="LAST_TURN_ANGLE_TRIM_DEG",
        description=(
            "Дополнительно только к последнему сегменту-повороту (замыкание). "
            "Если в конце квадрата нос смотрит вправо от старта — попробуйте +3…+8 (доворот влево)."
        ),
    )
    update_interval_sec: float = Field(
        default=0.1,
        gt=0,
        le=1.0,
        validation_alias="UPDATE_INTERVAL_SEC",
        description="Интервал обновления датчика (с).",
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

    heading_hold_enabled: bool = Field(
        default=True,
        validation_alias="HEADING_HOLD_ENABLED",
        description="Удержание курса по гироскопу при прямолинейных сегментах.",
    )
    heading_hold_kp: float = Field(
        default=2.8,
        ge=0.0,
        le=10.0,
        validation_alias="HEADING_HOLD_KP",
        description="Коэффициент P: усиление руления (%% дифференциала на 1° ошибки).",
    )
    heading_hold_steer_max: int = Field(
        default=45,
        ge=0,
        le=100,
        validation_alias="HEADING_HOLD_STEER_MAX",
        description="Максимальный дифференциал колёс при удержании курса (%).",
    )
    heading_hold_deadband_deg: float = Field(
        default=0.4,
        ge=0.0,
        le=5.0,
        validation_alias="HEADING_HOLD_DEADBAND_DEG",
        description="Зона нечувствительности по ошибке курса (°).",
    )
    heading_hold_steer_speed_ratio: float = Field(
        default=0.55,
        ge=0.05,
        le=0.58,
        validation_alias="HEADING_HOLD_STEER_SPEED_RATIO",
        description=(
            "Макс. |руление| ≤ скорость × ratio: при низкой скорости иначе одно колесо уходит в 0% и робот крутится."
        ),
    )
    heading_hold_min_speed_percent: float = Field(
        default=0.0,
        ge=0.0,
        le=30.0,
        validation_alias="HEADING_HOLD_MIN_SPEED_PERCENT",
        description=(
            "Ниже этой скорости (%) руление по курсу отключено. По умолчанию 0 — коррекция не обрубается "
            "в конце сегмента; ограничение только через steer_speed_ratio."
        ),
    )
    heading_hold_steer_trim: int = Field(
        default=0,
        ge=-20,
        le=20,
        validation_alias="HEADING_HOLD_STEER_TRIM",
        description=(
            "Постоянное смещение дифференциала (%) без калибровки TL_*: при уводе вправо попробуйте 2…5, "
            "влево — отрицательное."
        ),
    )
    heading_hold_invert_steer: bool = Field(
        default=False,
        validation_alias="HEADING_HOLD_INVERT_STEER",
        description=(
            "Инвертировать знак руления по курсу: True, если при прямой yaw уходит в минус, "
            "а машинка визуально вправо — коррекция «в разрез»."
        ),
    )
    forward_soft_start_sec: float = Field(
        default=0.35,
        ge=0.0,
        le=2.0,
        validation_alias="FORWARD_SOFT_START_SEC",
        description=(
            "Разгон в начале каждого прямого сегмента (с): 0 = выкл. Снижает рывок и ложный крен по гироскопу."
        ),
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
