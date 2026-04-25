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
    head_servo_channel: int = Field(
        default=4,
        ge=0,
        le=15,
        validation_alias="HEAD_SERVO_CHANNEL",
        description="PCA9685-канал серво головы/датчика.",
    )
    head_servo_home_angle_deg: float = Field(
        default=0.0,
        ge=0.0,
        le=180.0,
        validation_alias="HEAD_SERVO_HOME_ANGLE_DEG",
        description="Угол фиксации головы перед стартом движения.",
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
    invert_line_motion: bool = Field(
        default=False,
        validation_alias="INVERT_LINE_MOTION",
        description=(
            "Поменять местами физическое «вперёд» и «назад» для move_forward/move_backward "
            "(сегменты forward/backward в /v1/drive/route). Повороты на месте (set_tracks) не меняются."
        ),
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
    track_width_cm: float = Field(
        default=17.0,
        gt=0.0,
        le=100.0,
        validation_alias="TRACK_WIDTH_CM",
        description=(
            "Расстояние между центрами левого и правого борта (см). "
            "Временный ориентир до калибровки."
        ),
    )
    left_track_max_speed_cm_per_sec: float = Field(
        default=30.0,
        gt=0.0,
        le=100.0,
        validation_alias="LEFT_TRACK_MAX_SPEED_CM_PER_SEC",
        description=(
            "Скорость левого борта при команде 100% (см/с). Временный ориентир до калибровки."
        ),
    )
    right_track_max_speed_cm_per_sec: float = Field(
        default=30.0,
        gt=0.0,
        le=100.0,
        validation_alias="RIGHT_TRACK_MAX_SPEED_CM_PER_SEC",
        description=(
            "Скорость правого борта при команде 100% (см/с). Временный ориентир до калибровки."
        ),
    )
    l3_position_tolerance_cm: float = Field(
        default=5.0,
        gt=0.0,
        le=50.0,
        validation_alias="L3_POSITION_TOLERANCE_CM",
        description="Допустимая ошибка по расстоянию до целевой точки (см).",
    )
    l3_linear_speed_gain: float = Field(
        default=1.0,
        gt=0.0,
        le=10.0,
        validation_alias="L3_LINEAR_SPEED_GAIN",
        description="Коэффициент перевода ошибки расстояния в линейную скорость.",
    )
    l3_angular_speed_gain: float = Field(
        default=2.0,
        gt=0.0,
        le=20.0,
        validation_alias="L3_ANGULAR_SPEED_GAIN",
        description="Коэффициент перевода ошибки угла в угловую скорость.",
    )
    l3_max_linear_speed_cm_per_sec: float = Field(
        default=20.0,
        gt=0.0,
        le=100.0,
        validation_alias="L3_MAX_LINEAR_SPEED_CM_PER_SEC",
        description="Максимальная линейная скорость, которую L3 может запросить у L2.",
    )
    l3_max_angular_speed_deg_per_sec: float = Field(
        default=120.0,
        gt=0.0,
        le=720.0,
        validation_alias="L3_MAX_ANGULAR_SPEED_DEG_PER_SEC",
        description="Максимальная угловая скорость, которую L3 может запросить у L2.",
    )
    l3_obstacle_stop_distance_cm: float = Field(
        default=20.0,
        gt=0.0,
        le=200.0,
        validation_alias="L3_OBSTACLE_STOP_DISTANCE_CM",
        description="Расстояние до препятствия, при котором L3 останавливает движение (см).",
    )
    l3_obstacle_slowdown_distance_cm: float = Field(
        default=40.0,
        gt=0.0,
        le=300.0,
        validation_alias="L3_OBSTACLE_SLOWDOWN_DISTANCE_CM",
        description=(
            "Расстояние до препятствия, с которого L3 начинает плавно снижать скорость (см)."
        ),
    )
    l3_planner_obstacle_clearance_cm: float = Field(
        default=5.0,
        gt=0.0,
        le=100.0,
        validation_alias="L3_PLANNER_OBSTACLE_CLEARANCE_CM",
        description="Минимальный зазор планировщика при обходе заранее известных препятствий (см).",
    )
    l3_planner_max_detour_offset_cm: float = Field(
        default=40.0,
        gt=0.0,
        le=300.0,
        validation_alias="L3_PLANNER_MAX_DETOUR_OFFSET_CM",
        description=(
            "Максимальное допустимое боковое отклонение обходного пути от исходной прямой (см)."
        ),
    )
    l3_planner_max_waypoints: int = Field(
        default=24,
        ge=1,
        le=200,
        validation_alias="L3_PLANNER_MAX_WAYPOINTS",
        description=(
            "Максимальное число точек, которые планировщик может вставить в обходной маршрут."
        ),
    )
    l3_unknown_obstacle_radius_cm: float = Field(
        default=8.0,
        gt=0.0,
        le=100.0,
        validation_alias="L3_UNKNOWN_OBSTACLE_RADIUS_CM",
        description="Оценочный радиус препятствия, впервые обнаруженного только датчиком (см).",
    )

    model_config: SettingsConfigDict = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
    )
