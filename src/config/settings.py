from __future__ import annotations

from typing import final

from pydantic import Field, field_validator
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
    cors_origins: str = Field(
        default="*",
        validation_alias="CORS_ORIGINS",
        description="CORS: * или список Origin через запятую (например http://localhost:4200).",
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
    ultrasonic_enabled: bool = Field(
        default=True,
        validation_alias="ULTRASONIC_ENABLED",
        description="Включить HC-SR04. Отключить: ULTRASONIC_ENABLED=false.",
    )
    ultrasonic_trigger_pin: int = Field(
        default=23,
        ge=0,
        le=27,
        validation_alias="ULTRASONIC_TRIGGER_PIN",
        description="BCM-пин TRIG HC-SR04.",
    )
    ultrasonic_echo_pin: int = Field(
        default=24,
        ge=0,
        le=27,
        validation_alias="ULTRASONIC_ECHO_PIN",
        description="BCM-пин ECHO HC-SR04.",
    )
    gyro_yaw_integration_deadband_deg_per_sec: float = Field(
        default=0.2,
        ge=0.0,
        le=2.0,
        validation_alias="GYRO_YAW_INTEGRATION_DEADBAND_DEG_PER_SEC",
        description=(
            "Мёртвая зона |ω_z| (°/с) при интеграции yaw: ниже порога угол не копится — "
            "меньше дрейф стоя; 0 отключает. Акселерометр азимут на горизонтали не задаёт."
        ),
    )
    imu_mpu6050_dlpf_cfg: int = Field(
        default=3,
        ge=0,
        le=7,
        validation_alias="IMU_MPU6050_DLPF_CFG",
        description="MPU6050 DLPF_CFG (режим 3 ≈ 44/42 Гц).",
    )
    imu_mpu6050_smplrt_div: int = Field(
        default=9,
        ge=0,
        le=255,
        validation_alias="IMU_MPU6050_SMPLRT_DIV",
        description="MPU6050 SMPLRT_DIV.",
    )
    imu_accel_ema_alpha: float = Field(
        default=0.2,
        ge=0.01,
        le=1.0,
        validation_alias="IMU_ACCEL_EMA_ALPHA",
        description="EMA сглаживание акселерометра (0..1).",
    )
    imu_gyro_ema_alpha: float = Field(
        default=0.3,
        ge=0.01,
        le=1.0,
        validation_alias="IMU_GYRO_EMA_ALPHA",
        description="EMA сглаживание гироскопа (°/с).",
    )
    imu_ekf_enabled: bool = Field(
        default=True,
        validation_alias="IMU_EKF_ENABLED",
        description="Фильтр yaw + ZUPT (модуль ekf_imu).",
    )
    imu_ekf_q_angle: float = Field(
        default=0.001,
        ge=1e-12,
        validation_alias="IMU_EKF_Q_ANGLE",
        description="EKF: шум процесса угла.",
    )
    imu_ekf_q_bias: float = Field(
        default=0.0001,
        ge=1e-12,
        validation_alias="IMU_EKF_Q_BIAS",
        description="EKF: шум процесса bias.",
    )
    imu_ekf_r_accel: float = Field(
        default=0.5,
        ge=1e-12,
        validation_alias="IMU_EKF_R_ACCEL",
        description="EKF: зарезервированный параметр шума (в update не используется).",
    )
    imu_ekf_accel_gate: float = Field(
        default=0.5,
        ge=0.01,
        le=1.0,
        validation_alias="IMU_EKF_ACCEL_GATE",
        description="EKF: порог отклонения |a| от g для опоры roll/pitch.",
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
    l123_debug_trace_enabled: bool = Field(
        default=False,
        validation_alias="L123_DEBUG_TRACE_ENABLED",
        description="Логировать шаги математики L1->L2->L3.",
    )
    l123_debug_trace_every_n_steps: int = Field(
        default=1,
        ge=1,
        le=1000,
        validation_alias="L123_DEBUG_TRACE_EVERY_N_STEPS",
        description="Писать каждый N-й шаг debug-трейса L1->L2->L3.",
    )
    motion_diag_logging_enabled: bool = Field(
        default=True,
        validation_alias="MOTION_DIAG_LOGGING_ENABLED",
        description=(
            "Диагностика L1 и sync L1→L2 в stderr (DEBUG/WARNING). "
            "Отключить: MOTION_DIAG_LOGGING_ENABLED=false."
        ),
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
    m1_direction: int = Field(
        default=-1,
        validation_alias="M1_DIRECTION",
        description=(
            "Направление канала M1 (PCA9685): 1 или −1. "
            "По умолчанию −1 под типичную сборку RaspTank; "
            "для обратной полярности задайте M1_DIRECTION=1 и M2_DIRECTION=-1."
        ),
    )
    m2_direction: int = Field(
        default=1,
        validation_alias="M2_DIRECTION",
        description="Направление канала M2: 1 или −1 (по умолчанию 1, противоположно M1).",
    )

    @field_validator("m1_direction", "m2_direction", mode="before")
    @classmethod
    def _motor_direction_pm_one(cls, value: object) -> int:
        """Разрешить только ±1 (из .env приходят строки)."""
        as_int: int = int(value)  # type: ignore[arg-type]
        if as_int not in (-1, 1):
            msg: str = "M1_DIRECTION и M2_DIRECTION должны быть 1 или -1"
            raise ValueError(msg)
        return as_int

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
        default=False,
        validation_alias="HEADING_HOLD_INVERT_STEER",
        description=(
            "Инверсия знака руления для удержания курса. "
            "При дефолтных M1_DIRECTION=-1 и M2_DIRECTION=1 оставьте false; "
            "при классической паре 1/-1 часто нужно true — см. доку по моторам."
        ),
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
    l2_accel_speed_fusion_enabled: bool = Field(
        default=True,
        validation_alias="L2_ACCEL_SPEED_FUSION_ENABLED",
        description="Включить мягкую коррекцию скорости L2 по акселерометру.",
    )
    l2_accel_speed_blend_alpha: float = Field(
        default=0.05,
        ge=0.0,
        le=1.0,
        validation_alias="L2_ACCEL_SPEED_BLEND_ALPHA",
        description="Доля accel-скорости в оценке v_hat (0..1).",
    )
    l2_accel_stationary_threshold_m_s2: float = Field(
        default=0.30,
        ge=0.0,
        le=3.0,
        validation_alias="L2_ACCEL_STATIONARY_THRESHOLD_M_S2",
        description="Порог |a_long| (м/с²) для детектора покоя.",
    )
    l2_gyro_stationary_threshold_deg_per_sec: float = Field(
        default=2.5,
        ge=0.0,
        le=30.0,
        validation_alias="L2_GYRO_STATIONARY_THRESHOLD_DEG_PER_SEC",
        description="Порог |w| (°/с) для детектора покоя при bias-обучении.",
    )
    l2_accel_bias_learning_rate: float = Field(
        default=0.03,
        ge=0.0,
        le=1.0,
        validation_alias="L2_ACCEL_BIAS_LEARNING_RATE",
        description="Скорость обучения bias продольного акселя в покое.",
    )
    l2_accel_speed_limit_factor: float = Field(
        default=1.1,
        ge=0.5,
        le=3.0,
        validation_alias="L2_ACCEL_SPEED_LIMIT_FACTOR",
        description="Лимит интегральной accel-скорости относительно max скорости борта.",
    )
    l2_feedback_enabled: bool = Field(
        default=False,
        validation_alias="L2_FEEDBACK_ENABLED",
        description="Включить обратную связь по гироскопу в L2.",
    )
    l2_feedback_k_omega: float = Field(
        default=0.5,
        ge=0.0,
        le=10.0,
        validation_alias="L2_FEEDBACK_K_OMEGA",
        description="Коэффициент P по ошибке угловой скорости (прямолинейное движение).",
    )
    l2_feedback_k_theta: float = Field(
        default=0.3,
        ge=0.0,
        le=10.0,
        validation_alias="L2_FEEDBACK_K_THETA",
        description="Коэффициент P по ошибке угла к начальному курсу.",
    )
    l2_feedback_k_i: float = Field(
        default=0.05,
        ge=0.0,
        le=5.0,
        validation_alias="L2_FEEDBACK_K_I",
        description="Коэффициент I по накопленной ошибке.",
    )
    l2_feedback_i_max: float = Field(
        default=10.0,
        ge=0.0,
        le=100.0,
        validation_alias="L2_FEEDBACK_I_MAX",
        description="Лимит интегратора (антивайндап).",
    )
    l2_feedback_u_max_corr: float = Field(
        default=20.0,
        ge=0.0,
        le=100.0,
        validation_alias="L2_FEEDBACK_U_MAX_CORR",
        description="Максимальная коррекция ΔU для прямолинейного движения (%).",
    )
    l2_feedback_u_trim: float = Field(
        default=0.0,
        ge=-20.0,
        le=20.0,
        validation_alias="L2_FEEDBACK_U_TRIM",
        description="Постоянная поправка асимметрии гусениц (%).",
    )
    l2_feedback_k_omega_turn: float = Field(
        default=0.3,
        ge=0.0,
        le=10.0,
        validation_alias="L2_FEEDBACK_K_OMEGA_TURN",
        description="Коэффициент P по ошибке угловой скорости (поворот на месте).",
    )
    l2_feedback_u_max_turn: float = Field(
        default=15.0,
        ge=0.0,
        le=100.0,
        validation_alias="L2_FEEDBACK_U_MAX_TURN",
        description="Максимальная коррекция ΔU для поворота на месте (%).",
    )
    l2_state_space_enabled: bool = Field(
        default=False,
        validation_alias="L2_STATE_SPACE_ENABLED",
        description="Включить новый алгоритм МПС (LQR) в контуре L2.",
    )
    l2_state_space_t_v: float = Field(
        default=0.8,
        gt=0.0,
        le=5.0,
        validation_alias="L2_STATE_SPACE_T_V",
        description="Постоянная времени T_v для МПС.",
    )
    l2_state_space_t_w: float = Field(
        default=0.55,
        gt=0.0,
        le=5.0,
        validation_alias="L2_STATE_SPACE_T_W",
        description="Постоянная времени T_w для МПС.",
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
        populate_by_name=True,
    )
