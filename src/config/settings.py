from __future__ import annotations

from typing import final

from pydantic import Field, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


@final
class Settings(BaseSettings):
    """Настройки окружения приложения."""

    app_host: str = Field(
        default="0.0.0.0",
        validation_alias="APP_HOST",
        description="Адрес HTTP-сервера.",
    )
    app_port: int = Field(
        default=8010,
        validation_alias="APP_PORT",
        description="Порт HTTP-сервера.",
    )
    cors_origins: str = Field(
        default="*",
        validation_alias="CORS_ORIGINS",
        description="Разрешённые CORS origins через запятую.",
    )

    head_servo_channel: int = Field(
        default=4,
        ge=0,
        le=15,
        validation_alias="HEAD_SERVO_CHANNEL",
        description="PCA9685-канал сервопривода головы.",
    )
    head_servo_home_angle_deg: float = Field(
        default=90.0,
        ge=0.0,
        le=180.0,
        validation_alias="HEAD_SERVO_HOME_ANGLE_DEG",
        description="Домашний угол головы в градусах.",
    )
    ultrasonic_enabled: bool = Field(
        default=True,
        validation_alias="ULTRASONIC_ENABLED",
        description="Флаг включения ультразвукового датчика.",
    )
    ultrasonic_trigger_pin: int = Field(
        default=23,
        ge=0,
        le=27,
        validation_alias="ULTRASONIC_TRIGGER_PIN",
        description="BCM-пин TRIG ультразвукового датчика.",
    )
    ultrasonic_echo_pin: int = Field(
        default=24,
        ge=0,
        le=27,
        validation_alias="ULTRASONIC_ECHO_PIN",
        description="BCM-пин ECHO ультразвукового датчика.",
    )
    gyro_yaw_integration_deadband_deg_per_sec: float = Field(
        default=0.2,
        ge=0.0,
        le=2.0,
        validation_alias="GYRO_YAW_INTEGRATION_DEADBAND_DEG_PER_SEC",
        description="Порог интеграции yaw по гироскопу.",
    )
    imu_mpu6050_dlpf_cfg: int = Field(
        default=3,
        ge=0,
        le=7,
        validation_alias="IMU_MPU6050_DLPF_CFG",
        description="Параметр DLPF_CFG для MPU6050.",
    )
    imu_mpu6050_smplrt_div: int = Field(
        default=9,
        ge=0,
        le=255,
        validation_alias="IMU_MPU6050_SMPLRT_DIV",
        description="Параметр SMPLRT_DIV для MPU6050.",
    )
    imu_accel_ema_alpha: float = Field(
        default=0.2,
        ge=0.01,
        le=1.0,
        validation_alias="IMU_ACCEL_EMA_ALPHA",
        description="Коэффициент EMA для акселерометра.",
    )
    imu_gyro_ema_alpha: float = Field(
        default=0.3,
        ge=0.01,
        le=1.0,
        validation_alias="IMU_GYRO_EMA_ALPHA",
        description="Коэффициент EMA для гироскопа.",
    )
    imu_ekf_enabled: bool = Field(
        default=True,
        validation_alias="IMU_EKF_ENABLED",
        description="Флаг включения EKF для IMU.",
    )
    imu_ekf_q_angle: float = Field(
        default=0.001,
        ge=1e-12,
        validation_alias="IMU_EKF_Q_ANGLE",
        description="Шум процесса угла EKF.",
    )
    imu_ekf_q_bias: float = Field(
        default=0.0001,
        ge=1e-12,
        validation_alias="IMU_EKF_Q_BIAS",
        description="Шум процесса смещения EKF.",
    )
    imu_ekf_r_accel: float = Field(
        default=0.5,
        ge=1e-12,
        validation_alias="IMU_EKF_R_ACCEL",
        description="Шум измерения акселерометра EKF.",
    )
    imu_ekf_accel_gate: float = Field(
        default=0.5,
        ge=0.01,
        le=1.0,
        validation_alias="IMU_EKF_ACCEL_GATE",
        description="Порог доверия акселерометру для EKF.",
    )
    update_interval_sec: float = Field(
        default=0.2,
        gt=0,
        le=1.0,
        validation_alias="UPDATE_INTERVAL_SEC",
        description="Период фонового обновления L1-L3.",
    )
    m1_direction: int = Field(
        default=1,
        validation_alias="M1_DIRECTION",
        description="Направление канала M1.",
    )
    m2_direction: int = Field(
        default=-1,
        validation_alias="M2_DIRECTION",
        description="Направление канала M2.",
    )
    track_width_cm: float = Field(
        default=17.0,
        gt=0.0,
        le=100.0,
        validation_alias="TRACK_WIDTH_CM",
        description="Ширина базы между бортами.",
    )
    left_track_max_speed_cm_per_sec: float = Field(
        default=30.0,
        gt=0.0,
        le=100.0,
        validation_alias="LEFT_TRACK_MAX_SPEED_CM_PER_SEC",
        description="Максимальная скорость левого борта.",
    )
    right_track_max_speed_cm_per_sec: float = Field(
        default=30.0,
        gt=0.0,
        le=100.0,
        validation_alias="RIGHT_TRACK_MAX_SPEED_CM_PER_SEC",
        description="Максимальная скорость правого борта.",
    )

    l123_debug_trace_enabled: bool = Field(
        default=False,
        validation_alias="L123_DEBUG_TRACE_ENABLED",
        description="Флаг debug-трейса L1-L3.",
    )
    l123_debug_trace_every_n_steps: int = Field(
        default=1,
        ge=1,
        le=1000,
        validation_alias="L123_DEBUG_TRACE_EVERY_N_STEPS",
        description="Периодичность debug-трейса L1-L3.",
    )
    motion_diag_logging_enabled: bool = Field(
        default=True,
        validation_alias="MOTION_DIAG_LOGGING_ENABLED",
        description="Флаг диагностических логов движения.",
    )

    l2_feedback_enabled: bool = Field(
        default=False,
        validation_alias="L2_FEEDBACK_ENABLED",
        description="Флаг обратной связи L2 по гироскопу.",
    )
    l2_state_space_enabled: bool = Field(
        default=True,
        validation_alias="L2_STATE_SPACE_ENABLED",
        description="Флаг МПС-контроллера L2.",
    )
    l2_state_space_t_v: float = Field(
        default=0.8,
        gt=0.0,
        le=5.0,
        validation_alias="L2_STATE_SPACE_T_V",
        description="Постоянная времени линейной скорости МПС.",
    )
    l2_state_space_t_w: float = Field(
        default=0.55,
        gt=0.0,
        le=5.0,
        validation_alias="L2_STATE_SPACE_T_W",
        description="Постоянная времени угловой скорости МПС.",
    )

    model_config: SettingsConfigDict = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
        populate_by_name=True,
    )

    @field_validator("m1_direction", "m2_direction", mode="before")
    @classmethod
    def _motor_direction_pm_one(cls, value: object) -> int:
        """Проверить направление мотора."""
        as_int: int = int(value)  # type: ignore[arg-type]
        if as_int not in (-1, 1):
            msg: str = "M1_DIRECTION и M2_DIRECTION должны быть 1 или -1"
            raise ValueError(msg)
        return as_int
