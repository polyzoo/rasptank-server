from __future__ import annotations

import logging
import math
import time
from threading import Event, Lock, Thread
from typing import Any, final

from src.application.protocols import GyroscopeProtocol
from src.infrastructures.ekf_imu import EkfImu

try:
    import smbus2

    _HARDWARE_AVAILABLE: bool = True
except ImportError:
    smbus2: Any = None
    _HARDWARE_AVAILABLE: bool = False

logger: logging.Logger = logging.getLogger(__name__)


@final
class IMUSensor(GyroscopeProtocol):
    """Драйвер MPU6050: DLPF + пакетное чтение + EMA + опционально EKF/ZUPT."""

    I2C_BUS: int = 1
    I2C_ADDRESS: int = 0x68

    GYRO_SIGN_Z: float = -1.0

    REG_PWR_MGMT_1: int = 0x6B
    REG_ACCEL_XOUT_H: int = 0x3B
    REG_GYRO_ZOUT_H: int = 0x47
    REG_DLPF_CFG: int = 0x1A
    REG_SMPLRT_DIV: int = 0x19
    REG_GYRO_CONFIG: int = 0x1B
    REG_ACCEL_CONFIG: int = 0x1C

    GYRO_SCALE_FACTOR: float = 131.0
    ACCEL_SCALE_FACTOR: float = 16384.0
    STANDARD_GRAVITY_M_S2: float = 9.80665
    INT16_LIMIT: int = 32767
    INT16_OFFSET: int = 65536

    BYTES_PER_VALUE: int = 2
    ACCEL_BYTES_COUNT: int = 6
    MPU6050_BURST_BYTES: int = 14

    BYTE_SHIFT: int = 8

    CALIBRATION_SAMPLES: int = 100
    CALIBRATION_SLEEP_SEC: float = 0.01

    UPDATE_LOOP_SLEEP_SEC: float = 0.01

    ACCEL_SIGN_X: float = 1.0
    ACCEL_SIGN_Y: float = 1.0
    ACCEL_SIGN_Z: float = 1.0

    WAKEUP_DELAY_SEC: float = 0.1

    THREAD_JOIN_TIMEOUT_SEC: float = 1.0

    def __init__(
        self,
        bus_num: int = I2C_BUS,
        *,
        gyro_yaw_integration_deadband_deg_per_sec: float = 0.0,
        mpu6050_dlpf_cfg: int = 3,
        mpu6050_smplrt_div: int = 9,
        accel_ema_alpha: float = 0.2,
        gyro_ema_alpha: float = 0.3,
        ekf_enabled: bool = False,
        ekf_q_angle: float = 0.001,
        ekf_q_bias: float = 0.0001,
        ekf_r_accel: float = 0.5,
        ekf_accel_gate: float = 0.5,
    ) -> None:
        """Собрать сенсор с ограничением параметров фильтров допустимыми диапазонами.

        Args:
            bus_num: Номер I²C-шины Linux (обычно 1).
            gyro_yaw_integration_deadband_deg_per_sec: Мёртвая зона |ω_z| (°/с) без EKF.
            mpu6050_dlpf_cfg: Регистр DLPF_CFG MPU6050 (0…7).
            mpu6050_smplrt_div: Делитель частоты выборки SMPLRT_DIV (0…255).
            accel_ema_alpha: Коэффициент EMA для ускорений (0.01…1).
            gyro_ema_alpha: Коэффициент EMA для угловых скоростей (0.01…1).
            ekf_enabled: Включить :class:`EkfImu` для yaw и ZUPT.
            ekf_q_angle: Параметр шума процесса угла EKF.
            ekf_q_bias: Параметр шума процесса bias EKF.
            ekf_r_accel: Резерв совместимости (передаётся в EKF).
            ekf_accel_gate: Порог отклонения ‖a‖ от g для обновления roll/pitch в EKF.
        """
        self.bus_num: int = bus_num
        self._bus: Any = None
        self._is_initialized: bool = False
        self._gyro_yaw_integration_deadband_deg_per_sec: float = max(
            0.0,
            gyro_yaw_integration_deadband_deg_per_sec,
        )

        self._mpu6050_dlpf_cfg: int = max(0, min(7, mpu6050_dlpf_cfg))
        self._mpu6050_smplrt_div: int = max(0, min(255, mpu6050_smplrt_div))
        self._accel_ema_alpha: float = max(0.01, min(1.0, accel_ema_alpha))
        self._gyro_ema_alpha: float = max(0.01, min(1.0, gyro_ema_alpha))

        self._ema_ax: float | None = None
        self._ema_ay: float | None = None
        self._ema_az: float | None = None
        self._ema_gx_deg: float | None = None
        self._ema_gy_deg: float | None = None
        self._ema_gz_deg: float | None = None

        self._ekf_enabled: bool = ekf_enabled
        self._ekf: EkfImu | None
        if self._ekf_enabled:
            self._ekf = EkfImu(
                q_angle=ekf_q_angle,
                q_bias=ekf_q_bias,
                r_accel=ekf_r_accel,
                accel_gate=ekf_accel_gate,
                g=self.STANDARD_GRAVITY_M_S2,
            )
        else:
            self._ekf = None

        self._gyro_bias_raw_x: float = 0.0
        self._gyro_bias_raw_y: float = 0.0
        self._gyro_bias_raw_z: float = 0.0
        self._home_roll_rad: float = 0.0
        self._home_pitch_rad: float = 0.0

        self._yaw: float = 0.0
        self._gyro_z_bias: float = 0.0
        self._gyro_z_deg_per_sec: float = 0.0
        self._accel_x_m_s2: float = 0.0
        self._accel_y_m_s2: float = 0.0
        self._accel_z_m_s2: float = 0.0
        self._state_lock: Lock = Lock()

        self._stop_event: Event = Event()
        self._update_thread: Thread | None = None
        self._last_update_time: float = 0.0

    def start(self, calibrate: bool = True) -> None:
        """Открыть шину, при необходимости откалибровать и запустить фоновый цикл опроса.

        Args:
            calibrate: Выполнить :meth:`calibrate` перед навигацией (не вызывается повторно,
                если поток уже жив).
        """
        if not _HARDWARE_AVAILABLE:
            return

        self._setup()
        if not self._is_initialized:
            return

        if self._update_thread is not None and self._update_thread.is_alive():
            if calibrate:
                logger.info(
                    "IMU: поток уже работает — пропускаем повторный start() "
                    "(калибровка не выполняется, чтобы не ломать общий с L1-L3 сенсор)"
                )
            return

        self.stop()
        self._reset_ema_state()
        if self._ekf is not None:
            self._ekf.reset()

        if calibrate:
            self.calibrate()

        with self._state_lock:
            self._yaw = 0.0
            self._gyro_z_deg_per_sec = 0.0
            self._accel_x_m_s2 = 0.0
            self._accel_y_m_s2 = 0.0
            self._accel_z_m_s2 = 0.0
            self._last_update_time = time.monotonic()

        self._stop_event.clear()
        self._update_thread = Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()

        logger.info("IMU: Навигация запущена (0° = текущее положение)")

    def stop(self) -> None:
        """Запросить остановку потока опроса и дождаться его завершения."""
        self._stop_event.set()

        if self._update_thread:
            self._update_thread.join(timeout=self.THREAD_JOIN_TIMEOUT_SEC)

        self._update_thread = None
        self._stop_event.clear()

    def _reset_ema_state(self) -> None:
        """Сбросить накопленные значения EMA (при перезапуске навигации)."""
        self._ema_ax = None
        self._ema_ay = None
        self._ema_az = None
        self._ema_gx_deg = None
        self._ema_gy_deg = None
        self._ema_gz_deg = None

    def calibrate(self, samples: int | None = None) -> None:
        """Усреднить сырые показания в покое: bias гироскопа и «домашние» roll/pitch.

        Args:
            samples: Число пакетных чтений; по умолчанию :attr:`CALIBRATION_SAMPLES`.
        """
        if not self._is_initialized:
            return

        num_samples: int = samples or self.CALIBRATION_SAMPLES
        logger.info("IMU: Калибровка нуля... Не двигайте робота!")

        sum_ax = sum_ay = sum_az = 0.0
        sum_gx = sum_gy = sum_gz = 0.0
        for _ in range(num_samples):
            ax_r, ay_r, az_r, gx_r, gy_r, gz_r = self._read_burst_raw_int16()
            sum_ax += ax_r
            sum_ay += ay_r
            sum_az += az_r
            sum_gx += gx_r
            sum_gy += gy_r
            sum_gz += gz_r
            time.sleep(self.CALIBRATION_SLEEP_SEC)

        n = float(num_samples)
        self._gyro_bias_raw_x = sum_gx / n
        self._gyro_bias_raw_y = sum_gy / n
        self._gyro_bias_raw_z = sum_gz / n
        self._gyro_z_bias = self._gyro_bias_raw_z

        avg_ax_m2 = (
            self.ACCEL_SIGN_X * (sum_ax / n) / self.ACCEL_SCALE_FACTOR * self.STANDARD_GRAVITY_M_S2
        )
        avg_ay_m2 = (
            self.ACCEL_SIGN_Y * (sum_ay / n) / self.ACCEL_SCALE_FACTOR * self.STANDARD_GRAVITY_M_S2
        )
        avg_az_m2 = (
            self.ACCEL_SIGN_Z * (sum_az / n) / self.ACCEL_SCALE_FACTOR * self.STANDARD_GRAVITY_M_S2
        )

        self._home_roll_rad = math.atan2(avg_ay_m2, avg_az_m2)
        self._home_pitch_rad = math.atan2(
            -avg_ax_m2,
            math.sqrt(avg_ay_m2 * avg_ay_m2 + avg_az_m2 * avg_az_m2),
        )

        if self._ekf is not None:
            self._ekf.init_from_calibration(
                self._home_roll_rad,
                self._home_pitch_rad,
                0.0,
                (0.0, 0.0, 0.0),
            )

        logger.info(
            "IMU: Калибровка: bias raw gyro xyz=(%.2f, %.2f, %.2f), "
            "home roll/pitch=(%.4f, %.4f) rad",
            self._gyro_bias_raw_x,
            self._gyro_bias_raw_y,
            self._gyro_bias_raw_z,
            self._home_roll_rad,
            self._home_pitch_rad,
        )

    def get_yaw(self) -> float:
        """Текущий yaw в градусах (интеграл или оценка EKF)."""
        with self._state_lock:
            return self._yaw

    def get_angular_speed_z_deg_per_sec(self) -> float:
        """Сглаженная ω_z в °/с; без EKF ниже deadband наружу отдаётся 0."""
        with self._state_lock:
            return self._gyro_z_deg_per_sec

    def get_acceleration_xyz_m_s2(self) -> tuple[float, float, float]:
        """Ускорения по осям в м/с² после EMA (в потоке совпадают с входом EKF)."""
        with self._state_lock:
            return (
                self._accel_x_m_s2,
                self._accel_y_m_s2,
                self._accel_z_m_s2,
            )

    def reset_yaw(self) -> None:
        """Обнулить yaw и переинициализировать EKF текущими home roll/pitch."""
        with self._state_lock:
            self._yaw = 0.0
            if self._ekf is not None:
                self._ekf.init_from_calibration(
                    self._home_roll_rad,
                    self._home_pitch_rad,
                    0.0,
                    (0.0, 0.0, 0.0),
                )

    def destroy(self) -> None:
        """Остановить поток и закрыть I²C-шину."""
        self.stop()

        if self._bus:
            try:
                self._bus.close()
            except Exception as exc:
                logger.warning("IMU: Ошибка закрытия I2C: %s", exc)
            finally:
                self._bus = None

        self._is_initialized = False

    def _setup(self) -> None:
        """Пробудить MPU6050 и записать DLPF, делитель частоты и диапазоны гиро/акселя."""
        if not _HARDWARE_AVAILABLE or self._is_initialized:
            return

        try:
            self._bus = smbus2.SMBus(self.bus_num)
            self._bus.write_byte_data(self.I2C_ADDRESS, self.REG_PWR_MGMT_1, 0)
            time.sleep(self.WAKEUP_DELAY_SEC)
            self._bus.write_byte_data(self.I2C_ADDRESS, self.REG_DLPF_CFG, self._mpu6050_dlpf_cfg)
            self._bus.write_byte_data(
                self.I2C_ADDRESS,
                self.REG_SMPLRT_DIV,
                self._mpu6050_smplrt_div,
            )
            self._bus.write_byte_data(self.I2C_ADDRESS, self.REG_GYRO_CONFIG, 0x00)
            self._bus.write_byte_data(self.I2C_ADDRESS, self.REG_ACCEL_CONFIG, 0x00)
            self._is_initialized = True
            logger.info(
                "IMU: MPU6050 DLPF_CFG=%d SMPLRT_DIV=%d",
                self._mpu6050_dlpf_cfg,
                self._mpu6050_smplrt_div,
            )

        except Exception as exc:
            logger.error("IMU: Ошибка I2C: %s", exc)
            self._is_initialized = False

    def _read_burst_raw_int16(self) -> tuple[float, float, float, float, float, float]:
        """Прочитать 14 байт с ``REG_ACCEL_XOUT_H``: accel, temp, gyro (сырые int16)."""
        if not self._is_initialized:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        try:
            data: Any = self._bus.read_i2c_block_data(
                self.I2C_ADDRESS,
                self.REG_ACCEL_XOUT_H,
                self.MPU6050_BURST_BYTES,
            )
        except Exception as exc:
            logger.debug("IMU: Ошибка пакетного чтения: %s", exc)
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        ax = self._decode_int16(data[0], data[1])
        ay = self._decode_int16(data[2], data[3])
        az = self._decode_int16(data[4], data[5])
        gx = self._decode_int16(data[8], data[9])
        gy = self._decode_int16(data[10], data[11])
        gz = self._decode_int16(data[12], data[13])
        return ax, ay, az, gx, gy, gz

    def _read_raw_gyro_z(self) -> float:
        """Сырое значение гироскопа Z из пакетного чтения."""
        return self._read_burst_raw_int16()[5]

    def _read_raw_accel_xyz(self) -> tuple[float, float, float]:
        """Сырые значения акселерометра из того же пакета, что и гироскоп."""
        ax, ay, az, _, _, _ = self._read_burst_raw_int16()
        return ax, ay, az

    def _decode_int16(self, high_byte: int, low_byte: int) -> float:
        """Собрать signed int16 из старшего и младшего байта регистров MPU6050."""
        raw_val: int = (high_byte << self.BYTE_SHIFT) | low_byte
        if raw_val > self.INT16_LIMIT:
            raw_val -= self.INT16_OFFSET
        return float(raw_val)

    def _apply_ema(
        self,
        ax_m2: float,
        ay_m2: float,
        az_m2: float,
        gx_deg: float,
        gy_deg: float,
        gz_deg: float,
    ) -> tuple[float, float, float, float, float, float]:
        """Экспоненциальное сглаживание ускорений (м/с²) и угловых скоростей (°/с)."""
        if self._ema_ax is None:
            self._ema_ax = ax_m2
            self._ema_ay = ay_m2
            self._ema_az = az_m2
            self._ema_gx_deg = gx_deg
            self._ema_gy_deg = gy_deg
            self._ema_gz_deg = gz_deg
            return ax_m2, ay_m2, az_m2, gx_deg, gy_deg, gz_deg

        a = self._accel_ema_alpha
        a1 = 1.0 - a
        g = self._gyro_ema_alpha
        g1 = 1.0 - g

        self._ema_ax = a * ax_m2 + a1 * self._ema_ax
        self._ema_ay = a * ay_m2 + a1 * self._ema_ay
        self._ema_az = a * az_m2 + a1 * self._ema_az
        self._ema_gx_deg = g * gx_deg + g1 * self._ema_gx_deg
        self._ema_gy_deg = g * gy_deg + g1 * self._ema_gy_deg
        self._ema_gz_deg = g * gz_deg + g1 * self._ema_gz_deg

        return (
            self._ema_ax,
            self._ema_ay,
            self._ema_az,
            self._ema_gx_deg,
            self._ema_gy_deg,
            self._ema_gz_deg,
        )

    def _update_loop(self) -> None:
        """Фоновый цикл: пакетное чтение → масштаб → EMA → EKF или интеграл yaw."""
        while not self._stop_event.is_set():
            now: float = time.monotonic()

            with self._state_lock:
                dt: float = now - self._last_update_time
                self._last_update_time = now

            raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz = self._read_burst_raw_int16()

            ax_m2 = (
                self.ACCEL_SIGN_X * (raw_ax / self.ACCEL_SCALE_FACTOR) * self.STANDARD_GRAVITY_M_S2
            )
            ay_m2 = (
                self.ACCEL_SIGN_Y * (raw_ay / self.ACCEL_SCALE_FACTOR) * self.STANDARD_GRAVITY_M_S2
            )
            az_m2 = (
                self.ACCEL_SIGN_Z * (raw_az / self.ACCEL_SCALE_FACTOR) * self.STANDARD_GRAVITY_M_S2
            )

            gx_deg = self.GYRO_SIGN_Z * (raw_gx - self._gyro_bias_raw_x) / self.GYRO_SCALE_FACTOR
            gy_deg = self.GYRO_SIGN_Z * (raw_gy - self._gyro_bias_raw_y) / self.GYRO_SCALE_FACTOR
            gz_deg = self.GYRO_SIGN_Z * (raw_gz - self._gyro_bias_raw_z) / self.GYRO_SCALE_FACTOR

            ax_e, ay_e, az_e, gx_e, gy_e, gz_e = self._apply_ema(
                ax_m2,
                ay_m2,
                az_m2,
                gx_deg,
                gy_deg,
                gz_deg,
            )

            yaw_delta: float = 0.0
            reported_gz_deg_s: float = gz_e
            if self._ekf is not None:
                if dt > 0.0:
                    self._ekf.predict(
                        math.radians(gx_e),
                        math.radians(gy_e),
                        math.radians(gz_e),
                        dt,
                    )
                self._ekf.update(ax_e, ay_e, az_e)
            else:
                angular_rate_z_deg_s: float = gz_e
                deadband: float = self._gyro_yaw_integration_deadband_deg_per_sec
                if deadband > 0.0 and abs(angular_rate_z_deg_s) < deadband:
                    rate_for_yaw = 0.0
                else:
                    rate_for_yaw = angular_rate_z_deg_s
                yaw_delta = rate_for_yaw * dt
                reported_gz_deg_s = rate_for_yaw

            with self._state_lock:
                if self._ekf is not None:
                    self._yaw = math.degrees(self._ekf.yaw)
                else:
                    self._yaw += yaw_delta
                self._gyro_z_deg_per_sec = reported_gz_deg_s
                self._accel_x_m_s2 = ax_e
                self._accel_y_m_s2 = ay_e
                self._accel_z_m_s2 = az_e

            time.sleep(self.UPDATE_LOOP_SLEEP_SEC)
