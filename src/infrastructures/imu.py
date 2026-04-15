from __future__ import annotations

import logging
import time
from threading import Event, Lock, Thread
from typing import Any, final

from src.application.protocols import GyroscopeProtocol

try:
    import smbus2

    _HARDWARE_AVAILABLE: bool = True
except ImportError:
    smbus2: Any = None
    _HARDWARE_AVAILABLE: bool = False

logger: logging.Logger = logging.getLogger(__name__)


@final
class IMUSensor(GyroscopeProtocol):
    """Драйвер MPU6050 для отслеживания угла поворота."""

    # Настройки подключения
    I2C_BUS: int = 1
    I2C_ADDRESS: int = 0x68

    # Коэффициент инверсии для корректного угла при перевернутой установке платы.
    GYRO_SIGN_Z: float = -1.0

    # Регистры MPU6050
    REG_PWR_MGMT_1: int = 0x6B
    REG_ACCEL_XOUT_H: int = 0x3B
    REG_GYRO_ZOUT_H: int = 0x47

    # Технические коэффициенты
    GYRO_SCALE_FACTOR: float = 131.0
    ACCEL_SCALE_FACTOR: float = 16384.0
    STANDARD_GRAVITY_M_S2: float = 9.80665
    INT16_LIMIT: int = 32767
    INT16_OFFSET: int = 65536

    # Кол-во байт для чтения одного значения
    BYTES_PER_VALUE: int = 2
    ACCEL_BYTES_COUNT: int = 6

    # Сдвиг для склеивания байтов
    BYTE_SHIFT: int = 8

    # Параметры калибровки
    CALIBRATION_SAMPLES: int = 100
    CALIBRATION_SLEEP_SEC: float = 0.01

    # Параметры навигации
    UPDATE_LOOP_SLEEP_SEC: float = 0.01

    # Ориентация платы по осям акселерометра
    ACCEL_SIGN_X: float = 1.0
    ACCEL_SIGN_Y: float = 1.0
    ACCEL_SIGN_Z: float = 1.0

    # Пауза после пробуждения чипа (с)
    WAKEUP_DELAY_SEC: float = 0.1

    # Таймаут завершения потока (с)
    THREAD_JOIN_TIMEOUT_SEC: float = 1.0

    def __init__(self, bus_num: int = I2C_BUS) -> None:
        """Инициализация сенсора."""
        self.bus_num: int = bus_num
        self._bus: Any = None
        self._is_initialized: bool = False

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
        """Запуск отслеживания угла."""
        if not _HARDWARE_AVAILABLE:
            return

        self._setup()
        if not self._is_initialized:
            return

        self.stop()
        if calibrate:
            self.calibrate()

        with self._state_lock:
            self._yaw: float = 0.0
            self._gyro_z_deg_per_sec = 0.0
            self._accel_x_m_s2 = 0.0
            self._accel_y_m_s2 = 0.0
            self._accel_z_m_s2 = 0.0
            self._last_update_time: float = time.monotonic()

        self._stop_event.clear()
        self._update_thread: Thread = Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()

        logger.info("IMU: Навигация запущена (0° = текущее положение)")

    def stop(self) -> None:
        """Остановка фонового процесса отслеживания."""
        self._stop_event.set()

        if self._update_thread:
            self._update_thread.join(timeout=self.THREAD_JOIN_TIMEOUT_SEC)

        self._update_thread: Thread | None = None
        self._stop_event.clear()

    def calibrate(self, samples: int | None = None) -> None:
        """Калибровка для определения статического шума чипа."""
        if not self._is_initialized:
            return

        num_samples: int = samples or self.CALIBRATION_SAMPLES
        logger.info("IMU: Калибровка нуля... Не двигайте робота!")

        sums: float = 0.0
        for _ in range(num_samples):
            sums += self._read_raw_gyro_z()
            time.sleep(self.CALIBRATION_SLEEP_SEC)

        self._gyro_z_bias: float = sums / num_samples
        logger.info("IMU: Ноль откалиброван. Bias: %.4f", self._gyro_z_bias)

    def get_yaw(self) -> float:
        """Возврат текущего угла поворота."""
        with self._state_lock:
            return self._yaw

    def get_angular_speed_z_deg_per_sec(self) -> float:
        """Возврат текущей угловой скорости вокруг вертикальной оси."""
        with self._state_lock:
            return self._gyro_z_deg_per_sec

    def get_acceleration_xyz_m_s2(self) -> tuple[float, float, float]:
        """Возврат текущих ускорений по трём осям."""
        with self._state_lock:
            return (
                self._accel_x_m_s2,
                self._accel_y_m_s2,
                self._accel_z_m_s2,
            )

    def reset_yaw(self) -> None:
        """Сброс текущего значения угла в ноль."""
        with self._state_lock:
            self._yaw = 0.0

    def destroy(self) -> None:
        """Освобождение ресурсов."""
        self.stop()

        if self._bus:
            try:
                self._bus.close()
            except Exception as exc:
                logger.warning("IMU: Ошибка закрытия I2C: %s", exc)
            finally:
                self._bus: Any = None

        self._is_initialized: bool = False

    def _setup(self) -> None:
        """Инициализация шины I2C и пробуждение чипа MPU6050."""
        if not _HARDWARE_AVAILABLE or self._is_initialized:
            return

        try:
            self._bus: smbus2.SMBus = smbus2.SMBus(self.bus_num)
            self._bus.write_byte_data(self.I2C_ADDRESS, self.REG_PWR_MGMT_1, 0)
            time.sleep(self.WAKEUP_DELAY_SEC)
            self._is_initialized: bool = True

        except Exception as exc:
            logger.error("IMU: Ошибка I2C: %s", exc)
            self._is_initialized: bool = False

    def _read_raw_gyro_z(self) -> float:
        """Чтение сырых 16-битных данных угловой скорости по оси Z."""
        if not self._is_initialized:
            return 0.0

        try:
            data: Any = self._bus.read_i2c_block_data(
                self.I2C_ADDRESS,
                self.REG_GYRO_ZOUT_H,
                self.BYTES_PER_VALUE,
            )
            raw_val: int = (data[0] << self.BYTE_SHIFT) | data[1]

            if raw_val > self.INT16_LIMIT:
                raw_val -= self.INT16_OFFSET

            return float(raw_val)

        except Exception as exc:
            logger.debug("IMU: Ошибка чтения gyro_z: %s", exc)
            return 0.0

    def _read_raw_accel_xyz(self) -> tuple[float, float, float]:
        """Чтение сырых данных акселерометра по трём осям."""
        if not self._is_initialized:
            return 0.0, 0.0, 0.0

        try:
            data: Any = self._bus.read_i2c_block_data(
                self.I2C_ADDRESS,
                self.REG_ACCEL_XOUT_H,
                self.ACCEL_BYTES_COUNT,
            )
        except Exception as exc:
            logger.debug("IMU: Ошибка чтения accelerometer: %s", exc)
            return 0.0, 0.0, 0.0

        accel_x: float = self._decode_int16(data[0], data[1])
        accel_y: float = self._decode_int16(data[2], data[3])
        accel_z: float = self._decode_int16(data[4], data[5])

        return accel_x, accel_y, accel_z

    def _decode_int16(self, high_byte: int, low_byte: int) -> float:
        """Склеить два байта в signed int16."""
        raw_val: int = (high_byte << self.BYTE_SHIFT) | low_byte
        if raw_val > self.INT16_LIMIT:
            raw_val -= self.INT16_OFFSET
        return float(raw_val)

    def _update_loop(self) -> None:
        """Фоновый цикл интеграции угловой скорости."""
        while not self._stop_event.is_set():
            now: float = time.monotonic()

            with self._state_lock:
                dt: float = now - self._last_update_time
                self._last_update_time = now

            raw_gz: float = self._read_raw_gyro_z()
            raw_ax, raw_ay, raw_az = self._read_raw_accel_xyz()
            actual_gz: float = (raw_gz - self._gyro_z_bias) / self.GYRO_SCALE_FACTOR
            yaw_delta: float = (self.GYRO_SIGN_Z * actual_gz) * dt

            accel_x_m_s2: float = (
                self.ACCEL_SIGN_X * (raw_ax / self.ACCEL_SCALE_FACTOR) * self.STANDARD_GRAVITY_M_S2
            )

            accel_y_m_s2: float = (
                self.ACCEL_SIGN_Y * (raw_ay / self.ACCEL_SCALE_FACTOR) * self.STANDARD_GRAVITY_M_S2
            )

            accel_z_m_s2: float = (
                self.ACCEL_SIGN_Z * (raw_az / self.ACCEL_SCALE_FACTOR) * self.STANDARD_GRAVITY_M_S2
            )

            with self._state_lock:
                self._yaw += yaw_delta
                self._gyro_z_deg_per_sec = self.GYRO_SIGN_Z * actual_gz
                self._accel_x_m_s2 = accel_x_m_s2
                self._accel_y_m_s2 = accel_y_m_s2
                self._accel_z_m_s2 = accel_z_m_s2

            time.sleep(self.UPDATE_LOOP_SLEEP_SEC)
