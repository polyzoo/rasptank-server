from __future__ import annotations

import logging
import time
from threading import Event, Thread
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
    """Драйвер MPU6050 для отслеживания угла поворота (Yaw).

    Оптимизирован для перевернутой платы и высокой точности при резких движениях.
    """

    # Настройки подключения
    I2C_BUS: int = 1
    I2C_ADDRESS: int = 0x68

    # Коэффициент инверсии для корректного угла при перевернутой установке платы.
    GYRO_SIGN_Z: float = -1.0

    # Регистры MPU6050
    REG_PWR_MGMT_1: int = 0x6B
    REG_GYRO_ZOUT_H: int = 0x47

    # Технические коэффициенты
    GYRO_SCALE_FACTOR: float = 131.0
    INT16_LIMIT: int = 32767
    INT16_OFFSET: int = 65536

    # Кол-во байт для чтения одного значения
    BYTES_PER_VALUE: int = 2

    # Сдвиг для склеивания байтов
    BYTE_SHIFT: int = 8

    # Параметры калибровки
    CALIBRATION_SAMPLES: int = 100
    CALIBRATION_SLEEP_SEC: float = 0.01

    # Параметры навигации
    UPDATE_LOOP_SLEEP_SEC: float = 0.01

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

        self._stop_event: Event = Event()
        self._update_thread: Thread | None = None
        self._last_update_time: float = 0.0

    def start(self, calibrate: bool = True) -> None:
        """Запуск отслеживания угла."""
        if not _HARDWARE_AVAILABLE:
            logger.warning(
                "IMU: smbus2 недоступен — yaw не считается, повороты по углу не сработают",
            )
            return

        self._setup()
        if not self._is_initialized:
            logger.warning(
                "IMU: нет связи с MPU6050 по I2C — yaw не считается, повороты по углу не сработают",
            )
            return

        if calibrate:
            self.calibrate()

        self.reset_yaw()
        self._last_update_time: float = time.monotonic()
        self._stop_event.clear()

        self._update_thread = Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()
        logger.info("IMU: Навигация запущена (0° = текущее положение)")

    def stop(self) -> None:
        """Остановка фонового процесса отслеживания."""
        self._stop_event.set()

        if self._update_thread:
            self._update_thread.join(timeout=self.THREAD_JOIN_TIMEOUT_SEC)

        self._update_thread: Thread | None = None

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

        self._gyro_z_bias: int = sums / num_samples
        logger.info("IMU: Ноль откалиброван. Bias: %.4f", self._gyro_z_bias)

    def get_yaw(self) -> float:
        """Возврат текущего угла поворота."""
        return self._yaw

    def reset_yaw(self) -> None:
        """Сброс текущего значения угла в ноль."""
        self._yaw = 0.0

    def destroy(self) -> None:
        """Освобождение ресурсов."""
        self.stop()
        if self._bus:
            try:
                self._bus.close()
            except:
                pass
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

        except:
            return 0.0

    def _update_loop(self) -> None:
        """Фоновый цикл интеграции угловой скорости."""
        while not self._stop_event.is_set():
            now: float = time.monotonic()
            dt: float = now - self._last_update_time
            self._last_update_time: float = now

            raw_gz: float = self._read_raw_gyro_z()
            actual_gz: float = (raw_gz - self._gyro_z_bias) / self.GYRO_SCALE_FACTOR
            self._yaw += (self.GYRO_SIGN_Z * actual_gz) * dt

            time.sleep(self.UPDATE_LOOP_SLEEP_SEC)
