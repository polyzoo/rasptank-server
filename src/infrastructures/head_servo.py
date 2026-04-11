from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, final

from src.application.protocols import HeadServoProtocol

if TYPE_CHECKING:  # pragma: no cover
    from adafruit_pca9685 import PCA9685

try:
    import busio
    from adafruit_motor import servo
    from adafruit_pca9685 import PCA9685
    from board import SCL, SDA

    _HARDWARE_AVAILABLE: bool = True
except ImportError:
    SCL: Any = None
    SDA: Any = None
    busio: Any = None
    servo: Any = None
    PCA9685: Any = None
    _HARDWARE_AVAILABLE: bool = False

logger: logging.Logger = logging.getLogger(__name__)

_SETUP_EXCEPTIONS: tuple[type[BaseException], ...] = (
    OSError,
    RuntimeError,
    ConnectionError,
    ValueError,
)


@final
class HeadServoController(HeadServoProtocol):
    """Серво головы на PCA9685 CH4."""

    # Адрес PWM-контроллера PCA9685
    PCA9685_ADDRESS: int = 0x5F

    # Частота PWM для управления сервоприводом
    PWM_FREQUENCY: int = 50

    # Диапазон длительности управляющего импульса серво (мкс)
    MIN_PULSE_US: int = 500
    MAX_PULSE_US: int = 2400

    # Рабочий диапазон угла сервопривода
    ACTUATION_RANGE_DEG: int = 180

    def __init__(self, channel: int = 4, home_angle_deg: float = 0.0) -> None:
        """Инициализация серво."""
        self._channel: int = channel
        self._home_angle_deg: float = home_angle_deg
        self._pwm: object | None = None
        self._servo: object | None = None
        self._is_initialized: bool = False

    def set_angle(self, angle_deg: float) -> None:
        """Установить угол серво головы."""
        if not _HARDWARE_AVAILABLE:
            return

        self._setup()
        if self._servo is None:
            return

        angle: float = max(0.0, min(float(self.ACTUATION_RANGE_DEG), angle_deg))
        self._servo.angle = angle

    def fix_forward(self) -> None:
        """Зафиксировать голову в положении вперед."""
        self.set_angle(self._home_angle_deg)

    def destroy(self) -> None:
        """Освободить PCA9685, если он был инициализирован этим драйвером."""
        if not _HARDWARE_AVAILABLE:
            return

        if self._pwm is not None:
            try:
                self._pwm.deinit()
            except _SETUP_EXCEPTIONS as exc:
                logger.warning("Ошибка при освобождении PCA9685 головы: %s", exc)
            finally:
                self._pwm = None
                self._servo = None
                self._is_initialized = False

    def _setup(self) -> None:
        """Однократная инициализация PCA9685 и серво."""
        if not _HARDWARE_AVAILABLE or self._is_initialized:
            return

        try:
            i2c: busio.I2C = busio.I2C(SCL, SDA)
            pwm: PCA9685 = PCA9685(i2c, address=self.PCA9685_ADDRESS)
            pwm.frequency = self.PWM_FREQUENCY

            self._pwm: PCA9685 = pwm
            self._servo: servo.Servo = servo.Servo(
                pwm.channels[self._channel],
                min_pulse=self.MIN_PULSE_US,
                max_pulse=self.MAX_PULSE_US,
                actuation_range=self.ACTUATION_RANGE_DEG,
            )

            self._is_initialized: bool = True

        except _SETUP_EXCEPTIONS as exc:
            logger.exception(
                "Ошибка при инициализации серво головы (PCA9685 CH%d): %s",
                self._channel,
                exc,
            )
            self._pwm: object | None = None
            self._servo: object | None = None
            self._is_initialized: bool = False
