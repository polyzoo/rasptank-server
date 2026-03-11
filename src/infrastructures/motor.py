from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Optional, final

from src.application.protocols import MotorControllerProtocol

if TYPE_CHECKING:
    from adafruit_motor.motor import DCMotor
    from adafruit_pca9685 import PCA9685

try:
    from board import SCL, SDA
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import motor

    _HARDWARE_AVAILABLE: bool = True
except ImportError:
    SCL: Any = None
    SDA: Any = None
    busio: Any = None
    PCA9685: Any = None
    motor: Any = None
    _HARDWARE_AVAILABLE: bool = False

logger: logging.Logger = logging.getLogger(__name__)

# Исключения при инициализации I2C/PCA9685
_SETUP_EXCEPTIONS: tuple[type[BaseException], ...] = (
    OSError,
    RuntimeError,
    ConnectionError,
    ValueError,
)


@final
class MotorController(MotorControllerProtocol):
    """Контроллер управления моторами RaspTank.

    Использует PCA9685 по I2C (адрес 0x5F) для управления DC-моторами.
    M1 — правый мотор, M2 — левый. Поддерживает движение вперед и остановку.

    При отсутствии железа (ImportError) методы выполняются без ошибок, но не
    оказывают эффекта.
    """

    # Каналы PCA9685 для моторов
    MOTOR_M1_IN1: int = 8
    MOTOR_M1_IN2: int = 9
    MOTOR_M2_IN1: int = 10
    MOTOR_M2_IN2: int = 11

    # Направление моторов
    M1_DIRECTION: int = 1
    M2_DIRECTION: int = -1

    # Частота PWM для моторов (Гц)
    PWM_FREQUENCY: int = 50

    # I2C-адрес PCA9685 для моторов на HAT V3.1
    PCA9685_MOTOR_ADDRESS: int = 0x5F

    # Скорость (0 – 100%) и газ (0.0 = стоп, 1.0 = максимальный)
    SPEED_PERCENT_MIN: int = 0
    SPEED_PERCENT_MAX: int = 100
    THROTTLE_STOP: float = 0.0

    def __init__(self) -> None:
        """Инициализация контроллера без обращения к железу."""
        self._pwm_motor: Optional[PCA9685] = None
        self._motor1: Optional[DCMotor] = None
        self._motor2: Optional[DCMotor] = None
        self._is_initialized: bool = False

    def move_forward(self, speed_percent: int) -> None:
        """Движение вперед с заданной скоростью."""
        if not _HARDWARE_AVAILABLE:
            logger.debug("move_forward(%d): железо недоступно, пропуск", speed_percent)
            return

        self._setup()
        if self._motor1 is None or self._motor2 is None:
            logger.warning("move_forward(%d): моторы не инициализированы", speed_percent)
            return

        clamped_speed: int = max(self.SPEED_PERCENT_MIN, min(self.SPEED_PERCENT_MAX, speed_percent))
        throttle: float = clamped_speed / float(self.SPEED_PERCENT_MAX)

        self._motor1.throttle = throttle * self.M1_DIRECTION
        self._motor2.throttle = throttle * self.M2_DIRECTION
        logger.debug("move_forward: speed=%d%%, throttle=%.2f", clamped_speed, throttle)

    def move_backward(self, speed_percent: int) -> None:
        """Движение назад с заданной скоростью."""
        if not _HARDWARE_AVAILABLE:
            logger.debug("move_backward(%d): железо недоступно, пропуск", speed_percent)
            return

        self._setup()
        if self._motor1 is None or self._motor2 is None:
            logger.warning("move_backward(%d): моторы не инициализированы", speed_percent)
            return

        clamped_speed: int = max(self.SPEED_PERCENT_MIN, min(self.SPEED_PERCENT_MAX, speed_percent))
        throttle: float = clamped_speed / float(self.SPEED_PERCENT_MAX)

        self._motor1.throttle = -throttle * self.M1_DIRECTION
        self._motor2.throttle = -throttle * self.M2_DIRECTION
        logger.debug("move_backward: speed=%d%%, throttle=%.2f", clamped_speed, throttle)

    def stop(self) -> None:
        """Немедленная остановка обоих двигателей."""
        if not _HARDWARE_AVAILABLE:
            logger.debug("stop: железо недоступно, пропуск")
            return

        self._setup()

        if self._motor1 is not None:
            self._motor1.throttle = self.THROTTLE_STOP
        if self._motor2 is not None:
            self._motor2.throttle = self.THROTTLE_STOP
        logger.debug("stop: throttle=0")

    def destroy(self) -> None:
        """Освобождение ресурсов I2C и PCA9685.

        Вызывать при завершении работы приложения. После вызова контроллер переходит
        в неинициализированное состояние; повторный вызов move_forward или stop
        выполнит _setup() заново.
        """
        if not _HARDWARE_AVAILABLE:
            return

        self.stop()

        if self._pwm_motor is not None:
            try:
                self._pwm_motor.deinit()

            except _SETUP_EXCEPTIONS as exc:
                logger.warning("Ошибка при освобождении ресурсов PCA9685: %s", exc)

            finally:
                self._pwm_motor: Optional[PCA9685] = None
                self._motor1: Optional[DCMotor] = None
                self._motor2: Optional[DCMotor] = None
                self._is_initialized: bool = False

    def _setup(self) -> None:
        """Однократная инициализация PCA9685 и DC-моторов."""
        if not _HARDWARE_AVAILABLE:
            return

        if self._is_initialized:
            return

        try:
            i2c: busio.I2C = busio.I2C(SCL, SDA)
            pwm: PCA9685 = PCA9685(i2c, address=self.PCA9685_MOTOR_ADDRESS)
            pwm.frequency = self.PWM_FREQUENCY
            self._pwm_motor: Optional[PCA9685] = pwm

            dc_motor1: motor.DCMotor = motor.DCMotor(
                pwm.channels[self.MOTOR_M1_IN1],
                pwm.channels[self.MOTOR_M1_IN2],
            )
            dc_motor1.decay_mode = motor.SLOW_DECAY
            self._motor1: Optional[DCMotor] = dc_motor1

            dc_motor2: motor.DCMotor = motor.DCMotor(
                pwm.channels[self.MOTOR_M2_IN1],
                pwm.channels[self.MOTOR_M2_IN2],
            )
            dc_motor2.decay_mode = motor.SLOW_DECAY
            self._motor2: Optional[DCMotor] = dc_motor2

            self._is_initialized: bool = True
            logger.debug(
                "MotorController: PCA9685 0x%02X, M1(ch%d,%d) M2(ch%d,%d)",
                self.PCA9685_MOTOR_ADDRESS,
                self.MOTOR_M1_IN1,
                self.MOTOR_M1_IN2,
                self.MOTOR_M2_IN1,
                self.MOTOR_M2_IN2,
            )

        except _SETUP_EXCEPTIONS as exc:
            logger.exception(
                "Ошибка при инициализации моторов (I2C addr=0x%02X): %s",
                self.PCA9685_MOTOR_ADDRESS,
                exc,
            )
            self._pwm_motor: Optional[PCA9685] = None
            self._motor1: Optional[DCMotor] = None
            self._motor2: Optional[DCMotor] = None
