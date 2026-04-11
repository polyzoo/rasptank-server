from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, final

from src.application.protocols import MotorControllerProtocol

if TYPE_CHECKING:  # pragma: no cover
    from adafruit_pca9685 import PCA9685

try:
    import busio
    from adafruit_motor import motor
    from adafruit_pca9685 import PCA9685
    from board import SCL, SDA

    _HARDWARE_AVAILABLE: bool = True
except ImportError:
    SCL: Any = None
    SDA: Any = None
    busio: Any = None
    PCA9685: Any = None
    motor: Any = None
    _HARDWARE_AVAILABLE: bool = False

logger: logging.Logger = logging.getLogger(__name__)

_SETUP_EXCEPTIONS: tuple[type[BaseException], ...] = (
    OSError,
    RuntimeError,
    ConnectionError,
    ValueError,
)


@final
class MotorController(MotorControllerProtocol):
    """Контроллер управления моторами RaspTank."""

    # Каналы PCA9685 для входов драйвера мотора M1
    MOTOR_M1_IN1: int = 8
    MOTOR_M1_IN2: int = 9

    # Каналы PCA9685 для входов драйвера мотора M2
    MOTOR_M2_IN1: int = 10
    MOTOR_M2_IN2: int = 11

    # Коэффициенты направления, учитывающие физическую установку моторов
    M1_DIRECTION: int = 1
    M2_DIRECTION: int = -1

    # Настройки PWM-контроллера PCA9685
    PWM_FREQUENCY: int = 50
    PCA9685_MOTOR_ADDRESS: int = 0x5F

    # Диапазон пользовательской скорости в процентах
    SPEED_PERCENT_MIN: int = 0
    SPEED_PERCENT_MAX: int = 100

    # Диапазон signed-скорости для независимого управления гусеницами
    SPEED_PERCENT_SIGNED_MIN: int = -100
    SPEED_PERCENT_SIGNED_MAX: int = 100

    # Диапазон throttle для adafruit DCMotor
    THROTTLE_STOP: float = 0.0
    THROTTLE_REVERSE_MIN: float = -1.0
    THROTTLE_MIN: float = 0.0
    THROTTLE_MAX: float = 1.0

    def __init__(self, tl_left_offset: int = 0, tl_right_offset: int = 0) -> None:
        """Инициализация контроллера."""
        self.TL_LEFT_OFFSET: int = tl_left_offset
        self.TL_RIGHT_OFFSET: int = tl_right_offset
        self._pwm_motor: object | None = None
        self._motor1: object | None = None
        self._motor2: object | None = None
        self._is_initialized: bool = False

    def move_forward(self, speed_percent: int, steer_percent: int = 0) -> None:
        """Движение вперед с заданной скоростью."""
        self._move_line(speed_percent, steer_percent, reverse=False)

    def move_backward(self, speed_percent: int, steer_percent: int = 0) -> None:
        """Движение назад с заданной скоростью."""
        self._move_line(speed_percent, steer_percent, reverse=True)

    def turn_left(self, speed_percent: int) -> None:
        """Поворот налево на месте."""
        self._turn_in_place(speed_percent, left=True)

    def turn_right(self, speed_percent: int) -> None:
        """Поворот направо на месте."""
        self._turn_in_place(speed_percent, left=False)

    def stop(self) -> None:
        """Немедленная остановка обоих двигателей."""
        if not _HARDWARE_AVAILABLE:
            return

        self._setup()

        if self._motor1 is not None:
            self._motor1.throttle = self.THROTTLE_STOP

        if self._motor2 is not None:
            self._motor2.throttle = self.THROTTLE_STOP

    def destroy(self) -> None:
        """Освобождение ресурсов I2C и PCA9685."""
        if not _HARDWARE_AVAILABLE:
            return

        self.stop()

        if self._pwm_motor is not None:
            try:
                self._pwm_motor.deinit()
            except _SETUP_EXCEPTIONS as exc:
                logger.warning("Ошибка при освобождении ресурсов PCA9685: %s", exc)
            finally:
                self._pwm_motor: object | None = None
                self._motor1: object | None = None
                self._motor2: object | None = None
                self._is_initialized: bool = False

    def _move_line(self, speed_percent: int, steer_percent: int, *, reverse: bool) -> None:
        """Выполнить движение вперед или назад с подруливанием."""
        if not _HARDWARE_AVAILABLE:
            return

        self._setup()
        if self._motor1 is None or self._motor2 is None:
            return

        clamped_speed: int = max(self.SPEED_PERCENT_MIN, min(self.SPEED_PERCENT_MAX, speed_percent))
        steer: int = max(-self.SPEED_PERCENT_MAX, min(self.SPEED_PERCENT_MAX, steer_percent))

        left_pct: int = max(
            self.SPEED_PERCENT_MIN,
            min(self.SPEED_PERCENT_MAX, clamped_speed + self.TL_LEFT_OFFSET + steer),
        )
        right_pct: int = max(
            self.SPEED_PERCENT_MIN,
            min(self.SPEED_PERCENT_MAX, clamped_speed + self.TL_RIGHT_OFFSET - steer),
        )

        left_speed: float = max(
            self.THROTTLE_MIN,
            min(self.THROTTLE_MAX, left_pct / float(self.SPEED_PERCENT_MAX)),
        )
        right_speed: float = max(
            self.THROTTLE_MIN,
            min(self.THROTTLE_MAX, right_pct / float(self.SPEED_PERCENT_MAX)),
        )

        direction: float = -1.0 if reverse else 1.0
        self._motor1.throttle = direction * right_speed * self.M1_DIRECTION
        self._motor2.throttle = direction * left_speed * self.M2_DIRECTION

    def _turn_in_place(self, speed_percent: int, *, left: bool) -> None:
        """Выполнить поворот на месте в заданную сторону."""
        if not _HARDWARE_AVAILABLE:
            return

        self._setup()
        if self._motor1 is None or self._motor2 is None:
            return

        clamped_speed: int = max(self.SPEED_PERCENT_MIN, min(self.SPEED_PERCENT_MAX, speed_percent))
        throttle: float = clamped_speed / float(self.SPEED_PERCENT_MAX)

        if left:
            self._motor1.throttle = -throttle * self.M1_DIRECTION
            self._motor2.throttle = throttle * self.M2_DIRECTION
        else:
            self._motor1.throttle = throttle * self.M1_DIRECTION
            self._motor2.throttle = -throttle * self.M2_DIRECTION

    def _setup(self) -> None:
        """Однократная инициализация PCA9685 и DC-моторов."""
        if not _HARDWARE_AVAILABLE or self._is_initialized:
            return

        try:
            i2c: busio.I2C = busio.I2C(SCL, SDA)
            pwm: PCA9685 = PCA9685(i2c, address=self.PCA9685_MOTOR_ADDRESS)
            pwm.frequency = self.PWM_FREQUENCY
            self._pwm_motor: object | None = pwm

            dc_motor1: motor.DCMotor = motor.DCMotor(
                pwm.channels[self.MOTOR_M1_IN1],
                pwm.channels[self.MOTOR_M1_IN2],
            )
            dc_motor1.decay_mode = motor.SLOW_DECAY
            self._motor1: object | None = dc_motor1

            dc_motor2: motor.DCMotor = motor.DCMotor(
                pwm.channels[self.MOTOR_M2_IN1],
                pwm.channels[self.MOTOR_M2_IN2],
            )
            dc_motor2.decay_mode = motor.SLOW_DECAY
            self._motor2: object | None = dc_motor2

            self._is_initialized: bool = True

        except _SETUP_EXCEPTIONS as exc:
            logger.exception("Ошибка при инициализации моторов: %s", exc)
            self._pwm_motor: object | None = None
            self._motor1: object | None = None
            self._motor2: object | None = None
