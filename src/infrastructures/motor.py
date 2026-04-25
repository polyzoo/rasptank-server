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

    # Дефолты для конструктора (на проде совпадают с Settings; можно переопределить).
    _DEFAULT_M1_DIRECTION: int = -1
    _DEFAULT_M2_DIRECTION: int = 1

    # Настройки PWM-контроллера PCA9685
    PWM_FREQUENCY: int = 50
    PCA9685_MOTOR_ADDRESS: int = 0x5F

    # Диапазон пользовательской скорости в процентах
    SPEED_PERCENT_MIN: int = 0
    SPEED_PERCENT_MAX: int = 100

    # Диапазон скорости со знаком для независимого управления гусеницами
    SPEED_PERCENT_SIGNED_MIN: int = -100
    SPEED_PERCENT_SIGNED_MAX: int = 100

    # Диапазон throttle для adafruit DCMotor
    THROTTLE_STOP: float = 0.0
    THROTTLE_REVERSE_MIN: float = -1.0
    THROTTLE_MIN: float = 0.0
    THROTTLE_MAX: float = 1.0

    def __init__(
        self,
        tl_left_offset: int = 0,
        tl_right_offset: int = 0,
        *,
        m1_direction: int | None = None,
        m2_direction: int | None = None,
    ) -> None:
        """Инициализация контроллера."""
        self.TL_LEFT_OFFSET: int = tl_left_offset
        self.TL_RIGHT_OFFSET: int = tl_right_offset
        m1: int = self._DEFAULT_M1_DIRECTION if m1_direction is None else m1_direction
        m2: int = self._DEFAULT_M2_DIRECTION if m2_direction is None else m2_direction
        if m1 not in (-1, 1) or m2 not in (-1, 1):
            raise ValueError("m1_direction и m2_direction должны быть 1 или -1")
        self._m1_direction: int = m1
        self._m2_direction: int = m2
        self._pwm_motor: object | None = None
        self._motor1: object | None = None
        self._motor2: object | None = None
        self._is_initialized: bool = False

    def set_tracks(self, left_speed_percent: int, right_speed_percent: int) -> None:
        """Независимо задать скорость левой и правой гусеницы в процентах со знаком."""
        if not _HARDWARE_AVAILABLE:
            return

        self._setup()
        if self._motor1 is None or self._motor2 is None:
            return

        left_pct: int = self._clamp_signed_speed(left_speed_percent)
        right_pct: int = self._clamp_signed_speed(right_speed_percent)

        self._motor1.throttle = self._signed_percent_to_throttle(right_pct, self._m1_direction)
        self._motor2.throttle = self._signed_percent_to_throttle(left_pct, self._m2_direction)

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

        clamped_speed: int = self._clamp_unsigned_speed(speed_percent)
        steer: int = max(-self.SPEED_PERCENT_MAX, min(self.SPEED_PERCENT_MAX, steer_percent))

        effective_reverse: bool = reverse
        # При движении назад знак дифференциала лево/право для удержания курса обратный.
        steer_mix: int = -steer if effective_reverse else steer

        left_pct: int = max(
            self.SPEED_PERCENT_MIN,
            min(self.SPEED_PERCENT_MAX, clamped_speed + self.TL_LEFT_OFFSET + steer_mix),
        )
        right_pct: int = max(
            self.SPEED_PERCENT_MIN,
            min(self.SPEED_PERCENT_MAX, clamped_speed + self.TL_RIGHT_OFFSET - steer_mix),
        )

        direction: float = -1.0 if effective_reverse else 1.0
        self._motor1.throttle = self._signed_percent_to_throttle(
            int(direction * right_pct),
            self._m1_direction,
        )
        self._motor2.throttle = self._signed_percent_to_throttle(
            int(direction * left_pct),
            self._m2_direction,
        )

    def _turn_in_place(self, speed_percent: int, *, left: bool) -> None:
        """Выполнить поворот на месте в заданную сторону."""
        if not _HARDWARE_AVAILABLE:
            return

        self._setup()
        if self._motor1 is None or self._motor2 is None:
            return

        if left:
            self.set_tracks(left_speed_percent=speed_percent, right_speed_percent=-speed_percent)
        else:
            self.set_tracks(left_speed_percent=-speed_percent, right_speed_percent=speed_percent)

    def _clamp_unsigned_speed(self, speed_percent: int) -> int:
        """Ограничить скорость без знака допустимым диапазоном."""
        return max(self.SPEED_PERCENT_MIN, min(self.SPEED_PERCENT_MAX, speed_percent))

    def _clamp_signed_speed(self, speed_percent: int) -> int:
        """Ограничить скорость со знаком допустимым диапазоном."""
        return max(
            self.SPEED_PERCENT_SIGNED_MIN,
            min(self.SPEED_PERCENT_SIGNED_MAX, speed_percent),
        )

    def _signed_percent_to_throttle(self, speed_percent: int, direction: int) -> float:
        """Преобразовать скорость со знаком в мощность мотора с учетом его направления."""
        clamped_speed: int = self._clamp_signed_speed(speed_percent)
        throttle: float = clamped_speed / float(self.SPEED_PERCENT_MAX)
        return max(
            self.THROTTLE_REVERSE_MIN,
            min(self.THROTTLE_MAX, throttle * direction),
        )

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
