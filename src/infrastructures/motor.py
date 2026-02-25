from typing import Optional, final

from src.application.protocols import MotorControllerProtocol

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO: Optional[object] = None


@final
class MotorController(MotorControllerProtocol):
    """Контроллер управления моторами машинки на Raspberry Pi.

    Поддерживает только движение вперёд и остановку. Motor A — правый мотор,
    Motor B — левый; для движения вперёд оба вращаются «назад».
    """

    # Motor A (правый)
    MOTOR_A_EN: int = 4
    MOTOR_A_PIN1: int = 14
    MOTOR_A_PIN2: int = 15

    # Motor B (левый)
    MOTOR_B_EN: int = 17
    MOTOR_B_PIN1: int = 27
    MOTOR_B_PIN2: int = 18

    # PWM
    PWM_RATE: int = 1_000
    PWM_LEFT_START: int = 100
    PWM_RIGHT_START: int = 0

    def __init__(self) -> None:
        """Инициализация контроллера без обращения к GPIO."""
        self._pwm_a: Optional[object] = None
        self._pwm_b: Optional[object] = None
        self._is_initialized: bool = False

    def move_forward(self, speed_percent: int) -> None:
        """Движение машинки вперёд с заданной скоростью (0 – 100%)."""
        if GPIO is None:
            return

        self._setup()
        if self._pwm_a is None or self._pwm_b is None:
            return

        # Левый мотор (B): комбинация пинов для «назад» → машинка вперёд
        GPIO.output(self.MOTOR_B_PIN1, GPIO.HIGH)
        GPIO.output(self.MOTOR_B_PIN2, GPIO.LOW)
        self._pwm_b.start(self.PWM_LEFT_START)
        self._pwm_b.ChangeDutyCycle(speed_percent)

        # Правый мотор (A): комбинация пинов для «назад» → машинка вперёд
        GPIO.output(self.MOTOR_A_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR_A_PIN2, GPIO.HIGH)
        self._pwm_a.start(self.PWM_RIGHT_START)
        self._pwm_a.ChangeDutyCycle(speed_percent)

    def stop(self) -> None:
        """Немедленная остановка обоих двигателей."""
        if GPIO is None:
            return

        self._setup()

        GPIO.output(self.MOTOR_A_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR_A_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR_B_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR_B_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR_A_EN, GPIO.LOW)
        GPIO.output(self.MOTOR_B_EN, GPIO.LOW)

    def _setup(self) -> None:
        """Однократная настройка GPIO и PWM для обоих моторов."""
        if GPIO is None:
            return

        if self._is_initialized:
            return

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.MOTOR_A_EN, GPIO.OUT)
        GPIO.setup(self.MOTOR_B_EN, GPIO.OUT)
        GPIO.setup(self.MOTOR_A_PIN1, GPIO.OUT)
        GPIO.setup(self.MOTOR_A_PIN2, GPIO.OUT)
        GPIO.setup(self.MOTOR_B_PIN1, GPIO.OUT)
        GPIO.setup(self.MOTOR_B_PIN2, GPIO.OUT)

        GPIO.output(self.MOTOR_A_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR_A_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR_B_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR_B_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR_A_EN, GPIO.LOW)
        GPIO.output(self.MOTOR_B_EN, GPIO.LOW)

        try:
            self._pwm_a: object = GPIO.PWM(self.MOTOR_A_EN, self.PWM_RATE)
            self._pwm_b: object = GPIO.PWM(self.MOTOR_B_EN, self.PWM_RATE)
        except Exception:
            pass

        self._is_initialized: bool = True
