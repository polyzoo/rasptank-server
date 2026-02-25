import time
from typing import Optional, final

from src.application.protocols import UltrasonicSensorProtocol

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO: Optional[object] = None


@final
class UltrasonicSensor(UltrasonicSensorProtocol):
    """Контроллер работы с ультразвуковым датчиком HC-SR04."""

    # GPIO
    TRIGGER_PIN: int = 11
    ECHO_PIN: int = 8

    # Алгоритм измерения
    MEASURE_ATTEMPTS: int = 5
    OUTLIER_THRESHOLD_CM: float = 400.0
    SOUND_SPEED_CM_PER_S: float = 34_000.0
    ECHO_PATH_FACTOR: int = 2

    # Тайминги
    TRIGGER_PAUSE_S: float = 0.000002
    TRIGGER_PULSE_S: float = 0.000015

    # Таймаут ожидания эхо (с), ≈30 м при 340 м/с
    ECHO_TIMEOUT_S: float = 0.1

    # Fallback при отсутствии GPIO
    FALLBACK_DISTANCE_CM: float = 999.0

    def __init__(self) -> None:
        """Инициализация контроллера без обращения к GPIO."""
        self._is_initialized: bool = False

    def measure_distance_cm(self) -> float:
        """Измерить расстояние до препятствия в сантиметрах."""
        if GPIO is None:
            return self.FALLBACK_DISTANCE_CM

        self._setup()
        dist_cm: float = self.FALLBACK_DISTANCE_CM

        for i in range(self.MEASURE_ATTEMPTS):
            GPIO.output(self.TRIGGER_PIN, GPIO.LOW)
            time.sleep(self.TRIGGER_PAUSE_S)

            GPIO.output(self.TRIGGER_PIN, GPIO.HIGH)
            time.sleep(self.TRIGGER_PULSE_S)

            GPIO.output(self.TRIGGER_PIN, GPIO.LOW)

            timeout_at: float = time.monotonic() + self.ECHO_TIMEOUT_S
            while not GPIO.input(self.ECHO_PIN):
                if time.monotonic() >= timeout_at:
                    return dist_cm

            t1: float = time.monotonic()

            while GPIO.input(self.ECHO_PIN):
                if time.monotonic() >= timeout_at:
                    return dist_cm

            t2: float = time.monotonic()

            dist_cm: float = (t2 - t1) * self.SOUND_SPEED_CM_PER_S / self.ECHO_PATH_FACTOR
            if dist_cm <= self.OUTLIER_THRESHOLD_CM or i >= self.MEASURE_ATTEMPTS - 1:
                break

        return dist_cm

    def _setup(self) -> None:
        """Однократная настройка GPIO для датчика."""
        if GPIO is None:
            return

        if self._is_initialized:
            return

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.TRIGGER_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.ECHO_PIN, GPIO.IN)

        self._is_initialized: bool = True
