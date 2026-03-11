from typing import Any, final

from src.application.protocols import UltrasonicSensorProtocol

try:
    from gpiozero import DistanceSensor

    _HARDWARE_AVAILABLE: bool = True
except ImportError:
    DistanceSensor: Any = None
    _HARDWARE_AVAILABLE: bool = False

# Исключения при работе с датчиком
_SENSOR_EXCEPTIONS: tuple[type[BaseException], ...] = (
    OSError,
    RuntimeError,
    ConnectionError,
    ValueError,
)


@final
class UltrasonicSensor(UltrasonicSensorProtocol):
    """Контроллер для работы с ультразвуковым датчиком HC-SR04."""

    # Пины датчика: Trigger — импульс, Echo — отражённый сигнал
    TRIGGER_PIN: int = 23
    ECHO_PIN: int = 24

    # Максимальная дистанция (м)
    MAX_DISTANCE_M: float = 2.0

    # Значение расстояния до препятствия при ошибке или работе без Raspberry Pi (см)
    FALLBACK_DISTANCE_CM: float = 999.0

    # Коэффициент перевода метров в сантиметры
    METERS_TO_CM: float = 100.0

    def __init__(self) -> None:
        """Инициализация без обращения к GPIO."""
        self._sensor: object | None = None
        self._is_initialized: bool = False

    def measure_distance_cm(self) -> float:
        """Измерение расстояния до препятствия."""
        if not _HARDWARE_AVAILABLE:
            return self.FALLBACK_DISTANCE_CM

        self._setup()
        if self._sensor is None:
            return self.FALLBACK_DISTANCE_CM

        try:
            distance_m: float = self._sensor.distance
            return distance_m * self.METERS_TO_CM
        except _SENSOR_EXCEPTIONS:
            return self.FALLBACK_DISTANCE_CM

    def destroy(self) -> None:
        """Освобождение ресурсов GPIO."""
        if not _HARDWARE_AVAILABLE:
            return

        if self._sensor is not None:
            try:
                self._sensor.close()
            except _SENSOR_EXCEPTIONS:
                pass
            finally:
                self._sensor: object | None = None
                self._is_initialized: bool = False

    def _setup(self) -> None:
        """Однократная инициализация датчика."""
        if not _HARDWARE_AVAILABLE:
            return

        if self._is_initialized:
            return

        try:
            self._sensor: object | None = DistanceSensor(
                echo=self.ECHO_PIN,
                trigger=self.TRIGGER_PIN,
                max_distance=self.MAX_DISTANCE_M,
            )
            self._is_initialized: bool = True
        except _SENSOR_EXCEPTIONS:
            self._sensor: object | None = None
