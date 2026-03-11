from __future__ import annotations

import logging
from typing import Any, Optional, final

from src.application.protocols import UltrasonicSensorProtocol

logger: logging.Logger = logging.getLogger(__name__)

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
        self._sensor: Optional[object] = None
        self._is_initialized: bool = False

    def measure_distance_cm(self) -> float:
        """Измерение расстояния до препятствия."""
        if not _HARDWARE_AVAILABLE:
            logger.debug("measure_distance_cm: gpiozero недоступен, fallback=%.1f см", self.FALLBACK_DISTANCE_CM)
            return self.FALLBACK_DISTANCE_CM

        self._setup()
        if self._sensor is None:
            logger.debug("measure_distance_cm: датчик не инициализирован, fallback=%.1f см", self.FALLBACK_DISTANCE_CM)
            return self.FALLBACK_DISTANCE_CM

        try:
            distance_m: float = self._sensor.distance
            distance_cm: float = distance_m * self.METERS_TO_CM
            logger.debug("measure_distance_cm: %.1f см (raw=%.3f м)", distance_cm, distance_m)
            return distance_cm
        except _SENSOR_EXCEPTIONS as exc:
            logger.debug("measure_distance_cm: ошибка датчика %s, fallback=%.1f см", exc, self.FALLBACK_DISTANCE_CM)
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
                self._sensor: Optional[object] = None
                self._is_initialized: bool = False

    def _setup(self) -> None:
        """Однократная инициализация датчика."""
        if not _HARDWARE_AVAILABLE:
            return

        if self._is_initialized:
            return

        try:
            self._sensor: Optional[object] = DistanceSensor(
                echo=self.ECHO_PIN,
                trigger=self.TRIGGER_PIN,
                max_distance=self.MAX_DISTANCE_M,
            )
            self._is_initialized: bool = True
            logger.debug("UltrasonicSensor: инициализирован (echo=%d, trigger=%d)", self.ECHO_PIN, self.TRIGGER_PIN)
        except _SENSOR_EXCEPTIONS as exc:
            logger.warning("UltrasonicSensor: ошибка инициализации (echo=%d, trigger=%d): %s", self.ECHO_PIN, self.TRIGGER_PIN, exc)
            self._sensor: Optional[object] = None
