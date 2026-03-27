from __future__ import annotations

import logging
import warnings
from dataclasses import dataclass
from typing import Any, final

from src.application.protocols import UltrasonicSensorProtocol

try:
    from gpiozero import DistanceSensor
    from gpiozero.input_devices import DistanceSensorNoEcho

    _HARDWARE_AVAILABLE: bool = True
except ImportError:
    DistanceSensor: Any = None
    DistanceSensorNoEcho: type[Warning] = Warning
    _HARDWARE_AVAILABLE: bool = False

logger: logging.Logger = logging.getLogger(__name__)

# Исключения при работе с датчиком
_SENSOR_EXCEPTIONS: tuple[type[BaseException], ...] = (
    OSError,
    RuntimeError,
    ConnectionError,
    ValueError,
)


@dataclass(frozen=True, slots=True)
class UltrasonicMeasurementDiagnostics:
    """Диагностика последнего чтения ультразвукового датчика."""

    distance_cm: float
    no_echo: bool = False
    used_fallback_distance: bool = False
    warning_messages: tuple[str, ...] = ()


@final
class UltrasonicSensor(UltrasonicSensorProtocol):
    """Контроллер для работы с ультразвуковым датчиком HC-SR04."""

    # Пины датчика: Trigger — импульс, Echo — отражённый сигнал
    TRIGGER_PIN: int = 23
    ECHO_PIN: int = 24

    # Максимальная дистанция (м)
    MAX_DISTANCE_M: float = 2.0

    # Отключаем внутреннее smoothing gpiozero и полагаемся на свою медиану в контроллере.
    DISTANCE_QUEUE_LEN: int = 1

    # Значение расстояния до препятствия при ошибке или работе без Raspberry Pi (см)
    FALLBACK_DISTANCE_CM: float = 999.0

    # Коэффициент перевода метров в сантиметры
    METERS_TO_CM: float = 100.0

    def __init__(self) -> None:
        """Инициализация без обращения к GPIO."""
        self._sensor: object | None = None
        self._is_initialized: bool = False
        self._last_measurement_diagnostics: UltrasonicMeasurementDiagnostics = (
            UltrasonicMeasurementDiagnostics(
                distance_cm=self.FALLBACK_DISTANCE_CM,
                used_fallback_distance=True,
                warning_messages=("sensor_not_initialized",),
            )
        )

    def measure_distance_cm(self) -> float:
        """Измерение расстояния до препятствия."""
        if not _HARDWARE_AVAILABLE:
            self._last_measurement_diagnostics = UltrasonicMeasurementDiagnostics(
                distance_cm=self.FALLBACK_DISTANCE_CM,
                used_fallback_distance=True,
                warning_messages=("hardware_unavailable",),
            )
            return self.FALLBACK_DISTANCE_CM

        self._setup()
        if self._sensor is None:
            self._last_measurement_diagnostics = UltrasonicMeasurementDiagnostics(
                distance_cm=self.FALLBACK_DISTANCE_CM,
                used_fallback_distance=True,
                warning_messages=("sensor_setup_failed",),
            )
            return self.FALLBACK_DISTANCE_CM

        try:
            with warnings.catch_warnings(record=True) as caught_warnings:
                warnings.simplefilter("always")
                distance_m: float = self._sensor.distance
            distance_cm: float = distance_m * self.METERS_TO_CM
            warning_messages: tuple[str, ...] = tuple(
                str(caught.message) for caught in caught_warnings
            )
            self._last_measurement_diagnostics = UltrasonicMeasurementDiagnostics(
                distance_cm=distance_cm,
                no_echo=any(
                    (
                        caught.category is DistanceSensorNoEcho
                        or isinstance(caught.message, DistanceSensorNoEcho)
                    )
                    for caught in caught_warnings
                ),
                warning_messages=warning_messages,
            )
            return distance_cm
        except _SENSOR_EXCEPTIONS:
            self._last_measurement_diagnostics = UltrasonicMeasurementDiagnostics(
                distance_cm=self.FALLBACK_DISTANCE_CM,
                used_fallback_distance=True,
                warning_messages=("sensor_exception",),
            )
            return self.FALLBACK_DISTANCE_CM

    def get_last_measurement_diagnostics(self) -> UltrasonicMeasurementDiagnostics:
        """Возвращает диагностику последнего чтения для высокоуровневых логов."""
        return self._last_measurement_diagnostics

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
                queue_len=self.DISTANCE_QUEUE_LEN,
            )
            self._is_initialized: bool = True
        except _SENSOR_EXCEPTIONS:
            logger.exception(
                "Ошибка при инициализации датчика (echo=%d, trigger=%d)",
                self.ECHO_PIN,
                self.TRIGGER_PIN,
            )
            self._sensor: object | None = None
