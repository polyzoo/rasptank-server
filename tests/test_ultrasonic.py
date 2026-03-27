from __future__ import annotations

import unittest
import warnings
from unittest.mock import patch

from src.infrastructures import ultrasonic as ultrasonic_module
from src.infrastructures.ultrasonic import UltrasonicSensor


class FakeNoEchoWarning(Warning):
    """Тестовый аналог gpiozero DistanceSensorNoEcho."""


class FakeNoEchoSensor:
    """Сенсор, который возвращает максимальную дистанцию и пишет no-echo warning."""

    @property
    def distance(self) -> float:
        warnings.warn(FakeNoEchoWarning("no echo received"))
        return 2.0


class FakeBrokenSensor:
    """Сенсор, который падает при чтении distance."""

    @property
    def distance(self) -> float:
        raise RuntimeError("boom")


class UltrasonicSensorTests(unittest.TestCase):
    """Тесты драйвера ультразвука."""

    def test_setup_disables_gpiozero_internal_smoothing(self) -> None:
        """Side scan должен читать актуальное направление, а не длинную queue history."""
        with (
            patch.object(ultrasonic_module, "_HARDWARE_AVAILABLE", True),
            patch.object(ultrasonic_module, "DistanceSensor") as sensor_cls,
        ):
            sensor = UltrasonicSensor()
            sensor._setup()

        sensor_cls.assert_called_once_with(
            echo=UltrasonicSensor.ECHO_PIN,
            trigger=UltrasonicSensor.TRIGGER_PIN,
            max_distance=UltrasonicSensor.MAX_DISTANCE_M,
            queue_len=UltrasonicSensor.DISTANCE_QUEUE_LEN,
        )

    def test_measure_distance_records_no_echo_in_diagnostics(self) -> None:
        """no-echo должен попадать в структурированную диагностику последнего чтения."""
        with (
            patch.object(ultrasonic_module, "_HARDWARE_AVAILABLE", True),
            patch.object(ultrasonic_module, "DistanceSensorNoEcho", FakeNoEchoWarning),
        ):
            sensor = UltrasonicSensor()
            sensor._sensor = FakeNoEchoSensor()
            sensor._is_initialized = True

            distance_cm = sensor.measure_distance_cm()
            diagnostics = sensor.get_last_measurement_diagnostics()

        self.assertEqual(200.0, distance_cm)
        self.assertTrue(diagnostics.no_echo)
        self.assertFalse(diagnostics.used_fallback_distance)
        self.assertEqual(("no echo received",), diagnostics.warning_messages)

    def test_measure_distance_records_fallback_on_sensor_exception(self) -> None:
        """Исключение драйвера не должно теряться: диагностика должна отметить fallback."""
        with patch.object(ultrasonic_module, "_HARDWARE_AVAILABLE", True):
            sensor = UltrasonicSensor()
            sensor._sensor = FakeBrokenSensor()
            sensor._is_initialized = True

            distance_cm = sensor.measure_distance_cm()
            diagnostics = sensor.get_last_measurement_diagnostics()

        self.assertEqual(UltrasonicSensor.FALLBACK_DISTANCE_CM, distance_cm)
        self.assertFalse(diagnostics.no_echo)
        self.assertTrue(diagnostics.used_fallback_distance)
        self.assertEqual(("sensor_exception",), diagnostics.warning_messages)


if __name__ == "__main__":
    unittest.main()
