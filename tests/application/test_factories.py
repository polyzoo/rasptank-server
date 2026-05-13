from __future__ import annotations

from src.application.factories import _create_ultrasonic_sensor
from src.config.settings import Settings
from src.infrastructures.ultrasonic import DisabledUltrasonicSensor, UltrasonicSensor


def test_create_ultrasonic_sensor_returns_disabled_stub_when_setting_is_false() -> None:
    """ULTRASONIC_ENABLED=false отключает обращения к GPIO."""
    sensor = _create_ultrasonic_sensor(Settings(ultrasonic_enabled=False))

    assert isinstance(sensor, DisabledUltrasonicSensor)
    assert sensor.measure_distance_cm() == UltrasonicSensor.FALLBACK_DISTANCE_CM


def test_create_ultrasonic_sensor_applies_configured_bcm_pins() -> None:
    """Пины HC-SR04 можно задать через Settings/env без правки класса."""
    sensor = _create_ultrasonic_sensor(Settings(ultrasonic_trigger_pin=17, ultrasonic_echo_pin=27))

    assert isinstance(sensor, UltrasonicSensor)
    assert sensor.TRIGGER_PIN == 17
    assert sensor.ECHO_PIN == 27
