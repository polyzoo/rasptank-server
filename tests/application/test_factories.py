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


def test_create_l2_service_with_state_space() -> None:
    from unittest.mock import MagicMock

    from src.application.factories import create_l2_service
    from src.application.protocols import MotorControllerProtocol

    settings = Settings(
        l2_state_space_enabled=True, l2_state_space_t_v=0.8, l2_state_space_t_w=0.55
    )

    mock_motor = MagicMock(spec=MotorControllerProtocol)
    service = create_l2_service(settings, mock_motor)
    assert service._use_state_space is True
    assert service._state_space_controller is not None
    assert service._state_space_controller.t_v == 0.8
