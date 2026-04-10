from __future__ import annotations

from typing import Any

from src.infrastructures import ultrasonic as ultrasonic_module
from src.infrastructures.ultrasonic import UltrasonicSensor


class FakeDistanceSensor:
    """Заглушка gpiozero DistanceSensor."""

    instances: list["FakeDistanceSensor"] = []

    def __init__(self, **kwargs: Any) -> None:
        """Сохранить параметры создания датчика."""
        self.kwargs: dict[str, Any] = kwargs
        self.distance: float = 0.42
        self.close_called: bool = False
        FakeDistanceSensor.instances.append(self)

    def close(self) -> None:
        """Зафиксировать закрытие датчика."""
        self.close_called = True


class BrokenDistanceSensor(FakeDistanceSensor):
    """DistanceSensor, который падает при чтении."""

    @property
    def distance(self) -> float:
        """Сымитировать ошибку чтения."""
        raise RuntimeError("sensor failed")

    @distance.setter
    def distance(self, value: float) -> None:
        """Игнорировать установку начального значения."""
        self._distance: float = value


class BrokenSetupDistanceSensor(FakeDistanceSensor):
    """DistanceSensor, который падает при setup."""

    def __init__(self, **kwargs: Any) -> None:
        """Сымитировать ошибку setup."""
        super().__init__(**kwargs)
        raise OSError("setup failed")


class BrokenCloseDistanceSensor(FakeDistanceSensor):
    """DistanceSensor, который падает при close."""

    def close(self) -> None:
        """Сымитировать ошибку закрытия датчика."""
        raise RuntimeError("close failed")


def _enable_fake_hardware(monkeypatch: Any, sensor_cls: type[FakeDistanceSensor]) -> None:
    """Подключить fake gpiozero зависимости."""
    sensor_cls.instances = []
    monkeypatch.setattr(ultrasonic_module, "_HARDWARE_AVAILABLE", True)
    monkeypatch.setattr(ultrasonic_module, "DistanceSensor", sensor_cls)


def test_measure_distance_returns_fallback_without_hardware(monkeypatch: Any) -> None:
    """Без hardware-библиотек возвращается fallback distance."""
    monkeypatch.setattr(ultrasonic_module, "_HARDWARE_AVAILABLE", False)
    sensor: UltrasonicSensor = UltrasonicSensor()

    distance_cm: float = sensor.measure_distance_cm()

    assert distance_cm == sensor.FALLBACK_DISTANCE_CM


def test_setup_is_noop_without_hardware(monkeypatch: Any) -> None:
    """_setup ничего не делает без hardware-библиотек."""
    monkeypatch.setattr(ultrasonic_module, "_HARDWARE_AVAILABLE", False)
    sensor: UltrasonicSensor = UltrasonicSensor()

    sensor._setup()

    assert sensor._sensor is None
    assert sensor._is_initialized is False


def test_destroy_is_noop_without_hardware(monkeypatch: Any) -> None:
    """destroy ничего не делает без hardware-библиотек."""
    monkeypatch.setattr(ultrasonic_module, "_HARDWARE_AVAILABLE", False)
    sensor: UltrasonicSensor = UltrasonicSensor()
    sensor._sensor = object()
    sensor._is_initialized = True

    sensor.destroy()

    assert sensor._sensor is not None
    assert sensor._is_initialized is True


def test_setup_creates_distance_sensor_with_expected_pins(monkeypatch: Any) -> None:
    """_setup создает DistanceSensor с ожидаемыми GPIO-пинами."""
    _enable_fake_hardware(monkeypatch, FakeDistanceSensor)
    sensor: UltrasonicSensor = UltrasonicSensor()

    sensor._setup()

    created: FakeDistanceSensor = FakeDistanceSensor.instances[0]
    assert created.kwargs == {
        "echo": sensor.ECHO_PIN,
        "trigger": sensor.TRIGGER_PIN,
        "max_distance": sensor.MAX_DISTANCE_M,
        "queue_len": sensor.DISTANCE_QUEUE_LEN,
    }
    assert sensor._is_initialized is True

    sensor._setup()

    assert len(FakeDistanceSensor.instances) == 1


def test_measure_distance_records_centimeters(monkeypatch: Any) -> None:
    """Дистанция из метров переводится в сантиметры."""
    _enable_fake_hardware(monkeypatch, FakeDistanceSensor)
    sensor: UltrasonicSensor = UltrasonicSensor()

    distance_cm: float = sensor.measure_distance_cm()

    assert distance_cm == 42.0


def test_measure_distance_records_fallback_on_exception(monkeypatch: Any) -> None:
    """Ошибка чтения датчика переводится в fallback distance."""
    _enable_fake_hardware(monkeypatch, BrokenDistanceSensor)
    sensor: UltrasonicSensor = UltrasonicSensor()

    distance_cm: float = sensor.measure_distance_cm()

    assert distance_cm == sensor.FALLBACK_DISTANCE_CM


def test_measure_distance_records_fallback_on_setup_failure(monkeypatch: Any) -> None:
    """Ошибка setup переводится в fallback distance."""
    _enable_fake_hardware(monkeypatch, BrokenSetupDistanceSensor)
    sensor: UltrasonicSensor = UltrasonicSensor()

    distance_cm: float = sensor.measure_distance_cm()

    assert distance_cm == sensor.FALLBACK_DISTANCE_CM


def test_destroy_closes_sensor(monkeypatch: Any) -> None:
    """destroy закрывает DistanceSensor и сбрасывает state."""
    _enable_fake_hardware(monkeypatch, FakeDistanceSensor)
    sensor: UltrasonicSensor = UltrasonicSensor()
    sensor._setup()
    created: FakeDistanceSensor = FakeDistanceSensor.instances[0]

    sensor.destroy()

    assert created.close_called is True
    assert sensor._sensor is None
    assert sensor._is_initialized is False


def test_destroy_clears_state_when_close_fails(monkeypatch: Any) -> None:
    """destroy сбрасывает state даже при ошибке close."""
    _enable_fake_hardware(monkeypatch, BrokenCloseDistanceSensor)
    sensor: UltrasonicSensor = UltrasonicSensor()
    sensor._setup()

    sensor.destroy()

    assert sensor._sensor is None
    assert sensor._is_initialized is False
