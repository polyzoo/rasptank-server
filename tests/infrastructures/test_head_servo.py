from __future__ import annotations

from typing import Any

from src.infrastructures import head_servo as head_servo_module
from src.infrastructures.head_servo import HeadServoController


class FakeServo:
    """Заглушка adafruit Servo."""

    def __init__(self, channel: object, **kwargs: Any) -> None:
        """Сохранить параметры и начальный угол."""
        self.channel: object = channel
        self.kwargs: dict[str, Any] = kwargs
        self.angle: float | None = None


class FakeServoModule:
    """Заглушка модуля adafruit_motor.servo."""

    Servo = FakeServo


class FakeBusio:
    """Заглушка busio с фабрикой I2C."""

    calls: list[tuple[object, object]] = []

    @staticmethod
    def I2C(scl: object, sda: object) -> tuple[object, object]:
        """Вернуть fake I2C descriptor."""
        FakeBusio.calls.append((scl, sda))
        return (scl, sda)


class FakePCA9685:
    """Заглушка PCA9685 с 16 каналами."""

    instances: list["FakePCA9685"] = []

    def __init__(self, i2c: object, *, address: int) -> None:
        """Сохранить адрес и подготовить каналы."""
        self.i2c: object = i2c
        self.address: int = address
        self.frequency: int | None = None
        self.channels: list[str] = [f"ch{idx}" for idx in range(16)]
        self.deinit_called: bool = False
        FakePCA9685.instances.append(self)

    def deinit(self) -> None:
        """Зафиксировать освобождение PCA9685."""
        self.deinit_called = True


class BrokenDeinitPCA9685(FakePCA9685):
    """PCA9685-заглушка, которая падает при deinit."""

    def deinit(self) -> None:
        """Сымитировать ошибку освобождения PCA9685."""
        raise OSError("deinit failed")


def _enable_fake_hardware(monkeypatch: Any) -> None:
    """Подключить fake hardware зависимости к модулю head_servo."""
    FakeBusio.calls = []
    FakePCA9685.instances = []
    monkeypatch.setattr(head_servo_module, "_HARDWARE_AVAILABLE", True)
    monkeypatch.setattr(head_servo_module, "SCL", "scl")
    monkeypatch.setattr(head_servo_module, "SDA", "sda")
    monkeypatch.setattr(head_servo_module, "busio", FakeBusio)
    monkeypatch.setattr(head_servo_module, "PCA9685", FakePCA9685)
    monkeypatch.setattr(head_servo_module, "servo", FakeServoModule)


def test_set_angle_is_noop_when_hardware_is_unavailable(monkeypatch: Any) -> None:
    """set_angle ничего не делает без hardware-библиотек."""
    monkeypatch.setattr(head_servo_module, "_HARDWARE_AVAILABLE", False)
    controller: HeadServoController = HeadServoController()

    controller.set_angle(45.0)

    assert controller._servo is None


def test_destroy_is_noop_when_hardware_is_unavailable(monkeypatch: Any) -> None:
    """destroy ничего не делает без hardware-библиотек."""
    monkeypatch.setattr(head_servo_module, "_HARDWARE_AVAILABLE", False)
    controller: HeadServoController = HeadServoController()
    controller._pwm = object()
    controller._servo = object()
    controller._is_initialized = True

    controller.destroy()

    assert controller._pwm is not None
    assert controller._servo is not None
    assert controller._is_initialized is True


def test_set_angle_initializes_pca9685_and_clamps_angle(monkeypatch: Any) -> None:
    """set_angle инициализирует PCA9685 CH4 и ограничивает угол диапазоном 0..180."""
    _enable_fake_hardware(monkeypatch)
    controller: HeadServoController = HeadServoController(channel=4)

    controller.set_angle(220.0)

    assert FakeBusio.calls == [("scl", "sda")]
    assert FakePCA9685.instances[0].address == controller.PCA9685_ADDRESS
    assert FakePCA9685.instances[0].frequency == controller.PWM_FREQUENCY
    assert isinstance(controller._servo, FakeServo)
    assert controller._servo.channel == "ch4"
    assert controller._servo.angle == 180.0

    controller.set_angle(-10.0)

    assert len(FakePCA9685.instances) == 1
    assert controller._servo.angle == 0.0


def test_fix_forward_uses_home_angle(monkeypatch: Any) -> None:
    """fix_forward выставляет home angle."""
    _enable_fake_hardware(monkeypatch)
    controller: HeadServoController = HeadServoController(channel=4, home_angle_deg=15.0)

    controller.fix_forward()

    assert isinstance(controller._servo, FakeServo)
    assert controller._servo.angle == 15.0


def test_destroy_deinitializes_pca9685(monkeypatch: Any) -> None:
    """destroy вызывает deinit и сбрасывает ссылки."""
    _enable_fake_hardware(monkeypatch)
    controller: HeadServoController = HeadServoController()
    controller.set_angle(10.0)
    pwm: FakePCA9685 = FakePCA9685.instances[0]

    controller.destroy()

    assert pwm.deinit_called is True
    assert controller._pwm is None
    assert controller._servo is None
    assert controller._is_initialized is False


def test_destroy_clears_state_when_deinit_fails(monkeypatch: Any) -> None:
    """destroy сбрасывает state даже при ошибке deinit."""
    _enable_fake_hardware(monkeypatch)
    monkeypatch.setattr(head_servo_module, "PCA9685", BrokenDeinitPCA9685)
    controller: HeadServoController = HeadServoController()
    controller.set_angle(10.0)

    controller.destroy()

    assert controller._pwm is None
    assert controller._servo is None
    assert controller._is_initialized is False


def test_setup_failure_leaves_controller_uninitialized(monkeypatch: Any) -> None:
    """Ошибка при setup оставляет драйвер неинициализированным."""
    _enable_fake_hardware(monkeypatch)

    class BrokenPCA9685:
        """PCA9685-заглушка, которая падает при инициализации."""

        def __init__(self, i2c: object, *, address: int) -> None:
            """Сымитировать ошибку hardware setup."""
            raise OSError("boom")

    monkeypatch.setattr(head_servo_module, "PCA9685", BrokenPCA9685)
    controller: HeadServoController = HeadServoController()

    controller.set_angle(10.0)

    assert controller._pwm is None
    assert controller._servo is None
    assert controller._is_initialized is False
