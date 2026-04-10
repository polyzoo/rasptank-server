from __future__ import annotations

import importlib
import sys
import types

import pytest

from src.infrastructures import head_servo, imu, motor, ultrasonic


class FakePCA9685:
    """Минимальная заглушка PCA9685 для import-time проверки."""


class FakeDistanceSensor:
    """Минимальная заглушка DistanceSensor для import-time проверки."""


class FakeSMBus:
    """Минимальная заглушка SMBus для import-time проверки."""


def _module(name: str, **attrs: object) -> types.ModuleType:
    """Создать ModuleType с заданными атрибутами."""
    module: types.ModuleType = types.ModuleType(name)
    for attr_name, attr_value in attrs.items():
        setattr(module, attr_name, attr_value)
    return module


def test_optional_hardware_imports_enable_hardware_flags(monkeypatch: pytest.MonkeyPatch) -> None:
    """Успешный импорт fake hardware-библиотек включает hardware flags."""
    board_module: types.ModuleType = _module("board", SCL="scl", SDA="sda")
    busio_module: types.ModuleType = _module("busio")
    servo_module: types.ModuleType = _module("adafruit_motor.servo")
    dc_motor_module: types.ModuleType = _module("adafruit_motor.motor")
    adafruit_motor_module: types.ModuleType = _module(
        "adafruit_motor",
        servo=servo_module,
        motor=dc_motor_module,
    )
    pca_module: types.ModuleType = _module("adafruit_pca9685", PCA9685=FakePCA9685)
    gpiozero_module: types.ModuleType = _module("gpiozero", DistanceSensor=FakeDistanceSensor)
    smbus_module: types.ModuleType = _module("smbus2", SMBus=FakeSMBus)

    fake_modules: dict[str, types.ModuleType] = {
        "board": board_module,
        "busio": busio_module,
        "adafruit_motor": adafruit_motor_module,
        "adafruit_motor.servo": servo_module,
        "adafruit_motor.motor": dc_motor_module,
        "adafruit_pca9685": pca_module,
        "gpiozero": gpiozero_module,
        "smbus2": smbus_module,
    }

    try:
        for name, module in fake_modules.items():
            monkeypatch.setitem(sys.modules, name, module)

        importlib.reload(head_servo)
        importlib.reload(motor)
        importlib.reload(ultrasonic)
        importlib.reload(imu)

        assert head_servo._HARDWARE_AVAILABLE is True
        assert motor._HARDWARE_AVAILABLE is True
        assert ultrasonic._HARDWARE_AVAILABLE is True
        assert imu._HARDWARE_AVAILABLE is True

    finally:
        monkeypatch.undo()
        importlib.reload(head_servo)
        importlib.reload(motor)
        importlib.reload(ultrasonic)
        importlib.reload(imu)
