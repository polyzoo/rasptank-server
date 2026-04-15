from __future__ import annotations

from typing import Any

from src.infrastructures import motor as motor_module
from src.infrastructures.motor import MotorController


class FakeBusio:
    """Заглушка busio с фабрикой I2C."""

    calls: list[tuple[object, object]] = []

    @staticmethod
    def I2C(scl: object, sda: object) -> tuple[object, object]:
        """Вернуть fake I2C descriptor."""
        FakeBusio.calls.append((scl, sda))
        return scl, sda


class FakePCA9685:
    """Заглушка PCA9685 с каналами и deinit."""

    instances: list["FakePCA9685"] = []

    def __init__(self, i2c: object, *, address: int) -> None:
        """Сохранить параметры создания PWM-контроллера."""
        self.i2c: object = i2c
        self.address: int = address
        self.frequency: int | None = None
        self.channels: list[str] = [f"ch{index}" for index in range(16)]
        self.deinit_called: bool = False
        FakePCA9685.instances.append(self)

    def deinit(self) -> None:
        """Зафиксировать освобождение PCA9685."""
        self.deinit_called = True


class BrokenPCA9685:
    """PCA9685-заглушка, которая падает при setup."""

    def __init__(self, i2c: object, *, address: int) -> None:
        """Сымитировать ошибку инициализации."""
        raise OSError("pwm failed")


class BrokenDeinitPCA9685(FakePCA9685):
    """PCA9685-заглушка, которая падает при deinit."""

    def deinit(self) -> None:
        """Сымитировать ошибку освобождения ресурсов."""
        raise OSError("deinit failed")


class FakeDCMotor:
    """Заглушка adafruit_motor.motor.DCMotor."""

    instances: list["FakeDCMotor"] = []

    def __init__(self, in1: object, in2: object) -> None:
        """Сохранить каналы мотора."""
        self.in1: object = in1
        self.in2: object = in2
        self.throttle: float | None = None
        self.decay_mode: str | None = None
        FakeDCMotor.instances.append(self)


class FakeMotorModule:
    """Заглушка модуля adafruit_motor.motor."""

    SLOW_DECAY: str = "slow"
    DCMotor = FakeDCMotor


def _enable_fake_hardware(monkeypatch: Any, pca_cls: type[FakePCA9685] = FakePCA9685) -> None:
    """Подключить fake hardware зависимости к модулю motor."""
    FakeBusio.calls = []
    FakePCA9685.instances = []
    FakeDCMotor.instances = []
    monkeypatch.setattr(motor_module, "_HARDWARE_AVAILABLE", True)
    monkeypatch.setattr(motor_module, "SCL", "scl")
    monkeypatch.setattr(motor_module, "SDA", "sda")
    monkeypatch.setattr(motor_module, "busio", FakeBusio)
    monkeypatch.setattr(motor_module, "PCA9685", pca_cls)
    monkeypatch.setattr(motor_module, "motor", FakeMotorModule)


def test_motor_methods_are_noop_when_hardware_is_unavailable(monkeypatch: Any) -> None:
    """Команды моторов ничего не делают без hardware-библиотек."""
    monkeypatch.setattr(motor_module, "_HARDWARE_AVAILABLE", False)
    controller: MotorController = MotorController()

    controller.set_tracks(40, -40)
    controller.move_forward(50)
    controller.turn_left(50)
    controller.stop()
    controller.destroy()

    assert controller._pwm_motor is None
    assert controller._motor1 is None
    assert controller._motor2 is None


def test_move_forward_initializes_motors_and_applies_steering(monkeypatch: Any) -> None:
    """move_forward инициализирует PCA9685 и задает throttle с учетом steer."""
    _enable_fake_hardware(monkeypatch)
    controller: MotorController = MotorController(tl_left_offset=5, tl_right_offset=-5)

    controller.move_forward(speed_percent=50, steer_percent=10)

    pwm: FakePCA9685 = FakePCA9685.instances[0]
    motor1, motor2 = FakeDCMotor.instances
    assert FakeBusio.calls == [("scl", "sda")]
    assert pwm.address == controller.PCA9685_MOTOR_ADDRESS
    assert pwm.frequency == controller.PWM_FREQUENCY
    assert (motor1.in1, motor1.in2) == ("ch8", "ch9")
    assert (motor2.in1, motor2.in2) == ("ch10", "ch11")
    assert motor1.decay_mode == FakeMotorModule.SLOW_DECAY
    assert motor2.decay_mode == FakeMotorModule.SLOW_DECAY
    assert motor1.throttle == 0.35
    assert motor2.throttle == -0.65


def test_move_backward_reverses_track_direction(monkeypatch: Any) -> None:
    """move_backward инвертирует направление обеих гусениц."""
    _enable_fake_hardware(monkeypatch)
    controller: MotorController = MotorController()

    controller.move_backward(speed_percent=25)

    motor1, motor2 = FakeDCMotor.instances
    assert motor1.throttle == -0.25
    assert motor2.throttle == 0.25


def test_set_tracks_drives_left_and_right_tracks_independently(monkeypatch: Any) -> None:
    """set_tracks напрямую задает скорости со знаком для обоих бортов."""
    _enable_fake_hardware(monkeypatch)
    controller: MotorController = MotorController()

    controller.set_tracks(left_speed_percent=30, right_speed_percent=-45)

    motor1, motor2 = FakeDCMotor.instances
    assert motor1.throttle == -0.45
    assert motor2.throttle == -0.3


def test_set_tracks_clamps_signed_speed_range(monkeypatch: Any) -> None:
    """set_tracks ограничивает скорости со знаком диапазоном [-100; 100]."""
    _enable_fake_hardware(monkeypatch)
    controller: MotorController = MotorController()

    controller.set_tracks(left_speed_percent=-140, right_speed_percent=180)

    motor1, motor2 = FakeDCMotor.instances
    assert motor1.throttle == 1.0
    assert motor2.throttle == 1.0


def test_set_tracks_returns_when_setup_does_not_create_motors(monkeypatch: Any) -> None:
    """set_tracks завершается без записи, если setup не создал оба мотора."""
    _enable_fake_hardware(monkeypatch)
    controller: MotorController = MotorController()
    monkeypatch.setattr(controller, "_setup", lambda: None)

    controller.set_tracks(left_speed_percent=20, right_speed_percent=20)

    assert controller._motor1 is None
    assert controller._motor2 is None


def test_turn_methods_drive_tracks_in_opposite_directions(monkeypatch: Any) -> None:
    """Повороты на месте выставляют throttle для разворота."""
    _enable_fake_hardware(monkeypatch)
    controller: MotorController = MotorController()

    controller.turn_left(40)
    motor1, motor2 = FakeDCMotor.instances
    assert motor1.throttle == -0.4
    assert motor2.throttle == -0.4

    controller.turn_right(60)

    assert motor1.throttle == 0.6
    assert motor2.throttle == 0.6


def test_turn_returns_when_setup_does_not_create_motors(monkeypatch: Any) -> None:
    """turn_left завершается без throttle, если setup не создал оба мотора."""
    _enable_fake_hardware(monkeypatch)
    controller: MotorController = MotorController()
    monkeypatch.setattr(controller, "_setup", lambda: None)

    controller.turn_left(25)

    assert controller._motor1 is None
    assert controller._motor2 is None


def test_stop_sets_zero_throttle_for_both_motors(monkeypatch: Any) -> None:
    """stop выставляет нулевой throttle на оба мотора."""
    _enable_fake_hardware(monkeypatch)
    controller: MotorController = MotorController()
    controller.move_forward(80)

    controller.stop()

    motor1, motor2 = FakeDCMotor.instances
    assert motor1.throttle == 0.0
    assert motor2.throttle == 0.0


def test_destroy_deinitializes_pwm_and_clears_state(monkeypatch: Any) -> None:
    """destroy останавливает моторы, вызывает deinit и сбрасывает ссылки."""
    _enable_fake_hardware(monkeypatch)
    controller: MotorController = MotorController()
    controller.move_forward(80)
    pwm: FakePCA9685 = FakePCA9685.instances[0]

    controller.destroy()

    assert pwm.deinit_called is True
    assert controller._pwm_motor is None
    assert controller._motor1 is None
    assert controller._motor2 is None
    assert controller._is_initialized is False


def test_destroy_clears_state_when_deinit_fails(monkeypatch: Any) -> None:
    """destroy сбрасывает state даже при ошибке deinit."""
    _enable_fake_hardware(monkeypatch, BrokenDeinitPCA9685)
    controller: MotorController = MotorController()
    controller.move_forward(50)

    controller.destroy()

    assert controller._pwm_motor is None
    assert controller._motor1 is None
    assert controller._motor2 is None
    assert controller._is_initialized is False


def test_setup_failure_leaves_controller_uninitialized(monkeypatch: Any) -> None:
    """Ошибка setup оставляет контроллер моторов неинициализированным."""
    _enable_fake_hardware(monkeypatch, BrokenPCA9685)
    controller: MotorController = MotorController()

    controller.move_forward(50)

    assert controller._pwm_motor is None
    assert controller._motor1 is None
    assert controller._motor2 is None
    assert controller._is_initialized is False
