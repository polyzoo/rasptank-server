from __future__ import annotations

from typing import Callable

import pytest

from src.infrastructures import imu as imu_module
from src.infrastructures.imu import IMUSensor


class FakeBus:
    """Заглушка SMBus."""

    def __init__(self, bus_num: int) -> None:
        """Сохранить номер I2C-шины."""
        self.bus_num: int = bus_num
        self.write_calls: list[tuple[int, int, int]] = []
        self.read_responses: dict[tuple[int, int, int], list[int] | Exception] = {}
        self.close_called: bool = False

    def write_byte_data(self, address: int, register: int, value: int) -> None:
        """Сохранить запись байта в регистр."""
        self.write_calls.append((address, register, value))

    def read_i2c_block_data(self, address: int, register: int, count: int) -> list[int]:
        """Вернуть подготовленный ответ чтения или выбросить ошибку."""
        response: list[int] | Exception = self.read_responses[(address, register, count)]
        if isinstance(response, Exception):
            raise response
        return response

    def close(self) -> None:
        """Зафиксировать закрытие шины."""
        self.close_called = True


class BrokenCloseBus(FakeBus):
    """SMBus-заглушка, которая падает при close."""

    def close(self) -> None:
        """Сымитировать ошибку закрытия шины."""
        raise RuntimeError("close failed")


class FakeSMBusModule:
    """Заглушка модуля smbus2."""

    last_bus: FakeBus | None = None
    bus_cls: type[FakeBus] = FakeBus

    @classmethod
    def SMBus(cls, bus_num: int) -> FakeBus:
        """Создать fake SMBus."""
        cls.last_bus = cls.bus_cls(bus_num)
        return cls.last_bus


class BrokenSMBusModule:
    """Заглушка smbus2, которая падает при создании шины."""

    @staticmethod
    def SMBus(bus_num: int) -> FakeBus:
        """Сымитировать ошибку I2C."""
        raise OSError("i2c failed")


class FakeThread:
    """Заглушка Thread без фонового выполнения target."""

    instances: list["FakeThread"] = []

    def __init__(self, *, target: Callable[[], None], daemon: bool) -> None:
        """Сохранить параметры потока."""
        self.target: Callable[[], None] = target
        self.daemon: bool = daemon
        self.started: bool = False
        self.join_timeout: float | None = None
        FakeThread.instances.append(self)

    def start(self) -> None:
        """Зафиксировать старт потока без запуска target."""
        self.started = True

    def join(self, *, timeout: float) -> None:
        """Зафиксировать timeout ожидания потока."""
        self.join_timeout = timeout


def _enable_fake_hardware(
    monkeypatch: pytest.MonkeyPatch,
    smbus_module: object = FakeSMBusModule,
) -> None:
    """Подключить fake smbus2 зависимости к модулю imu."""
    FakeSMBusModule.last_bus = None
    FakeSMBusModule.bus_cls = FakeBus
    monkeypatch.setattr(imu_module, "_HARDWARE_AVAILABLE", True)
    monkeypatch.setattr(imu_module, "smbus2", smbus_module)
    monkeypatch.setattr(imu_module.time, "sleep", lambda seconds: None)


def test_start_is_noop_when_hardware_is_unavailable(monkeypatch: pytest.MonkeyPatch) -> None:
    """start ничего не делает без hardware-библиотек."""
    monkeypatch.setattr(imu_module, "_HARDWARE_AVAILABLE", False)
    sensor: IMUSensor = IMUSensor()

    sensor.start()

    assert sensor._is_initialized is False
    assert sensor._update_thread is None


def test_setup_wakes_mpu6050(monkeypatch: pytest.MonkeyPatch) -> None:
    """_setup открывает SMBus и будит MPU6050."""
    _enable_fake_hardware(monkeypatch)
    sensor: IMUSensor = IMUSensor(bus_num=7)

    sensor._setup()

    assert FakeSMBusModule.last_bus is not None
    assert FakeSMBusModule.last_bus.bus_num == 7
    assert FakeSMBusModule.last_bus.write_calls == [
        (sensor.I2C_ADDRESS, sensor.REG_PWR_MGMT_1, 0),
    ]
    assert sensor._is_initialized is True


def test_setup_failure_keeps_sensor_uninitialized(monkeypatch: pytest.MonkeyPatch) -> None:
    """Ошибка I2C оставляет IMU неинициализированным."""
    _enable_fake_hardware(monkeypatch, BrokenSMBusModule)
    sensor: IMUSensor = IMUSensor()

    sensor._setup()

    assert sensor._is_initialized is False


def test_setup_is_noop_when_hardware_is_unavailable(monkeypatch: pytest.MonkeyPatch) -> None:
    """_setup ничего не делает без hardware-библиотек."""
    monkeypatch.setattr(imu_module, "_HARDWARE_AVAILABLE", False)
    sensor: IMUSensor = IMUSensor()

    sensor._setup()

    assert sensor._bus is None
    assert sensor._is_initialized is False


def test_start_returns_when_setup_fails(monkeypatch: pytest.MonkeyPatch) -> None:
    """start не запускает поток, если setup не инициализировал IMU."""
    _enable_fake_hardware(monkeypatch, BrokenSMBusModule)
    monkeypatch.setattr(imu_module, "Thread", FakeThread)
    FakeThread.instances = []
    sensor: IMUSensor = IMUSensor()

    sensor.start()

    assert sensor._is_initialized is False
    assert FakeThread.instances == []


def test_start_resets_state_and_starts_update_thread(monkeypatch: pytest.MonkeyPatch) -> None:
    """start сбрасывает state, опционально калибрует и запускает поток обновления."""
    _enable_fake_hardware(monkeypatch)
    FakeThread.instances = []
    monkeypatch.setattr(imu_module, "Thread", FakeThread)
    monkeypatch.setattr(imu_module.time, "monotonic", lambda: 123.0)
    sensor: IMUSensor = IMUSensor()
    sensor._yaw = 99.0
    sensor._gyro_z_deg_per_sec = 9.0
    sensor._accel_x_m_s2 = 1.0
    sensor._accel_y_m_s2 = 2.0
    sensor._accel_z_m_s2 = 3.0
    calibrate_calls: list[int | None] = []
    monkeypatch.setattr(sensor, "calibrate", lambda samples=None: calibrate_calls.append(samples))

    sensor.start(calibrate=True)

    assert calibrate_calls == [None]
    assert sensor.get_yaw() == 0.0
    assert sensor._last_update_time == 123.0
    assert isinstance(sensor._update_thread, FakeThread)
    assert sensor._update_thread.started is True


def test_stop_joins_running_thread(monkeypatch: pytest.MonkeyPatch) -> None:
    """stop останавливает event, ожидает поток и очищает ссылку."""
    FakeThread.instances = []
    thread: FakeThread = FakeThread(target=lambda: None, daemon=True)
    sensor: IMUSensor = IMUSensor()
    sensor._update_thread = thread  # type: ignore[assignment]

    sensor.stop()

    assert thread.join_timeout == sensor.THREAD_JOIN_TIMEOUT_SEC
    assert sensor._update_thread is None
    assert sensor._stop_event.is_set() is False


def test_calibrate_sets_gyro_bias(monkeypatch: pytest.MonkeyPatch) -> None:
    """calibrate усредняет сырые значения gyro_z."""
    sensor: IMUSensor = IMUSensor()
    sensor._is_initialized = True
    values: list[float] = [10.0, 20.0, 30.0]
    monkeypatch.setattr(sensor, "_read_raw_gyro_z", lambda: values.pop(0))
    monkeypatch.setattr(imu_module.time, "sleep", lambda seconds: None)

    sensor.calibrate(samples=3)

    assert sensor._gyro_z_bias == 20.0


def test_calibrate_is_noop_before_setup() -> None:
    """calibrate ничего не меняет до инициализации."""
    sensor: IMUSensor = IMUSensor()

    sensor.calibrate(samples=1)

    assert sensor._gyro_z_bias == 0.0


def test_angle_speed_and_acceleration_getters_are_thread_safe() -> None:
    """Методы чтения состояния возвращают актуальные угол, скорость и ускорение."""
    sensor: IMUSensor = IMUSensor()
    sensor._yaw = 14.0
    sensor._gyro_z_deg_per_sec = -2.0
    sensor._accel_x_m_s2 = 1.0
    sensor._accel_y_m_s2 = 2.0
    sensor._accel_z_m_s2 = 3.0

    assert sensor.get_yaw() == 14.0
    assert sensor.get_angular_speed_z_deg_per_sec() == -2.0
    assert sensor.get_acceleration_xyz_m_s2() == (1.0, 2.0, 3.0)

    sensor.reset_yaw()
    assert sensor.get_yaw() == 0.0


def test_raw_gyro_reader_decodes_signed_values() -> None:
    """_read_raw_gyro_z декодирует signed int16 и возвращает 0 при ошибке."""
    sensor: IMUSensor = IMUSensor()
    assert sensor._read_raw_gyro_z() == 0.0
    bus: FakeBus = FakeBus(bus_num=1)
    sensor._bus = bus
    sensor._is_initialized = True

    bus.read_responses[(sensor.I2C_ADDRESS, sensor.REG_GYRO_ZOUT_H, sensor.BYTES_PER_VALUE)] = [
        0x00,
        0x10,
    ]
    assert sensor._read_raw_gyro_z() == 16.0

    bus.read_responses[(sensor.I2C_ADDRESS, sensor.REG_GYRO_ZOUT_H, sensor.BYTES_PER_VALUE)] = [
        0xFF,
        0xFE,
    ]
    assert sensor._read_raw_gyro_z() == -2.0

    bus.read_responses[(sensor.I2C_ADDRESS, sensor.REG_GYRO_ZOUT_H, sensor.BYTES_PER_VALUE)] = (
        RuntimeError("read failed")
    )
    assert sensor._read_raw_gyro_z() == 0.0


def test_raw_accel_reader_decodes_axes() -> None:
    """_read_raw_accel_xyz декодирует три signed int16 значения."""
    sensor: IMUSensor = IMUSensor()
    assert sensor._read_raw_accel_xyz() == (0.0, 0.0, 0.0)
    bus: FakeBus = FakeBus(bus_num=1)
    sensor._bus = bus
    sensor._is_initialized = True

    bus.read_responses[(sensor.I2C_ADDRESS, sensor.REG_ACCEL_XOUT_H, sensor.ACCEL_BYTES_COUNT)] = [
        0x00,
        0x01,
        0xFF,
        0xFF,
        0x7F,
        0xFF,
    ]
    assert sensor._read_raw_accel_xyz() == (1.0, -1.0, 32767.0)
    assert sensor._decode_int16(0x80, 0x00) == -32768.0

    bus.read_responses[(sensor.I2C_ADDRESS, sensor.REG_ACCEL_XOUT_H, sensor.ACCEL_BYTES_COUNT)] = (
        RuntimeError("read failed")
    )
    assert sensor._read_raw_accel_xyz() == (0.0, 0.0, 0.0)


def test_update_loop_integrates_gyro_and_accel(monkeypatch: pytest.MonkeyPatch) -> None:
    """_update_loop обновляет yaw, gyro и ускорения за одну итерацию."""
    sensor: IMUSensor = IMUSensor()
    sensor._is_initialized = True
    sensor._last_update_time = 10.0
    sensor._gyro_z_bias = 1.0
    monkeypatch.setattr(imu_module.time, "monotonic", lambda: 11.0)
    monkeypatch.setattr(sensor, "_read_raw_gyro_z", lambda: 132.0)
    monkeypatch.setattr(sensor, "_read_raw_accel_xyz", lambda: (16384.0, 0.0, -16384.0))

    def stop_after_sleep(seconds: float) -> None:
        """Остановить цикл после первой итерации."""
        sensor._stop_event.set()

    monkeypatch.setattr(imu_module.time, "sleep", stop_after_sleep)

    sensor._update_loop()

    assert sensor.get_yaw() == -1.0


def test_destroy_closes_bus_and_clears_state(monkeypatch: pytest.MonkeyPatch) -> None:
    """destroy останавливает поток, закрывает шину и сбрасывает initialized."""
    _enable_fake_hardware(monkeypatch)
    sensor: IMUSensor = IMUSensor()
    sensor._setup()
    bus: FakeBus = FakeSMBusModule.last_bus
    assert bus is not None

    sensor.destroy()

    assert bus.close_called is True
    assert sensor._bus is None
    assert sensor._is_initialized is False

    sensor._setup()
    bus_counted_once: FakeBus = FakeSMBusModule.last_bus
    assert bus_counted_once is not None
    sensor._setup()

    assert FakeSMBusModule.last_bus is bus_counted_once


def test_destroy_clears_bus_when_close_fails(monkeypatch: pytest.MonkeyPatch) -> None:
    """destroy очищает bus даже при ошибке close."""
    _enable_fake_hardware(monkeypatch)
    FakeSMBusModule.bus_cls = BrokenCloseBus
    sensor: IMUSensor = IMUSensor()
    sensor._setup()

    sensor.destroy()

    assert sensor._bus is None
    assert sensor._is_initialized is False
