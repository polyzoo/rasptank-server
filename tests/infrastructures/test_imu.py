from __future__ import annotations

from collections.abc import Callable
from typing import Any

import pytest

from src.infrastructures import imu as imu_module
from src.infrastructures.imu import IMUSensor


def _i16_be(v: int) -> list[int]:
    """Два байта big-endian для signed int16 (для пакета MPU6050)."""
    iv = int(v) & 0xFFFF
    return [iv >> 8, iv & 0xFF]


def _burst14(
    ax: int = 0,
    ay: int = 0,
    az: int = 0,
    gx: int = 0,
    gy: int = 0,
    gz: int = 0,
) -> list[int]:
    """14 байт: accel xyz, temp, gyro xyz."""
    return (
        _i16_be(ax) + _i16_be(ay) + _i16_be(az) + [0, 0] + _i16_be(gx) + _i16_be(gy) + _i16_be(gz)
    )


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
        """Создать заглушку SMBus."""
        cls.last_bus = cls.bus_cls(bus_num)
        return cls.last_bus


class BrokenSMBusModule:
    """Заглушка smbus2, которая падает при создании шины."""

    @staticmethod
    def SMBus(bus_num: int) -> FakeBus:
        """Сымитировать ошибку I2C."""
        raise OSError("i2c failed")


class FakeThread:
    """Заглушка Thread без фонового выполнения функции."""

    instances: list["FakeThread"] = []

    def __init__(self, *, target: Callable[[], None], daemon: bool) -> None:
        """Сохранить параметры потока."""
        self.target: Callable[[], None] = target
        self.daemon: bool = daemon
        self.started: bool = False
        self.join_timeout: float | None = None
        self._alive: bool = False
        FakeThread.instances.append(self)

    def start(self) -> None:
        """Зафиксировать старт потока без запуска target."""
        self.started = True
        self._alive = True

    def is_alive(self) -> bool:
        """Имитация threading.Thread.is_alive для повторного IMUSensor.start()."""
        return self._alive

    def join(self, *, timeout: float) -> None:
        """Зафиксировать timeout ожидания потока."""
        self.join_timeout = timeout
        self._alive = False


def _enable_fake_hardware(
    monkeypatch: pytest.MonkeyPatch,
    smbus_module: object = FakeSMBusModule,
) -> None:
    """Подключить аппаратные заглушки smbus2 к модулю imu."""
    FakeSMBusModule.last_bus = None
    FakeSMBusModule.bus_cls = FakeBus
    monkeypatch.setattr(imu_module, "_HARDWARE_AVAILABLE", True)
    monkeypatch.setattr(imu_module, "smbus2", smbus_module)
    monkeypatch.setattr(imu_module.time, "sleep", lambda seconds: None)


def test_start_is_noop_when_hardware_is_unavailable(monkeypatch: pytest.MonkeyPatch) -> None:
    """start ничего не делает без аппаратных библиотек."""
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
        (sensor.I2C_ADDRESS, sensor.REG_DLPF_CFG, sensor._mpu6050_dlpf_cfg),
        (sensor.I2C_ADDRESS, sensor.REG_SMPLRT_DIV, sensor._mpu6050_smplrt_div),
        (sensor.I2C_ADDRESS, sensor.REG_GYRO_CONFIG, 0x00),
        (sensor.I2C_ADDRESS, sensor.REG_ACCEL_CONFIG, 0x00),
    ]
    assert sensor._is_initialized is True


def test_setup_failure_keeps_sensor_uninitialized(monkeypatch: pytest.MonkeyPatch) -> None:
    """Ошибка I2C оставляет IMU неинициализированным."""
    _enable_fake_hardware(monkeypatch, BrokenSMBusModule)
    sensor: IMUSensor = IMUSensor()

    sensor._setup()

    assert sensor._is_initialized is False


def test_setup_is_noop_when_hardware_is_unavailable(monkeypatch: pytest.MonkeyPatch) -> None:
    """_setup ничего не делает без аппаратных библиотек."""
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


def test_second_start_skips_recalibrate_when_thread_still_alive(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Повторный start(calibrate=True) при живом потоке не вызывает calibrate снова."""
    _enable_fake_hardware(monkeypatch)
    FakeThread.instances = []
    monkeypatch.setattr(imu_module, "Thread", FakeThread)
    monkeypatch.setattr(imu_module.time, "monotonic", lambda: 0.0)
    sensor: IMUSensor = IMUSensor()
    calibrate_calls: list[object | None] = []
    monkeypatch.setattr(sensor, "calibrate", lambda samples=None: calibrate_calls.append(samples))

    sensor.start(calibrate=True)
    sensor.start(calibrate=True)

    assert calibrate_calls == [None]


def test_start_resets_state_and_starts_update_thread(monkeypatch: pytest.MonkeyPatch) -> None:
    """start сбрасывает состояние, калибрует и запускает поток обновления."""
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
    """calibrate усредняет сырые значения гироскопа Z."""
    sensor: IMUSensor = IMUSensor()
    sensor._is_initialized = True
    bursts: list[tuple[int, int, int, int, int, int]] = [
        (0, 0, 16384, 0, 0, 10),
        (0, 0, 16384, 0, 0, 20),
        (0, 0, 16384, 0, 0, 30),
    ]

    def next_burst() -> tuple[float, float, float, float, float, float]:
        ax, ay, az, gx, gy, gz = bursts.pop(0)
        return float(ax), float(ay), float(az), float(gx), float(gy), float(gz)

    monkeypatch.setattr(sensor, "_read_burst_raw_int16", next_burst)
    monkeypatch.setattr(imu_module.time, "sleep", lambda seconds: None)

    sensor.calibrate(samples=3)

    assert sensor._gyro_bias_raw_z == 20.0
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
    """_read_raw_gyro_z читает Z из пакета MPU6050 и возвращает 0 при ошибке."""
    sensor: IMUSensor = IMUSensor()
    assert sensor._read_raw_gyro_z() == 0.0
    bus: FakeBus = FakeBus(bus_num=1)
    sensor._bus = bus
    sensor._is_initialized = True

    key = (sensor.I2C_ADDRESS, sensor.REG_ACCEL_XOUT_H, sensor.MPU6050_BURST_BYTES)
    bus.read_responses[key] = _burst14(gz=16)
    assert sensor._read_raw_gyro_z() == 16.0

    bus.read_responses[key] = _burst14(gz=-2)
    assert sensor._read_raw_gyro_z() == -2.0

    bus.read_responses[key] = RuntimeError("read failed")
    assert sensor._read_raw_gyro_z() == 0.0


def test_raw_accel_reader_decodes_axes() -> None:
    """_read_raw_accel_xyz декодирует три signed int16 из пакета MPU6050."""
    sensor: IMUSensor = IMUSensor()
    assert sensor._read_raw_accel_xyz() == (0.0, 0.0, 0.0)
    bus: FakeBus = FakeBus(bus_num=1)
    sensor._bus = bus
    sensor._is_initialized = True

    key = (sensor.I2C_ADDRESS, sensor.REG_ACCEL_XOUT_H, sensor.MPU6050_BURST_BYTES)
    bus.read_responses[key] = _burst14(ax=1, ay=-1, az=32767)
    assert sensor._read_raw_accel_xyz() == (1.0, -1.0, 32767.0)
    assert sensor._decode_int16(0x80, 0x00) == -32768.0

    bus.read_responses[key] = RuntimeError("read failed")
    assert sensor._read_raw_accel_xyz() == (0.0, 0.0, 0.0)


def test_update_loop_integrates_gyro_and_accel(monkeypatch: pytest.MonkeyPatch) -> None:
    """_update_loop обновляет yaw, gyro и ускорения за одну итерацию."""
    sensor: IMUSensor = IMUSensor(ekf_enabled=False)
    sensor._is_initialized = True
    sensor._last_update_time = 10.0
    sensor._gyro_bias_raw_z = 1.0
    sensor._gyro_z_bias = 1.0
    monkeypatch.setattr(imu_module.time, "monotonic", lambda: 11.0)
    monkeypatch.setattr(
        sensor,
        "_read_burst_raw_int16",
        lambda: (16384.0, 0.0, -16384.0, 0.0, 0.0, 132.0),
    )

    def stop_after_sleep(seconds: float) -> None:
        """Остановить цикл после первой итерации."""
        sensor._stop_event.set()

    monkeypatch.setattr(imu_module.time, "sleep", stop_after_sleep)

    sensor._update_loop()

    assert sensor.get_yaw() == -1.0


def test_update_loop_deadband_suppresses_small_yaw_rate(monkeypatch: pytest.MonkeyPatch) -> None:
    """Малая угловая скорость ниже порога не интегрирует yaw."""
    sensor: IMUSensor = IMUSensor(
        ekf_enabled=False,
        gyro_yaw_integration_deadband_deg_per_sec=1.0,
    )
    sensor._is_initialized = True
    sensor._last_update_time = 10.0
    sensor._gyro_bias_raw_z = 0.0
    sensor._gyro_z_bias = 0.0
    monkeypatch.setattr(imu_module.time, "monotonic", lambda: 11.0)
    # Значение около -0.5 град/с остаётся внутри порога 1 град/с.
    monkeypatch.setattr(
        sensor,
        "_read_burst_raw_int16",
        lambda: (0.0, 0.0, 0.0, 0.0, 0.0, 65.0),
    )

    def stop_after_sleep(seconds: float) -> None:
        sensor._stop_event.set()

    monkeypatch.setattr(imu_module.time, "sleep", stop_after_sleep)

    sensor._update_loop()

    assert sensor.get_yaw() == 0.0
    assert sensor.get_angular_speed_z_deg_per_sec() == 0.0


def test_destroy_closes_bus_and_clears_state(monkeypatch: pytest.MonkeyPatch) -> None:
    """destroy останавливает поток, закрывает шину и сбрасывает состояние."""
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
    """destroy очищает шину даже при ошибке close."""
    _enable_fake_hardware(monkeypatch)
    FakeSMBusModule.bus_cls = BrokenCloseBus
    sensor: IMUSensor = IMUSensor()
    sensor._setup()

    sensor.destroy()

    assert sensor._bus is None
    assert sensor._is_initialized is False


def test_start_invokes_ekf_reset_when_enabled(monkeypatch: pytest.MonkeyPatch) -> None:
    """При ekf_enabled start() сбрасывает фильтр перед калибровкой/циклом."""
    _enable_fake_hardware(monkeypatch)
    FakeThread.instances = []
    monkeypatch.setattr(imu_module, "Thread", FakeThread)
    monkeypatch.setattr(imu_module.time, "monotonic", lambda: 0.0)
    sensor: IMUSensor = IMUSensor(ekf_enabled=True)
    assert sensor._ekf is not None
    reset_calls: list[bool] = []
    original_reset: Callable[[], None] = sensor._ekf.reset

    def wrapped_reset() -> None:
        reset_calls.append(True)
        original_reset()

    monkeypatch.setattr(sensor._ekf, "reset", wrapped_reset)
    monkeypatch.setattr(sensor, "calibrate", lambda samples=None: None)
    sensor.start(calibrate=False)
    assert reset_calls == [True]


def test_calibrate_with_ekf_calls_init_from_calibration(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """calibrate передаёт home углы в EKF при включённом фильтре."""
    sensor: IMUSensor = IMUSensor(ekf_enabled=True)
    sensor._is_initialized = True
    init_calls: list[Any] = []

    def capture_init(*args: object, **kwargs: object) -> None:
        init_calls.append((args, kwargs))

    monkeypatch.setattr(sensor._ekf, "init_from_calibration", capture_init)

    def one_burst() -> tuple[float, float, float, float, float, float]:
        return (0.0, 0.0, 16384.0, 0.0, 0.0, 0.0)

    monkeypatch.setattr(sensor, "_read_burst_raw_int16", one_burst)
    monkeypatch.setattr(imu_module.time, "sleep", lambda seconds: None)
    sensor.calibrate(samples=1)
    assert len(init_calls) == 1


def test_reset_yaw_with_ekf_calls_init_from_calibration(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """reset_yaw переинициализирует EKF текущими home roll/pitch."""
    sensor: IMUSensor = IMUSensor(ekf_enabled=True)
    captured: list[tuple[float, float, float, tuple[float, float, float]]] = []

    def capture(
        roll: float,
        pitch: float,
        yaw: float,
        gyro_bias: tuple[float, float, float],
    ) -> None:
        captured.append((roll, pitch, yaw, gyro_bias))

    monkeypatch.setattr(sensor._ekf, "init_from_calibration", capture)
    sensor._home_roll_rad = 0.12
    sensor._home_pitch_rad = -0.34
    sensor.reset_yaw()
    assert captured[0][0] == pytest.approx(0.12)
    assert captured[0][1] == pytest.approx(-0.34)
    assert captured[0][2] == 0.0


def test_apply_ema_second_iteration_blends_values() -> None:
    """После первого кадра EMA смешивает новые измерения с предыдущими."""
    sensor: IMUSensor = IMUSensor(
        ekf_enabled=False,
        accel_ema_alpha=0.2,
        gyro_ema_alpha=0.3,
    )
    first: tuple[float, float, float, float, float, float] = sensor._apply_ema(
        1.0,
        2.0,
        3.0,
        0.0,
        0.0,
        10.0,
    )
    assert first == (1.0, 2.0, 3.0, 0.0, 0.0, 10.0)
    second: tuple[float, float, float, float, float, float] = sensor._apply_ema(
        3.0,
        0.0,
        0.0,
        0.0,
        0.0,
        20.0,
    )
    assert second[0] == pytest.approx(0.2 * 3.0 + 0.8 * 1.0)
    assert second[5] == pytest.approx(0.3 * 20.0 + 0.7 * 10.0)


def test_update_loop_second_tick_applies_ema_smoothing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Вторая итерация цикла проходит по ветке сглаживания EMA."""
    sensor: IMUSensor = IMUSensor(ekf_enabled=False, accel_ema_alpha=0.2, gyro_ema_alpha=0.5)
    sensor._is_initialized = True
    sensor._last_update_time = 10.0
    time_seq: list[float] = [11.0, 12.0]
    monkeypatch.setattr(imu_module.time, "monotonic", time_seq.pop)

    bursts: list[tuple[float, float, float, float, float, float]] = [
        (16384.0, 0.0, -16384.0, 0.0, 0.0, 0.0),
        (0.0, 16384.0, -16384.0, 0.0, 0.0, 0.0),
    ]

    def next_burst() -> tuple[float, float, float, float, float, float]:
        return bursts.pop(0)

    monkeypatch.setattr(sensor, "_read_burst_raw_int16", next_burst)
    sleep_count: int = 0

    def stop_after_two_sleeps(seconds: float) -> None:
        nonlocal sleep_count
        sleep_count += 1
        if sleep_count >= 2:
            sensor._stop_event.set()

    monkeypatch.setattr(imu_module.time, "sleep", stop_after_two_sleeps)
    sensor._update_loop()
    ax1, ay1, _ = sensor.get_acceleration_xyz_m_s2()
    assert abs(ax1 - ay1) > 1e-3


def test_update_loop_with_ekf_sets_yaw_from_filter(monkeypatch: pytest.MonkeyPatch) -> None:
    """Ветка EKF обновляет yaw через оценку фильтра."""
    sensor: IMUSensor = IMUSensor(ekf_enabled=True)
    sensor._is_initialized = True
    sensor._last_update_time = 10.0
    sensor._gyro_bias_raw_z = 0.0
    monkeypatch.setattr(imu_module.time, "monotonic", lambda: 11.0)
    monkeypatch.setattr(
        sensor,
        "_read_burst_raw_int16",
        lambda: (0.0, 0.0, 16384.0, 0.0, 0.0, 0.0),
    )

    def stop_after_sleep(seconds: float) -> None:
        sensor._stop_event.set()

    monkeypatch.setattr(imu_module.time, "sleep", stop_after_sleep)
    sensor._update_loop()
    assert isinstance(sensor.get_yaw(), float)


def test_update_loop_ekf_skips_predict_when_dt_zero(monkeypatch: pytest.MonkeyPatch) -> None:
    """При нулевом dt predict не вызывается, update — да."""
    sensor: IMUSensor = IMUSensor(ekf_enabled=True)
    sensor._is_initialized = True
    sensor._last_update_time = 11.0
    monkeypatch.setattr(imu_module.time, "monotonic", lambda: 11.0)
    predict_calls: list[bool] = []
    original_predict: Any = sensor._ekf.predict

    def traced_predict(*args: object, **kwargs: object) -> None:
        predict_calls.append(True)
        original_predict(*args, **kwargs)

    monkeypatch.setattr(sensor._ekf, "predict", traced_predict)
    monkeypatch.setattr(
        sensor,
        "_read_burst_raw_int16",
        lambda: (0.0, 0.0, 16384.0, 0.0, 0.0, 0.0),
    )

    def stop_after_sleep(seconds: float) -> None:
        sensor._stop_event.set()

    monkeypatch.setattr(imu_module.time, "sleep", stop_after_sleep)
    sensor._update_loop()
    assert predict_calls == []
