from __future__ import annotations

from src.application.services.l1_models import L1SensorState, L1TrackCommand
from src.application.services.l1_service import L1Service


class FakeMotor:
    """Заглушка моторов для нового нижнего уровня."""

    def __init__(self) -> None:
        """Подготовить хранилище команд и событий."""
        self.track_commands: list[tuple[int, int]] = []
        self.stop_calls: int = 0
        self.destroy_calls: int = 0

    def set_tracks(self, left_speed_percent: int, right_speed_percent: int) -> None:
        """Сохранить команду левого и правого борта."""
        self.track_commands.append((left_speed_percent, right_speed_percent))

    def stop(self) -> None:
        """Зафиксировать остановку."""
        self.stop_calls += 1

    def destroy(self) -> None:
        """Зафиксировать освобождение ресурсов."""
        self.destroy_calls += 1


class FakeGyroscope:
    """Заглушка IMU для нового нижнего уровня."""

    def __init__(self) -> None:
        """Подготовить фиксированные данные датчика."""
        self.start_calls: list[bool] = []
        self.stop_calls: int = 0
        self.destroy_calls: int = 0

    def start(self, calibrate: bool = True) -> None:
        """Зафиксировать запуск IMU."""
        self.start_calls.append(calibrate)

    def stop(self) -> None:
        """Зафиксировать остановку IMU."""
        self.stop_calls += 1

    def get_angular_speed_z_deg_per_sec(self) -> float:
        """Вернуть текущую угловую скорость."""
        return 12.5

    def get_acceleration_xyz_m_s2(self) -> tuple[float, float, float]:
        """Вернуть ускорения по трём осям."""
        return 1.0, 2.0, 3.0

    def destroy(self) -> None:
        """Зафиксировать освобождение ресурсов."""
        self.destroy_calls += 1


class FakeUltrasonic:
    """Заглушка дальномера."""

    def __init__(self) -> None:
        """Подготовить счётчик освобождения ресурсов."""
        self.destroy_calls: int = 0

    def measure_distance_cm(self) -> float:
        """Вернуть расстояние до препятствия."""
        return 42.0

    def destroy(self) -> None:
        """Зафиксировать освобождение ресурсов."""
        self.destroy_calls += 1


class FakeHeadServo:
    """Заглушка сервопривода головы."""

    def __init__(self) -> None:
        """Подготовить историю углов и счётчик destroy."""
        self.angles: list[float] = []
        self.destroy_calls: int = 0

    def set_angle(self, angle_deg: float) -> None:
        """Сохранить угол."""
        self.angles.append(angle_deg)

    def destroy(self) -> None:
        """Зафиксировать освобождение ресурсов."""
        self.destroy_calls += 1


def _service(
    *, with_head_servo: bool = True
) -> tuple[
    L1Service,
    FakeMotor,
    FakeGyroscope,
    FakeUltrasonic,
    FakeHeadServo | None,
]:
    """Создать чистый сервис нижнего уровня с заглушками."""
    motor: FakeMotor = FakeMotor()
    gyroscope: FakeGyroscope = FakeGyroscope()
    ultrasonic: FakeUltrasonic = FakeUltrasonic()
    head_servo: FakeHeadServo | None = FakeHeadServo() if with_head_servo else None
    service = L1Service(
        motor_controller=motor,  # type: ignore[arg-type]
        gyroscope=gyroscope,  # type: ignore[arg-type]
        ultrasonic_sensor=ultrasonic,  # type: ignore[arg-type]
        head_servo=head_servo,  # type: ignore[arg-type]
    )
    return service, motor, gyroscope, ultrasonic, head_servo


def test_apply_track_command_directly_passes_values_to_motor() -> None:
    """Новый L1 передаёт команды бортов прямо в моторы."""
    service, motor, _, _, _ = _service()

    service.apply_track_command(L1TrackCommand(left_percent=17, right_percent=-19))

    assert motor.track_commands == [(17, -19)]


def test_read_sensors_returns_only_measured_values() -> None:
    """Новый L1 возвращает только измеренные данные датчиков."""
    service, _, _, _, _ = _service()

    state: L1SensorState = service.read_sensors()

    assert state.angular_speed_z_deg_per_sec == 12.5
    assert state.accel_x_m_s2 == 1.0
    assert state.accel_y_m_s2 == 2.0
    assert state.accel_z_m_s2 == 3.0
    assert state.distance_cm == 42.0


def test_l1_service_controls_imu_head_and_destroy() -> None:
    """Новый L1 умеет запускать IMU, управлять головой и освобождать ресурсы."""
    service, motor, gyroscope, ultrasonic, head_servo = _service()

    service.start_imu(calibrate=False)
    service.stop_imu()
    service.set_head_angle(15.0)
    service.stop_motion()
    service.destroy()

    assert gyroscope.start_calls == [False]
    assert gyroscope.stop_calls == 1
    assert head_servo is not None
    assert head_servo.angles == [15.0]
    assert motor.stop_calls == 1
    assert motor.destroy_calls == 1
    assert gyroscope.destroy_calls == 1
    assert ultrasonic.destroy_calls == 1
    assert head_servo.destroy_calls == 1


def test_set_head_angle_is_noop_without_servo() -> None:
    """Если сервопривода нет, установка угла просто пропускается."""
    service, _, _, _, _ = _service(with_head_servo=False)

    service.set_head_angle(30.0)


def test_destroy_skips_devices_when_release_disabled() -> None:
    """destroy(release_devices=False) не вызывает destroy у железа."""
    service, motor, gyro, ultrasonic, head_servo = _service()

    service.destroy(release_devices=False)

    assert motor.destroy_calls == 0
    assert gyro.destroy_calls == 0
    assert ultrasonic.destroy_calls == 0
    assert head_servo is not None
    assert head_servo.destroy_calls == 0
