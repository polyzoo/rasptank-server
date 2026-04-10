from __future__ import annotations

import pytest

from src.application.services import linear_motion_executor as linear_module
from src.application.services.linear_motion_executor import (
    LinearMotionExecutionResult,
    LinearMotionExecutor,
)
from src.application.services.motion_lifecycle import MotionLifecycle


class FakeMotor:
    """Заглушка контроллера моторов."""

    def __init__(self) -> None:
        """Подготовить счетчик stop."""
        self.stop_calls: int = 0

    def stop(self) -> None:
        """Зафиксировать остановку."""
        self.stop_calls += 1


class FakeUltrasonic:
    """Заглушка ультразвукового датчика."""

    def __init__(self, distance_cm: float = 999.0) -> None:
        """Сохранить возвращаемую дистанцию."""
        self.distance_cm: float = distance_cm

    def measure_distance_cm(self) -> float:
        """Вернуть fake дистанцию."""
        return self.distance_cm


class FakeGyroscope:
    """Заглушка гироскопа."""

    def __init__(self, yaw: float = 0.0) -> None:
        """Сохранить текущий yaw."""
        self.yaw: float = yaw

    def get_yaw(self) -> float:
        """Вернуть текущий yaw."""
        return self.yaw


def _executor(
    *,
    motor: FakeMotor | None = None,
    ultrasonic: FakeUltrasonic | None = None,
    gyroscope: FakeGyroscope | None = None,
    lifecycle: MotionLifecycle | None = None,
) -> LinearMotionExecutor:
    """Создать LinearMotionExecutor с fake зависимостями."""
    return LinearMotionExecutor(
        motor_controller=motor or FakeMotor(),  # type: ignore[arg-type]
        ultrasonic_sensor=ultrasonic or FakeUltrasonic(),  # type: ignore[arg-type]
        gyroscope=gyroscope or FakeGyroscope(),  # type: ignore[arg-type]
        min_obstacle_distance_cm=20.0,
        deceleration_distance_cm=40.0,
        max_speed_cm_per_sec=100.0,
        update_interval_sec=0.0,
        forward_soft_start_sec=0.0,
        heading_hold_enabled=True,
        heading_hold_kp=1.0,
        heading_hold_steer_max=30,
        heading_hold_deadband_deg=1.0,
        heading_hold_steer_speed_ratio=0.5,
        heading_hold_min_speed_percent=5.0,
        heading_hold_steer_cap_min_spd_percent=20.0,
        heading_hold_steer_trim=0,
        heading_hold_invert_steer=False,
        lifecycle=lifecycle or MotionLifecycle(),
    )


def test_run_stops_immediately_for_non_positive_distance() -> None:
    """run_with_result сразу останавливает мотор при нулевой дистанции."""
    motor: FakeMotor = FakeMotor()
    executor: LinearMotionExecutor = _executor(motor=motor)

    result: LinearMotionExecutionResult = executor.run_with_result(
        distance_cm=0.0,
        speed_percent=50,
        move_fn=lambda **kwargs: None,
    )

    assert result.completed is True
    assert result.traveled_cm == 0.0
    assert motor.stop_calls == 1


def test_run_returns_completed_flag() -> None:
    """run возвращает только completed из расширенного результата."""
    executor: LinearMotionExecutor = _executor()

    completed: bool = executor.run(
        distance_cm=0.0,
        speed_percent=50,
        move_fn=lambda **kwargs: None,
    )

    assert completed is True


def test_calculates_speed_and_angle_helpers() -> None:
    """Вспомогательные расчеты скорости, дистанции и ошибки угла работают стабильно."""
    executor: LinearMotionExecutor = _executor()

    assert executor._calculate_obstacle_aware_speed(10.0, remaining_cm=50.0, max_speed=80) == 0.0
    assert executor._calculate_obstacle_aware_speed(30.0, remaining_cm=50.0, max_speed=80) == 40.0
    assert executor._calculate_distance_speed_factor(10.0) == 0.25
    executor.deceleration_distance_cm = 0.0
    assert executor._calculate_obstacle_aware_speed(30.0, remaining_cm=50.0, max_speed=80) == 80.0
    assert executor._calculate_distance_speed_factor(10.0) == 1.0
    assert executor._estimate_traveled_distance(speed_percent=50.0, time_interval=2.0) == 100.0
    assert executor._angle_error_deg(10.0, 350.0) == 20.0
    assert executor._angle_error_deg(350.0, 10.0) == -20.0


def test_heading_steer_respects_disabled_deadband_caps_and_inversion() -> None:
    """_heading_steer_percent учитывает флаги, deadband, cap и инверсию."""
    gyroscope: FakeGyroscope = FakeGyroscope(yaw=-20.0)
    executor: LinearMotionExecutor = _executor(gyroscope=gyroscope)

    assert executor._heading_steer_percent(heading_setpoint_deg=0.0, base_speed_percent=50.0) == 20

    executor.heading_hold_invert_steer = True
    assert executor._heading_steer_percent(heading_setpoint_deg=0.0, base_speed_percent=50.0) == -20

    executor.heading_hold_enabled = False
    assert executor._heading_steer_percent(heading_setpoint_deg=0.0, base_speed_percent=50.0) == 0

    executor.heading_hold_enabled = True
    assert executor._heading_steer_percent(heading_setpoint_deg=0.0, base_speed_percent=1.0) == 0

    gyroscope.yaw = 0.5
    executor.heading_hold_invert_steer = False
    assert executor._heading_steer_percent(heading_setpoint_deg=0.0, base_speed_percent=50.0) == 0

    executor.heading_hold_steer_speed_ratio = 0.0
    gyroscope.yaw = -20.0
    assert executor._heading_steer_percent(heading_setpoint_deg=0.0, base_speed_percent=50.0) == 0


def test_publish_progress_reports_positive_delta_and_obstacle() -> None:
    """_publish_progress отправляет delta и obstacle в callback."""
    executor: LinearMotionExecutor = _executor()
    calls: list[tuple[float, float, int, float | None]] = []

    reported: float = executor._publish_progress(
        progress_callback=lambda traveled, delta, direction, obstacle: calls.append(
            (traveled, delta, direction, obstacle)
        ),
        traveled_cm=5.0,
        reported_traveled_cm=2.0,
        direction=-1,
        obstacle_cm=None,
    )
    unchanged: float = executor._publish_progress(
        progress_callback=lambda traveled, delta, direction, obstacle: calls.append(
            (traveled, delta, direction, obstacle)
        ),
        traveled_cm=5.0,
        reported_traveled_cm=5.0,
        direction=1,
        obstacle_cm=None,
    )
    obstacle_reported: float = executor._publish_progress(
        progress_callback=lambda traveled, delta, direction, obstacle: calls.append(
            (traveled, delta, direction, obstacle)
        ),
        traveled_cm=5.0,
        reported_traveled_cm=5.0,
        direction=1,
        obstacle_cm=12.0,
    )

    assert reported == 5.0
    assert unchanged == 5.0
    assert obstacle_reported == 5.0
    assert calls == [(5.0, -3.0, -1, None), (5.0, 0.0, 1, 12.0)]


def test_publish_progress_and_coast_can_skip_work() -> None:
    """No-op ветки progress и coast не вызывают внешние callbacks."""
    executor: LinearMotionExecutor = _executor()
    move_calls: list[dict[str, int]] = []

    reported: float = executor._publish_progress(
        progress_callback=None,
        traveled_cm=5.0,
        reported_traveled_cm=2.0,
        direction=1,
        obstacle_cm=None,
    )
    executor._coast_before_stop(
        move_fn=lambda **kwargs: move_calls.append(kwargs),
        heading_setpoint_deg=0.0,
        last_cmd_speed=0.0,
    )

    assert reported == 2.0
    assert move_calls == []


def test_run_with_result_completes_segment(monkeypatch: pytest.MonkeyPatch) -> None:
    """run_with_result доезжает до цели и останавливает мотор."""
    lifecycle: MotionLifecycle = MotionLifecycle()
    lifecycle.set_running(is_route_running=False)
    motor: FakeMotor = FakeMotor()
    executor: LinearMotionExecutor = _executor(motor=motor, lifecycle=lifecycle)
    times: list[float] = [0.0, 0.0, 1.0, 2.0, 2.0]
    move_calls: list[dict[str, int]] = []
    monkeypatch.setattr(linear_module.time, "monotonic", lambda: times.pop(0))
    monkeypatch.setattr(linear_module.time, "sleep", lambda seconds: None)

    result: LinearMotionExecutionResult = executor.run_with_result(
        distance_cm=10.0,
        speed_percent=50,
        move_fn=lambda **kwargs: move_calls.append(kwargs),
        obstacle_aware=False,
    )

    assert result.completed is True
    assert result.traveled_cm == 10.0
    assert motor.stop_calls == 1
    assert move_calls


def test_run_with_result_blocks_on_obstacle(monkeypatch: pytest.MonkeyPatch) -> None:
    """run_with_result возвращает blocked, когда препятствие ближе минимума."""
    lifecycle: MotionLifecycle = MotionLifecycle()
    lifecycle.set_running(is_route_running=False)
    motor: FakeMotor = FakeMotor()
    executor: LinearMotionExecutor = _executor(motor=motor, lifecycle=lifecycle)
    times: list[float] = [0.0, 0.0, 0.1]
    progress_calls: list[tuple[float, float, int, float | None]] = []
    monkeypatch.setattr(linear_module.time, "monotonic", lambda: times.pop(0))
    monkeypatch.setattr(linear_module.time, "sleep", lambda seconds: None)

    result: LinearMotionExecutionResult = executor.run_with_result(
        distance_cm=100.0,
        speed_percent=50,
        move_fn=lambda **kwargs: None,
        obstacle_aware=True,
        obstacle_distance_provider=lambda: 10.0,
        progress_callback=lambda traveled, delta, direction, obstacle: progress_calls.append(
            (traveled, delta, direction, obstacle)
        ),
    )

    assert result.completed is False
    assert result.blocked is True
    assert motor.stop_calls == 1
    assert progress_calls[-1][3] == 10.0


def test_run_with_result_handles_move_errors(monkeypatch: pytest.MonkeyPatch) -> None:
    """run_with_result возвращает неуспешный результат при runtime ошибке движения."""
    lifecycle: MotionLifecycle = MotionLifecycle()
    lifecycle.set_running(is_route_running=False)
    executor: LinearMotionExecutor = _executor(lifecycle=lifecycle)
    times: list[float] = [0.0, 0.0, 0.1, 0.2]
    monkeypatch.setattr(linear_module.time, "monotonic", lambda: times.pop(0))

    result: LinearMotionExecutionResult = executor.run_with_result(
        distance_cm=100.0,
        speed_percent=50,
        move_fn=lambda **kwargs: (_ for _ in ()).throw(RuntimeError("move failed")),
        obstacle_aware=False,
    )

    assert result.completed is False
    assert result.blocked is False


def test_run_with_result_returns_incomplete_when_lifecycle_is_stopped(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """run_with_result возвращает incomplete, если lifecycle уже остановлен."""
    executor: LinearMotionExecutor = _executor()
    monkeypatch.setattr(linear_module.time, "monotonic", lambda: 0.0)

    result: LinearMotionExecutionResult = executor.run_with_result(
        distance_cm=100.0,
        speed_percent=50,
        move_fn=lambda **kwargs: None,
    )

    assert result.completed is False
    assert result.traveled_cm == 0.0
