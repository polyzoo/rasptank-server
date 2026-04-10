from __future__ import annotations

import pytest

from src.application.services import turn_executor as turn_module
from src.application.services.motion_lifecycle import MotionLifecycle
from src.application.services.turn_executor import TurnExecutionResult, TurnExecutor


class FakeMotor:
    """Заглушка контроллера моторов для поворотов."""

    def __init__(self) -> None:
        """Подготовить историю команд."""
        self.left_speeds: list[int] = []
        self.right_speeds: list[int] = []
        self.stop_calls: int = 0

    def turn_left(self, speed_percent: int) -> None:
        """Зафиксировать команду поворота налево."""
        self.left_speeds.append(speed_percent)

    def turn_right(self, speed_percent: int) -> None:
        """Зафиксировать команду поворота направо."""
        self.right_speeds.append(speed_percent)

    def stop(self) -> None:
        """Зафиксировать остановку."""
        self.stop_calls += 1


class FakeUltrasonic:
    """Заглушка ультразвукового датчика."""

    def __init__(self, distance_cm: float = 999.0) -> None:
        """Сохранить возвращаемую дистанцию."""
        self.distance_cm: float = distance_cm

    def measure_distance_cm(self) -> float:
        """Вернуть текущую fake дистанцию."""
        return self.distance_cm


class FakeGyroscope:
    """Заглушка гироскопа с последовательностью yaw."""

    def __init__(self, yaw_values: list[float] | None = None) -> None:
        """Подготовить yaw значения."""
        self.yaw_values: list[float] = yaw_values or [0.0]
        self.reset_calls: int = 0

    def reset_yaw(self) -> None:
        """Зафиксировать сброс yaw."""
        self.reset_calls += 1

    def get_yaw(self) -> float:
        """Вернуть следующий yaw или последний доступный."""
        if len(self.yaw_values) > 1:
            return self.yaw_values.pop(0)
        return self.yaw_values[0]


def _turn_executor(
    *,
    motor: FakeMotor | None = None,
    ultrasonic: FakeUltrasonic | None = None,
    gyroscope: FakeGyroscope | None = None,
    lifecycle: MotionLifecycle | None = None,
) -> TurnExecutor:
    """Создать TurnExecutor с fake зависимостями."""
    return TurnExecutor(
        motor_controller=motor or FakeMotor(),  # type: ignore[arg-type]
        ultrasonic_sensor=ultrasonic or FakeUltrasonic(),  # type: ignore[arg-type]
        gyroscope=gyroscope or FakeGyroscope(),  # type: ignore[arg-type]
        min_obstacle_distance_cm=20.0,
        turn_speed_percent=60,
        turn_slowdown_remaining_deg=20.0,
        turn_creep_speed_percent=25,
        turn_angle_trim_deg=2.0,
        last_turn_angle_trim_deg=3.0,
        turn_check_interval_sec=0.0,
        turn_obstacle_check_interval_sec=0.0,
        turn_timeout_per_deg=0.01,
        turn_timeout_min=0.1,
        lifecycle=lifecycle or MotionLifecycle(),
    )


def test_run_relative_returns_immediately_for_zero_angle() -> None:
    """run_relative сразу завершает нулевой угол."""
    result: TurnExecutionResult = _turn_executor().run_relative(
        target_angle_deg=0.0,
        turn_left=True,
        stop_on_front_obstacle=True,
    )

    assert result.completed is True
    assert result.angle_deg == 0.0


def test_run_and_run_relative_delegate_to_execute_turn() -> None:
    """run и run_relative делегируют расчетные параметры в _execute_turn."""
    executor: TurnExecutor = _turn_executor()
    calls: list[dict[str, object]] = []

    def fake_execute_turn(**kwargs: object) -> TurnExecutionResult:
        """Сохранить параметры и вернуть результат."""
        calls.append(kwargs)
        return TurnExecutionResult(completed=True, angle_deg=float(kwargs["target_angle"]))

    executor._execute_turn = fake_execute_turn  # type: ignore[method-assign]

    route_result: TurnExecutionResult = executor.run(
        requested_angle_deg=90.0,
        turn_left=True,
        segment_index=0,
        total_segments=2,
    )
    relative_result: TurnExecutionResult = executor.run_relative(
        target_angle_deg=20.0,
        turn_left=False,
        stop_on_front_obstacle=False,
    )

    assert route_result.angle_deg == 92.0
    assert relative_result.angle_deg == 20.0
    assert calls[0]["stop_on_front_obstacle"] is True
    assert calls[1]["stop_on_front_obstacle"] is False


def test_route_turn_target_angle_applies_trim_and_clamps() -> None:
    """_route_turn_target_angle применяет trim и ограничивает диапазон."""
    executor: TurnExecutor = _turn_executor()

    assert executor._route_turn_target_angle(90.0, segment_index=0, total_segments=2) == 92.0
    assert executor._route_turn_target_angle(90.0, segment_index=1, total_segments=2) == 95.0
    assert executor._route_turn_target_angle(500.0, segment_index=0, total_segments=1) == 179.0
    assert executor._route_turn_target_angle(-50.0, segment_index=0, total_segments=1) == 1.0


def test_turn_speed_uses_creep_speed_in_slowdown_zone() -> None:
    """_turn_motor_speed_percent снижает скорость рядом с целью."""
    executor: TurnExecutor = _turn_executor()

    assert executor._turn_motor_speed_percent(target_angle=90.0, current_yaw_abs=0.0) == 60
    assert executor._turn_motor_speed_percent(target_angle=90.0, current_yaw_abs=80.0) == 25
    executor.turn_slowdown_remaining_deg = 0.0
    assert executor._turn_slowdown_zone_deg(target_angle=90.0) == 0.0
    assert executor._turn_motor_speed_percent(target_angle=90.0, current_yaw_abs=89.0) == 60


def test_execute_turn_stops_on_target_reached() -> None:
    """_execute_turn останавливается при достижении целевого yaw."""
    lifecycle: MotionLifecycle = MotionLifecycle()
    lifecycle.set_running(is_route_running=True)
    motor: FakeMotor = FakeMotor()
    gyroscope: FakeGyroscope = FakeGyroscope([0.0, 20.0, 40.0])
    executor: TurnExecutor = _turn_executor(motor=motor, gyroscope=gyroscope, lifecycle=lifecycle)

    result: TurnExecutionResult = executor._execute_turn(
        turn_left=True,
        target_angle=30.0,
        timeout_sec=1.0,
        stop_on_front_obstacle=False,
    )

    assert result.completed is True
    assert result.stop_reason == "target_reached"
    assert motor.left_speeds == [60, 25]
    assert motor.stop_calls == 1


def test_execute_turn_stops_on_front_obstacle() -> None:
    """_execute_turn останавливается при фронтальном препятствии."""
    lifecycle: MotionLifecycle = MotionLifecycle()
    lifecycle.set_running(is_route_running=True)
    motor: FakeMotor = FakeMotor()
    executor: TurnExecutor = _turn_executor(
        motor=motor,
        ultrasonic=FakeUltrasonic(distance_cm=10.0),
        lifecycle=lifecycle,
    )

    result: TurnExecutionResult = executor._execute_turn(
        turn_left=False,
        target_angle=30.0,
        timeout_sec=1.0,
        stop_on_front_obstacle=True,
    )

    assert result.completed is False
    assert result.stop_reason == "front_obstacle"
    assert motor.right_speeds == []
    assert motor.stop_calls == 1


def test_execute_turn_stops_on_timeout(monkeypatch: pytest.MonkeyPatch) -> None:
    """_execute_turn останавливается по timeout и умеет поворачивать направо."""
    lifecycle: MotionLifecycle = MotionLifecycle()
    lifecycle.set_running(is_route_running=True)
    motor: FakeMotor = FakeMotor()
    executor: TurnExecutor = _turn_executor(motor=motor, lifecycle=lifecycle)
    times: list[float] = [0.0, 2.0]
    monkeypatch.setattr(turn_module.time, "monotonic", lambda: times.pop(0))

    result: TurnExecutionResult = executor._execute_turn(
        turn_left=False,
        target_angle=30.0,
        timeout_sec=1.0,
        stop_on_front_obstacle=False,
    )

    assert result.completed is False
    assert result.stop_reason == "timeout"
    assert motor.right_speeds == []
    assert motor.stop_calls == 1


def test_execute_turn_checks_clear_path_and_turns_right() -> None:
    """_execute_turn обновляет obstacle-check timestamp и отправляет команду направо."""
    lifecycle: MotionLifecycle = MotionLifecycle()
    lifecycle.set_running(is_route_running=True)
    motor: FakeMotor = FakeMotor()
    gyroscope: FakeGyroscope = FakeGyroscope([0.0, 40.0])
    executor: TurnExecutor = _turn_executor(
        motor=motor,
        ultrasonic=FakeUltrasonic(distance_cm=999.0),
        gyroscope=gyroscope,
        lifecycle=lifecycle,
    )

    result: TurnExecutionResult = executor._execute_turn(
        turn_left=False,
        target_angle=30.0,
        timeout_sec=1.0,
        stop_on_front_obstacle=True,
    )

    assert result.completed is True
    assert result.stop_reason == "target_reached"
    assert motor.right_speeds == [60]
    assert motor.stop_calls == 1
