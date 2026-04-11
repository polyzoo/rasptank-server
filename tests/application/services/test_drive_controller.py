from __future__ import annotations

import pytest

from src.application.models.route import ForwardSegment, Route
from src.application.services.drive_controller import (
    AvoidanceContext,
    AvoidanceResult,
    AvoidanceSide,
    DistanceMeasurementSummary,
    DriveController,
    LinearMoveResult,
    SideScanResult,
    TurnResult,
)
from src.application.services.linear_motion_executor import LinearMotionExecutionResult
from src.application.services.motion_events import MotionEvent, MotionEventHub
from src.application.services.turn_executor import TurnExecutionResult


class FakeMotor:
    """Заглушка контроллера моторов."""

    def __init__(self) -> None:
        """Подготовить историю вызовов."""
        self.forward_calls: list[dict[str, int]] = []
        self.backward_calls: list[dict[str, int]] = []
        self.stop_calls: int = 0
        self.destroy_calls: int = 0

    def move_forward(self, speed_percent: int, steer_percent: int = 0) -> None:
        """Зафиксировать движение вперед."""
        self.forward_calls.append({"speed_percent": speed_percent, "steer_percent": steer_percent})

    def move_backward(self, speed_percent: int, steer_percent: int = 0) -> None:
        """Зафиксировать движение назад."""
        self.backward_calls.append({"speed_percent": speed_percent, "steer_percent": steer_percent})

    def turn_left(self, speed_percent: int) -> None:
        """Заглушка поворота налево."""

    def turn_right(self, speed_percent: int) -> None:
        """Заглушка поворота направо."""

    def stop(self) -> None:
        """Зафиксировать остановку."""
        self.stop_calls += 1

    def destroy(self) -> None:
        """Зафиксировать освобождение ресурсов."""
        self.destroy_calls += 1


class FakeUltrasonic:
    """Заглушка ультразвукового датчика."""

    def __init__(self, distances: list[float] | None = None) -> None:
        """Подготовить последовательность дистанций."""
        self.distances: list[float] = distances or [999.0]
        self.destroy_calls: int = 0

    def measure_distance_cm(self) -> float:
        """Вернуть следующую дистанцию или последнюю известную."""
        if len(self.distances) > 1:
            return self.distances.pop(0)
        return self.distances[0]

    def destroy(self) -> None:
        """Зафиксировать освобождение ресурсов."""
        self.destroy_calls += 1


class FakeGyroscope:
    """Заглушка гироскопа."""

    def __init__(self) -> None:
        """Подготовить историю вызовов."""
        self.start_calls: list[bool] = []
        self.stop_calls: int = 0
        self.destroy_calls: int = 0

    def start(self, calibrate: bool = True) -> None:
        """Зафиксировать запуск."""
        self.start_calls.append(calibrate)

    def get_yaw(self) -> float:
        """Вернуть нулевой yaw."""
        return 0.0

    def stop(self) -> None:
        """Зафиксировать остановку."""
        self.stop_calls += 1

    def reset_yaw(self) -> None:
        """Заглушка сброса yaw."""

    def destroy(self) -> None:
        """Зафиксировать освобождение ресурсов."""
        self.destroy_calls += 1


class FakeHeadServo:
    """Заглушка серво головы."""

    def __init__(self, *, fail: bool = False) -> None:
        """Подготовить историю углов."""
        self.fail: bool = fail
        self.angles: list[float] = []
        self.destroy_calls: int = 0

    def set_angle(self, angle_deg: float) -> None:
        """Зафиксировать угол или сымитировать ошибку."""
        if self.fail:
            raise RuntimeError("servo failed")
        self.angles.append(angle_deg)

    def destroy(self) -> None:
        """Зафиксировать освобождение ресурсов."""
        self.destroy_calls += 1


class FakeLinearMotion:
    """Заглушка LinearMotionExecutor."""

    def __init__(self, result: LinearMotionExecutionResult) -> None:
        """Сохранить результат запуска."""
        self.result: LinearMotionExecutionResult = result
        self.calls: list[dict[str, object]] = []

    def run_with_result(self, **kwargs: object) -> LinearMotionExecutionResult:
        """Сохранить параметры и вернуть настроенный результат."""
        self.calls.append(kwargs)
        return self.result


class FakeTurnExecutor:
    """Заглушка TurnExecutor."""

    def __init__(self, result: TurnExecutionResult) -> None:
        """Сохранить результат поворота."""
        self.result: TurnExecutionResult = result
        self.calls: list[dict[str, object]] = []
        self.min_obstacle_distance_cm: float = 0.0
        self.turn_speed_percent: int = 0

    def run_relative(self, **kwargs: object) -> TurnExecutionResult:
        """Сохранить параметры и вернуть настроенный результат."""
        self.calls.append(kwargs)
        return self.result


class FakeRouteExecutor:
    """Заглушка RouteExecutor."""

    def __init__(self) -> None:
        """Подготовить историю вызовов."""
        self.execute_calls: list[Route] = []
        self.execute_sync_calls: list[Route] = []
        self.stop_calls: int = 0

    def execute(self, route: Route) -> None:
        """Зафиксировать фоновый запуск маршрута."""
        self.execute_calls.append(route)

    def execute_sync(self, route: Route) -> None:
        """Зафиксировать синхронный запуск маршрута."""
        self.execute_sync_calls.append(route)

    def stop(self) -> None:
        """Зафиксировать stop."""
        self.stop_calls += 1


def _controller(
    *,
    ultrasonic: FakeUltrasonic | None = None,
    motion_events: MotionEventHub | None = None,
    head_servo: FakeHeadServo | None = None,
    workspace_limit_cm: float = 40.0,
) -> DriveController:
    """Создать DriveController с fake зависимостями."""
    return DriveController(
        motor_controller=FakeMotor(),
        ultrasonic_sensor=ultrasonic or FakeUltrasonic(),
        gyroscope=FakeGyroscope(),
        min_obstacle_distance_cm=20.0,
        deceleration_distance_cm=40.0,
        base_speed_percent=50,
        update_interval_sec=0.0,
        avoidance_side_step_cm=10.0,
        avoidance_forward_step_cm=12.0,
        avoidance_rejoin_step_cm=10.0,
        avoidance_max_attempts=3,
        avoidance_confirm_readings=3,
        avoidance_min_side_clearance_cm=25.0,
        avoidance_max_lateral_offset_cm=30.0,
        avoidance_max_bypass_distance_cm=50.0,
        motion_events=motion_events,
        workspace_limit_cm=workspace_limit_cm,
        head_servo=head_servo,
        head_servo_home_angle_deg=7.0,
    )


def test_avoidance_side_and_scan_result_helpers() -> None:
    """Enum и is_selectable отражают базовую логику выбора стороны."""
    selectable: SideScanResult = SideScanResult(
        side=AvoidanceSide.LEFT,
        target_angle_deg=45.0,
        turned_angle_deg=45.0,
        clearance_cm=30.0,
        heading_restored=True,
        scan_completed=True,
        scan_useful=True,
    )
    blocked: SideScanResult = SideScanResult(
        side=AvoidanceSide.RIGHT,
        target_angle_deg=45.0,
        turned_angle_deg=45.0,
        clearance_cm=30.0,
        heading_restored=True,
        scan_completed=True,
        scan_useful=True,
        selection_blocked=True,
    )

    assert AvoidanceSide.LEFT.turn_left is True
    assert AvoidanceSide.RIGHT.turn_left is False
    assert AvoidanceSide.LEFT.opposite() is AvoidanceSide.RIGHT
    assert AvoidanceSide.RIGHT.opposite() is AvoidanceSide.LEFT
    assert selectable.is_selectable is True
    assert blocked.is_selectable is False


def test_is_moving_compatibility_properties() -> None:
    """Старые private properties синхронизируются с lifecycle."""
    controller: DriveController = _controller()

    controller._is_moving = True
    assert controller._is_moving is True
    controller._is_route_running = True
    assert controller._is_route_running is True
    controller._is_moving = False
    assert controller._is_moving is False
    controller._is_route_running = False
    assert controller._is_route_running is False


def test_forward_cm_sync_runs_forward_segment_and_stops() -> None:
    """forward_cm_sync поднимает lifecycle, запускает гироскоп и останавливает ресурсы."""
    head_servo: FakeHeadServo = FakeHeadServo()
    controller: DriveController = _controller(head_servo=head_servo)
    calls: list[tuple[float, int]] = []
    controller._run_forward_segment = lambda distance, speed: (
        calls.append((distance, speed)) or True
    )  # type: ignore[method-assign]

    controller.forward_cm_sync(distance_cm=12.0, max_speed_percent=40)

    assert calls == [(12.0, 40)]
    assert head_servo.angles == [7.0]
    assert controller.gyroscope.start_calls == [True]  # type: ignore[attr-defined]
    assert controller.gyroscope.stop_calls == 1  # type: ignore[attr-defined]
    assert controller.motor_controller.stop_calls >= 1  # type: ignore[attr-defined]
    assert controller.lifecycle.is_moving is False


def test_execute_route_public_methods_publish_and_delegate() -> None:
    """execute_route, execute_route_sync и stop делегируют RouteExecutor и публикуют события."""
    hub: MotionEventHub = MotionEventHub()
    queue = hub.subscribe()
    queue.get_nowait()
    controller: DriveController = _controller(motion_events=hub)
    fake_executor: FakeRouteExecutor = FakeRouteExecutor()
    controller._route_executor = fake_executor  # type: ignore[assignment]
    route: Route = Route(segments=[ForwardSegment(distance_cm=5.0)])

    controller.execute_route(route)
    controller.execute_route_sync(route)
    controller.stop()

    statuses: list[str] = [queue.get_nowait().status for _ in range(queue.qsize())]
    assert fake_executor.execute_calls == [route]
    assert fake_executor.execute_sync_calls == [route]
    assert fake_executor.stop_calls == 1
    assert "moving" in statuses
    assert "stopped" in statuses


def test_destroy_releases_all_resources() -> None:
    """destroy вызывает stop/destroy у всех подключенных зависимостей."""
    head_servo: FakeHeadServo = FakeHeadServo()
    controller: DriveController = _controller(head_servo=head_servo)
    fake_executor: FakeRouteExecutor = FakeRouteExecutor()
    controller._route_executor = fake_executor  # type: ignore[assignment]

    controller.destroy()

    assert fake_executor.stop_calls == 1
    assert controller.motor_controller.destroy_calls == 1  # type: ignore[attr-defined]
    assert controller.ultrasonic_sensor.destroy_calls == 1  # type: ignore[attr-defined]
    assert controller.gyroscope.destroy_calls == 1  # type: ignore[attr-defined]
    assert head_servo.destroy_calls == 1


def test_fix_head_forward_ignores_missing_or_failed_servo() -> None:
    """_fix_head_forward безопасен без серво и при ошибке серво."""
    controller: DriveController = _controller()
    controller._fix_head_forward()
    failing_servo: FakeHeadServo = FakeHeadServo(fail=True)
    controller._head_servo = failing_servo

    controller._fix_head_forward()

    assert failing_servo.angles == []


def test_motion_event_helpers_update_position_obstacle_and_workspace_limit() -> None:
    """Progress callback учитывает допуск по границе зоны и публикует ошибку только после него."""
    hub: MotionEventHub = MotionEventHub()
    queue = hub.subscribe()
    queue.get_nowait()
    controller: DriveController = _controller(
        motion_events=hub,
        workspace_limit_cm=5.0,
    )
    controller._heading_deg = 90.0
    controller.lifecycle.set_running(is_route_running=False)

    controller._handle_linear_motion_progress(
        traveled_cm=5.5,
        delta_cm=5.5,
        direction=1,
        obstacle_cm=10.0,
    )

    events: list[MotionEvent] = [queue.get_nowait() for _ in range(queue.qsize())]
    assert controller._position_x_cm == pytest.approx(5.5)
    assert controller._position_y_cm == pytest.approx(0.0)
    assert [event.type for event in events] == ["position", "obstacle"]
    assert controller._motion_error_reported is False
    assert controller.lifecycle.is_moving is True

    controller._handle_linear_motion_progress(
        traveled_cm=6.2,
        delta_cm=0.7,
        direction=1,
        obstacle_cm=None,
    )

    events = [queue.get_nowait() for _ in range(queue.qsize())]
    assert controller._position_x_cm == pytest.approx(6.2)
    assert [event.type for event in events] == ["position", "error"]
    assert controller._motion_error_reported is True
    assert controller.lifecycle.is_moving is False

    controller.motor_controller.stop_calls = 0  # type: ignore[attr-defined]
    controller._stop_if_workspace_limit_exceeded()
    assert controller.motor_controller.stop_calls == 0  # type: ignore[attr-defined]

    controller_without_hub: DriveController = _controller()
    controller_without_hub._publish_obstacle_event(10.0)
    controller_without_hub._stop_if_workspace_limit_exceeded()
    assert controller_without_hub.motor_controller.stop_calls == 0  # type: ignore[attr-defined]


def test_report_motion_error_ignores_repeated_error() -> None:
    """Повторный вызов _report_motion_error ничего не делает после первой ошибки."""
    hub: MotionEventHub = MotionEventHub()
    queue = hub.subscribe()
    queue.get_nowait()
    controller: DriveController = _controller(motion_events=hub)
    controller._motion_error_reported = True

    controller._report_motion_error("Повторная ошибка")

    assert queue.qsize() == 0
    assert controller.motor_controller.stop_calls == 0  # type: ignore[attr-defined]
    assert controller.lifecycle.is_moving is False


def test_turn_and_route_finished_event_helpers() -> None:
    """Поворот обновляет heading, route_finished публикует stopped без error."""
    hub: MotionEventHub = MotionEventHub()
    queue = hub.subscribe()
    queue.get_nowait()
    controller: DriveController = _controller(motion_events=hub)

    controller._handle_turn_completed(angle_deg=90.0, turn_left=True)
    controller._handle_turn_completed(angle_deg=45.0, turn_left=False)
    controller._handle_route_finished()

    events: list[MotionEvent] = [queue.get_nowait() for _ in range(queue.qsize())]
    assert controller._heading_deg == 315.0
    assert [event.status for event in events] == ["turning", "turning", "stopped"]

    controller._motion_error_reported = True
    controller._handle_route_finished()
    assert queue.qsize() == 0


def test_run_linear_motion_short_circuits_and_delegates() -> None:
    """_run_linear_motion закрывает нулевую дистанцию и делегирует обычную."""
    controller: DriveController = _controller()

    done: LinearMoveResult = controller._run_linear_motion(
        distance_cm=0.1,
        speed_percent=50,
        move_forward=True,
        obstacle_aware=True,
    )
    fake_linear: FakeLinearMotion = FakeLinearMotion(
        LinearMotionExecutionResult(completed=False, traveled_cm=4.0, blocked=True)
    )
    controller._linear_motion = fake_linear  # type: ignore[assignment]
    moved: LinearMoveResult = controller._run_linear_motion(
        distance_cm=10.0,
        speed_percent=45,
        move_forward=False,
        obstacle_aware=False,
    )

    assert done.completed is True
    assert moved == LinearMoveResult(completed=False, traveled_cm=4.0, blocked=True)
    assert fake_linear.calls[0]["move_fn"] == controller.motor_controller.move_backward
    assert fake_linear.calls[0]["obstacle_distance_provider"] is None
    assert fake_linear.calls[0]["progress_direction"] == -1


def test_run_forward_segment_success_blocked_and_error_paths() -> None:
    """_run_forward_segment обрабатывает успех, obstacle avoidance и исключения."""
    controller: DriveController = _controller()
    controller.lifecycle.set_running(is_route_running=False)
    controller._run_linear_motion = lambda **kwargs: LinearMoveResult(True, 10.0)  # type: ignore[method-assign]
    assert controller._run_forward_segment(10.0, 50) is True

    controller.lifecycle.set_running(is_route_running=False)
    calls: list[str] = []
    controller._run_linear_motion = lambda **kwargs: LinearMoveResult(False, 2.0, blocked=True)  # type: ignore[method-assign]
    controller._run_obstacle_avoidance = lambda remaining_cm, speed_percent: (
        calls.append("avoid") or AvoidanceResult(True, remaining_cm)
    )  # type: ignore[method-assign]
    assert controller._run_forward_segment(10.0, 50) is True
    assert calls == ["avoid"]

    controller.lifecycle.set_running(is_route_running=False)
    controller._run_linear_motion = lambda **kwargs: LinearMoveResult(False, 0.0, blocked=False)  # type: ignore[method-assign]
    assert controller._run_forward_segment(10.0, 50) is False

    controller.lifecycle.set_running(is_route_running=False)
    assert controller._run_forward_segment(0.0, 50) is True

    controller.lifecycle.set_running(is_route_running=False)
    controller._run_linear_motion = lambda **kwargs: LinearMoveResult(False, 2.0, blocked=True)  # type: ignore[method-assign]
    controller._run_obstacle_avoidance = lambda remaining_cm, speed_percent: AvoidanceResult(
        False, 0.0
    )  # type: ignore[method-assign]
    assert controller._run_forward_segment(10.0, 50) is False

    controller.lifecycle.set_running(is_route_running=False)
    controller._run_linear_motion = lambda **kwargs: (_ for _ in ()).throw(RuntimeError("boom"))  # type: ignore[method-assign]
    assert controller._run_forward_segment(10.0, 50) is False

    stopped_controller: DriveController = _controller()
    assert stopped_controller._run_forward_segment(10.0, 50) is False


def test_backward_side_forward_and_rejoin_step_helpers() -> None:
    """Небольшие helper-методы делегируют в нужные направления и дистанции."""
    controller: DriveController = _controller()
    run_calls: list[dict[str, object]] = []
    lateral_calls: list[dict[str, object]] = []
    controller._run_linear_motion = lambda **kwargs: (
        run_calls.append(kwargs) or LinearMoveResult(True, 3.0)
    )  # type: ignore[method-assign]
    controller._perform_lateral_step = lambda **kwargs: (
        lateral_calls.append(kwargs) or LinearMoveResult(True, 4.0)
    )  # type: ignore[method-assign]

    assert controller._run_backward_segment(9.0, 40) is True
    assert controller._perform_forward_step(20.0, 40).traveled_cm == 3.0
    assert controller._perform_side_step(AvoidanceSide.LEFT, 40).traveled_cm == 4.0
    assert controller._attempt_rejoin_step(AvoidanceSide.LEFT, 6.0, 40).traveled_cm == 4.0

    assert run_calls[0]["move_forward"] is False
    assert run_calls[1]["distance_cm"] == 12.0
    assert lateral_calls[0]["side"] is AvoidanceSide.LEFT
    assert lateral_calls[1]["side"] is AvoidanceSide.RIGHT
    assert lateral_calls[1]["distance_cm"] == 6.0


def test_perform_lateral_step_variants() -> None:
    """_perform_lateral_step покрывает нулевую дистанцию, restore success/fail и малый угол."""
    controller: DriveController = _controller()
    assert controller._perform_lateral_step(AvoidanceSide.LEFT, 0.0, 40).completed is True

    turn_results: list[TurnResult] = [TurnResult(True, 0.1)]
    controller._turn_relative = lambda **kwargs: turn_results.pop(0)  # type: ignore[method-assign]
    result: LinearMoveResult = controller._perform_lateral_step(AvoidanceSide.LEFT, 5.0, 40)
    assert result.traveled_cm == 0.0

    turn_results = [TurnResult(True, 90.0), TurnResult(True, 90.0)]
    controller._turn_relative = lambda **kwargs: turn_results.pop(0)  # type: ignore[method-assign]
    controller._run_linear_motion = lambda **kwargs: LinearMoveResult(False, 2.0, blocked=True)  # type: ignore[method-assign]
    restored: LinearMoveResult = controller._perform_lateral_step(AvoidanceSide.LEFT, 5.0, 40)
    assert restored.heading_restored is True

    turn_results = [TurnResult(True, 90.0), TurnResult(False, 30.0)]
    controller._turn_relative = lambda **kwargs: turn_results.pop(0)  # type: ignore[method-assign]
    failed_restore: LinearMoveResult = controller._perform_lateral_step(AvoidanceSide.LEFT, 5.0, 40)
    assert failed_restore.heading_restored is False


def test_turn_relative_updates_executor_and_returns_result() -> None:
    """_turn_relative синхронизирует настройки и обновляет heading."""
    controller: DriveController = _controller()
    fake_turn: FakeTurnExecutor = FakeTurnExecutor(
        TurnExecutionResult(completed=False, angle_deg=30.0, stop_reason="timeout")
    )
    controller._turn_executor = fake_turn  # type: ignore[assignment]

    result: TurnResult = controller._turn_relative(
        turn_left=False,
        target_angle=45.0,
        stop_on_front_obstacle=True,
    )

    assert result == TurnResult(completed=False, angle_deg=30.0, stop_reason="timeout")
    assert fake_turn.min_obstacle_distance_cm == controller.min_obstacle_distance_cm
    assert fake_turn.turn_speed_percent == controller.turn_speed_percent
    assert controller._heading_deg == 30.0


def test_measurement_and_front_clear_helpers(monkeypatch: pytest.MonkeyPatch) -> None:
    """Измерения сортируются, оценивают spread и подтверждают свободный фронт."""
    controller: DriveController = _controller(ultrasonic=FakeUltrasonic([30.0, 10.0, 50.0]))
    summary: DistanceMeasurementSummary = controller._measure_front_distance_summary(samples=3)

    assert summary.median_cm == 30.0
    assert summary.raw_samples_cm == (10.0, 30.0, 50.0)
    assert summary.reliable is False
    assert controller._measure_front_distance(samples=3) == 50.0

    controller.ultrasonic_sensor = FakeUltrasonic([30.0, 30.0])  # type: ignore[assignment]
    monkeypatch.setattr(
        "src.application.services.drive_controller.time.sleep",
        lambda seconds: None,
    )
    assert controller._is_front_clear_confirmed() is True
    controller.ultrasonic_sensor = FakeUltrasonic([10.0])  # type: ignore[assignment]
    assert controller._is_front_clear_confirmed() is False


def test_measure_motion_obstacle_distance_confirms_blocked_reading() -> None:
    """_measure_motion_obstacle_distance подтверждает только заблокированное чтение."""
    controller: DriveController = _controller(ultrasonic=FakeUltrasonic([25.0]))
    assert controller._measure_motion_obstacle_distance() == 25.0

    controller.ultrasonic_sensor = FakeUltrasonic([10.0, 11.0, 12.0, 13.0])  # type: ignore[assignment]
    assert controller._measure_motion_obstacle_distance() == 12.0


def test_avoidance_limit_and_progress_helpers() -> None:
    """Лимиты обхода публикуют ошибки и прогресс считает attempts/offset/distance."""
    hub: MotionEventHub = MotionEventHub()
    queue = hub.subscribe()
    queue.get_nowait()
    controller: DriveController = _controller(motion_events=hub)
    context: AvoidanceContext = AvoidanceContext(attempts=3)
    assert controller._has_exceeded_avoidance_limits(context) is True
    attempt_event: MotionEvent = queue.get_nowait()
    assert attempt_event.type == "error"
    assert "превышено число попыток" in (attempt_event.message or "")

    controller._motion_error_reported = False
    context = AvoidanceContext(lateral_offset_cm=30.0)
    assert controller._has_exceeded_avoidance_limits(context) is True
    lateral_event: MotionEvent = queue.get_nowait()
    assert lateral_event.type == "error"
    assert "боковое смещение" in (lateral_event.message or "")

    controller._motion_error_reported = False
    context = AvoidanceContext(bypass_distance_cm=50.0)
    assert controller._has_exceeded_avoidance_limits(context) is True
    bypass_event: MotionEvent = queue.get_nowait()
    assert bypass_event.type == "error"
    assert "суммарная длина обхода" in (bypass_event.message or "")

    controller._motion_error_reported = False
    context = AvoidanceContext()
    assert controller._has_exceeded_avoidance_limits(context) is False

    controller._update_avoidance_progress(context, traveled_cm=4.0, attempts_delta=2)
    assert context.attempts == 2
    assert context.bypass_distance_cm == 4.0
    assert controller._is_front_blocked(20.0) is True
    assert controller._has_reached_target_distance(0.5) is True


def test_rejoin_and_angle_threshold_helpers() -> None:
    """Rejoin recovery и side-scan threshold helpers возвращают ожидаемые значения."""
    controller: DriveController = _controller()

    assert (
        controller._effective_rejoin_lateral_recovery_cm(
            traveled_cm=0.0,
            lateral_offset_before_cm=3.0,
        )
        == 0.0
    )
    assert (
        controller._effective_rejoin_lateral_recovery_cm(
            traveled_cm=5.0,
            lateral_offset_before_cm=20.0,
        )
        == 5.0
    )
    assert controller._effective_rejoin_lateral_recovery_cm(
        traveled_cm=5.0,
        lateral_offset_before_cm=5.0,
    ) == pytest.approx(4.75)
    assert controller._should_retry_final_rejoin_immediately(lateral_offset_cm=0.1) is False
    assert controller._should_retry_final_rejoin_immediately(lateral_offset_cm=0.75) is True
    assert controller._side_scan_useful_angle_threshold(45.0) == 22.5
    assert controller._side_scan_effective_angle_threshold(45.0) == pytest.approx(29.25)
    assert (
        controller._is_useful_partial_side_scan(
            target_angle_deg=45.0,
            turned_angle_deg=23.0,
        )
        is True
    )
    assert (
        controller._is_effective_side_scan_angle(
            target_angle_deg=45.0,
            turned_angle_deg=20.0,
        )
        is False
    )


def test_side_scan_formatting_and_assessment_helpers() -> None:
    """Форматирование и assessment side scan покрывают основные классификации."""
    controller: DriveController = _controller()
    summary: DistanceMeasurementSummary = DistanceMeasurementSummary(
        median_cm=20.0,
        raw_samples_cm=(18.0, 20.0, 22.0),
        min_cm=18.0,
        max_cm=22.0,
        spread_cm=4.0,
    )
    result: SideScanResult = SideScanResult(
        side=AvoidanceSide.LEFT,
        target_angle_deg=45.0,
        turned_angle_deg=30.0,
        clearance_cm=30.0,
        heading_restored=True,
        scan_completed=False,
        scan_useful=True,
        limited_confidence=True,
        measurement_summary=summary,
    )
    controller._annotate_side_scan_result(
        result=result,
        primary_result=result,
        secondary_result=SideScanResult(
            side=AvoidanceSide.LEFT,
            target_angle_deg=60.0,
            turned_angle_deg=60.0,
            clearance_cm=35.0,
            heading_restored=True,
            scan_completed=True,
        ),
        selection_reason="ok",
    )

    assert controller._scan_clearance_for_log([result], AvoidanceSide.LEFT) == "30.0"
    result.clearance_cm = None
    assert controller._scan_clearance_for_log([result], AvoidanceSide.LEFT) == "n/a"
    result.clearance_cm = 30.0
    assert controller._scan_clearance_for_log([result], AvoidanceSide.RIGHT) == "n/a"
    assert "measurement=samples" in controller._format_side_scan_result(result)
    assert controller._format_measurement_summary(None) == "n/a"
    assert (
        controller._side_selection_assessment(
            result=result,
            preferred_threshold_cm=25.0,
            exploratory_threshold_cm=22.0,
        )[0]
        == "preferred"
    )

    result.clearance_cm = 23.0
    assert (
        controller._side_selection_assessment(
            result=result,
            preferred_threshold_cm=25.0,
            exploratory_threshold_cm=22.0,
        )[0]
        == "borderline"
    )
    result.clearance_cm = 10.0
    result.turn_stop_reason = "timeout"
    assert (
        controller._side_selection_assessment(
            result=result,
            preferred_threshold_cm=25.0,
            exploratory_threshold_cm=22.0,
        )[0]
        == "inconclusive"
    )
    result.turn_stop_reason = "target_reached"
    result.limited_confidence = False
    assert (
        controller._side_selection_assessment(
            result=result,
            preferred_threshold_cm=25.0,
            exploratory_threshold_cm=22.0,
        )[0]
        == "rejected"
    )


def _scan_result(
    side: AvoidanceSide,
    *,
    clearance_cm: float | None,
    selectable: bool = True,
    limited_confidence: bool = False,
    stop_reason: str = "target_reached",
    selection_blocked: bool = False,
    secondary: bool = False,
) -> SideScanResult:
    """Создать SideScanResult для тестов выбора стороны."""
    return SideScanResult(
        side=side,
        target_angle_deg=60.0 if secondary else 45.0,
        turned_angle_deg=60.0 if secondary else 45.0,
        clearance_cm=clearance_cm,
        heading_restored=selectable,
        scan_completed=True,
        scan_useful=selectable and clearance_cm is not None,
        limited_confidence=limited_confidence,
        turn_stop_reason=stop_reason,
        selection_blocked=selection_blocked,
        secondary_target_angle_deg=60.0 if secondary else None,
    )


def test_select_avoidance_side_preferred_borderline_inconclusive_and_none() -> None:
    """_select_avoidance_side выбирает preferred, borderline, inconclusive или None."""
    controller: DriveController = _controller()
    cases: list[tuple[list[SideScanResult], AvoidanceSide | None]] = [
        (
            [
                _scan_result(AvoidanceSide.LEFT, clearance_cm=30.0),
                _scan_result(AvoidanceSide.RIGHT, clearance_cm=26.0),
            ],
            AvoidanceSide.LEFT,
        ),
        (
            [
                _scan_result(AvoidanceSide.LEFT, clearance_cm=23.0),
                _scan_result(AvoidanceSide.RIGHT, clearance_cm=10.0),
            ],
            AvoidanceSide.LEFT,
        ),
        (
            [
                _scan_result(AvoidanceSide.LEFT, clearance_cm=10.0, limited_confidence=True),
                _scan_result(AvoidanceSide.RIGHT, clearance_cm=9.0),
            ],
            AvoidanceSide.LEFT,
        ),
        (
            [
                _scan_result(AvoidanceSide.LEFT, clearance_cm=None, selectable=False),
                _scan_result(AvoidanceSide.RIGHT, clearance_cm=None, selectable=False),
            ],
            None,
        ),
    ]

    for results, expected_side in cases:
        queue: list[SideScanResult] = list(results)
        controller._scan_side_observation = lambda side, queue=queue: queue.pop(0)  # type: ignore[method-assign]
        assert controller._select_avoidance_side() is expected_side


def test_scan_side_observation_primary_secondary_branches() -> None:
    """_scan_side_observation покрывает primary return и secondary fallback ветки."""
    controller: DriveController = _controller()
    primary_good: SideScanResult = _scan_result(AvoidanceSide.LEFT, clearance_cm=30.0)
    controller._scan_side_observation_at_angle = lambda **kwargs: primary_good  # type: ignore[method-assign]
    assert controller._scan_side_observation(AvoidanceSide.LEFT) is primary_good

    primary_low: SideScanResult = _scan_result(AvoidanceSide.LEFT, clearance_cm=21.0)
    secondary_bad: SideScanResult = _scan_result(
        AvoidanceSide.LEFT,
        clearance_cm=None,
        selectable=False,
        secondary=True,
    )
    queue: list[SideScanResult] = [primary_low, secondary_bad]
    controller._scan_side_observation_at_angle = lambda **kwargs: queue.pop(0)  # type: ignore[method-assign]
    unusable: SideScanResult = controller._scan_side_observation(AvoidanceSide.LEFT)
    assert unusable.selection_blocked is True
    assert "unusable" in (unusable.selection_reason or "")

    primary_low = _scan_result(AvoidanceSide.LEFT, clearance_cm=21.0)
    secondary_low: SideScanResult = _scan_result(
        AvoidanceSide.LEFT,
        clearance_cm=21.5,
        secondary=True,
    )
    queue = [primary_low, secondary_low]
    controller._scan_side_observation_at_angle = lambda **kwargs: queue.pop(0)  # type: ignore[method-assign]
    blocked: SideScanResult = controller._scan_side_observation(AvoidanceSide.LEFT)
    assert blocked.selection_blocked is True
    assert blocked.secondary_clearance_cm == 21.5

    primary_low = _scan_result(AvoidanceSide.LEFT, clearance_cm=21.0)
    secondary_good: SideScanResult = _scan_result(
        AvoidanceSide.LEFT,
        clearance_cm=24.0,
        secondary=True,
    )
    queue = [primary_low, secondary_good]
    controller._scan_side_observation_at_angle = lambda **kwargs: queue.pop(0)  # type: ignore[method-assign]
    accepted: SideScanResult = controller._scan_side_observation(AvoidanceSide.LEFT)
    assert accepted.selection_blocked is False
    assert "accepted after 60" in (accepted.selection_reason or "")


def test_scan_side_observation_at_angle_variants(monkeypatch: pytest.MonkeyPatch) -> None:
    """_scan_side_observation_at_angle покрывает not_needed, partial, rejection и restore fail."""
    controller: DriveController = _controller()
    controller._measure_front_distance_summary = lambda samples=None: DistanceMeasurementSummary(  # type: ignore[method-assign]
        median_cm=33.0,
        raw_samples_cm=(33.0, 33.0, 33.0),
        min_cm=33.0,
        max_cm=33.0,
        spread_cm=0.0,
    )
    controller._log_side_scan_result = lambda result: None  # type: ignore[method-assign]
    monkeypatch.setattr(
        "src.application.services.drive_controller.time.sleep",
        lambda seconds: None,
    )

    not_needed: SideScanResult = controller._scan_side_observation_at_angle(
        side=AvoidanceSide.LEFT,
        target_angle=0.0,
    )
    assert not_needed.turn_stop_reason == "not_needed"
    assert not_needed.clearance_cm == 33.0

    turns: list[TurnResult] = [TurnResult(False, 25.0, "timeout"), TurnResult(True, 25.0)]
    controller._turn_relative = lambda **kwargs: turns.pop(0)  # type: ignore[method-assign]
    partial: SideScanResult = controller._scan_side_observation_at_angle(
        side=AvoidanceSide.LEFT,
        target_angle=45.0,
    )
    assert partial.used_partial_scan is True
    assert partial.limited_confidence is True

    turns = [TurnResult(False, 1.0, "timeout"), TurnResult(True, 1.0)]
    controller._turn_relative = lambda **kwargs: turns.pop(0)  # type: ignore[method-assign]
    rejected: SideScanResult = controller._scan_side_observation_at_angle(
        side=AvoidanceSide.LEFT,
        target_angle=45.0,
    )
    assert rejected.clearance_cm is None
    assert "недостаточный угол" in (rejected.rejection_reason or "")

    turns = [TurnResult(True, 45.0, "target_reached"), TurnResult(False, 10.0)]
    controller._turn_relative = lambda **kwargs: turns.pop(0)  # type: ignore[method-assign]
    restore_failed: SideScanResult = controller._scan_side_observation_at_angle(
        side=AvoidanceSide.LEFT,
        target_angle=45.0,
    )
    assert restore_failed.heading_restored is False
    assert "восстановить курс" in (restore_failed.rejection_reason or "")


def test_secondary_target_default_reason_annotate_and_clearance_helpers() -> None:
    """Side scan мелкие helpers покрывают none/reason/rejection branches."""
    controller: DriveController = _controller()
    assert controller._secondary_side_scan_target_angle(60.0) is None
    assert controller._secondary_side_scan_target_angle(45.0) == 60.0

    unusable: SideScanResult = _scan_result(AvoidanceSide.LEFT, clearance_cm=None, selectable=False)
    unusable.rejection_reason = "bad"
    assert (
        controller._default_side_scan_selection_reason(
            result=unusable,
            exploratory_threshold_cm=22.0,
        )
        == "bad"
    )
    below: SideScanResult = _scan_result(AvoidanceSide.LEFT, clearance_cm=10.0)
    assert "will try" in controller._default_side_scan_selection_reason(
        result=below,
        exploratory_threshold_cm=22.0,
    )

    annotated: SideScanResult = controller._annotate_side_scan_result(
        result=below,
        primary_result=below,
        selection_blocked=True,
        selection_reason="blocked reason",
    )
    assert annotated.rejection_reason == "blocked reason"
    controller._scan_side_observation = lambda side: unusable  # type: ignore[method-assign]
    assert controller._scan_side_clearance(AvoidanceSide.LEFT) is None
    usable: SideScanResult = _scan_result(AvoidanceSide.LEFT, clearance_cm=42.0)
    controller._scan_side_observation = lambda side: usable  # type: ignore[method-assign]
    assert controller._scan_side_clearance(AvoidanceSide.LEFT) == 42.0


def test_describe_side_selection_failure_variants() -> None:
    """_describe_side_selection_failure возвращает разные сводные причины."""
    controller: DriveController = _controller()
    assert "restore heading" in controller._describe_side_selection_failure(
        scan_results=[
            _scan_result(AvoidanceSide.LEFT, clearance_cm=10.0, selectable=False),
            _scan_result(AvoidanceSide.RIGHT, clearance_cm=10.0, selectable=False),
        ],
        exploratory_threshold_cm=22.0,
    )
    assert "not useful" in controller._describe_side_selection_failure(
        scan_results=[
            SideScanResult(AvoidanceSide.LEFT, 45.0, 10.0, None, True, False),
            SideScanResult(AvoidanceSide.RIGHT, 45.0, 10.0, None, True, False),
        ],
        exploratory_threshold_cm=22.0,
    )
    assert "60" in controller._describe_side_selection_failure(
        scan_results=[
            _scan_result(
                AvoidanceSide.LEFT,
                clearance_cm=10.0,
                selection_blocked=True,
                secondary=True,
            ),
            _scan_result(
                AvoidanceSide.RIGHT,
                clearance_cm=10.0,
                selection_blocked=True,
                secondary=True,
            ),
        ],
        exploratory_threshold_cm=22.0,
    )
    assert "below exploratory" in controller._describe_side_selection_failure(
        scan_results=[
            _scan_result(AvoidanceSide.LEFT, clearance_cm=10.0),
            _scan_result(AvoidanceSide.RIGHT, clearance_cm=11.0),
        ],
        exploratory_threshold_cm=22.0,
    )
    assert "no usable" in controller._describe_side_selection_failure(
        scan_results=[
            _scan_result(AvoidanceSide.LEFT, clearance_cm=10.0, limited_confidence=True),
            _scan_result(AvoidanceSide.RIGHT, clearance_cm=None, selectable=False),
        ],
        exploratory_threshold_cm=22.0,
    )


def test_wait_and_log_helpers(monkeypatch: pytest.MonkeyPatch) -> None:
    """Wait/log helpers покрывают ранний return и обычный sleep/log path."""
    controller: DriveController = _controller()
    sleep_calls: list[float] = []
    monkeypatch.setattr("src.application.services.drive_controller.time.sleep", sleep_calls.append)

    controller.update_interval_sec = -1.0
    controller.SIDE_SCAN_SENSOR_SETTLE_SEC = -1.0
    controller._wait_for_side_scan_sensor_settle()
    controller.SIDE_SCAN_SENSOR_SETTLE_SEC = 0.07
    controller._wait_for_side_scan_sensor_settle()
    controller._log_lateral_offset_recovery(reason="test", lateral_offset_cm=0.0)

    assert sleep_calls == [0.07]


def test_run_obstacle_avoidance_immediate_success_and_no_side() -> None:
    """FSM обхода покрывает ранний успех и отсутствие стороны обхода."""
    controller: DriveController = _controller()
    controller.lifecycle.set_running(is_route_running=False)
    assert controller._run_obstacle_avoidance(remaining_cm=0.0, speed_percent=50).completed is True

    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: None  # type: ignore[method-assign]
    result: AvoidanceResult = controller._run_obstacle_avoidance(
        remaining_cm=10.0,
        speed_percent=50,
    )

    assert result.completed is False
    assert controller.motor_controller.stop_calls >= 1  # type: ignore[attr-defined]


def test_run_obstacle_avoidance_happy_path() -> None:
    """FSM обхода проходит side step, forward step и rejoin до успешного завершения."""
    controller: DriveController = _controller()
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    controller._perform_side_step = lambda side, speed_percent: LinearMoveResult(True, 10.0)  # type: ignore[method-assign]
    controller._is_front_clear_confirmed = lambda: True  # type: ignore[method-assign]
    controller._perform_forward_step = lambda remaining_cm, speed_percent: LinearMoveResult(
        True, 9.0
    )  # type: ignore[method-assign]
    controller._attempt_rejoin_step = lambda side, lateral_offset_cm, speed_percent: (
        LinearMoveResult(True, 10.0)
    )  # type: ignore[method-assign]

    result: AvoidanceResult = controller._run_obstacle_avoidance(
        remaining_cm=20.0,
        speed_percent=50,
    )

    assert result.completed is True
    assert result.forward_progress_cm == 9.0


@pytest.mark.parametrize(
    ("side_step", "expected_progress"),
    [
        (LinearMoveResult(False, 0.0, blocked=False, heading_restored=False), 0.0),
        (LinearMoveResult(False, 2.0, blocked=False), 0.0),
        (LinearMoveResult(False, 0.0, blocked=True), 0.0),
    ],
)
def test_run_obstacle_avoidance_side_step_failure_branches(
    side_step: LinearMoveResult,
    expected_progress: float,
) -> None:
    """FSM обхода останавливается на неудачных вариантах бокового шага."""
    controller: DriveController = _controller()
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    controller._perform_side_step = lambda side, speed_percent: side_step  # type: ignore[method-assign]

    result: AvoidanceResult = controller._run_obstacle_avoidance(
        remaining_cm=10.0,
        speed_percent=50,
    )

    assert result.completed is False
    assert result.forward_progress_cm == expected_progress


def test_run_obstacle_avoidance_front_not_clear_retries_until_limit() -> None:
    """FSM возвращается к SIDE_STEP, если фронт после бокового шага не свободен."""
    controller: DriveController = _controller()
    controller.avoidance_max_attempts = 2
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    side_steps: list[LinearMoveResult] = [
        LinearMoveResult(True, 2.0),
        LinearMoveResult(False, 0.0, blocked=True),
    ]
    controller._perform_side_step = lambda side, speed_percent: side_steps.pop(0)  # type: ignore[method-assign]
    front_results: list[bool] = [False]
    controller._is_front_clear_confirmed = lambda: front_results.pop(0)  # type: ignore[method-assign]

    result: AvoidanceResult = controller._run_obstacle_avoidance(
        remaining_cm=10.0,
        speed_percent=50,
    )

    assert result.completed is False
    assert result.forward_progress_cm == 0.0


def test_run_obstacle_avoidance_front_clear_with_recovered_lateral_offset() -> None:
    """FSM завершает обход, если после side step фронт чистый и offset в допуске."""
    controller: DriveController = _controller()
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    controller._perform_side_step = lambda side, speed_percent: LinearMoveResult(True, 0.2)  # type: ignore[method-assign]
    controller._is_front_clear_confirmed = lambda: True  # type: ignore[method-assign]

    result: AvoidanceResult = controller._run_obstacle_avoidance(
        remaining_cm=10.0,
        speed_percent=50,
    )

    assert result.completed is True


def test_run_obstacle_avoidance_forward_progress_completion_branches() -> None:
    """FSM покрывает завершение remaining через forward step с разным lateral offset."""
    controller: DriveController = _controller()
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    controller._perform_side_step = lambda side, speed_percent: LinearMoveResult(True, 0.2)  # type: ignore[method-assign]
    controller._is_front_clear_confirmed = lambda: True  # type: ignore[method-assign]
    controller._perform_forward_step = lambda remaining_cm, speed_percent: LinearMoveResult(
        True, 20.0
    )  # type: ignore[method-assign]
    recovered: AvoidanceResult = controller._run_obstacle_avoidance(
        remaining_cm=10.0,
        speed_percent=50,
    )
    assert recovered.completed is True

    controller = _controller()
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    controller._perform_side_step = lambda side, speed_percent: LinearMoveResult(True, 0.2)  # type: ignore[method-assign]
    controller._is_front_clear_confirmed = lambda: True  # type: ignore[method-assign]
    controller._perform_forward_step = lambda remaining_cm, speed_percent: LinearMoveResult(
        True, 20.0
    )  # type: ignore[method-assign]
    controller._attempt_rejoin_step = lambda side, lateral_offset_cm, speed_percent: (
        LinearMoveResult(
            False,
            0.0,
            heading_restored=False,
        )
    )  # type: ignore[method-assign]
    lateral_checks: list[bool] = [False, True]

    def reached_for_forward_completion(distance_cm: float) -> bool:
        """Сначала пропустить CHECK_FRONT по lateral offset, затем завершить forward branch."""
        if abs(distance_cm - 0.2) < 0.001:
            return lateral_checks.pop(0)
        return distance_cm <= controller.DISTANCE_TOLERANCE_CM

    controller._has_reached_target_distance = reached_for_forward_completion  # type: ignore[method-assign]
    completed_after_forward: AvoidanceResult = controller._run_obstacle_avoidance(
        remaining_cm=10.0,
        speed_percent=50,
    )
    assert completed_after_forward.completed is True


def test_run_obstacle_avoidance_forward_step_completed_with_recovered_offset() -> None:
    """FSM завершает обход после forward step, если offset уже в допуске."""
    controller: DriveController = _controller()
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    controller._perform_side_step = lambda side, speed_percent: LinearMoveResult(True, 0.2)  # type: ignore[method-assign]
    front_results: list[bool] = [True]
    controller._is_front_clear_confirmed = lambda: front_results.pop(0)  # type: ignore[method-assign]
    controller._perform_forward_step = lambda remaining_cm, speed_percent: LinearMoveResult(
        True, 1.0
    )  # type: ignore[method-assign]
    lateral_checks: list[bool] = [False, True]

    def reached_for_offset_recovery(distance_cm: float) -> bool:
        """Сначала пропустить CHECK_FRONT, затем завершить branch после forward step."""
        if abs(distance_cm - 0.2) < 0.001:
            return lateral_checks.pop(0)
        return distance_cm <= controller.DISTANCE_TOLERANCE_CM

    controller._has_reached_target_distance = reached_for_offset_recovery  # type: ignore[method-assign]

    result: AvoidanceResult = controller._run_obstacle_avoidance(
        remaining_cm=10.0,
        speed_percent=50,
    )

    assert result.completed is True


def test_run_obstacle_avoidance_forward_completion_switches_to_rejoin() -> None:
    """FSM переходит в TRY_REJOIN, если forward step закрыл remaining, но lateral offset остался."""
    controller: DriveController = _controller()
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    controller._perform_side_step = lambda side, speed_percent: LinearMoveResult(True, 2.0)  # type: ignore[method-assign]
    controller._is_front_clear_confirmed = lambda: True  # type: ignore[method-assign]
    controller._perform_forward_step = lambda remaining_cm, speed_percent: LinearMoveResult(
        True, 20.0
    )  # type: ignore[method-assign]
    controller._attempt_rejoin_step = lambda side, lateral_offset_cm, speed_percent: (
        LinearMoveResult(
            False,
            0.0,
            heading_restored=False,
        )
    )  # type: ignore[method-assign]

    result: AvoidanceResult = controller._run_obstacle_avoidance(
        remaining_cm=10.0,
        speed_percent=50,
    )

    assert result.completed is False


def test_run_obstacle_avoidance_forward_step_failure_branches() -> None:
    """FSM покрывает blocked и interrupted варианты движения вдоль препятствия."""
    controller: DriveController = _controller()
    controller.avoidance_max_attempts = 2
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    controller._perform_side_step = lambda side, speed_percent: LinearMoveResult(True, 2.0)  # type: ignore[method-assign]
    controller._is_front_clear_confirmed = lambda: True  # type: ignore[method-assign]
    controller._perform_forward_step = lambda remaining_cm, speed_percent: LinearMoveResult(
        False, 1.0, blocked=True
    )  # type: ignore[method-assign]
    assert (
        controller._run_obstacle_avoidance(remaining_cm=10.0, speed_percent=50).completed is False
    )

    controller = _controller()
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    controller._perform_side_step = lambda side, speed_percent: LinearMoveResult(True, 2.0)  # type: ignore[method-assign]
    controller._is_front_clear_confirmed = lambda: True  # type: ignore[method-assign]
    controller._perform_forward_step = lambda remaining_cm, speed_percent: LinearMoveResult(
        False, 1.0, blocked=False
    )  # type: ignore[method-assign]
    assert (
        controller._run_obstacle_avoidance(remaining_cm=10.0, speed_percent=50).completed is False
    )


def test_run_obstacle_avoidance_rejoin_failure_and_retry_branches() -> None:
    """FSM покрывает неудачные и retry варианты rejoin."""
    controller: DriveController = _controller()
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    controller._perform_side_step = lambda side, speed_percent: LinearMoveResult(True, 2.0)  # type: ignore[method-assign]
    controller._is_front_clear_confirmed = lambda: True  # type: ignore[method-assign]
    controller._perform_forward_step = lambda remaining_cm, speed_percent: LinearMoveResult(
        True, 1.0
    )  # type: ignore[method-assign]
    controller._attempt_rejoin_step = lambda side, lateral_offset_cm, speed_percent: (
        LinearMoveResult(
            False,
            0.0,
            heading_restored=False,
        )
    )  # type: ignore[method-assign]
    assert (
        controller._run_obstacle_avoidance(remaining_cm=10.0, speed_percent=50).completed is False
    )

    controller = _controller()
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    controller._perform_side_step = lambda side, speed_percent: LinearMoveResult(True, 2.0)  # type: ignore[method-assign]
    controller._is_front_clear_confirmed = lambda: True  # type: ignore[method-assign]
    controller._perform_forward_step = lambda remaining_cm, speed_percent: LinearMoveResult(
        True, 1.0
    )  # type: ignore[method-assign]
    controller._attempt_rejoin_step = lambda side, lateral_offset_cm, speed_percent: (
        LinearMoveResult(
            False,
            1.0,
            blocked=False,
        )
    )  # type: ignore[method-assign]
    assert (
        controller._run_obstacle_avoidance(remaining_cm=10.0, speed_percent=50).completed is False
    )

    controller = _controller()
    controller.avoidance_max_attempts = 3
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    controller._perform_side_step = lambda side, speed_percent: LinearMoveResult(True, 2.0)  # type: ignore[method-assign]
    controller._is_front_clear_confirmed = lambda: True  # type: ignore[method-assign]
    controller._perform_forward_step = lambda remaining_cm, speed_percent: LinearMoveResult(
        True, 1.0
    )  # type: ignore[method-assign]
    controller._attempt_rejoin_step = lambda side, lateral_offset_cm, speed_percent: (
        LinearMoveResult(
            False,
            0.0,
            blocked=True,
        )
    )  # type: ignore[method-assign]
    assert (
        controller._run_obstacle_avoidance(remaining_cm=10.0, speed_percent=50).completed is False
    )


def test_run_obstacle_avoidance_rejoin_front_blocked_and_final_retry() -> None:
    """FSM покрывает rejoin с заблокированным фронтом и immediate final retry."""
    controller: DriveController = _controller()
    controller.avoidance_max_attempts = 3
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    controller._perform_side_step = lambda side, speed_percent: LinearMoveResult(True, 1.0)  # type: ignore[method-assign]
    front_results: list[bool] = [True, False]
    controller._is_front_clear_confirmed = lambda: front_results.pop(0)  # type: ignore[method-assign]
    controller._perform_forward_step = lambda remaining_cm, speed_percent: LinearMoveResult(
        True, 0.0
    )  # type: ignore[method-assign]
    controller._attempt_rejoin_step = lambda side, lateral_offset_cm, speed_percent: (
        LinearMoveResult(True, 1.0)
    )  # type: ignore[method-assign]
    assert (
        controller._run_obstacle_avoidance(remaining_cm=10.0, speed_percent=50).completed is False
    )

    controller = _controller()
    controller.avoidance_max_attempts = 5
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    controller._perform_side_step = lambda side, speed_percent: LinearMoveResult(True, 1.0)  # type: ignore[method-assign]
    controller._is_front_clear_confirmed = lambda: True  # type: ignore[method-assign]
    controller._perform_forward_step = lambda remaining_cm, speed_percent: LinearMoveResult(
        True, 0.0
    )  # type: ignore[method-assign]
    rejoin_steps: list[LinearMoveResult] = [
        LinearMoveResult(True, 0.25),
        LinearMoveResult(True, 0.25),
        LinearMoveResult(True, 0.25),
    ]
    controller._attempt_rejoin_step = lambda side, lateral_offset_cm, speed_percent: (
        rejoin_steps.pop(0)
    )  # type: ignore[method-assign]
    assert controller._run_obstacle_avoidance(remaining_cm=10.0, speed_percent=50).completed is True


def test_run_obstacle_avoidance_rejoin_returns_to_front_check() -> None:
    """FSM после неполного rejoin возвращается в CHECK_FRONT, если retry window не подходит."""
    controller: DriveController = _controller()
    controller.avoidance_max_attempts = 4
    controller.lifecycle.set_running(is_route_running=False)
    controller._select_avoidance_side = lambda: AvoidanceSide.LEFT  # type: ignore[method-assign]
    controller._perform_side_step = lambda side, speed_percent: LinearMoveResult(True, 10.0)  # type: ignore[method-assign]
    front_results: list[bool] = [True, False]
    controller._is_front_clear_confirmed = lambda: front_results.pop(0)  # type: ignore[method-assign]
    controller._perform_forward_step = lambda remaining_cm, speed_percent: LinearMoveResult(
        True, 1.0
    )  # type: ignore[method-assign]
    controller._attempt_rejoin_step = lambda side, lateral_offset_cm, speed_percent: (
        LinearMoveResult(True, 1.0)
    )  # type: ignore[method-assign]

    result: AvoidanceResult = controller._run_obstacle_avoidance(
        remaining_cm=10.0,
        speed_percent=50,
    )

    assert result.completed is False


def test_run_obstacle_avoidance_remaining_exhausted_switches_to_rejoin() -> None:
    """FSM переводит состояние в TRY_REJOIN, если remaining уже исчерпан, а offset ещё есть."""
    controller: DriveController = _controller()
    controller.lifecycle.set_running(is_route_running=False)
    reached_results: list[bool] = [True, False, False]
    controller._has_reached_target_distance = lambda distance_cm: reached_results.pop(0)  # type: ignore[method-assign]
    controller._has_exceeded_avoidance_limits = lambda context: True  # type: ignore[method-assign]

    result: AvoidanceResult = controller._run_obstacle_avoidance(
        remaining_cm=10.0,
        speed_percent=50,
    )

    assert result.completed is False
