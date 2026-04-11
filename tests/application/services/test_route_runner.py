from __future__ import annotations

from dataclasses import dataclass

from src.application.models.route import (
    BackwardSegment,
    ForwardSegment,
    TurnLeftSegment,
    TurnRightSegment,
)
from src.application.services.motion_lifecycle import MotionLifecycle
from src.application.services.route_runner import RouteRunner
from src.application.services.turn_executor import TurnExecutionResult


class FakeGyroscope:
    """Заглушка гироскопа."""

    def __init__(self) -> None:
        """Подготовить счетчики start/stop."""
        self.start_calls: list[bool] = []
        self.stop_calls: int = 0

    def start(self, calibrate: bool = True) -> None:
        """Зафиксировать запуск."""
        self.start_calls.append(calibrate)

    def stop(self) -> None:
        """Зафиксировать остановку."""
        self.stop_calls += 1


class FakeLinearMotion:
    """Заглушка linear motion executor."""

    def __init__(self) -> None:
        """Подготовить историю вызовов."""
        self.calls: list[dict[str, object]] = []
        self.result: bool = True

    def run(self, **kwargs: object) -> bool:
        """Сохранить параметры запуска и вернуть настроенный результат."""
        self.calls.append(kwargs)
        return self.result


class FakeTurnExecutor:
    """Заглушка turn executor."""

    def __init__(self) -> None:
        """Подготовить историю поворотов."""
        self.calls: list[dict[str, object]] = []

    def run(self, **kwargs: object) -> TurnExecutionResult:
        """Сохранить параметры поворота и вернуть успешный результат."""
        self.calls.append(kwargs)
        return TurnExecutionResult(completed=True, angle_deg=float(kwargs["requested_angle_deg"]))


@dataclass(slots=True)
class UnknownSegment:
    """Сегмент с неизвестным action."""

    action: str = "unknown"


def _route_runner(
    *,
    lifecycle: MotionLifecycle | None = None,
    linear_motion: FakeLinearMotion | None = None,
    turn_executor: FakeTurnExecutor | None = None,
    gyroscope: FakeGyroscope | None = None,
    forward_segment_runner=None,
    backward_segment_runner=None,
    turn_completed_callback=None,
) -> RouteRunner:
    """Создать RouteRunner с fake зависимостями."""
    return RouteRunner(
        linear_motion=linear_motion or FakeLinearMotion(),  # type: ignore[arg-type]
        turn_executor=turn_executor or FakeTurnExecutor(),  # type: ignore[arg-type]
        move_forward=lambda **kwargs: None,
        move_backward=lambda **kwargs: None,
        gyroscope=gyroscope or FakeGyroscope(),  # type: ignore[arg-type]
        lifecycle=lifecycle or MotionLifecycle(),
        base_speed_percent=55,
        forward_segment_runner=forward_segment_runner,
        backward_segment_runner=backward_segment_runner,
        turn_completed_callback=turn_completed_callback,
    )


def test_run_starts_and_stops_gyroscope() -> None:
    """run запускает гироскоп на маршрут и останавливает его в finally."""
    lifecycle: MotionLifecycle = MotionLifecycle()
    lifecycle.set_running(is_route_running=True)
    gyroscope: FakeGyroscope = FakeGyroscope()
    runner: RouteRunner = _route_runner(lifecycle=lifecycle, gyroscope=gyroscope)

    runner.run([UnknownSegment()])  # type: ignore[list-item]

    assert gyroscope.start_calls == [True]
    assert gyroscope.stop_calls == 1


def test_segments_use_custom_runners_and_turn_callback() -> None:
    """Forward/backward runners и turn callback вызываются для соответствующих сегментов."""
    lifecycle: MotionLifecycle = MotionLifecycle()
    lifecycle.set_running(is_route_running=True)
    forward_calls: list[tuple[float, int]] = []
    backward_calls: list[tuple[float, int]] = []
    turn_calls: list[tuple[float, bool]] = []
    turn_executor: FakeTurnExecutor = FakeTurnExecutor()

    def forward_runner(distance: float, speed: int) -> bool:
        """Сохранить вызов прямого сегмента и вернуть успех."""
        forward_calls.append((distance, speed))
        return True

    def backward_runner(distance: float, speed: int) -> bool:
        """Сохранить вызов заднего сегмента и вернуть успех."""
        backward_calls.append((distance, speed))
        return True

    runner: RouteRunner = _route_runner(
        lifecycle=lifecycle,
        turn_executor=turn_executor,
        forward_segment_runner=forward_runner,
        backward_segment_runner=backward_runner,
        turn_completed_callback=lambda angle, left: turn_calls.append((angle, left)),
    )

    runner._run_segments(
        [
            ForwardSegment(distance_cm=10.0),
            BackwardSegment(distance_cm=5.0),
            TurnLeftSegment(angle_deg=45.0),
            TurnRightSegment(angle_deg=90.0),
        ]
    )

    assert forward_calls == [(10.0, 55)]
    assert backward_calls == [(5.0, 55)]
    assert turn_calls == [(45.0, True), (90.0, False)]
    assert turn_executor.calls[0]["turn_left"] is True
    assert turn_executor.calls[1]["turn_left"] is False


def test_segments_fall_back_to_linear_motion() -> None:
    """Без custom runners прямые сегменты выполняются через LinearMotionExecutor."""
    lifecycle: MotionLifecycle = MotionLifecycle()
    lifecycle.set_running(is_route_running=True)
    linear_motion: FakeLinearMotion = FakeLinearMotion()
    runner: RouteRunner = _route_runner(lifecycle=lifecycle, linear_motion=linear_motion)

    runner._run_segments([ForwardSegment(distance_cm=10.0), BackwardSegment(distance_cm=5.0)])

    assert linear_motion.calls[0]["distance_cm"] == 10.0
    assert linear_motion.calls[0]["obstacle_aware"] is True
    assert linear_motion.calls[1]["distance_cm"] == 5.0
    assert linear_motion.calls[1]["obstacle_aware"] is False


def test_run_segments_stops_when_lifecycle_stops_or_segment_fails() -> None:
    """_run_segments прекращает маршрут при остановке lifecycle или False от сегмента."""
    stopped_lifecycle: MotionLifecycle = MotionLifecycle()
    stopped_runner: RouteRunner = _route_runner(lifecycle=stopped_lifecycle)

    stopped_runner._run_segments([ForwardSegment(distance_cm=10.0)])

    assert stopped_runner._linear_motion.calls == []  # type: ignore[attr-defined]

    lifecycle: MotionLifecycle = MotionLifecycle()
    lifecycle.set_running(is_route_running=True)
    forward_calls: list[tuple[float, int]] = []

    def forward_runner(distance: float, speed: int) -> bool:
        """Сохранить вызов и вернуть признак неудачи сегмента."""
        forward_calls.append((distance, speed))
        return False

    runner: RouteRunner = _route_runner(
        lifecycle=lifecycle,
        forward_segment_runner=forward_runner,
    )

    runner._run_segments([ForwardSegment(distance_cm=10.0), BackwardSegment(distance_cm=5.0)])

    assert forward_calls == [(10.0, 55)]

    backward_calls: list[tuple[float, int]] = []

    def backward_runner_fn(distance: float, speed: int) -> bool:
        """Сохранить вызов и вернуть признак неудачи сегмента."""
        backward_calls.append((distance, speed))
        return False

    backward_runner: RouteRunner = _route_runner(
        lifecycle=lifecycle,
        backward_segment_runner=backward_runner_fn,
    )

    backward_runner._run_segments(
        [
            BackwardSegment(distance_cm=5.0),
            ForwardSegment(distance_cm=10.0),
        ]
    )

    assert backward_calls == [(5.0, 55)]
