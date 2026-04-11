from __future__ import annotations

from src.application.models.route import ForwardSegment, Route
from src.application.services import route_executor as route_executor_module
from src.application.services.motion_lifecycle import MotionLifecycle
from src.application.services.route_executor import RouteExecutor


class FakeMotor:
    """Заглушка контроллера моторов."""

    def __init__(self) -> None:
        """Подготовить счетчик остановок."""
        self.stop_calls: int = 0

    def stop(self) -> None:
        """Зафиксировать остановку моторов."""
        self.stop_calls += 1


class FakeRouteRunner:
    """Заглушка RouteRunner."""

    def __init__(self) -> None:
        """Подготовить историю сегментов."""
        self.calls: list[list[object]] = []
        self.raise_error: bool = False

    def run(self, segments: list[object]) -> None:
        """Сохранить сегменты или сымитировать ошибку."""
        self.calls.append(segments)
        if self.raise_error:
            raise RuntimeError("route failed")


class FakeThread:
    """Заглушка Thread для проверки execute без фонового запуска."""

    instances: list["FakeThread"] = []

    def __init__(self, *, target, args: tuple[object, ...], daemon: bool) -> None:
        """Сохранить параметры потока."""
        self.target = target
        self.args: tuple[object, ...] = args
        self.daemon: bool = daemon
        self.started: bool = False
        FakeThread.instances.append(self)

    def start(self) -> None:
        """Зафиксировать запуск."""
        self.started = True

    def is_alive(self) -> bool:
        """Вернуть, что поток активен."""
        return True


def test_execute_sync_runs_segments() -> None:
    """execute_sync передает сегменты в RouteRunner."""
    runner: FakeRouteRunner = FakeRouteRunner()
    executor: RouteExecutor = RouteExecutor(
        route_runner=runner,  # type: ignore[arg-type]
        motor_controller=FakeMotor(),  # type: ignore[arg-type]
        lifecycle=MotionLifecycle(),
    )
    route: Route = Route(segments=[ForwardSegment(distance_cm=10.0)])

    executor.execute_sync(route)

    assert runner.calls == [route.segments]


def test_execute_creates_and_attaches_background_thread(monkeypatch) -> None:
    """execute создает daemon Thread, сохраняет его в lifecycle и запускает."""
    FakeThread.instances = []
    monkeypatch.setattr(route_executor_module, "Thread", FakeThread)
    lifecycle: MotionLifecycle = MotionLifecycle()
    route: Route = Route(segments=[ForwardSegment(distance_cm=10.0)])
    executor: RouteExecutor = RouteExecutor(
        route_runner=FakeRouteRunner(),  # type: ignore[arg-type]
        motor_controller=FakeMotor(),  # type: ignore[arg-type]
        lifecycle=lifecycle,
    )

    executor.execute(route)

    thread: FakeThread = FakeThread.instances[0]
    assert thread.args == (route.segments,)
    assert thread.daemon is True
    assert thread.started is True
    assert lifecycle._movement_thread is thread


def test_stop_marks_lifecycle_stopped() -> None:
    """stop переводит lifecycle в stopped."""
    lifecycle: MotionLifecycle = MotionLifecycle()
    lifecycle.set_running(is_route_running=True)
    executor: RouteExecutor = RouteExecutor(
        route_runner=FakeRouteRunner(),  # type: ignore[arg-type]
        motor_controller=FakeMotor(),  # type: ignore[arg-type]
        lifecycle=lifecycle,
    )

    executor.stop()

    assert lifecycle.is_moving is False
    assert lifecycle.stop_event.is_set() is True


def test_run_route_loop_stops_motor_calls_callback_and_clears_lifecycle() -> None:
    """_run_route_loop корректно завершает маршрут и вызывает callback."""
    lifecycle: MotionLifecycle = MotionLifecycle()
    motor: FakeMotor = FakeMotor()
    runner: FakeRouteRunner = FakeRouteRunner()
    finished_calls: list[None] = []
    executor: RouteExecutor = RouteExecutor(
        route_runner=runner,  # type: ignore[arg-type]
        motor_controller=motor,  # type: ignore[arg-type]
        lifecycle=lifecycle,
        on_finished=lambda: finished_calls.append(None),
    )
    segments: list[object] = [ForwardSegment(distance_cm=10.0)]

    executor._run_route_loop(segments)

    assert runner.calls == [segments]
    assert motor.stop_calls == 1
    assert lifecycle.is_moving is False
    assert finished_calls == [None]


def test_run_route_loop_handles_runner_errors() -> None:
    """_run_route_loop останавливает моторы даже при ошибке RouteRunner."""
    motor: FakeMotor = FakeMotor()
    runner: FakeRouteRunner = FakeRouteRunner()
    runner.raise_error = True
    executor: RouteExecutor = RouteExecutor(
        route_runner=runner,  # type: ignore[arg-type]
        motor_controller=motor,  # type: ignore[arg-type]
        lifecycle=MotionLifecycle(),
    )

    executor._run_route_loop([ForwardSegment(distance_cm=10.0)])

    assert motor.stop_calls == 1
