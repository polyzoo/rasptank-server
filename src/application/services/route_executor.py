from __future__ import annotations

import logging
from threading import Thread

from src.application.models.route import Route
from src.application.protocols import MotorControllerProtocol
from src.application.services.motion_lifecycle import MotionLifecycle
from src.application.services.route_runner import RouteRunner

logger: logging.Logger = logging.getLogger(__name__)


class RouteExecutor:
    """Исполнитель выполнения маршрута и управления его фоновым потоком."""

    def __init__(
        self,
        route_runner: RouteRunner,
        motor_controller: MotorControllerProtocol,
        lifecycle: MotionLifecycle,
    ) -> None:
        """Инициализация исполнителя маршрута."""
        self._route_runner: RouteRunner = route_runner
        self._motor_controller: MotorControllerProtocol = motor_controller
        self._lifecycle: MotionLifecycle = lifecycle

    def execute(self, route: Route) -> None:
        """Запустить маршрут в фоновом потоке."""
        movement_thread: Thread = Thread(
            target=self._run_route_loop,
            args=(route.segments,),
            daemon=True,
        )
        self._lifecycle.attach_thread(movement_thread)
        movement_thread.start()

    def execute_sync(self, route: Route) -> None:
        """Запустить маршрут в текущем потоке."""
        self._run_route_segments(route.segments)

    def stop(self) -> None:
        """Остановить текущий маршрут."""
        self._lifecycle.set_stopped()

    def _run_route_loop(self, segments: list) -> None:
        """Выполнить маршрут в фоне и корректно завершить поток."""
        try:
            self._run_route_segments(segments)
        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.exception("Ошибка при выполнении маршрута: %s", exc)
        finally:
            self._motor_controller.stop()
            self._lifecycle.set_stopped()
            self._lifecycle.clear_thread_if_current()

    def _run_route_segments(self, segments: list) -> None:
        """Передать сегменты в маршрутный обработчик."""
        self._route_runner.run(segments)
