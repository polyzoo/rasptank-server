from __future__ import annotations

import logging
from collections.abc import Callable
from threading import Event, Thread
from typing import final

from src.application.models.route import Route, RouteSegment
from src.application.protocols import (
    DriveControllerProtocol,
    MotorControllerProtocol,
    UltrasonicSensorProtocol,
)
from src.application.services.route_executor import RouteExecutor, RouteExecutorConfig

logger: logging.Logger = logging.getLogger(__name__)


@final
class DriveController(DriveControllerProtocol):
    """Контроллер движения с учетом препятствий и целевой дистанции."""

    # Таймаут ожидания завершения фонового потока движения (секунды)
    STOP_JOIN_TIMEOUT_SEC: float = 1.0

    # Минимальная дистанция до препятствия по умолчанию (см)
    DEFAULT_MIN_OBSTACLE_DISTANCE_CM: float = 20.0

    # Зона плавного торможения по умолчанию (см)
    DEFAULT_DECELERATION_DISTANCE_CM: float = 50.0

    # Базовая скорость движения по умолчанию (%)
    DEFAULT_BASE_SPEED_PERCENT: int = 60

    # Скорость поворотов по умолчанию (%)
    DEFAULT_TURN_SPEED_PERCENT: int = 50

    # Линейная скорость при 100% по умолчанию (см/с)
    DEFAULT_MAX_SPEED_CM_PER_SEC: float = 30.0

    # Интервал опроса датчика по умолчанию (с)
    DEFAULT_UPDATE_INTERVAL_SEC: float = 0.1

    # Таймаут ожидания перед препятствием по умолчанию (с)
    DEFAULT_OBSTACLE_CANNOT_BYPASS_TIMEOUT_SEC: float = 30.0

    # Длительность поворота на 90° по умолчанию (с)
    DEFAULT_TURN_DURATION_90_DEG_SEC: float = 0.5

    def __init__(
        self,
        motor_controller: MotorControllerProtocol,
        ultrasonic_sensor: UltrasonicSensorProtocol,
        min_obstacle_distance_cm: float = DEFAULT_MIN_OBSTACLE_DISTANCE_CM,
        deceleration_distance_cm: float = DEFAULT_DECELERATION_DISTANCE_CM,
        base_speed_percent: int = DEFAULT_BASE_SPEED_PERCENT,
        turn_speed_percent: int = DEFAULT_TURN_SPEED_PERCENT,
        max_speed_cm_per_sec: float = DEFAULT_MAX_SPEED_CM_PER_SEC,
        update_interval_sec: float = DEFAULT_UPDATE_INTERVAL_SEC,
        obstacle_cannot_bypass_timeout_sec: float = DEFAULT_OBSTACLE_CANNOT_BYPASS_TIMEOUT_SEC,
        on_obstacle_cannot_bypass: Callable[[], None] | None = None,
        turn_duration_90_deg_sec: float = DEFAULT_TURN_DURATION_90_DEG_SEC,
        on_trajectory_event: Callable[[dict], None] | None = None,
    ) -> None:
        """Инициализация контроллера движения.

        Args:
            motor_controller: Низкоуровневое управление моторами.
            ultrasonic_sensor: Датчик расстояния до препятствий.
            min_obstacle_distance_cm: Дистанция остановки перед препятствием (см).
            deceleration_distance_cm: Зона плавного торможения (см).
            base_speed_percent: Базовая скорость движения (%).
            turn_speed_percent: Скорость поворотов (%).
            max_speed_cm_per_sec: Линейная скорость при 100% для оценки пути.
            update_interval_sec: Интервал опроса датчика (с).
            obstacle_cannot_bypass_timeout_sec: Ожидание перед препятствием до уведомления (с).
            on_obstacle_cannot_bypass: Callback при невозможности объезда.
            turn_duration_90_deg_sec: Длительность поворота на 90° (с).
            on_trajectory_event: Callback для событий траектории.
        """
        config = RouteExecutorConfig(
            min_obstacle_distance_cm=min_obstacle_distance_cm,
            deceleration_distance_cm=deceleration_distance_cm,
            base_speed_percent=base_speed_percent,
            turn_speed_percent=turn_speed_percent,
            max_speed_cm_per_sec=max_speed_cm_per_sec,
            update_interval_sec=update_interval_sec,
            obstacle_cannot_bypass_timeout_sec=obstacle_cannot_bypass_timeout_sec,
            turn_duration_90_deg_sec=turn_duration_90_deg_sec,
        )
        self._executor: RouteExecutor = RouteExecutor(
            motor_controller=motor_controller,
            ultrasonic_sensor=ultrasonic_sensor,
            config=config,
            on_obstacle_cannot_bypass=on_obstacle_cannot_bypass,
            on_trajectory_event=on_trajectory_event,
        )
        self._motor: MotorControllerProtocol = motor_controller
        self._base_speed_percent: int = base_speed_percent
        self._sensor: UltrasonicSensorProtocol = ultrasonic_sensor

        self._is_moving: bool = False
        self._is_route_running: bool = False
        self._movement_thread: Thread | None = None
        self._stop_event: Event = Event()

    def forward_cm_sync(self, distance_cm: float, max_speed_percent: int | None = None) -> None:
        """Движение вперёд на заданное расстояние в текущем потоке."""
        if self._is_moving:
            self.stop()

        self._is_moving: bool = True
        self._stop_event.clear()

        try:
            speed: int = max_speed_percent or self._base_speed_percent
            self._executor.run_forward_segment(distance_cm, speed, self._should_continue)
        finally:
            self._motor.stop()
            self._is_moving: bool = False

    def execute_route(self, route: Route) -> None:
        """Выполнение маршрута в фоновом потоке."""
        if self._is_moving or self._is_route_running:
            self.stop()

        self._is_moving: bool = True
        self._is_route_running: bool = True
        self._stop_event.clear()

        self._movement_thread: Thread = Thread(
            target=self._run_route,
            args=(route.segments,),
            daemon=True,
        )
        self._movement_thread.start()

    def execute_route_sync(self, route: Route) -> None:
        """Выполнение маршрута в текущем потоке."""
        if self._is_moving or self._is_route_running:
            self.stop()

        self._is_moving: bool = True
        self._is_route_running: bool = True
        self._stop_event.clear()

        try:
            self._executor.execute(route.segments, self._should_continue)
        finally:
            self._motor.stop()
            self._is_moving: bool = False
            self._is_route_running: bool = False

    def _run_route(self, segments: list[RouteSegment]) -> None:
        """Цикл выполнения маршрута в daemon-потоке."""
        try:
            self._executor.execute(segments, self._should_continue)
        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.exception("Ошибка при выполнении маршрута: %s", exc)
        finally:
            self._motor.stop()
            self._is_moving: bool = False
            self._is_route_running: bool = False

    def stop(self) -> None:
        """Немедленная остановка движения."""
        self._is_moving: bool = False
        self._is_route_running: bool = False
        self._stop_event.set()
        self._motor.stop()

        if self._movement_thread and self._movement_thread.is_alive():
            self._movement_thread.join(timeout=self.STOP_JOIN_TIMEOUT_SEC)

        self._movement_thread: Thread | None = None

    def destroy(self) -> None:
        """Освобождение ресурсов."""
        self.stop()
        self._motor.destroy()
        self._sensor.destroy()

    def _should_continue(self) -> bool:
        """Флаг продолжения движения для исполнителя движения."""
        return self._is_moving and not self._stop_event.is_set()
