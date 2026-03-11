from __future__ import annotations

import logging
import time
from threading import Event, Thread
from typing import Optional, final

from src.application.models.route import (
    BackwardSegment,
    ForwardSegment,
    Route,
    RouteSegment,
    TurnLeftSegment,
    TurnRightSegment,
)
from src.application.protocols import (
    DriveControllerProtocol,
    MotorControllerProtocol,
    UltrasonicSensorProtocol,
)

logger: logging.Logger = logging.getLogger(__name__)


@final
class DriveController(DriveControllerProtocol):
    """Контроллер движения с учетом препятствий и целевой дистанции."""

    def __init__(
        self,
        motor_controller: MotorControllerProtocol,
        ultrasonic_sensor: UltrasonicSensorProtocol,
        min_obstacle_distance_cm: float = 20.0,
        deceleration_distance_cm: float = 50.0,
        base_speed_percent: int = 60,
        turn_speed_percent: int = 50,
        max_speed_cm_per_sec: float = 30.0,
        update_interval_sec: float = 0.1,
    ) -> None:
        """Инициализация контроллера движения.

        Args:
            motor_controller: Низкоуровневое управление моторами.
            ultrasonic_sensor: Датчик расстояния до препятствий.
            min_obstacle_distance_cm: Дистанция остановки (см). Ниже — скорость 0.
            deceleration_distance_cm: Зона торможения по препятствию (см).
            base_speed_percent: Базовая скорость (%), если 'max_speed' не задан.
            turn_speed_percent: Скорость поворотов на месте (%).
            max_speed_cm_per_sec: Линейная скорость при 100% для оценки пройденного пути.
            update_interval_sec: Интервал опроса датчика и пересчета скорости (с).
        """
        self.motor_controller: MotorControllerProtocol = motor_controller
        self.ultrasonic_sensor: UltrasonicSensorProtocol = ultrasonic_sensor
        self.min_obstacle_distance_cm: float = min_obstacle_distance_cm
        self.deceleration_distance_cm: float = deceleration_distance_cm
        self.base_speed_percent: int = base_speed_percent
        self.turn_speed_percent: int = turn_speed_percent
        self.max_speed_cm_per_sec: float = max_speed_cm_per_sec
        self.update_interval_sec: float = update_interval_sec

        self._is_moving: bool = False
        self._is_route_running: bool = False
        self._target_distance_cm: Optional[float] = None
        self._traveled_distance_cm: float = 0.0
        self._max_speed_percent: Optional[int] = None
        self._movement_thread: Optional[Thread] = None
        self._stop_event: Event = Event()

    def forward_cm(self, distance_cm: float, max_speed_percent: Optional[int] = None) -> None:
        """Запуск движения вперед на заданное расстояние.

        Не блокирует: движение выполняется в daemon-потоке. При уже идущем движении
        предыдущее останавливается и запускается новое.

        Args:
            distance_cm: Целевое расстояние в сантиметрах.
            max_speed_percent: Ограничение скорости (0 – 100%).
        """
        if self._is_moving:
            self.stop()

        self._target_distance_cm: float = distance_cm
        self._traveled_distance_cm: float = 0.0
        self._max_speed_percent: int = max_speed_percent or self.base_speed_percent
        self._is_moving: bool = True
        self._stop_event.clear()

        self._movement_thread: Thread = Thread(target=self._autonomous_movement, daemon=True)
        self._movement_thread.start()

    def forward_cm_sync(self, distance_cm: float, max_speed_percent: Optional[int] = None) -> None:
        """Движение вперед в текущем потоке (удобно для тестов/отладки)."""
        if self._is_moving or self._is_route_running:
            self.stop()
        self._is_moving = True
        self._stop_event.clear()
        try:
            self._run_forward_segment(
                distance_cm=distance_cm,
                max_speed_percent=max_speed_percent or self.base_speed_percent,
            )
        finally:
            self.motor_controller.stop()
            self._is_moving = False

    def backward_cm_sync(self, distance_cm: float, speed_percent: Optional[int] = None) -> None:
        """Движение назад в текущем потоке (для маршрутов и тестов).

        Препятствия не проверяются (датчик смотрит вперед).
        """
        if self._is_moving or self._is_route_running:
            self.stop()
        self._is_moving = True
        self._stop_event.clear()
        try:
            self._run_backward_segment(
                distance_cm=distance_cm,
                speed_percent=speed_percent or self.base_speed_percent,
            )
        finally:
            self.motor_controller.stop()
            self._is_moving = False

    def execute_route(self, route: Route) -> None:
        """Запуск выполнения маршрута.

        Не блокирует: маршрут выполняется в daemon-потоке. При обнаружении
        препятствия на сегменте «вперед» — плавная остановка. При уже идущем
        движении предыдущее останавливается.

        Args:
            route: Маршрут — последовательность сегментов (forward, turn_left, turn_right).
        """
        if self._is_moving or self._is_route_running:
            self.stop()

        self._is_moving = True
        self._is_route_running = True
        self._stop_event.clear()

        self._movement_thread = Thread(
            target=self._execute_route_loop,
            args=(route.segments,),
            daemon=True,
        )
        self._movement_thread.start()

    def execute_route_sync(self, route: Route) -> None:
        """Выполнение маршрута в текущем потоке (для тестов/отладки)."""
        if self._is_moving or self._is_route_running:
            self.stop()
        self._is_moving = True
        self._is_route_running = True
        self._stop_event.clear()
        try:
            self._execute_route_segments(route.segments)
        finally:
            self.motor_controller.stop()
            self._is_moving = False
            self._is_route_running = False

    def stop(self) -> None:
        """Немедленная остановка движения.

        Останавливает моторы и ждет завершения потока автономного движения (до 1 с).
        """
        self._is_moving = False
        self._is_route_running = False
        self._stop_event.set()
        self.motor_controller.stop()

        if self._movement_thread and self._movement_thread.is_alive():
            self._movement_thread.join(timeout=1.0)

        self._movement_thread: Optional[Thread] = None

    def destroy(self) -> None:
        """Освобождение ресурсов: остановка движения, моторов и датчика."""
        self.stop()
        self.motor_controller.destroy()
        self.ultrasonic_sensor.destroy()
    
    @property
    def is_moving(self) -> bool:
        """Идет ли движение."""
        return self._is_moving or self._is_route_running

    def _calculate_speed(self, obstacle_distance_cm: float, remaining_distance_cm: float) -> float:
        """Скорость с учетом препятствий и оставшейся дистанции.

        По препятствию: 0 при 'distance' <= 'min', линейный рост до 'max' в зоне торможения.
        По цели: дополнительное снижение в зоне торможения (не ниже 10%).

        Returns:
            Скорость в процентах (0–100).
        """
        if obstacle_distance_cm <= self.min_obstacle_distance_cm:
            return 0.0

        speed: float = self._max_speed_percent

        if obstacle_distance_cm <= self.deceleration_distance_cm:
            decel_range: float = self.deceleration_distance_cm - self.min_obstacle_distance_cm
            distance_above_min: float = obstacle_distance_cm - self.min_obstacle_distance_cm
            speed *= max(0.0, min(1.0, distance_above_min / decel_range))

        if remaining_distance_cm and remaining_distance_cm < self.deceleration_distance_cm:
            speed *= max(0.1, min(1.0, remaining_distance_cm / self.deceleration_distance_cm))

        return max(0.0, speed)

    def _estimate_traveled_distance(self, speed_percent: float, time_interval: float) -> float:
        """Оценка пройденного расстояния за интервал.

        Модель: при 100% скорость 'MAX_SPEED_CM_PER_SEC' см/с. Не учитывает
        проскальзывание и уклон.

        Returns:
            Оценочное расстояние в сантиметрах.
        """
        return self.max_speed_cm_per_sec * (speed_percent / 100.0) * time_interval

    def _autonomous_movement(self) -> None:
        """Цикл движения вперед в daemon-потоке."""
        try:
            self._run_forward_segment(
                distance_cm=self._target_distance_cm,
                max_speed_percent=self._max_speed_percent,
            )
        finally:
            self.motor_controller.stop()
            self._is_moving = False

    def _run_backward_segment(self, distance_cm: float, speed_percent: int) -> bool:
        """Сегмент «назад» на заданную дистанцию.

        Returns:
            True — сегмент пройден полностью; False — остановка по stop_event/stop.
        """
        traveled_cm: float = 0.0
        clamped_speed: int = max(0, min(100, speed_percent))
        try:
            while self._is_moving and not self._stop_event.is_set():
                remaining_distance: float = distance_cm - traveled_cm
                if remaining_distance <= 0:
                    return True

                # Плавное снижение к концу (как и вперед), но без учета препятствий
                speed_factor: float = 1.0
                if remaining_distance < self.deceleration_distance_cm:
                    speed_factor = max(0.1, min(1.0, remaining_distance / self.deceleration_distance_cm))
                current_speed: float = clamped_speed * speed_factor

                self.motor_controller.move_backward(speed_percent=int(current_speed))
                traveled_cm += self._estimate_traveled_distance(
                    speed_percent=current_speed,
                    time_interval=self.update_interval_sec,
                )
                time.sleep(self.update_interval_sec)

            return False

        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.exception("Ошибка в сегменте движения назад: %s", exc)
            return False

    def _run_forward_segment(
        self,
        distance_cm: float,
        max_speed_percent: int,
    ) -> bool:
        """Запуск сегмента «вперед» с учетом препятствий и целевой дистанции.

        Блокирует до завершения или остановки по препятствию/stop_event.

        Returns:
            True — сегмент пройден полностью; False — остановка по препятствию или stop.
        """
        traveled_cm: float = 0.0
        self._max_speed_percent = max_speed_percent
        try:
            while self._is_moving and not self._stop_event.is_set():
                obstacle_distance_cm: float = self.ultrasonic_sensor.measure_distance_cm()
                remaining_distance: float = distance_cm - traveled_cm

                if remaining_distance <= 0:
                    return True

                current_speed: float = self._calculate_speed(
                    obstacle_distance_cm=obstacle_distance_cm,
                    remaining_distance_cm=remaining_distance,
                )

                if current_speed <= 0.0:
                    self.motor_controller.stop()
                    time.sleep(self.update_interval_sec)
                    return False
                self.motor_controller.move_forward(speed_percent=int(current_speed))
                traveled_cm += self._estimate_traveled_distance(
                    speed_percent=current_speed,
                    time_interval=self.update_interval_sec,
                )

                time.sleep(self.update_interval_sec)

            return False

        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.exception("Ошибка в сегменте движения вперед: %s", exc)
            return False

    def _execute_route_loop(self, segments: list[RouteSegment]) -> None:
        """Цикл выполнения маршрута в daemon-потоке."""
        try:
            self._execute_route_segments(segments)

        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.exception("Ошибка при выполнении маршрута: %s", exc)

        finally:
            self.motor_controller.stop()
            self._is_moving = False
            self._is_route_running = False

    def _execute_route_segments(self, segments: list[RouteSegment]) -> None:
        """Общая реализация выполнения сегментов маршрута (sync и thread)."""
        for segment in segments:
            if not self._is_moving or self._stop_event.is_set():
                break

            if isinstance(segment, ForwardSegment):
                completed: bool = self._run_forward_segment(
                    distance_cm=segment.distance_cm,
                    max_speed_percent=self.base_speed_percent,
                )
                if not completed:
                    break

            elif isinstance(segment, BackwardSegment):
                completed_back: bool = self._run_backward_segment(
                    distance_cm=segment.distance_cm,
                    speed_percent=self.base_speed_percent,
                )
                if not completed_back:
                    break

            elif isinstance(segment, TurnLeftSegment):
                self._run_turn_segment(
                    turn_left=True,
                    duration_sec=segment.duration_sec,
                )

            elif isinstance(segment, TurnRightSegment):
                self._run_turn_segment(
                    turn_left=False,
                    duration_sec=segment.duration_sec,
                )

    def _run_turn_segment(self, turn_left: bool, duration_sec: float) -> None:
        """Выполнение поворота на месте с проверкой препятствий."""
        elapsed: float = 0.0
        check_interval: float = min(0.1, duration_sec / 10.0)

        while elapsed < duration_sec and self._is_moving and not self._stop_event.is_set():
            obstacle_cm: float = self.ultrasonic_sensor.measure_distance_cm()
            if obstacle_cm <= self.min_obstacle_distance_cm:
                return

            if turn_left:
                self.motor_controller.turn_left(speed_percent=self.turn_speed_percent)
            else:
                self.motor_controller.turn_right(speed_percent=self.turn_speed_percent)

            time.sleep(check_interval)
            elapsed += check_interval

        self.motor_controller.stop()
