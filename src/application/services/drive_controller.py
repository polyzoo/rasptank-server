from __future__ import annotations

import logging
import time
from threading import Event, Thread
from typing import final

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
    GyroscopeProtocol,
    MotorControllerProtocol,
    UltrasonicSensorProtocol,
)

logger: logging.Logger = logging.getLogger(__name__)


@final
class DriveController(DriveControllerProtocol):
    """Контроллер движения с учетом препятствий и целевой дистанции."""

    # Ограничения и коэффициенты для пересчёта скорости
    SPEED_PERCENT_MIN: int = 0
    SPEED_PERCENT_MAX: int = 100
    SPEED_FACTOR_MIN: float = 0.1
    SPEED_FACTOR_MAX: float = 1.0

    # Ограничения для расчёта скорости по препятствию
    OBSTACLE_SPEED_FACTOR_MIN: float = 0.0
    OBSTACLE_SPEED_FACTOR_MAX: float = 1.0

    # Ограничения для навигации
    ESTIMATE_SPEED_DIVISOR: float = 100.0
    TURN_CHECK_INTERVAL_SEC: float = 0.05
    TURN_TIMEOUT_PER_DEG: float = 0.05
    TURN_TIMEOUT_MIN: float = 1.0

    # Ожидание завершения потока
    STOP_JOIN_TIMEOUT_SEC: float = 1.0

    def __init__(
        self,
        motor_controller: MotorControllerProtocol,
        ultrasonic_sensor: UltrasonicSensorProtocol,
        gyroscope: GyroscopeProtocol,
        min_obstacle_distance_cm: float = 20.0,
        deceleration_distance_cm: float = 50.0,
        base_speed_percent: int = 60,
        turn_speed_percent: int = 50,
        max_speed_cm_per_sec: float = 30.0,
        update_interval_sec: float = 0.1,
    ) -> None:
        """Инициализация контроллера движения."""
        self.motor_controller: MotorControllerProtocol = motor_controller
        self.ultrasonic_sensor: UltrasonicSensorProtocol = ultrasonic_sensor
        self.gyroscope: GyroscopeProtocol = gyroscope
        
        self.min_obstacle_distance_cm: float = min_obstacle_distance_cm
        self.deceleration_distance_cm: float = deceleration_distance_cm
        self.base_speed_percent: int = base_speed_percent
        self.turn_speed_percent: int = turn_speed_percent
        self.max_speed_cm_per_sec: float = max_speed_cm_per_sec
        self.update_interval_sec: float = update_interval_sec

        self._is_moving: bool = False
        self._is_route_running: bool = False
        self._movement_thread: Thread | None = None
        self._stop_event: Event = Event()

    def forward_cm_sync(self, distance_cm: float, max_speed_percent: int | None = None) -> None:
        """Запуск движения вперед на заданное расстояние в текущем потоке."""
        if self._is_moving:
            self.stop()

        self._is_moving: bool = True
        self._stop_event.clear()

        try:
            self._run_forward_segment(distance_cm, max_speed_percent or self.base_speed_percent)
        finally:
            self.motor_controller.stop()
            self._is_moving: bool = False

    def execute_route(self, route: Route) -> None:
        """Запуск выполнения маршрута в фоновом потоке."""
        if self._is_moving or self._is_route_running:
            self.stop()

        self._is_moving: bool = True
        self._is_route_running: bool = True
        self._stop_event.clear()

        self._movement_thread: Thread = Thread(
            target=self._execute_route_loop,
            args=(route.segments,),
            daemon=True,
        )
        self._movement_thread.start()

    def execute_route_sync(self, route: Route) -> None:
        """Запуск выполнения маршрута в текущем потоке."""
        if self._is_moving or self._is_route_running:
            self.stop()

        self._is_moving: bool = True
        self._is_route_running: bool = True
        self._stop_event.clear()

        try:
            self._execute_route_segments(route.segments)
        finally:
            self.motor_controller.stop()
            self._is_moving: bool = False
            self._is_route_running: bool = False

    def stop(self) -> None:
        """Немедленная остановка движения."""
        self._is_moving: bool = False
        self._is_route_running: bool = False
        self._stop_event.set()
        self.motor_controller.stop()

        if self._movement_thread and self._movement_thread.is_alive():
            self._movement_thread.join(timeout=self.STOP_JOIN_TIMEOUT_SEC)

        self._movement_thread: Thread | None = None

    def destroy(self) -> None:
        """Освобождение ресурсов."""
        self.stop()
        self.motor_controller.destroy()
        self.ultrasonic_sensor.destroy()
        self.gyroscope.destroy()

    def _execute_route_loop(self, segments: list[RouteSegment]) -> None:
        """Цикл выполнения маршрута в daemon-потоке."""
        try:
            self._execute_route_segments(segments)
        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.exception("Ошибка при выполнении маршрута: %s", exc)
        finally:
            self.motor_controller.stop()
            self._is_moving: bool = False
            self._is_route_running: bool = False

    def _execute_route_segments(self, segments: list[RouteSegment]) -> None:
        """Последовательное выполнение сегментов маршрута."""
        self.gyroscope.start(calibrate=True)

        for segment in segments:
            if not self._is_moving or self._stop_event.is_set():
                break

            if isinstance(segment, ForwardSegment):
                if not self._run_forward_segment(segment.distance_cm, self.base_speed_percent):
                    break

            elif isinstance(segment, BackwardSegment):
                if not self._run_backward_segment(segment.distance_cm, self.base_speed_percent):
                    break

            elif isinstance(segment, TurnLeftSegment):
                self._run_turn_segment(
                    turn_left=True,
                    target_angle=segment.angle_deg,
                    timeout_sec=max(
                        self.TURN_TIMEOUT_MIN,
                        segment.angle_deg * self.TURN_TIMEOUT_PER_DEG,
                    ),
                )

            elif isinstance(segment, TurnRightSegment):
                self._run_turn_segment(
                    turn_left=False,
                    target_angle=segment.angle_deg,
                    timeout_sec=max(
                        self.TURN_TIMEOUT_MIN,
                        segment.angle_deg * self.TURN_TIMEOUT_PER_DEG,
                    ),
                )

    def _run_forward_segment(self, distance_cm: float, speed_percent: int) -> bool:
        """Запуск сегмента «вперед» с учетом препятствий."""
        traveled_cm: float = 0.0
        current_speed: float = 0.0
        last_time: float = time.monotonic()

        clamped_speed: int = max(self.SPEED_PERCENT_MIN, min(self.SPEED_PERCENT_MAX, speed_percent))
        try:
            while self._is_moving and not self._stop_event.is_set():
                obstacle_cm: float = self.ultrasonic_sensor.measure_distance_cm()

                now: float = time.monotonic()
                dt: float = now - last_time
                last_time: float = now

                traveled_cm += self._estimate_traveled_distance(current_speed, dt)

                remaining_dist: float = distance_cm - traveled_cm
                if remaining_dist <= 0:
                    self.motor_controller.stop()
                    return True

                current_speed: float = self._calculate_speed(
                    obstacle_cm=obstacle_cm,
                    remaining_cm=remaining_dist,
                    max_speed=clamped_speed,
                )

                if current_speed <= 0.0:
                    self.motor_controller.stop()
                    time.sleep(self.update_interval_sec)
                    return False

                self.motor_controller.move_forward(speed_percent=int(current_speed))
                time.sleep(self.update_interval_sec)

            return False

        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.exception("Ошибка в сегменте движения вперед: %s", exc)
            return False

    def _run_backward_segment(self, distance_cm: float, speed_percent: int) -> bool:
        """Запуск сегмента «назад»."""
        traveled_cm: float = 0.0
        current_speed: float = 0.0
        last_time: float = time.monotonic()

        clamped_speed: int = max(self.SPEED_PERCENT_MIN, min(self.SPEED_PERCENT_MAX, speed_percent))
        try:
            while self._is_moving and not self._stop_event.is_set():
                now: float = time.monotonic()
                dt: float = now - last_time
                last_time: float = now

                traveled_cm += self._estimate_traveled_distance(current_speed, dt)

                remaining_dist: float = distance_cm - traveled_cm
                if remaining_dist <= 0:
                    self.motor_controller.stop()
                    return True

                speed_factor: float = (
                    self.SPEED_FACTOR_MAX
                    if remaining_dist >= self.deceleration_distance_cm
                    else max(self.SPEED_FACTOR_MIN, remaining_dist / self.deceleration_distance_cm)
                )

                current_speed: float = clamped_speed * speed_factor
                self.motor_controller.move_backward(speed_percent=int(current_speed))
                time.sleep(self.update_interval_sec)

            return False

        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.exception("Ошибка в сегменте движения назад: %s", exc)
            return False

    def _run_turn_segment(self, turn_left: bool, target_angle: float, timeout_sec: float) -> None:
        """Запуск сегмента поворота на месте с проверкой препятствий."""
        self.gyroscope.reset_yaw()
        start_time: float = time.monotonic()

        while self._is_moving and not self._stop_event.is_set():
            current_yaw: float = self.gyroscope.get_yaw()
            
            if abs(current_yaw) >= target_angle:
                break

            if (time.monotonic() - start_time) >= timeout_sec:
                break

            obstacle_cm: float = self.ultrasonic_sensor.measure_distance_cm()
            if obstacle_cm <= self.min_obstacle_distance_cm:
                break

            if turn_left:
                self.motor_controller.turn_left(speed_percent=self.turn_speed_percent)
            else:
                self.motor_controller.turn_right(speed_percent=self.turn_speed_percent)

            time.sleep(self.TURN_CHECK_INTERVAL_SEC)

        self.motor_controller.stop()

    def _calculate_speed(self, obstacle_cm: float, remaining_cm: float, max_speed: int) -> float:
        """Расчет скорости с учетом препятствий и цели."""
        if obstacle_cm <= self.min_obstacle_distance_cm:
            return 0.0

        speed: float = float(max_speed)

        if obstacle_cm <= self.deceleration_distance_cm:
            decel_range: float = self.deceleration_distance_cm - self.min_obstacle_distance_cm
            dist_above_min: float = obstacle_cm - self.min_obstacle_distance_cm
            speed *= max(self.OBSTACLE_SPEED_FACTOR_MIN, dist_above_min / decel_range)

        if remaining_cm < self.deceleration_distance_cm:
            speed *= max(self.SPEED_FACTOR_MIN, remaining_cm / self.deceleration_distance_cm)

        return max(self.OBSTACLE_SPEED_FACTOR_MIN, speed)

    def _estimate_traveled_distance(self, speed_percent: float, time_interval: float) -> float:
        """Оценка пройденного пути."""
        speed_ratio: float = speed_percent / self.ESTIMATE_SPEED_DIVISOR
        return self.max_speed_cm_per_sec * speed_ratio * time_interval
