from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum
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


class AvoidanceState(Enum):
    """Состояния пошагового обхода препятствия."""

    SELECT_SIDE = "select_side"
    SIDE_STEP = "side_step"
    CHECK_FRONT = "check_front"
    ADVANCE_ALONG_OBSTACLE = "advance_along_obstacle"
    TRY_REJOIN = "try_rejoin"


class AvoidanceSide(Enum):
    """Сторона обхода препятствия."""

    LEFT = "left"
    RIGHT = "right"

    @property
    def turn_left(self) -> bool:
        """Направление поворота для данной стороны."""
        return self is AvoidanceSide.LEFT

    def opposite(self) -> AvoidanceSide:
        """Противоположная сторона."""
        if self is AvoidanceSide.LEFT:
            return AvoidanceSide.RIGHT
        return AvoidanceSide.LEFT


@dataclass(slots=True)
class LinearMoveResult:
    """Результат линейного движения."""

    completed: bool
    traveled_cm: float
    blocked: bool = False


@dataclass(slots=True)
class TurnResult:
    """Результат относительного поворота."""

    completed: bool
    angle_deg: float


@dataclass(slots=True)
class AvoidanceResult:
    """Результат обхода препятствия."""

    completed: bool
    forward_progress_cm: float = 0.0


@dataclass(slots=True)
class AvoidanceContext:
    """Текущее состояние автомата обхода препятствия."""

    state: AvoidanceState = AvoidanceState.SELECT_SIDE
    side: AvoidanceSide | None = None
    forward_progress_cm: float = 0.0
    lateral_offset_cm: float = 0.0
    bypass_distance_cm: float = 0.0
    attempts: int = 0


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

    # Технические допуски
    DISTANCE_TOLERANCE_CM: float = 0.5
    BLOCK_CONFIRMATION_SAMPLES_MIN: int = 3

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
        avoidance_scan_angle_deg: float = 45.0,
        avoidance_side_step_cm: float = 12.0,
        avoidance_forward_step_cm: float = 15.0,
        avoidance_rejoin_step_cm: float = 12.0,
        avoidance_max_attempts: int = 24,
        avoidance_confirm_readings: int = 3,
        avoidance_min_side_clearance_cm: float = 25.0,
        avoidance_max_lateral_offset_cm: float = 60.0,
        avoidance_max_bypass_distance_cm: float = 200.0,
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

        self.avoidance_scan_angle_deg: float = avoidance_scan_angle_deg
        self.avoidance_side_step_cm: float = avoidance_side_step_cm
        self.avoidance_forward_step_cm: float = avoidance_forward_step_cm
        self.avoidance_rejoin_step_cm: float = avoidance_rejoin_step_cm
        self.avoidance_max_attempts: int = avoidance_max_attempts
        self.avoidance_confirm_readings: int = avoidance_confirm_readings
        self.avoidance_min_side_clearance_cm: float = avoidance_min_side_clearance_cm
        self.avoidance_max_lateral_offset_cm: float = avoidance_max_lateral_offset_cm
        self.avoidance_max_bypass_distance_cm: float = avoidance_max_bypass_distance_cm

        self._is_moving: bool = False
        self._is_route_running: bool = False
        self._movement_thread: Thread | None = None
        self._stop_event: Event = Event()

    def forward_cm_sync(self, distance_cm: float, max_speed_percent: int | None = None) -> None:
        """Запуск движения вперед на заданное расстояние в текущем потоке."""
        if self._is_moving:
            self.stop()

        self._is_moving = True
        self._stop_event.clear()
        self._start_navigation_tracking()

        try:
            self._run_forward_segment(distance_cm, max_speed_percent or self.base_speed_percent)
        finally:
            self.motor_controller.stop()
            self._is_moving = False

    def execute_route(self, route: Route) -> None:
        """Запуск выполнения маршрута в фоновом потоке."""
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
        """Запуск выполнения маршрута в текущем потоке."""
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
        """Немедленная остановка движения."""
        self._is_moving = False
        self._is_route_running = False
        self._stop_event.set()
        self.motor_controller.stop()

        if self._movement_thread and self._movement_thread.is_alive():
            self._movement_thread.join(timeout=self.STOP_JOIN_TIMEOUT_SEC)

        self._movement_thread = None

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
            self._is_moving = False
            self._is_route_running = False

    def _execute_route_segments(self, segments: list[RouteSegment]) -> None:
        """Последовательное выполнение сегментов маршрута."""
        self._start_navigation_tracking()

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
                    timeout_sec=self._calculate_turn_timeout(segment.angle_deg),
                )

            elif isinstance(segment, TurnRightSegment):
                self._run_turn_segment(
                    turn_left=False,
                    target_angle=segment.angle_deg,
                    timeout_sec=self._calculate_turn_timeout(segment.angle_deg),
                )

    def _run_forward_segment(self, distance_cm: float, speed_percent: int) -> bool:
        """Запуск сегмента «вперед» с учетом препятствий и обхода."""
        remaining_cm: float = max(0.0, distance_cm)
        clamped_speed: int = max(self.SPEED_PERCENT_MIN, min(self.SPEED_PERCENT_MAX, speed_percent))

        try:
            while self._is_moving and not self._stop_event.is_set():
                if self._has_reached_target_distance(remaining_cm):
                    self.motor_controller.stop()
                    return True

                move_result: LinearMoveResult = self._run_linear_motion(
                    distance_cm=remaining_cm,
                    speed_percent=clamped_speed,
                    move_forward=True,
                    obstacle_aware=True,
                )
                remaining_cm = max(0.0, remaining_cm - move_result.traveled_cm)

                if move_result.completed or self._has_reached_target_distance(remaining_cm):
                    self.motor_controller.stop()
                    return True

                if not move_result.blocked:
                    return False

                avoidance_result: AvoidanceResult = self._run_obstacle_avoidance(
                    remaining_cm=remaining_cm,
                    speed_percent=clamped_speed,
                )
                remaining_cm = max(0.0, remaining_cm - avoidance_result.forward_progress_cm)

                if self._has_reached_target_distance(remaining_cm):
                    self.motor_controller.stop()
                    return True

                if not avoidance_result.completed:
                    return False

            return False

        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.exception("Ошибка в сегменте движения вперед: %s", exc)
            return False

    def _run_backward_segment(self, distance_cm: float, speed_percent: int) -> bool:
        """Запуск сегмента «назад»."""
        move_result: LinearMoveResult = self._run_linear_motion(
            distance_cm=distance_cm,
            speed_percent=speed_percent,
            move_forward=False,
            obstacle_aware=False,
        )
        return move_result.completed

    def _run_turn_segment(self, turn_left: bool, target_angle: float, timeout_sec: float) -> None:
        """Запуск сегмента поворота на месте с проверкой препятствий."""
        self._turn_relative(
            turn_left=turn_left,
            target_angle=target_angle,
            timeout_sec=timeout_sec,
            stop_on_front_obstacle=True,
        )

    def _run_linear_motion(
        self,
        distance_cm: float,
        speed_percent: int,
        move_forward: bool,
        obstacle_aware: bool,
    ) -> LinearMoveResult:
        """Движение по прямой на ограниченную дистанцию."""
        if self._has_reached_target_distance(distance_cm):
            self.motor_controller.stop()
            return LinearMoveResult(completed=True, traveled_cm=0.0)

        traveled_cm: float = 0.0
        current_speed: float = 0.0
        last_time: float = time.monotonic()
        clamped_speed: int = max(self.SPEED_PERCENT_MIN, min(self.SPEED_PERCENT_MAX, speed_percent))

        try:
            while self._is_moving and not self._stop_event.is_set():
                obstacle_cm: float | None = None
                if obstacle_aware:
                    obstacle_cm = self._measure_motion_obstacle_distance()

                now: float = time.monotonic()
                dt: float = now - last_time
                last_time = now

                traveled_cm += self._estimate_traveled_distance(current_speed, dt)
                remaining_dist: float = max(0.0, distance_cm - traveled_cm)

                if self._has_reached_target_distance(remaining_dist):
                    self.motor_controller.stop()
                    return LinearMoveResult(completed=True, traveled_cm=distance_cm)

                if obstacle_aware and obstacle_cm is not None:
                    current_speed = self._calculate_speed(
                        obstacle_cm=obstacle_cm,
                        remaining_cm=remaining_dist,
                        max_speed=clamped_speed,
                    )
                    if current_speed <= 0.0:
                        self.motor_controller.stop()
                        time.sleep(self.update_interval_sec)
                        return LinearMoveResult(
                            completed=False,
                            traveled_cm=min(traveled_cm, distance_cm),
                            blocked=True,
                        )
                else:
                    current_speed = clamped_speed * self._calculate_distance_speed_factor(remaining_dist)

                if move_forward:
                    self.motor_controller.move_forward(speed_percent=int(current_speed))
                else:
                    self.motor_controller.move_backward(speed_percent=int(current_speed))

                time.sleep(self.update_interval_sec)

            return LinearMoveResult(
                completed=False,
                traveled_cm=min(traveled_cm, distance_cm),
                blocked=False,
            )

        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            direction: str = "вперед" if move_forward else "назад"
            logger.exception("Ошибка в линейном движении %s: %s", direction, exc)
            return LinearMoveResult(
                completed=False,
                traveled_cm=min(traveled_cm, distance_cm),
                blocked=False,
            )

    def _run_obstacle_avoidance(self, remaining_cm: float, speed_percent: int) -> AvoidanceResult:
        """Пошаговый автомат обхода препятствия."""
        context: AvoidanceContext = AvoidanceContext()

        while self._is_moving and not self._stop_event.is_set():
            if self._has_reached_target_distance(remaining_cm - context.forward_progress_cm):
                return AvoidanceResult(completed=True, forward_progress_cm=context.forward_progress_cm)

            if self._has_exceeded_avoidance_limits(context):
                break

            if context.state is AvoidanceState.SELECT_SIDE:
                context.side = self._select_avoidance_side()
                if context.side is None:
                    logger.warning("Обход препятствия невозможен: свободная сторона не найдена.")
                    break
                context.state = AvoidanceState.SIDE_STEP
                continue

            if context.state is AvoidanceState.SIDE_STEP:
                side_step: LinearMoveResult = self._perform_side_step(context.side, speed_percent)
                self._update_avoidance_progress(
                    context=context,
                    traveled_cm=side_step.traveled_cm,
                    attempts_delta=1,
                )
                context.lateral_offset_cm += side_step.traveled_cm

                if not side_step.completed and side_step.traveled_cm <= self.DISTANCE_TOLERANCE_CM:
                    logger.warning("Обход остановлен: боковой шаг не удался.")
                    break

                context.state = AvoidanceState.CHECK_FRONT
                continue

            if context.state is AvoidanceState.CHECK_FRONT:
                obstacle_cm: float = self._measure_front_distance()
                if self._is_front_blocked(obstacle_cm):
                    context.state = AvoidanceState.SIDE_STEP
                    continue

                if self._has_reached_target_distance(context.lateral_offset_cm):
                    return AvoidanceResult(completed=True, forward_progress_cm=context.forward_progress_cm)

                context.state = AvoidanceState.ADVANCE_ALONG_OBSTACLE
                continue

            if context.state is AvoidanceState.ADVANCE_ALONG_OBSTACLE:
                forward_step: LinearMoveResult = self._perform_forward_step(
                    remaining_cm=remaining_cm - context.forward_progress_cm,
                    speed_percent=speed_percent,
                )
                self._update_avoidance_progress(
                    context=context,
                    traveled_cm=forward_step.traveled_cm,
                    attempts_delta=1,
                )
                context.forward_progress_cm += forward_step.traveled_cm

                if self._has_reached_target_distance(remaining_cm - context.forward_progress_cm):
                    return AvoidanceResult(completed=True, forward_progress_cm=context.forward_progress_cm)

                if not forward_step.completed:
                    if forward_step.blocked:
                        context.state = AvoidanceState.SIDE_STEP
                        continue
                    logger.warning("Обход остановлен: шаг вдоль препятствия не завершён.")
                    break

                if self._has_reached_target_distance(context.lateral_offset_cm):
                    return AvoidanceResult(completed=True, forward_progress_cm=context.forward_progress_cm)

                context.state = AvoidanceState.TRY_REJOIN
                continue

            if context.state is AvoidanceState.TRY_REJOIN:
                rejoin_step: LinearMoveResult = self._attempt_rejoin_step(
                    side=context.side,
                    lateral_offset_cm=context.lateral_offset_cm,
                    speed_percent=speed_percent,
                )
                self._update_avoidance_progress(
                    context=context,
                    traveled_cm=rejoin_step.traveled_cm,
                    attempts_delta=1,
                )
                context.lateral_offset_cm = max(0.0, context.lateral_offset_cm - rejoin_step.traveled_cm)

                if not rejoin_step.completed and rejoin_step.traveled_cm <= self.DISTANCE_TOLERANCE_CM:
                    context.state = AvoidanceState.CHECK_FRONT
                    continue

                if self._has_reached_target_distance(context.lateral_offset_cm):
                    obstacle_cm = self._measure_front_distance()
                    if self._is_front_blocked(obstacle_cm):
                        context.state = AvoidanceState.SIDE_STEP
                        continue
                    return AvoidanceResult(completed=True, forward_progress_cm=context.forward_progress_cm)

                context.state = AvoidanceState.CHECK_FRONT
                continue

        self.motor_controller.stop()
        return AvoidanceResult(completed=False, forward_progress_cm=context.forward_progress_cm)

    def _select_avoidance_side(self) -> AvoidanceSide | None:
        """Выбор стороны обхода по данным короткого сканирования."""
        left_clearance: float | None = self._scan_side_clearance(AvoidanceSide.LEFT)
        if left_clearance is None:
            logger.warning("Выбор стороны отменён: после сканирования влево курс не восстановлен.")
            return None

        right_clearance: float | None = self._scan_side_clearance(AvoidanceSide.RIGHT)
        if right_clearance is None:
            logger.warning("Выбор стороны отменён: после сканирования вправо курс не восстановлен.")
            return None

        best_clearance: float = max(left_clearance, right_clearance)
        if best_clearance < self.avoidance_min_side_clearance_cm:
            return None

        if left_clearance >= right_clearance:
            return AvoidanceSide.LEFT
        return AvoidanceSide.RIGHT

    def _scan_side_clearance(self, side: AvoidanceSide) -> float | None:
        """Короткий поворот в сторону для оценки свободного пространства."""
        target_angle: float = max(0.0, min(90.0, self.avoidance_scan_angle_deg))
        if self._has_reached_target_distance(target_angle):
            return self._measure_front_distance()

        turn_result: TurnResult = self._turn_relative(
            turn_left=side.turn_left,
            target_angle=target_angle,
            timeout_sec=self._calculate_turn_timeout(target_angle),
            stop_on_front_obstacle=False,
        )
        clearance: float = 0.0
        heading_restored: bool = True

        try:
            if turn_result.completed:
                clearance = self._measure_front_distance()
        finally:
            if turn_result.angle_deg > self.DISTANCE_TOLERANCE_CM:
                restore_result: TurnResult = self._turn_relative(
                    turn_left=not side.turn_left,
                    target_angle=turn_result.angle_deg,
                    timeout_sec=self._calculate_turn_timeout(turn_result.angle_deg),
                    stop_on_front_obstacle=False,
                )
                if not restore_result.completed:
                    logger.warning("Сканирование стороны прервано: исходный курс не восстановлен.")
                    heading_restored = False

        if not heading_restored:
            return None
        return clearance

    def _perform_side_step(self, side: AvoidanceSide, speed_percent: int) -> LinearMoveResult:
        """Один осторожный шаг в выбранную сторону."""
        return self._perform_lateral_step(
            side=side,
            distance_cm=self.avoidance_side_step_cm,
            speed_percent=speed_percent,
        )

    def _perform_forward_step(self, remaining_cm: float, speed_percent: int) -> LinearMoveResult:
        """Один короткий шаг вдоль исходного курса."""
        step_cm: float = min(self.avoidance_forward_step_cm, max(0.0, remaining_cm))
        return self._run_linear_motion(
            distance_cm=step_cm,
            speed_percent=speed_percent,
            move_forward=True,
            obstacle_aware=True,
        )

    def _attempt_rejoin_step(
        self,
        side: AvoidanceSide,
        lateral_offset_cm: float,
        speed_percent: int,
    ) -> LinearMoveResult:
        """Осторожная попытка вернуться к исходной траектории."""
        step_cm: float = min(self.avoidance_rejoin_step_cm, max(0.0, lateral_offset_cm))
        return self._perform_lateral_step(
            side=side.opposite(),
            distance_cm=step_cm,
            speed_percent=speed_percent,
        )

    def _perform_lateral_step(
        self,
        side: AvoidanceSide,
        distance_cm: float,
        speed_percent: int,
    ) -> LinearMoveResult:
        """Шаг в сторону с обязательным восстановлением исходного курса."""
        if self._has_reached_target_distance(distance_cm):
            return LinearMoveResult(completed=True, traveled_cm=0.0)

        turn_result: TurnResult = self._turn_relative(
            turn_left=side.turn_left,
            target_angle=90.0,
            timeout_sec=self._calculate_turn_timeout(90.0),
            stop_on_front_obstacle=False,
        )

        try:
            if not turn_result.completed:
                return LinearMoveResult(completed=False, traveled_cm=0.0)

            return self._run_linear_motion(
                distance_cm=distance_cm,
                speed_percent=speed_percent,
                move_forward=True,
                obstacle_aware=True,
            )

        finally:
            if turn_result.angle_deg > self.DISTANCE_TOLERANCE_CM:
                restore_result: TurnResult = self._turn_relative(
                    turn_left=not side.turn_left,
                    target_angle=turn_result.angle_deg,
                    timeout_sec=self._calculate_turn_timeout(turn_result.angle_deg),
                    stop_on_front_obstacle=False,
                )
                if not restore_result.completed:
                    logger.warning("Не удалось полностью восстановить исходный курс.")

    def _turn_relative(
        self,
        turn_left: bool,
        target_angle: float,
        timeout_sec: float,
        stop_on_front_obstacle: bool,
    ) -> TurnResult:
        """Поворот на относительный угол по данным гироскопа."""
        if target_angle <= 0.0:
            return TurnResult(completed=True, angle_deg=0.0)

        self.gyroscope.reset_yaw()
        start_time: float = time.monotonic()

        while self._is_moving and not self._stop_event.is_set():
            current_yaw: float = abs(self.gyroscope.get_yaw())
            if current_yaw >= target_angle:
                break

            if (time.monotonic() - start_time) >= timeout_sec:
                break

            if stop_on_front_obstacle:
                obstacle_cm: float = self.ultrasonic_sensor.measure_distance_cm()
                if self._is_front_blocked(obstacle_cm):
                    break

            if turn_left:
                self.motor_controller.turn_left(speed_percent=self.turn_speed_percent)
            else:
                self.motor_controller.turn_right(speed_percent=self.turn_speed_percent)

            time.sleep(self.TURN_CHECK_INTERVAL_SEC)

        self.motor_controller.stop()
        turned_angle: float = abs(self.gyroscope.get_yaw())
        return TurnResult(completed=turned_angle >= target_angle, angle_deg=turned_angle)

    def _measure_front_distance(self, samples: int | None = None) -> float:
        """Измерение фронтальной дистанции с подавлением одиночных шумовых чтений."""
        sample_count: int = max(1, samples or self.avoidance_confirm_readings)
        measurements: list[float] = [
            self.ultrasonic_sensor.measure_distance_cm()
            for _ in range(sample_count)
        ]
        measurements.sort()
        return measurements[len(measurements) // 2]

    def _measure_motion_obstacle_distance(self) -> float:
        """Чтение дистанции для движения с подтверждением критически малого расстояния."""
        obstacle_cm: float = self.ultrasonic_sensor.measure_distance_cm()
        if not self._is_front_blocked(obstacle_cm):
            return obstacle_cm

        return self._measure_front_distance(
            samples=max(
                self.BLOCK_CONFIRMATION_SAMPLES_MIN,
                self.avoidance_confirm_readings,
            ),
        )

    def _has_exceeded_avoidance_limits(self, context: AvoidanceContext) -> bool:
        """Проверка жёстких лимитов на безопасный обход препятствия."""
        if context.attempts >= self.avoidance_max_attempts:
            logger.warning("Обход остановлен: превышено число попыток (%d).", context.attempts)
            return True

        if context.lateral_offset_cm > self.avoidance_max_lateral_offset_cm:
            logger.warning(
                "Обход остановлен: превышено боковое смещение (%.1f см).",
                context.lateral_offset_cm,
            )
            return True

        if context.bypass_distance_cm > self.avoidance_max_bypass_distance_cm:
            logger.warning(
                "Обход остановлен: превышена суммарная длина обхода (%.1f см).",
                context.bypass_distance_cm,
            )
            return True

        return False

    def _update_avoidance_progress(
        self,
        context: AvoidanceContext,
        traveled_cm: float,
        attempts_delta: int = 0,
    ) -> None:
        """Обновление накопленных метрик обхода."""
        context.attempts += attempts_delta
        context.bypass_distance_cm += traveled_cm

    def _start_navigation_tracking(self) -> None:
        """Подготовка гироскопа к поворотам и обходу препятствий."""
        self.gyroscope.start(calibrate=True)

    def _is_front_blocked(self, obstacle_cm: float) -> bool:
        """Проверка, что впереди недостаточно места для безопасного движения."""
        return obstacle_cm <= self.min_obstacle_distance_cm

    def _has_reached_target_distance(self, distance_cm: float) -> bool:
        """Проверка достижения цели с учётом допуска на оценку пути."""
        return distance_cm <= self.DISTANCE_TOLERANCE_CM

    def _calculate_turn_timeout(self, target_angle: float) -> float:
        """Расчёт безопасного таймаута для поворота."""
        return max(self.TURN_TIMEOUT_MIN, target_angle * self.TURN_TIMEOUT_PER_DEG)

    def _calculate_distance_speed_factor(self, remaining_cm: float) -> float:
        """Коэффициент замедления при приближении к целевой точке."""
        if self.deceleration_distance_cm <= 0:
            return self.SPEED_FACTOR_MAX

        if remaining_cm >= self.deceleration_distance_cm:
            return self.SPEED_FACTOR_MAX

        return max(self.SPEED_FACTOR_MIN, remaining_cm / self.deceleration_distance_cm)

    def _calculate_speed(self, obstacle_cm: float, remaining_cm: float, max_speed: int) -> float:
        """Расчет скорости с учетом препятствий и цели."""
        if self._is_front_blocked(obstacle_cm):
            return 0.0

        speed: float = float(max_speed)

        if (
            obstacle_cm <= self.deceleration_distance_cm
            and self.deceleration_distance_cm > self.min_obstacle_distance_cm
        ):
            decel_range: float = self.deceleration_distance_cm - self.min_obstacle_distance_cm
            dist_above_min: float = obstacle_cm - self.min_obstacle_distance_cm
            speed *= max(self.OBSTACLE_SPEED_FACTOR_MIN, dist_above_min / decel_range)

        speed *= self._calculate_distance_speed_factor(remaining_cm)
        return max(self.OBSTACLE_SPEED_FACTOR_MIN, speed)

    def _estimate_traveled_distance(self, speed_percent: float, time_interval: float) -> float:
        """Оценка пройденного пути."""
        speed_ratio: float = speed_percent / self.ESTIMATE_SPEED_DIVISOR
        return self.max_speed_cm_per_sec * speed_ratio * time_interval
