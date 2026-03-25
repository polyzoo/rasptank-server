from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum
from threading import Event
from typing import final

from src.application.models.route import Route
from src.application.protocols import (
    DriveControllerProtocol,
    GyroscopeProtocol,
    MotorControllerProtocol,
    UltrasonicSensorProtocol,
)
from src.application.services.linear_motion_executor import (
    LinearMotionExecutionResult,
    LinearMotionExecutor,
)
from src.application.services.motion_config import MotionConfig
from src.application.services.motion_lifecycle import MotionLifecycle
from src.application.services.route_executor import RouteExecutor
from src.application.services.route_runner import RouteRunner
from src.application.services.turn_executor import TurnExecutionResult, TurnExecutor

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
    heading_restored: bool = True


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
    """Контроллер движения поверх модульных executors с obstacle avoidance."""

    DISTANCE_TOLERANCE_CM: float = 0.5
    BLOCK_CONFIRMATION_SAMPLES_MIN: int = 3
    CLEAR_CONFIRMATION_CHECKS: int = 2
    CLEARANCE_HYSTERESIS_CM: float = 3.0
    STOP_JOIN_TIMEOUT_SEC: float = 1.0

    def __init__(
        self,
        motor_controller: MotorControllerProtocol,
        ultrasonic_sensor: UltrasonicSensorProtocol,
        gyroscope: GyroscopeProtocol,
        config: MotionConfig | None = None,
        *,
        min_obstacle_distance_cm: float = 20.0,
        deceleration_distance_cm: float = 10.0,
        base_speed_percent: int = 55,
        turn_speed_percent: int = 72,
        max_speed_cm_per_sec: float = 28.0,
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
        turn_slowdown_remaining_deg: float = 8.0,
        turn_creep_speed_percent: int = 42,
        turn_angle_trim_deg: float = -2.0,
        last_turn_angle_trim_deg: float = 2.0,
        heading_hold_enabled: bool = True,
        heading_hold_kp: float = 6.0,
        heading_hold_steer_max: int = 85,
        heading_hold_deadband_deg: float = 0.15,
        heading_hold_steer_speed_ratio: float = 0.58,
        heading_hold_min_speed_percent: float = 0.0,
        heading_hold_steer_cap_min_speed_percent: float = 45.0,
        heading_hold_steer_trim: int = 0,
        heading_hold_invert_steer: bool = True,
        forward_soft_start_sec: float = 0.35,
        turn_check_interval_sec: float = 0.01,
        turn_obstacle_check_interval_sec: float = 0.20,
        turn_timeout_per_deg: float = 0.05,
        turn_timeout_min: float = 1.0,
    ) -> None:
        """Инициализация контроллера движения."""
        self.motor_controller: MotorControllerProtocol = motor_controller
        self.ultrasonic_sensor: UltrasonicSensorProtocol = ultrasonic_sensor
        self.gyroscope: GyroscopeProtocol = gyroscope
        self.config: MotionConfig = config or MotionConfig(
            min_obstacle_distance_cm=min_obstacle_distance_cm,
            deceleration_distance_cm=deceleration_distance_cm,
            base_speed_percent=base_speed_percent,
            turn_speed_percent=turn_speed_percent,
            turn_slowdown_remaining_deg=turn_slowdown_remaining_deg,
            turn_creep_speed_percent=turn_creep_speed_percent,
            turn_angle_trim_deg=turn_angle_trim_deg,
            last_turn_angle_trim_deg=last_turn_angle_trim_deg,
            max_speed_cm_per_sec=max_speed_cm_per_sec,
            update_interval_sec=update_interval_sec,
            avoidance_scan_angle_deg=avoidance_scan_angle_deg,
            avoidance_side_step_cm=avoidance_side_step_cm,
            avoidance_forward_step_cm=avoidance_forward_step_cm,
            avoidance_rejoin_step_cm=avoidance_rejoin_step_cm,
            avoidance_max_attempts=avoidance_max_attempts,
            avoidance_confirm_readings=avoidance_confirm_readings,
            avoidance_min_side_clearance_cm=avoidance_min_side_clearance_cm,
            avoidance_max_lateral_offset_cm=avoidance_max_lateral_offset_cm,
            avoidance_max_bypass_distance_cm=avoidance_max_bypass_distance_cm,
            heading_hold_enabled=heading_hold_enabled,
            heading_hold_kp=heading_hold_kp,
            heading_hold_steer_max=heading_hold_steer_max,
            heading_hold_deadband_deg=heading_hold_deadband_deg,
            heading_hold_steer_speed_ratio=heading_hold_steer_speed_ratio,
            heading_hold_min_speed_percent=heading_hold_min_speed_percent,
            heading_hold_steer_cap_min_spd_percent=heading_hold_steer_cap_min_speed_percent,
            heading_hold_steer_trim=heading_hold_steer_trim,
            heading_hold_invert_steer=heading_hold_invert_steer,
            forward_soft_start_sec=forward_soft_start_sec,
            turn_check_interval_sec=turn_check_interval_sec,
            turn_obstacle_check_interval_sec=turn_obstacle_check_interval_sec,
            turn_timeout_per_deg=turn_timeout_per_deg,
            turn_timeout_min=turn_timeout_min,
        )
        self.lifecycle: MotionLifecycle = MotionLifecycle()
        self._stop_event: Event = self.lifecycle.stop_event
        self._sync_runtime_settings_from_config()

        self._linear_motion: LinearMotionExecutor = LinearMotionExecutor(
            motor_controller=motor_controller,
            ultrasonic_sensor=ultrasonic_sensor,
            gyroscope=gyroscope,
            min_obstacle_distance_cm=self.config.min_obstacle_distance_cm,
            deceleration_distance_cm=self.config.deceleration_distance_cm,
            max_speed_cm_per_sec=self.config.max_speed_cm_per_sec,
            update_interval_sec=self.config.update_interval_sec,
            forward_soft_start_sec=self.config.forward_soft_start_sec,
            heading_hold_enabled=self.config.heading_hold_enabled,
            heading_hold_kp=self.config.heading_hold_kp,
            heading_hold_steer_max=self.config.heading_hold_steer_max,
            heading_hold_deadband_deg=self.config.heading_hold_deadband_deg,
            heading_hold_steer_speed_ratio=self.config.heading_hold_steer_speed_ratio,
            heading_hold_min_speed_percent=self.config.heading_hold_min_speed_percent,
            heading_hold_steer_cap_min_spd_percent=self.config.heading_hold_steer_cap_min_spd_percent,
            heading_hold_steer_trim=self.config.heading_hold_steer_trim,
            heading_hold_invert_steer=self.config.heading_hold_invert_steer,
            lifecycle=self.lifecycle,
        )

        self._turn_executor: TurnExecutor = TurnExecutor(
            motor_controller=motor_controller,
            ultrasonic_sensor=ultrasonic_sensor,
            gyroscope=gyroscope,
            min_obstacle_distance_cm=self.config.min_obstacle_distance_cm,
            turn_speed_percent=self.config.turn_speed_percent,
            turn_slowdown_remaining_deg=self.config.turn_slowdown_remaining_deg,
            turn_creep_speed_percent=self.config.turn_creep_speed_percent,
            turn_angle_trim_deg=self.config.turn_angle_trim_deg,
            last_turn_angle_trim_deg=self.config.last_turn_angle_trim_deg,
            turn_check_interval_sec=self.config.turn_check_interval_sec,
            turn_obstacle_check_interval_sec=self.config.turn_obstacle_check_interval_sec,
            turn_timeout_per_deg=self.config.turn_timeout_per_deg,
            turn_timeout_min=self.config.turn_timeout_min,
            lifecycle=self.lifecycle,
        )

        self._route_runner: RouteRunner = RouteRunner(
            linear_motion=self._linear_motion,
            turn_executor=self._turn_executor,
            move_forward=motor_controller.move_forward,
            move_backward=motor_controller.move_backward,
            gyroscope=gyroscope,
            lifecycle=self.lifecycle,
            base_speed_percent=self.base_speed_percent,
            forward_segment_runner=self._run_forward_segment,
        )

        self._route_executor: RouteExecutor = RouteExecutor(
            route_runner=self._route_runner,
            motor_controller=motor_controller,
            lifecycle=self.lifecycle,
        )

    @property
    def _is_moving(self) -> bool:
        """Совместимость со старыми тестами, ожидающими приватный флаг."""
        return self.lifecycle.is_moving

    @_is_moving.setter
    def _is_moving(self, value: bool) -> None:
        if value:
            self.lifecycle.set_running(is_route_running=self.lifecycle.is_route_running)
            return
        self.lifecycle.set_stopped()

    @property
    def _is_route_running(self) -> bool:
        """Совместимость со старыми тестами, ожидающими приватный флаг маршрута."""
        return self.lifecycle.is_route_running

    @_is_route_running.setter
    def _is_route_running(self, value: bool) -> None:
        if value or self.lifecycle.is_moving:
            self.lifecycle.set_running(is_route_running=value)
            return
        self.lifecycle.set_stopped()

    def forward_cm_sync(self, distance_cm: float, max_speed_percent: int | None = None) -> None:
        """Проехать заданную дистанцию в текущем потоке."""
        self._stop_motion()
        self.lifecycle.set_running(is_route_running=False)
        self.gyroscope.start(calibrate=True)

        try:
            speed_percent: int = (
                self.base_speed_percent if max_speed_percent is None else max_speed_percent
            )
            self._run_forward_segment(distance_cm, speed_percent)
        finally:
            self.motor_controller.stop()
            self.gyroscope.stop()
            self.lifecycle.set_stopped()

    def execute_route(self, route: Route) -> None:
        """Запустить выполнение маршрута в фоновом потоке."""
        self._stop_motion()
        self._route_runner.base_speed_percent = self.base_speed_percent
        self.lifecycle.set_running(is_route_running=True)
        self._route_executor.execute(route)

    def execute_route_sync(self, route: Route) -> None:
        """Запустить выполнение маршрута в текущем потоке."""
        self._stop_motion()
        self._route_runner.base_speed_percent = self.base_speed_percent
        self.lifecycle.set_running(is_route_running=True)

        try:
            self._route_executor.execute_sync(route)
        finally:
            self.motor_controller.stop()
            self.lifecycle.set_stopped()

    def stop(self) -> None:
        """Остановить текущее движение."""
        self._route_executor.stop()
        self._stop_motion()

    def destroy(self) -> None:
        """Освободить все ресурсы, связанные с движением."""
        self.stop()
        self.motor_controller.destroy()
        self.ultrasonic_sensor.destroy()
        self.gyroscope.destroy()

    def _run_forward_segment(self, distance_cm: float, speed_percent: int) -> bool:
        """Запустить forward-сегмент с obstacle avoidance поверх linear executor."""
        remaining_cm: float = max(0.0, distance_cm)

        try:
            while self.lifecycle.should_keep_running():
                if self._has_reached_target_distance(remaining_cm):
                    self.motor_controller.stop()
                    return True

                move_result: LinearMoveResult = self._run_linear_motion(
                    distance_cm=remaining_cm,
                    speed_percent=speed_percent,
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
                    speed_percent=speed_percent,
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
        """Запуск сегмента «назад» без фронтальной obstacle-check логики."""
        move_result: LinearMoveResult = self._run_linear_motion(
            distance_cm=distance_cm,
            speed_percent=speed_percent,
            move_forward=False,
            obstacle_aware=False,
        )
        return move_result.completed

    def _run_linear_motion(
        self,
        distance_cm: float,
        speed_percent: int,
        move_forward: bool,
        obstacle_aware: bool,
    ) -> LinearMoveResult:
        """Выполнить линейное движение через reusable executor с расширенным результатом."""
        if self._has_reached_target_distance(distance_cm):
            self.motor_controller.stop()
            return LinearMoveResult(completed=True, traveled_cm=0.0)

        self._linear_motion.min_obstacle_distance_cm = self.min_obstacle_distance_cm
        self._linear_motion.deceleration_distance_cm = self.deceleration_distance_cm
        self._linear_motion.max_speed_cm_per_sec = self.max_speed_cm_per_sec
        self._linear_motion.update_interval_sec = self.update_interval_sec

        move_fn = self.motor_controller.move_forward if move_forward else self.motor_controller.move_backward
        result: LinearMotionExecutionResult = self._linear_motion.run_with_result(
            distance_cm=distance_cm,
            speed_percent=speed_percent,
            move_fn=move_fn,
            obstacle_aware=obstacle_aware,
            obstacle_distance_provider=(
                self._measure_motion_obstacle_distance if obstacle_aware else None
            ),
        )
        return LinearMoveResult(
            completed=result.completed,
            traveled_cm=result.traveled_cm,
            blocked=result.blocked,
        )

    def _run_obstacle_avoidance(self, remaining_cm: float, speed_percent: int) -> AvoidanceResult:
        """Пошаговый автомат обхода препятствия."""
        context: AvoidanceContext = AvoidanceContext()

        while self.lifecycle.should_keep_running():
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
                if not side_step.heading_restored:
                    logger.warning("Обход остановлен: после бокового шага не удалось восстановить курс.")
                    break
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
                if not self._is_front_clear_confirmed():
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
                if not rejoin_step.heading_restored:
                    logger.warning(
                        "Обход остановлен: после попытки возврата не удалось восстановить курс."
                    )
                    break
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
                    if not self._is_front_clear_confirmed():
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
            stop_on_front_obstacle=False,
        )

        move_result: LinearMoveResult = LinearMoveResult(completed=False, traveled_cm=0.0)
        if turn_result.completed:
            move_result = self._run_linear_motion(
                distance_cm=distance_cm,
                speed_percent=speed_percent,
                move_forward=True,
                obstacle_aware=True,
            )

        if turn_result.angle_deg <= self.DISTANCE_TOLERANCE_CM:
            return move_result

        restore_result: TurnResult = self._turn_relative(
            turn_left=not side.turn_left,
            target_angle=turn_result.angle_deg,
            stop_on_front_obstacle=False,
        )
        if restore_result.completed:
            return move_result

        logger.warning("Не удалось полностью восстановить исходный курс.")
        return LinearMoveResult(
            completed=False,
            traveled_cm=move_result.traveled_cm,
            blocked=move_result.blocked,
            heading_restored=False,
        )

    def _turn_relative(
        self,
        turn_left: bool,
        target_angle: float,
        stop_on_front_obstacle: bool,
    ) -> TurnResult:
        """Поворот на относительный угол по данным turn executor."""
        self._turn_executor.min_obstacle_distance_cm = self.min_obstacle_distance_cm
        self._turn_executor.turn_speed_percent = self.turn_speed_percent

        turn_result: TurnExecutionResult = self._turn_executor.run_relative(
            target_angle_deg=target_angle,
            turn_left=turn_left,
            stop_on_front_obstacle=stop_on_front_obstacle,
        )
        return TurnResult(completed=turn_result.completed, angle_deg=turn_result.angle_deg)

    def _measure_front_distance(self, samples: int | None = None) -> float:
        """Измерение фронтальной дистанции с подавлением одиночных шумовых чтений."""
        sample_count: int = max(
            self.BLOCK_CONFIRMATION_SAMPLES_MIN,
            samples or self.avoidance_confirm_readings,
        )
        measurements: list[float] = [
            self.ultrasonic_sensor.measure_distance_cm()
            for _ in range(sample_count)
        ]
        measurements.sort()
        return measurements[len(measurements) // 2]

    def _is_front_clear_confirmed(self) -> bool:
        """Проверка, что фронт стабильно свободен с небольшим запасом."""
        clear_threshold_cm: float = self.min_obstacle_distance_cm + self.CLEARANCE_HYSTERESIS_CM

        for check_index in range(self.CLEAR_CONFIRMATION_CHECKS):
            obstacle_cm: float = self._measure_front_distance()
            if obstacle_cm <= clear_threshold_cm:
                return False

            if check_index + 1 < self.CLEAR_CONFIRMATION_CHECKS:
                time.sleep(self.update_interval_sec)

        return True

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

    def _is_front_blocked(self, obstacle_cm: float) -> bool:
        """Проверка, что впереди недостаточно места для безопасного движения."""
        return obstacle_cm <= self.min_obstacle_distance_cm

    def _has_reached_target_distance(self, distance_cm: float) -> bool:
        """Проверка достижения цели с учётом допуска на оценку пути."""
        return distance_cm <= self.DISTANCE_TOLERANCE_CM

    def _sync_runtime_settings_from_config(self) -> None:
        """Разворачивает конфиг в mutable runtime-поля контроллера."""
        self.min_obstacle_distance_cm: float = self.config.min_obstacle_distance_cm
        self.deceleration_distance_cm: float = self.config.deceleration_distance_cm
        self.base_speed_percent: int = self.config.base_speed_percent
        self.turn_speed_percent: int = self.config.turn_speed_percent
        self.max_speed_cm_per_sec: float = self.config.max_speed_cm_per_sec
        self.update_interval_sec: float = self.config.update_interval_sec
        self.avoidance_scan_angle_deg: float = self.config.avoidance_scan_angle_deg
        self.avoidance_side_step_cm: float = self.config.avoidance_side_step_cm
        self.avoidance_forward_step_cm: float = self.config.avoidance_forward_step_cm
        self.avoidance_rejoin_step_cm: float = self.config.avoidance_rejoin_step_cm
        self.avoidance_max_attempts: int = self.config.avoidance_max_attempts
        self.avoidance_confirm_readings: int = self.config.avoidance_confirm_readings
        self.avoidance_min_side_clearance_cm: float = self.config.avoidance_min_side_clearance_cm
        self.avoidance_max_lateral_offset_cm: float = self.config.avoidance_max_lateral_offset_cm
        self.avoidance_max_bypass_distance_cm: float = self.config.avoidance_max_bypass_distance_cm

    def _stop_motion(self) -> None:
        """Остановить движение и дождаться фонового потока при необходимости."""
        self.lifecycle.stop_and_join(self.motor_controller.stop, timeout_sec=self.STOP_JOIN_TIMEOUT_SEC)
