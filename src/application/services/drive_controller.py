from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from enum import Enum
from threading import Event
from typing import final

from src.application.models.route import Route
from src.application.protocols import (
    DriveControllerProtocol,
    GyroscopeProtocol,
    HeadServoProtocol,
    MotorControllerProtocol,
    UltrasonicSensorProtocol,
)
from src.application.services.linear_motion_executor import (
    LinearMotionExecutionResult,
    LinearMotionExecutor,
)
from src.application.services.motion_config import MotionConfig
from src.application.services.motion_events import (
    MotionEvent,
    MotionEventHub,
    MotionEventType,
    MotionStatus,
)
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
    stop_reason: str = "unknown"


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


@dataclass(frozen=True, slots=True)
class DistanceMeasurementSummary:
    """Сводка серии измерений фронтальной дистанции."""

    median_cm: float
    raw_samples_cm: tuple[float, ...]
    min_cm: float
    max_cm: float
    spread_cm: float
    reliable: bool = True
    reliability_reason: str = "stable"


@dataclass(slots=True)
class SideScanResult:
    """Результат сканирования одной стороны перед началом обхода."""

    side: AvoidanceSide
    target_angle_deg: float
    turned_angle_deg: float
    clearance_cm: float | None
    heading_restored: bool
    scan_completed: bool
    used_partial_scan: bool = False
    scan_useful: bool = False
    limited_confidence: bool = False
    turn_stop_reason: str = "unknown"
    rejection_reason: str | None = None
    selection_blocked: bool = False
    primary_target_angle_deg: float | None = None
    primary_turned_angle_deg: float | None = None
    primary_clearance_cm: float | None = None
    secondary_target_angle_deg: float | None = None
    secondary_turned_angle_deg: float | None = None
    secondary_clearance_cm: float | None = None
    secondary_rejection_reason: str | None = None
    selection_reason: str | None = None
    measurement_summary: DistanceMeasurementSummary | None = None

    @property
    def is_selectable(self) -> bool:
        """Можно ли использовать сторону для выбора направления обхода."""
        return (
            not self.selection_blocked
            and self.heading_restored
            and self.scan_useful
            and self.clearance_cm is not None
        )


@final
class DriveController(DriveControllerProtocol):
    """Контроллер движения поверх модульных executors с obstacle avoidance."""

    DISTANCE_TOLERANCE_CM: float = 0.5
    WORKSPACE_LIMIT_TOLERANCE_CM: float = 1.0
    BLOCK_CONFIRMATION_SAMPLES_MIN: int = 3
    CLEAR_CONFIRMATION_CHECKS: int = 2
    CLEARANCE_HYSTERESIS_CM: float = 3.0
    SIDE_SCAN_MIN_ADVISORY_ANGLE_DEG: float = 18.0
    SIDE_SCAN_MIN_ADVISORY_RATIO: float = 0.50
    SIDE_SCAN_THRESHOLD_RELAXATION_CM: float = 3.0
    SIDE_SCAN_SECONDARY_ANGLE_DEG: float = 60.0
    SIDE_SCAN_MIN_EFFECTIVE_ANGLE_DEG: float = 20.0
    SIDE_SCAN_MIN_EFFECTIVE_RATIO: float = 0.65
    SIDE_SCAN_MEASUREMENT_MAX_SPREAD_CM: float = 8.0
    SIDE_SCAN_SENSOR_SETTLE_SEC: float = 0.07
    REJOIN_FINAL_LATERAL_RECOVERY_RATIO: float = 0.95
    STOP_JOIN_TIMEOUT_SEC: float = 1.0
    DEFAULT_WORKSPACE_LIMIT_CM: float = 100.0

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
        motion_events: MotionEventHub | None = None,
        workspace_limit_cm: float = DEFAULT_WORKSPACE_LIMIT_CM,
        head_servo: HeadServoProtocol | None = None,
        head_servo_home_angle_deg: float = 0.0,
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
        self._motion_events: MotionEventHub | None = motion_events
        self._head_servo: HeadServoProtocol | None = head_servo
        self._head_servo_home_angle_deg: float = head_servo_home_angle_deg
        self._workspace_limit_cm: float = workspace_limit_cm
        self._position_x_cm: float = 0.0
        self._position_y_cm: float = 0.0
        self._heading_deg: float = 0.0
        self._motion_error_reported: bool = False
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
            backward_segment_runner=self._run_backward_segment,
            turn_completed_callback=self._handle_turn_completed,
        )

        self._route_executor: RouteExecutor = RouteExecutor(
            route_runner=self._route_runner,
            motor_controller=motor_controller,
            lifecycle=self.lifecycle,
            on_finished=self._handle_route_finished,
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
        self._fix_head_forward()
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
        self._fix_head_forward()
        self._reset_motion_state()
        self._publish_motion_event(status="moving", event_type="status", message="Маршрут запущен")
        self._route_runner.base_speed_percent = self.base_speed_percent
        self.lifecycle.set_running(is_route_running=True)
        self._route_executor.execute(route)

    def execute_route_sync(self, route: Route) -> None:
        """Запустить выполнение маршрута в текущем потоке."""
        self._stop_motion()
        self._fix_head_forward()
        self._reset_motion_state()
        self._publish_motion_event(status="moving", event_type="status", message="Маршрут запущен")
        self._route_runner.base_speed_percent = self.base_speed_percent
        self.lifecycle.set_running(is_route_running=True)

        try:
            self._route_executor.execute_sync(route)
        finally:
            self.motor_controller.stop()
            self.lifecycle.set_stopped()
            self._publish_motion_event(
                status="stopped",
                event_type="status",
                message="Маршрут завершен",
            )

    def stop(self) -> None:
        """Остановить текущее движение."""
        self._route_executor.stop()
        self._stop_motion()
        self._publish_motion_event(
            status="stopped",
            event_type="status",
            message="Движение остановлено",
        )

    def destroy(self) -> None:
        """Освободить все ресурсы, связанные с движением."""
        self.stop()
        self.motor_controller.destroy()
        self.ultrasonic_sensor.destroy()
        self.gyroscope.destroy()
        if self._head_servo is not None:
            self._head_servo.destroy()

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

        move_fn = (
            self.motor_controller.move_forward
            if move_forward
            else self.motor_controller.move_backward
        )
        result: LinearMotionExecutionResult = self._linear_motion.run_with_result(
            distance_cm=distance_cm,
            speed_percent=speed_percent,
            move_fn=move_fn,
            obstacle_aware=obstacle_aware,
            obstacle_distance_provider=(
                self._measure_motion_obstacle_distance if obstacle_aware else None
            ),
            progress_callback=self._handle_linear_motion_progress,
            progress_direction=1 if move_forward else -1,
        )
        return LinearMoveResult(
            completed=result.completed,
            traveled_cm=result.traveled_cm,
            blocked=result.blocked,
        )

    def _reset_motion_state(self) -> None:
        """Сбросить расчетную позицию маршрута."""
        self._position_x_cm = 0.0
        self._position_y_cm = 0.0
        self._heading_deg = 0.0
        self._motion_error_reported = False

    def _fix_head_forward(self) -> None:
        """Зафиксировать голову/датчик перед началом движения."""
        if self._head_servo is None:
            return

        try:
            self._head_servo.set_angle(self._head_servo_home_angle_deg)
        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.warning("Не удалось зафиксировать голову перед движением: %s", exc)

    def _handle_linear_motion_progress(
        self,
        traveled_cm: float,
        delta_cm: float,
        direction: int,
        obstacle_cm: float | None,
    ) -> None:
        """Обновить расчетную позицию по прогрессу линейного движения."""
        if delta_cm != 0.0:
            heading_rad: float = math.radians(self._heading_deg)
            self._position_x_cm += math.sin(heading_rad) * delta_cm
            self._position_y_cm += math.cos(heading_rad) * delta_cm

        status: MotionStatus = "moving"
        message: str | None = None
        if obstacle_cm is not None and obstacle_cm <= self.min_obstacle_distance_cm:
            status = "blocked"
            message = f"Препятствие ближе {self.min_obstacle_distance_cm:.0f} см"

        self._publish_motion_event(
            status=status,
            event_type="position",
            message=message,
            obstacle_cm=obstacle_cm,
        )
        if status == "blocked" and obstacle_cm is not None:
            self._publish_obstacle_event(obstacle_cm)
        self._stop_if_workspace_limit_exceeded()

    def _handle_turn_completed(self, angle_deg: float, turn_left: bool) -> None:
        """Обновить расчетный курс после маршрутного поворота."""
        signed_angle: float = -angle_deg if turn_left else angle_deg
        self._heading_deg = (self._heading_deg + signed_angle) % 360.0
        self._publish_motion_event(
            status="turning",
            event_type="position",
            message=f"Поворот {'налево' if turn_left else 'направо'}: {angle_deg:.0f}°",
        )

    def _handle_route_finished(self) -> None:
        """Опубликовать финальное событие фонового маршрута."""
        if not self._motion_error_reported:
            self._publish_motion_event(
                status="stopped",
                event_type="status",
                message="Маршрут завершен",
            )

    def _publish_motion_event(
        self,
        *,
        status: MotionStatus,
        event_type: MotionEventType,
        message: str | None = None,
        obstacle_cm: float | None = None,
    ) -> None:
        """Опубликовать событие движения, если подключен event hub."""
        if self._motion_events is None:
            return

        self._motion_events.publish(
            MotionEvent(
                type=event_type,
                status=status,
                x_cm=self._position_x_cm,
                y_cm=self._position_y_cm,
                heading_deg=self._heading_deg,
                message=message,
                obstacle_cm=obstacle_cm,
            )
        )

    def _publish_obstacle_event(self, obstacle_cm: float) -> None:
        """Опубликовать расчетные координаты обнаруженного препятствия."""
        if self._motion_events is None:
            return

        heading_rad: float = math.radians(self._heading_deg)
        obstacle_x_cm: float = self._position_x_cm + math.sin(heading_rad) * obstacle_cm
        obstacle_y_cm: float = self._position_y_cm + math.cos(heading_rad) * obstacle_cm
        self._motion_events.publish(
            MotionEvent(
                type="obstacle",
                status="blocked",
                x_cm=self._position_x_cm,
                y_cm=self._position_y_cm,
                heading_deg=self._heading_deg,
                message="Обнаружено препятствие",
                obstacle_cm=obstacle_cm,
                obstacle_x_cm=obstacle_x_cm,
                obstacle_y_cm=obstacle_y_cm,
            )
        )

    def _stop_if_workspace_limit_exceeded(self) -> None:
        """Остановить движение и уведомить UI при выходе из зоны 40 см."""
        if self._motion_error_reported:
            return

        effective_workspace_limit_cm: float = (
            self._workspace_limit_cm + self.WORKSPACE_LIMIT_TOLERANCE_CM
        )
        if (
            abs(self._position_x_cm) <= effective_workspace_limit_cm
            and abs(self._position_y_cm) <= effective_workspace_limit_cm
        ):
            return

        self._motion_error_reported = True
        self.motor_controller.stop()
        self.lifecycle.set_stopped()
        self._publish_motion_event(
            status="error",
            event_type="error",
            message=f"Машинка вышла за пределы зоны {self._workspace_limit_cm:.0f} см",
        )

    def _run_obstacle_avoidance(self, remaining_cm: float, speed_percent: int) -> AvoidanceResult:
        """Пошаговый автомат обхода препятствия."""
        context: AvoidanceContext = AvoidanceContext()

        while self.lifecycle.should_keep_running():
            if self._has_reached_target_distance(remaining_cm - context.forward_progress_cm):
                if self._has_reached_target_distance(context.lateral_offset_cm):
                    self._log_lateral_offset_recovery(
                        reason=(
                            "remaining segment exhausted and lateral offset "
                            "already within tolerance"
                        ),
                        lateral_offset_cm=context.lateral_offset_cm,
                    )
                    return AvoidanceResult(
                        completed=True,
                        forward_progress_cm=context.forward_progress_cm,
                    )
                if context.state is not AvoidanceState.TRY_REJOIN:
                    context.state = AvoidanceState.TRY_REJOIN
                    continue

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
                    logger.warning(
                        "Обход остановлен: после бокового шага "
                        "не удалось восстановить курс."
                    )
                    break
                self._update_avoidance_progress(
                    context=context,
                    traveled_cm=side_step.traveled_cm,
                    attempts_delta=1,
                )
                offset_before_cm: float = context.lateral_offset_cm
                context.lateral_offset_cm += side_step.traveled_cm
                logger.info(
                    "Обход: lateral offset after side step. before=%.1f added=%.1f after=%.1f "
                    "completed=%s blocked=%s",
                    offset_before_cm,
                    side_step.traveled_cm,
                    context.lateral_offset_cm,
                    side_step.completed,
                    side_step.blocked,
                )

                if not side_step.completed:
                    if not side_step.blocked:
                        logger.warning(
                            "Обход остановлен: боковой шаг прерван без obstacle-stop "
                            "(traveled=%.1f см).",
                            side_step.traveled_cm,
                        )
                        break

                    if side_step.traveled_cm <= self.DISTANCE_TOLERANCE_CM:
                        logger.warning("Обход остановлен: боковой шаг не удался.")
                        break

                context.state = AvoidanceState.CHECK_FRONT
                continue

            if context.state is AvoidanceState.CHECK_FRONT:
                if not self._is_front_clear_confirmed():
                    context.state = AvoidanceState.SIDE_STEP
                    continue

                if self._has_reached_target_distance(context.lateral_offset_cm):
                    self._log_lateral_offset_recovery(
                        reason=(
                            "front is clear after side step and lateral offset "
                            "is within tolerance"
                        ),
                        lateral_offset_cm=context.lateral_offset_cm,
                    )
                    return AvoidanceResult(
                        completed=True,
                        forward_progress_cm=context.forward_progress_cm,
                    )

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
                    if self._has_reached_target_distance(context.lateral_offset_cm):
                        self._log_lateral_offset_recovery(
                            reason=(
                                "forward progress completed and lateral offset "
                                "is within tolerance"
                            ),
                            lateral_offset_cm=context.lateral_offset_cm,
                        )
                        return AvoidanceResult(
                            completed=True,
                            forward_progress_cm=context.forward_progress_cm,
                        )
                    context.state = AvoidanceState.TRY_REJOIN
                    continue

                if not forward_step.completed:
                    if forward_step.blocked:
                        context.state = AvoidanceState.SIDE_STEP
                        continue
                    logger.warning("Обход остановлен: шаг вдоль препятствия не завершён.")
                    break

                if self._has_reached_target_distance(context.lateral_offset_cm):
                    self._log_lateral_offset_recovery(
                        reason="forward step completed and lateral offset is within tolerance",
                        lateral_offset_cm=context.lateral_offset_cm,
                    )
                    return AvoidanceResult(
                        completed=True,
                        forward_progress_cm=context.forward_progress_cm,
                    )

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
                offset_before_cm = context.lateral_offset_cm
                effective_recovery_cm: float = self._effective_rejoin_lateral_recovery_cm(
                    traveled_cm=rejoin_step.traveled_cm,
                    lateral_offset_before_cm=offset_before_cm,
                )
                context.lateral_offset_cm = max(
                    0.0,
                    context.lateral_offset_cm - effective_recovery_cm,
                )
                logger.info(
                    "Обход: lateral offset after rejoin. before=%.1f raw_rejoin=%.1f "
                    "effective_recovery=%.1f ratio=%.2f after=%.1f completed=%s blocked=%s",
                    offset_before_cm,
                    rejoin_step.traveled_cm,
                    effective_recovery_cm,
                    (
                        self.REJOIN_FINAL_LATERAL_RECOVERY_RATIO
                        if offset_before_cm <= self.avoidance_rejoin_step_cm
                        else 1.0
                    ),
                    context.lateral_offset_cm,
                    rejoin_step.completed,
                    rejoin_step.blocked,
                )

                if not rejoin_step.completed:
                    if not rejoin_step.blocked:
                        logger.warning(
                            "Обход остановлен: попытка возврата прервана без obstacle-stop "
                            "(traveled=%.1f см).",
                            rejoin_step.traveled_cm,
                        )
                        break

                    if rejoin_step.traveled_cm <= self.DISTANCE_TOLERANCE_CM:
                        context.state = AvoidanceState.CHECK_FRONT
                        continue

                if self._has_reached_target_distance(context.lateral_offset_cm):
                    if not self._is_front_clear_confirmed():
                        context.state = AvoidanceState.SIDE_STEP
                        continue
                    self._log_lateral_offset_recovery(
                        reason="rejoin reduced lateral offset to tolerance and front is clear",
                        lateral_offset_cm=context.lateral_offset_cm,
                    )
                    return AvoidanceResult(
                        completed=True,
                        forward_progress_cm=context.forward_progress_cm,
                    )

                if self._should_retry_final_rejoin_immediately(
                    lateral_offset_cm=context.lateral_offset_cm,
                ):
                    logger.info(
                        "Обход: immediate final rejoin retry requested. lateral_offset_cm=%.1f "
                        "retry_window_cm=%.1f",
                        context.lateral_offset_cm,
                        self._final_rejoin_retry_window_cm(),
                    )
                    context.state = AvoidanceState.TRY_REJOIN
                    continue

                context.state = AvoidanceState.CHECK_FRONT
                continue

        self.motor_controller.stop()
        return AvoidanceResult(completed=False, forward_progress_cm=context.forward_progress_cm)

    def _select_avoidance_side(self) -> AvoidanceSide | None:
        """Выбор стороны обхода по данным короткого сканирования."""
        scan_results: list[SideScanResult] = [
            self._scan_side_observation(AvoidanceSide.LEFT),
            self._scan_side_observation(AvoidanceSide.RIGHT),
        ]
        preferred_threshold_cm: float = self.avoidance_min_side_clearance_cm
        exploratory_threshold_cm: float = self._exploratory_side_clearance_threshold()
        for result in scan_results:
            self._log_side_selection_assessment(
                result=result,
                preferred_threshold_cm=preferred_threshold_cm,
                exploratory_threshold_cm=exploratory_threshold_cm,
            )

        preferred_candidates: list[SideScanResult] = [
            result
            for result in scan_results
            if result.is_selectable and (result.clearance_cm or 0.0) >= preferred_threshold_cm
        ]
        if preferred_candidates:
            selected: SideScanResult = max(
                preferred_candidates,
                key=lambda result: result.clearance_cm or 0.0,
            )
            logger.info(
                "Обход: выбрана сторона %s. left=%s right=%s threshold=%.1f exploratory=%.1f",
                selected.side.value,
                self._scan_clearance_for_log(scan_results, AvoidanceSide.LEFT),
                self._scan_clearance_for_log(scan_results, AvoidanceSide.RIGHT),
                preferred_threshold_cm,
                exploratory_threshold_cm,
            )
            return selected.side

        exploratory_candidates: list[SideScanResult] = [
            result
            for result in scan_results
            if result.is_selectable and (result.clearance_cm or 0.0) >= exploratory_threshold_cm
        ]
        if exploratory_candidates:
            selected = max(
                exploratory_candidates,
                key=lambda result: result.clearance_cm or 0.0,
            )
            logger.warning(
                "Обход: используем пограничный side scan для стороны %s. "
                "clearance=%.1f preferred_threshold=%.1f exploratory_threshold=%.1f",
                selected.side.value,
                selected.clearance_cm,
                preferred_threshold_cm,
                exploratory_threshold_cm,
            )
            return selected.side

        inconclusive_usable_candidates: list[SideScanResult] = [
            result
            for result in scan_results
            if result.is_selectable and (
                result.limited_confidence or result.turn_stop_reason != "target_reached"
            )
        ]
        if inconclusive_usable_candidates:
            selected = max(
                inconclusive_usable_candidates,
                key=lambda result: result.clearance_cm or 0.0,
            )
            logger.warning(
                "Обход: side scan неполный/неуверенный, но usable. "
                "Выбираем сторону %s как exploratory candidate. clearance=%.1f "
                "turn_stop_reason=%s limited_confidence=%s",
                selected.side.value,
                selected.clearance_cm,
                selected.turn_stop_reason,
                selected.limited_confidence,
            )
            return selected.side

        logger.warning(
            "Обход невозможен по результатам side scan. final_reason=%s threshold=%.1f "
            "exploratory_threshold=%.1f "
            "left={%s} right={%s}",
            self._describe_side_selection_failure(
                scan_results=scan_results,
                exploratory_threshold_cm=exploratory_threshold_cm,
            ),
            preferred_threshold_cm,
            exploratory_threshold_cm,
            self._format_side_scan_result(scan_results[0]),
            self._format_side_scan_result(scan_results[1]),
        )
        return None

    def _scan_side_observation(self, side: AvoidanceSide) -> SideScanResult:
        """Сканирует сторону ступенчато: сначала обычный угол, затем более широкий fallback."""
        exploratory_threshold_cm: float = self._exploratory_side_clearance_threshold()
        primary_result: SideScanResult = self._scan_side_observation_at_angle(
            side=side,
            target_angle=self.avoidance_scan_angle_deg,
        )
        self._annotate_side_scan_result(
            result=primary_result,
            primary_result=primary_result,
            selection_reason=self._default_side_scan_selection_reason(
                result=primary_result,
                exploratory_threshold_cm=exploratory_threshold_cm,
            ),
        )

        secondary_angle: float | None = self._secondary_side_scan_target_angle(
            primary_target_angle_deg=primary_result.target_angle_deg,
        )
        if (
            secondary_angle is None
            or not primary_result.is_selectable
            or (primary_result.clearance_cm or 0.0) >= exploratory_threshold_cm
        ):
            self._log_side_scan_result(primary_result)
            return primary_result

        secondary_result: SideScanResult = self._scan_side_observation_at_angle(
            side=side,
            target_angle=secondary_angle,
        )
        secondary_clearance: float = secondary_result.clearance_cm or 0.0
        primary_clearance: float = primary_result.clearance_cm or 0.0

        if not secondary_result.is_selectable:
            secondary_rejection_reason: str = (
                secondary_result.rejection_reason
                or "scan unusable or heading not restored"
            )
            final_result: SideScanResult = self._annotate_side_scan_result(
                result=primary_result,
                primary_result=primary_result,
                secondary_result=secondary_result,
                selection_blocked=True,
                selection_reason=(
                    "45° clearance below exploratory threshold; "
                    "60° fallback scan unusable"
                ),
                rejection_reason=(
                    "45° scan insufficient "
                    f"({primary_clearance:.1f} < {exploratory_threshold_cm:.1f}), "
                    "а 60° scan unusable: "
                    f"{secondary_rejection_reason}"
                ),
            )
            self._log_side_scan_result(final_result)
            return final_result

        if secondary_clearance < exploratory_threshold_cm:
            final_result = self._annotate_side_scan_result(
                result=secondary_result,
                primary_result=primary_result,
                secondary_result=secondary_result,
                selection_blocked=True,
                selection_reason=(
                    "both 45° and 60° clearances stayed below exploratory threshold"
                ),
                rejection_reason=(
                    "45° clearance="
                    f"{primary_clearance:.1f}, 60° clearance={secondary_clearance:.1f}, "
                    f"оба ниже exploratory threshold {exploratory_threshold_cm:.1f}"
                ),
            )
            self._log_side_scan_result(final_result)
            return final_result

        final_result = self._annotate_side_scan_result(
            result=secondary_result,
            primary_result=primary_result,
            secondary_result=secondary_result,
            selection_reason=(
                "accepted after 60° fallback scan because 45° clearance "
                "was below exploratory threshold"
            ),
        )
        self._log_side_scan_result(final_result)
        return final_result

    def _scan_side_observation_at_angle(
        self,
        *,
        side: AvoidanceSide,
        target_angle: float,
    ) -> SideScanResult:
        """Сканирует одну сторону на конкретном угле и возвращает одиночный результат."""
        target_angle = max(0.0, min(90.0, target_angle))
        if self._has_reached_target_distance(target_angle):
            measurement_summary: DistanceMeasurementSummary = self._measure_front_distance_summary()
            clearance: float = measurement_summary.median_cm
            result = SideScanResult(
                side=side,
                target_angle_deg=target_angle,
                turned_angle_deg=0.0,
                clearance_cm=clearance,
                heading_restored=True,
                scan_completed=True,
                scan_useful=True,
                turn_stop_reason="not_needed",
                measurement_summary=measurement_summary,
            )
            self._log_side_scan_result(result)
            return result

        turn_result: TurnResult = self._turn_relative(
            turn_left=side.turn_left,
            target_angle=target_angle,
            stop_on_front_obstacle=False,
        )
        useful_partial_scan: bool = (
            not turn_result.completed
            and self._is_useful_partial_side_scan(
                target_angle_deg=target_angle,
                turned_angle_deg=turn_result.angle_deg,
            )
        )
        used_partial_scan: bool = useful_partial_scan
        clearance: float | None = None
        measurement_summary: DistanceMeasurementSummary | None = None
        heading_restored: bool = True
        rejection_reason: str | None = None

        try:
            if turn_result.completed or useful_partial_scan:
                self._wait_for_side_scan_sensor_settle()
                measurement_summary = self._measure_front_distance_summary()
                clearance = measurement_summary.median_cm
            else:
                rejection_reason = (
                    "недостаточный угол сканирования "
                    f"({turn_result.angle_deg:.1f}/{target_angle:.1f}°, "
                    f"min_useful={self._side_scan_useful_angle_threshold(target_angle):.1f}°)"
                )
        finally:
            if turn_result.angle_deg > self.DISTANCE_TOLERANCE_CM:
                restore_result: TurnResult = self._turn_relative(
                    turn_left=not side.turn_left,
                    target_angle=turn_result.angle_deg,
                    stop_on_front_obstacle=False,
                )
                if not restore_result.completed:
                    heading_restored = False
                    rejection_reason = (
                        "не удалось восстановить курс "
                        f"(restore_angle={restore_result.angle_deg:.1f}/{turn_result.angle_deg:.1f}°)"
                    )

        result = SideScanResult(
            side=side,
            target_angle_deg=target_angle,
            turned_angle_deg=turn_result.angle_deg,
            clearance_cm=clearance,
            heading_restored=heading_restored,
            scan_completed=turn_result.completed,
            used_partial_scan=used_partial_scan,
            scan_useful=clearance is not None,
            limited_confidence=clearance is not None and not turn_result.completed,
            turn_stop_reason=turn_result.stop_reason,
            rejection_reason=rejection_reason,
            measurement_summary=measurement_summary,
        )
        return result

    def _secondary_side_scan_target_angle(self, primary_target_angle_deg: float) -> float | None:
        """Возвращает угол fallback side scan или `None`, если дополнительный scan не нужен."""
        secondary_angle: float = max(primary_target_angle_deg, self.SIDE_SCAN_SECONDARY_ANGLE_DEG)
        if self._has_reached_target_distance(secondary_angle - primary_target_angle_deg):
            return None
        return min(90.0, secondary_angle)

    def _default_side_scan_selection_reason(
        self,
        *,
        result: SideScanResult,
        exploratory_threshold_cm: float,
    ) -> str:
        """Объясняет базовое решение после первого side scan до возможного fallback scan."""
        if not result.is_selectable:
            return result.rejection_reason or "primary side scan unusable"

        clearance_cm: float = result.clearance_cm or 0.0
        if clearance_cm >= exploratory_threshold_cm:
            return (
                "accepted on primary side scan "
                f"(clearance {clearance_cm:.1f} >= exploratory threshold "
                f"{exploratory_threshold_cm:.1f})"
            )

        return (
            "primary side scan below exploratory threshold; "
            "will try 60° fallback scan before rejecting the side"
        )

    def _annotate_side_scan_result(
        self,
        *,
        result: SideScanResult,
        primary_result: SideScanResult,
        secondary_result: SideScanResult | None = None,
        selection_blocked: bool = False,
        selection_reason: str | None = None,
        rejection_reason: str | None = None,
    ) -> SideScanResult:
        """Добавляет в итог side scan данные по первому и fallback-скану для логов и выбора."""
        result.primary_target_angle_deg = primary_result.target_angle_deg
        result.primary_turned_angle_deg = primary_result.turned_angle_deg
        result.primary_clearance_cm = primary_result.clearance_cm
        if secondary_result is not None:
            result.secondary_target_angle_deg = secondary_result.target_angle_deg
            result.secondary_turned_angle_deg = secondary_result.turned_angle_deg
            result.secondary_clearance_cm = secondary_result.clearance_cm
            result.secondary_rejection_reason = secondary_result.rejection_reason
        result.selection_blocked = selection_blocked
        result.selection_reason = selection_reason
        if rejection_reason is not None:
            result.rejection_reason = rejection_reason
        elif selection_blocked and selection_reason is not None:
            result.rejection_reason = selection_reason
        return result

    def _scan_side_clearance(self, side: AvoidanceSide) -> float | None:
        """Короткий поворот в сторону для оценки свободного пространства."""
        result: SideScanResult = self._scan_side_observation(side)
        if not result.is_selectable:
            return None
        return result.clearance_cm

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
        self._handle_turn_completed(turn_result.angle_deg, turn_left)
        return TurnResult(
            completed=turn_result.completed,
            angle_deg=turn_result.angle_deg,
            stop_reason=turn_result.stop_reason,
        )

    def _measure_front_distance(self, samples: int | None = None) -> float:
        """Измерение фронтальной дистанции с подавлением одиночных шумовых чтений."""
        return self._measure_front_distance_summary(samples=samples).median_cm

    def _measure_front_distance_summary(
        self,
        samples: int | None = None,
    ) -> DistanceMeasurementSummary:
        """Серия измерений фронтальной дистанции с оценкой стабильности."""
        sample_count: int = max(
            self.BLOCK_CONFIRMATION_SAMPLES_MIN,
            samples or self.avoidance_confirm_readings,
        )
        measurements: list[float] = []

        for _ in range(sample_count):
            distance_cm: float = self.ultrasonic_sensor.measure_distance_cm()
            measurements.append(distance_cm)

        measurements.sort()
        min_cm: float = measurements[0]
        max_cm: float = measurements[-1]
        spread_cm: float = max_cm - min_cm
        reliability_issues: list[str] = []
        if spread_cm > self.SIDE_SCAN_MEASUREMENT_MAX_SPREAD_CM:
            reliability_issues.append(
                "wide_sample_spread="
                f"{spread_cm:.1f}cm>{self.SIDE_SCAN_MEASUREMENT_MAX_SPREAD_CM:.1f}cm"
            )
        reliable: bool = not reliability_issues
        return DistanceMeasurementSummary(
            median_cm=measurements[len(measurements) // 2],
            raw_samples_cm=tuple(measurements),
            min_cm=min_cm,
            max_cm=max_cm,
            spread_cm=spread_cm,
            reliable=reliable,
            reliability_reason=(
                "stable median sample set"
                if reliable
                else ", ".join(reliability_issues)
            ),
        )

    def _wait_for_side_scan_sensor_settle(self) -> None:
        """Даёт ультразвуковому сенсору время обновить чтение после поворота корпуса."""
        settle_sec: float = max(self.update_interval_sec, self.SIDE_SCAN_SENSOR_SETTLE_SEC)
        if settle_sec <= 0.0:
            return
        time.sleep(settle_sec)

    def _effective_rejoin_lateral_recovery_cm(
        self,
        *,
        traveled_cm: float,
        lateral_offset_before_cm: float,
    ) -> float:
        """Консервативно оценивает, сколько бокового смещения действительно снял final rejoin."""
        if traveled_cm <= 0.0:
            return 0.0
        if lateral_offset_before_cm > self.avoidance_rejoin_step_cm:
            return traveled_cm
        return traveled_cm * self.REJOIN_FINAL_LATERAL_RECOVERY_RATIO

    def _final_rejoin_retry_window_cm(self) -> float:
        """Окно остаточного offset, при котором лучше сразу добить rejoin без нового advance."""
        return (
            self.avoidance_rejoin_step_cm * (1.0 - self.REJOIN_FINAL_LATERAL_RECOVERY_RATIO)
        ) + self.DISTANCE_TOLERANCE_CM

    def _should_retry_final_rejoin_immediately(self, *, lateral_offset_cm: float) -> bool:
        """Определяет, нужен ли ещё один короткий rejoin после почти успешного возврата."""
        return (
            lateral_offset_cm > self.DISTANCE_TOLERANCE_CM
            and lateral_offset_cm <= self._final_rejoin_retry_window_cm()
        )

    def _log_lateral_offset_recovery(
        self,
        *,
        reason: str,
        lateral_offset_cm: float,
    ) -> None:
        """Логирует, почему FSM считает боковое смещение уже достаточно скомпенсированным."""
        logger.info(
            "Обход: lateral offset considered recovered. lateral_offset_cm=%.1f "
            "tolerance=%.1f reason=%s",
            lateral_offset_cm,
            self.DISTANCE_TOLERANCE_CM,
            reason,
        )

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

        if context.lateral_offset_cm >= self.avoidance_max_lateral_offset_cm:
            logger.warning(
                "Обход остановлен: достигнуто/превышено боковое смещение (%.1f см).",
                context.lateral_offset_cm,
            )
            return True

        if context.bypass_distance_cm >= self.avoidance_max_bypass_distance_cm:
            logger.warning(
                "Обход остановлен: достигнута/превышена суммарная длина обхода (%.1f см).",
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

    def _exploratory_side_clearance_threshold(self) -> float:
        """Ослабленный порог для шумных пограничных scan-измерений."""
        relaxed_threshold: float = self._raw_exploratory_side_clearance_threshold()
        return min(self.avoidance_min_side_clearance_cm, relaxed_threshold)

    def _raw_exploratory_side_clearance_threshold(self) -> float:
        """Возвращает неограниченный exploratory threshold до защиты от ужесточения."""
        return max(
            self.min_obstacle_distance_cm + self.CLEARANCE_HYSTERESIS_CM,
            self.avoidance_min_side_clearance_cm - self.SIDE_SCAN_THRESHOLD_RELAXATION_CM,
        )

    def _side_scan_useful_angle_threshold(self, target_angle_deg: float) -> float:
        """Минимальный угол для advisory side scan на реальном железе."""
        return min(
            target_angle_deg,
            max(
                self.SIDE_SCAN_MIN_ADVISORY_ANGLE_DEG,
                target_angle_deg * self.SIDE_SCAN_MIN_ADVISORY_RATIO,
            ),
        )

    def _side_scan_effective_angle_threshold(self, target_angle_deg: float) -> float:
        """Минимальный угол для достаточно уверенного side scan."""
        return min(
            target_angle_deg,
            max(
                self.SIDE_SCAN_MIN_EFFECTIVE_ANGLE_DEG,
                target_angle_deg * self.SIDE_SCAN_MIN_EFFECTIVE_RATIO,
            ),
        )

    def _is_useful_partial_side_scan(
        self,
        *,
        target_angle_deg: float,
        turned_angle_deg: float,
    ) -> bool:
        """Проверяет, даёт ли частичный поворот уже хоть сколько-то полезный обзор."""
        return turned_angle_deg >= self._side_scan_useful_angle_threshold(target_angle_deg)

    def _is_effective_side_scan_angle(
        self,
        *,
        target_angle_deg: float,
        turned_angle_deg: float,
    ) -> bool:
        """Определяет, достаточно ли фактического угла для полезного side scan."""
        return turned_angle_deg >= self._side_scan_effective_angle_threshold(target_angle_deg)

    def _scan_clearance_for_log(
        self,
        scan_results: list[SideScanResult],
        side: AvoidanceSide,
    ) -> str:
        """Возвращает clearance для логов или `n/a`, если измерение недоступно."""
        for result in scan_results:
            if result.side is side:
                if result.clearance_cm is None:
                    return "n/a"
                return f"{result.clearance_cm:.1f}"
        return "n/a"

    def _format_side_scan_result(self, result: SideScanResult) -> str:
        """Форматирует результат side scan для диагностического сообщения."""
        clearance_repr: str = (
            f"{result.clearance_cm:.1f}см" if result.clearance_cm is not None else "n/a"
        )
        primary_clearance_repr: str = (
            f"{result.primary_clearance_cm:.1f}см"
            if result.primary_clearance_cm is not None
            else clearance_repr
        )
        secondary_clearance_repr: str = (
            f"{result.secondary_clearance_cm:.1f}см"
            if result.secondary_clearance_cm is not None
            else "n/a"
        )
        reason: str = result.rejection_reason or "ok"
        selection_reason: str = result.selection_reason or "n/a"
        measurement_repr: str = self._format_measurement_summary(result.measurement_summary)
        primary_target_deg: float = (
            result.primary_target_angle_deg or result.target_angle_deg
        )
        primary_turned_deg: float = (
            result.primary_turned_angle_deg or result.turned_angle_deg
        )
        secondary_target_repr: str = (
            f"{result.secondary_target_angle_deg:.1f}°"
            if result.secondary_target_angle_deg is not None
            else "n/a"
        )
        secondary_turned_repr: str = (
            f"{result.secondary_turned_angle_deg:.1f}°"
            if result.secondary_turned_angle_deg is not None
            else "n/a"
        )
        return (
            f"side={result.side.value}, clearance={clearance_repr}, "
            f"target={result.target_angle_deg:.1f}°, turned={result.turned_angle_deg:.1f}°, "
            f"primary_target={primary_target_deg:.1f}°, "
            f"primary_turned={primary_turned_deg:.1f}°, "
            f"primary_clearance={primary_clearance_repr}, "
            f"secondary_target={secondary_target_repr}, "
            f"secondary_turned={secondary_turned_repr}, "
            f"secondary_clearance={secondary_clearance_repr}, "
            f"completed={result.scan_completed}, partial={result.used_partial_scan}, "
            f"useful={result.scan_useful}, limited_confidence={result.limited_confidence}, "
            f"restored={result.heading_restored}, "
            f"turn_stop_reason={result.turn_stop_reason}, "
            f"measurement={measurement_repr}, "
            f"selection_blocked={result.selection_blocked}, selection_reason={selection_reason}, "
            f"reason={reason}"
        )

    def _log_side_scan_result(self, result: SideScanResult) -> None:
        """Пишет подробный лог по side scan и причине возможного отклонения стороны."""
        clearance_repr: str = (
            f"{result.clearance_cm:.1f}" if result.clearance_cm is not None else "n/a"
        )
        primary_clearance_repr: str = (
            f"{result.primary_clearance_cm:.1f}"
            if result.primary_clearance_cm is not None
            else clearance_repr
        )
        secondary_clearance_repr: str = (
            f"{result.secondary_clearance_cm:.1f}"
            if result.secondary_clearance_cm is not None
            else "n/a"
        )
        measurement_summary: DistanceMeasurementSummary | None = result.measurement_summary
        secondary_target_repr: str = (
            f"{result.secondary_target_angle_deg:.1f}"
            if result.secondary_target_angle_deg is not None
            else "n/a"
        )
        measurement_reason: str = (
            measurement_summary.reliability_reason
            if measurement_summary is not None
            else "n/a"
        )
        logger.info(
            "Обход: side scan side=%s clearance_cm=%s threshold=%.1f exploratory_threshold=%.1f "
            "raw_exploratory_threshold=%.1f "
            "target_angle_deg=%.1f turned_angle_deg=%.1f primary_angle_deg=%.1f "
            "primary_turned_angle_deg=%.1f primary_clearance_cm=%s secondary_angle_deg=%s "
            "secondary_turned_angle_deg=%s secondary_clearance_cm=%s scan_completed=%s "
            "partial_scan=%s scan_useful=%s limited_confidence=%s heading_restored=%s "
            "turn_stop_reason=%s measurement_samples=%s measurement_spread_cm=%s "
            "measurement_reliable=%s measurement_reason=%s "
            "selection_blocked=%s selection_reason=%s "
            "useful_angle_min_deg=%.1f confident_angle_min_deg=%.1f",
            result.side.value,
            clearance_repr,
            self.avoidance_min_side_clearance_cm,
            self._exploratory_side_clearance_threshold(),
            self._raw_exploratory_side_clearance_threshold(),
            result.target_angle_deg,
            result.turned_angle_deg,
            result.primary_target_angle_deg or result.target_angle_deg,
            result.primary_turned_angle_deg or result.turned_angle_deg,
            primary_clearance_repr,
            secondary_target_repr,
            (
                f"{result.secondary_turned_angle_deg:.1f}"
                if result.secondary_turned_angle_deg is not None
                else "n/a"
            ),
            secondary_clearance_repr,
            result.scan_completed,
            result.used_partial_scan,
            result.scan_useful,
            result.limited_confidence,
            result.heading_restored,
            result.turn_stop_reason,
            (
                list(measurement_summary.raw_samples_cm)
                if measurement_summary is not None
                else "n/a"
            ),
            (
                f"{measurement_summary.spread_cm:.1f}"
                if measurement_summary is not None
                else "n/a"
            ),
            measurement_summary.reliable if measurement_summary is not None else True,
            measurement_reason,
            result.selection_blocked,
            result.selection_reason or "n/a",
            self._side_scan_useful_angle_threshold(result.target_angle_deg),
            self._side_scan_effective_angle_threshold(result.target_angle_deg),
        )
        if result.rejection_reason is not None:
            logger.warning(
                "Обход: сторона %s отклонена. reason=%s clearance_cm=%s primary_clearance_cm=%s "
                "secondary_clearance_cm=%s threshold=%.1f exploratory_threshold=%.1f "
                "raw_exploratory_threshold=%.1f heading_restored=%s "
                "turn_stop_reason=%s measurement_reliable=%s measurement_reason=%s "
                "selection_reason=%s",
                result.side.value,
                result.rejection_reason,
                clearance_repr,
                primary_clearance_repr,
                secondary_clearance_repr,
                self.avoidance_min_side_clearance_cm,
                self._exploratory_side_clearance_threshold(),
                self._raw_exploratory_side_clearance_threshold(),
                result.heading_restored,
                result.turn_stop_reason,
                measurement_summary.reliable if measurement_summary is not None else True,
                measurement_reason,
                result.selection_reason or "n/a",
            )

    def _log_side_selection_assessment(
        self,
        *,
        result: SideScanResult,
        preferred_threshold_cm: float,
        exploratory_threshold_cm: float,
    ) -> None:
        """Логирует, как именно side scan был интерпретирован на этапе выбора стороны."""
        selection_status, reason = self._side_selection_assessment(
            result=result,
            preferred_threshold_cm=preferred_threshold_cm,
            exploratory_threshold_cm=exploratory_threshold_cm,
        )
        clearance_repr: str = (
            f"{result.clearance_cm:.1f}" if result.clearance_cm is not None else "n/a"
        )
        logger.info(
            "Обход: оценка стороны side=%s status=%s clearance_cm=%s "
            "preferred_threshold=%.1f exploratory_threshold=%.1f "
            "raw_exploratory_threshold=%.1f reason=%s",
            result.side.value,
            selection_status,
            clearance_repr,
            preferred_threshold_cm,
            exploratory_threshold_cm,
            self._raw_exploratory_side_clearance_threshold(),
            reason,
        )

    def _side_selection_assessment(
        self,
        *,
        result: SideScanResult,
        preferred_threshold_cm: float,
        exploratory_threshold_cm: float,
    ) -> tuple[str, str]:
        """Классифицирует результат side scan для диагностики выбора стороны."""
        if not result.is_selectable:
            return (
                "rejected",
                result.selection_reason
                or result.rejection_reason
                or "scan unusable or heading not restored",
            )

        clearance_cm: float = result.clearance_cm or 0.0
        if clearance_cm >= preferred_threshold_cm:
            return (
                "preferred",
                f"clearance {clearance_cm:.1f} >= preferred threshold {preferred_threshold_cm:.1f}",
            )

        if clearance_cm >= exploratory_threshold_cm:
            return (
                "borderline",
                "clearance is below preferred threshold but above exploratory threshold",
            )

        if result.limited_confidence or result.turn_stop_reason != "target_reached":
            return (
                "inconclusive",
                "partial scan stayed below thresholds; measurement kept only as advisory signal",
            )

        return (
            "rejected",
            f"clearance {clearance_cm:.1f} < exploratory threshold {exploratory_threshold_cm:.1f}",
        )

    def _format_measurement_summary(
        self,
        summary: DistanceMeasurementSummary | None,
    ) -> str:
        """Коротко форматирует диагностическую сводку серии измерений."""
        if summary is None:
            return "n/a"
        return (
            f"samples={list(summary.raw_samples_cm)}, median={summary.median_cm:.1f}см, "
            f"range=[{summary.min_cm:.1f},{summary.max_cm:.1f}]см, "
            f"spread={summary.spread_cm:.1f}см, reliable={summary.reliable}, "
            f"reason={summary.reliability_reason}"
        )

    def _describe_side_selection_failure(
        self,
        *,
        scan_results: list[SideScanResult],
        exploratory_threshold_cm: float,
    ) -> str:
        """Сводная причина, почему выбор стороны не состоялся."""
        if all(not result.heading_restored for result in scan_results):
            return "restore heading failed after both side scans"

        if all(not result.scan_useful for result in scan_results):
            return "both side scans were rejected as not useful by actual turned angle"

        if all(
            result.selection_blocked and result.secondary_target_angle_deg is not None
            for result in scan_results
        ):
            return "both sides stayed blocked after primary and 60° fallback side scans"

        if all(
            result.is_selectable
            and not result.limited_confidence
            and (result.clearance_cm or 0.0) < exploratory_threshold_cm
            for result in scan_results
        ):
            return "both confident side scans measured clearance below exploratory threshold"

        return "no usable side scan produced a selectable avoidance side"

    def _stop_motion(self) -> None:
        """Остановить движение и дождаться фонового потока при необходимости."""
        self.lifecycle.stop_and_join(
            self.motor_controller.stop,
            timeout_sec=self.STOP_JOIN_TIMEOUT_SEC,
        )
