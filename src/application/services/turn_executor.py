from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from src.application.protocols import (
    GyroscopeProtocol,
    MotorControllerProtocol,
    UltrasonicSensorProtocol,
)
from src.application.services.motion_lifecycle import MotionLifecycle

logger: logging.Logger = logging.getLogger(__name__)


@dataclass(slots=True)
class TurnExecutionResult:
    """Результат относительного поворота."""

    completed: bool
    angle_deg: float
    stop_reason: str = "unknown"


class TurnExecutor:
    """Исполнитель поворотов на месте с поправкой угла и замедлением."""

    TURN_EFFECTIVE_ANGLE_MIN_DEG: float = 1.0
    TURN_EFFECTIVE_ANGLE_MAX_DEG: float = 179.0
    TURN_SLOWDOWN_ZONE_MIN_DEG: float = 5.0
    TURN_CREEP_SPEED_MIN_PERCENT: int = 15
    TURN_SLOWDOWN_ZONE_RATIO: float = 0.40

    def __init__(
        self,
        motor_controller: MotorControllerProtocol,
        ultrasonic_sensor: UltrasonicSensorProtocol,
        gyroscope: GyroscopeProtocol,
        *,
        min_obstacle_distance_cm: float,
        turn_speed_percent: int,
        turn_slowdown_remaining_deg: float,
        turn_creep_speed_percent: int,
        turn_angle_trim_deg: float,
        last_turn_angle_trim_deg: float,
        turn_check_interval_sec: float,
        turn_obstacle_check_interval_sec: float,
        turn_timeout_per_deg: float,
        turn_timeout_min: float,
        lifecycle: MotionLifecycle,
    ) -> None:
        """Инициализация исполнителя поворотов."""
        self.motor_controller: MotorControllerProtocol = motor_controller
        self.ultrasonic_sensor: UltrasonicSensorProtocol = ultrasonic_sensor
        self.gyroscope: GyroscopeProtocol = gyroscope
        self.min_obstacle_distance_cm: float = min_obstacle_distance_cm
        self.turn_speed_percent: int = turn_speed_percent
        self.turn_slowdown_remaining_deg: float = turn_slowdown_remaining_deg
        self.turn_creep_speed_percent: int = turn_creep_speed_percent
        self.turn_angle_trim_deg: float = turn_angle_trim_deg
        self.last_turn_angle_trim_deg: float = last_turn_angle_trim_deg
        self.turn_check_interval_sec: float = turn_check_interval_sec
        self.turn_obstacle_check_interval_sec: float = turn_obstacle_check_interval_sec
        self.turn_timeout_per_deg: float = turn_timeout_per_deg
        self.turn_timeout_min: float = turn_timeout_min
        self.lifecycle: MotionLifecycle = lifecycle

    def run(
        self,
        requested_angle_deg: float,
        *,
        turn_left: bool,
        segment_index: int,
        total_segments: int,
    ) -> None:
        """Выполнить поворот маршрута с учетом поправки угла и последнего сегмента."""
        target_angle: float = self._route_turn_target_angle(
            requested_angle_deg=requested_angle_deg,
            segment_index=segment_index,
            total_segments=total_segments,
        )
        timeout_sec: float = max(self.turn_timeout_min, target_angle * self.turn_timeout_per_deg)
        self._execute_turn(
            turn_left=turn_left,
            target_angle=target_angle,
            timeout_sec=timeout_sec,
            stop_on_front_obstacle=True,
        )

    def run_relative(
        self,
        target_angle_deg: float,
        *,
        turn_left: bool,
        stop_on_front_obstacle: bool,
    ) -> TurnExecutionResult:
        """Выполнить поворот без route-specific поправок и вернуть результат."""
        target_angle: float = max(0.0, min(self.TURN_EFFECTIVE_ANGLE_MAX_DEG, target_angle_deg))
        if target_angle <= 0.0:
            return TurnExecutionResult(completed=True, angle_deg=0.0)

        timeout_sec: float = max(self.turn_timeout_min, target_angle * self.turn_timeout_per_deg)
        return self._execute_turn(
            turn_left=turn_left,
            target_angle=target_angle,
            timeout_sec=timeout_sec,
            stop_on_front_obstacle=stop_on_front_obstacle,
        )

    def _execute_turn(
        self,
        *,
        turn_left: bool,
        target_angle: float,
        timeout_sec: float,
        stop_on_front_obstacle: bool,
    ) -> TurnExecutionResult:
        """Выполнить поворот до цели, таймаута или срабатывания safety-check."""
        self.gyroscope.reset_yaw()

        start_time: float = time.monotonic()
        last_obstacle_check_t: float = start_time - self.turn_obstacle_check_interval_sec
        stop_reason: str = "movement_stopped"
        try:
            while self.lifecycle.should_keep_running():
                current_yaw: float = self.gyroscope.get_yaw()
                if abs(current_yaw) >= target_angle:
                    stop_reason = "target_reached"
                    break

                if (time.monotonic() - start_time) >= timeout_sec:
                    stop_reason = "timeout"
                    break

                now: float = time.monotonic()
                if stop_on_front_obstacle and (now - last_obstacle_check_t) >= self.turn_obstacle_check_interval_sec:
                    if self.ultrasonic_sensor.measure_distance_cm() <= self.min_obstacle_distance_cm:
                        stop_reason = "front_obstacle"
                        break
                    last_obstacle_check_t = now

                sp: int = self._turn_motor_speed_percent(target_angle, abs(current_yaw))
                if turn_left:
                    self.motor_controller.turn_left(speed_percent=sp)
                else:
                    self.motor_controller.turn_right(speed_percent=sp)

                time.sleep(self.turn_check_interval_sec)

            turned_angle: float = abs(self.gyroscope.get_yaw())
            return TurnExecutionResult(
                completed=turned_angle >= target_angle,
                angle_deg=turned_angle,
                stop_reason=stop_reason,
            )
        finally:
            self.motor_controller.stop()

    def _route_turn_target_angle(
        self,
        requested_angle_deg: float,
        *,
        segment_index: int,
        total_segments: int,
    ) -> float:
        """Посчитать эффективный угол поворота с учетом поправки угла."""
        idx_segment: int = total_segments - 1
        last_extra: float = self.last_turn_angle_trim_deg if segment_index == idx_segment else 0.0
        effective_angle: float = requested_angle_deg + self.turn_angle_trim_deg + last_extra
        return max(
            self.TURN_EFFECTIVE_ANGLE_MIN_DEG,
            min(self.TURN_EFFECTIVE_ANGLE_MAX_DEG, effective_angle),
        )

    def _turn_slowdown_zone_deg(self, target_angle: float) -> float:
        """Определить, с какого остатка угла переходить на ползучую скорость."""
        if self.turn_slowdown_remaining_deg <= 0.0:
            return 0.0

        return min(
            self.turn_slowdown_remaining_deg,
            max(self.TURN_SLOWDOWN_ZONE_MIN_DEG, target_angle * self.TURN_SLOWDOWN_ZONE_RATIO),
        )

    def _turn_motor_speed_percent(self, target_angle: float, current_yaw_abs: float) -> int:
        """Посчитать скорость моторов для текущего остатка угла."""
        zone: float = self._turn_slowdown_zone_deg(target_angle)

        if zone <= 0.0 or (target_angle - current_yaw_abs) > zone:
            return self.turn_speed_percent

        return max(
            self.TURN_CREEP_SPEED_MIN_PERCENT,
            min(self.turn_speed_percent, self.turn_creep_speed_percent),
        )
