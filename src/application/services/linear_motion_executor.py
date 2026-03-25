from __future__ import annotations

import logging
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from src.application.protocols import (
    GyroscopeProtocol,
    MotorControllerProtocol,
    UltrasonicSensorProtocol,
)
from src.application.services.motion_lifecycle import MotionLifecycle

logger: logging.Logger = logging.getLogger(__name__)

ObstacleDistanceProvider = Callable[[], float]


@dataclass(slots=True)
class LinearMotionExecutionResult:
    """Расширенный результат линейного движения."""

    completed: bool
    traveled_cm: float
    blocked: bool = False


class MoveCommand(Protocol):
    """Команда движения с подруливанием."""

    def __call__(self, *, speed_percent: int, steer_percent: int = 0) -> None: ...


class LinearMotionExecutor:
    """Исполнитель прямолинейного движения и удержания курса."""

    SPEED_PERCENT_MIN: int = 0
    SPEED_PERCENT_MAX: int = 100
    SPEED_FACTOR_MIN: float = 0.1
    RAMP_MAX_RATIO: float = 1.0
    ESTIMATE_SPEED_DIVISOR: float = 100.0
    FORWARD_COAST_BEFORE_STOP_SEC: float = 0.08
    TURN_ANGLE_FULL_ROTATION_DEG: float = 360.0
    TURN_ANGLE_HALF_ROTATION_DEG: float = 180.0

    def __init__(
        self,
        motor_controller: MotorControllerProtocol,
        ultrasonic_sensor: UltrasonicSensorProtocol,
        gyroscope: GyroscopeProtocol,
        *,
        min_obstacle_distance_cm: float,
        deceleration_distance_cm: float,
        max_speed_cm_per_sec: float,
        update_interval_sec: float,
        forward_soft_start_sec: float,
        heading_hold_enabled: bool,
        heading_hold_kp: float,
        heading_hold_steer_max: int,
        heading_hold_deadband_deg: float,
        heading_hold_steer_speed_ratio: float,
        heading_hold_min_speed_percent: float,
        heading_hold_steer_cap_min_spd_percent: float,
        heading_hold_steer_trim: int,
        heading_hold_invert_steer: bool,
        lifecycle: MotionLifecycle,
    ) -> None:
        """Инициализация исполнителя прямолинейного движения и удержания курса."""
        self.motor_controller: MotorControllerProtocol = motor_controller
        self.ultrasonic_sensor: UltrasonicSensorProtocol = ultrasonic_sensor
        self.gyroscope: GyroscopeProtocol = gyroscope
        self.min_obstacle_distance_cm: float = min_obstacle_distance_cm
        self.deceleration_distance_cm: float = deceleration_distance_cm
        self.max_speed_cm_per_sec: float = max_speed_cm_per_sec
        self.update_interval_sec: float = update_interval_sec
        self.forward_soft_start_sec: float = forward_soft_start_sec
        self.heading_hold_enabled: bool = heading_hold_enabled
        self.heading_hold_kp: float = heading_hold_kp
        self.heading_hold_steer_max: int = heading_hold_steer_max
        self.heading_hold_deadband_deg: float = heading_hold_deadband_deg
        self.heading_hold_steer_speed_ratio: float = heading_hold_steer_speed_ratio
        self.heading_hold_min_speed_percent: float = heading_hold_min_speed_percent
        self.heading_hold_steer_cap_min_spd_percent: float = heading_hold_steer_cap_min_spd_percent
        self.heading_hold_steer_trim: int = heading_hold_steer_trim
        self.heading_hold_invert_steer: bool = heading_hold_invert_steer
        self.lifecycle: MotionLifecycle = lifecycle

    def run(
        self,
        distance_cm: float,
        speed_percent: int,
        move_fn: MoveCommand,
        *,
        obstacle_aware: bool = True,
        obstacle_distance_provider: ObstacleDistanceProvider | None = None,
    ) -> bool:
        """Пройти заданную дистанцию с учетом препятствий и коррекции курса."""
        return self.run_with_result(
            distance_cm=distance_cm,
            speed_percent=speed_percent,
            move_fn=move_fn,
            obstacle_aware=obstacle_aware,
            obstacle_distance_provider=obstacle_distance_provider,
        ).completed

    def run_with_result(
        self,
        distance_cm: float,
        speed_percent: int,
        move_fn: MoveCommand,
        *,
        obstacle_aware: bool = True,
        obstacle_distance_provider: ObstacleDistanceProvider | None = None,
    ) -> LinearMotionExecutionResult:
        """Пройти заданную дистанцию и вернуть расширенный результат исполнения."""
        if distance_cm <= 0.0:
            self.motor_controller.stop()
            return LinearMotionExecutionResult(completed=True, traveled_cm=0.0)

        traveled_cm: float = 0.0
        last_effective_speed: float = 0.0
        last_cmd_speed: float = 0.0

        last_time: float = time.monotonic()
        heading_setpoint_deg: float = self.gyroscope.get_yaw()
        segment_start_t: float = time.monotonic()
        clamped_speed: int = max(self.SPEED_PERCENT_MIN, min(self.SPEED_PERCENT_MAX, speed_percent))
        distance_provider: ObstacleDistanceProvider = (
            self.ultrasonic_sensor.measure_distance_cm
            if obstacle_distance_provider is None
            else obstacle_distance_provider
        )

        try:
            while self.lifecycle.should_keep_running():
                obstacle_cm: float | None = distance_provider() if obstacle_aware else None
                now: float = time.monotonic()
                dt: float = now - last_time

                last_time = now
                traveled_cm += self._estimate_traveled_distance(last_effective_speed, dt)

                remaining_dist: float = max(0.0, distance_cm - traveled_cm)
                if remaining_dist <= 0.0:
                    self._coast_before_stop(
                        move_fn=move_fn,
                        heading_setpoint_deg=heading_setpoint_deg,
                        last_cmd_speed=last_cmd_speed,
                    )
                    self.motor_controller.stop()
                    return LinearMotionExecutionResult(completed=True, traveled_cm=distance_cm)

                if obstacle_aware and obstacle_cm is not None:
                    current_speed: float = self._calculate_obstacle_aware_speed(
                        obstacle_cm=obstacle_cm,
                        remaining_cm=remaining_dist,
                        max_speed=clamped_speed,
                    )
                    if current_speed <= 0.0:
                        self.motor_controller.stop()
                        time.sleep(self.update_interval_sec)
                        return LinearMotionExecutionResult(
                            completed=False,
                            traveled_cm=min(traveled_cm, distance_cm),
                            blocked=True,
                        )
                else:
                    current_speed = clamped_speed * self._calculate_distance_speed_factor(remaining_dist)

                elapsed_seg: float = time.monotonic() - segment_start_t
                ramp: float = (
                    min(self.RAMP_MAX_RATIO, elapsed_seg / self.forward_soft_start_sec)
                    if self.forward_soft_start_sec > 0.0
                    else self.RAMP_MAX_RATIO
                )

                effective_speed: float = current_speed * ramp
                steer: int = self._heading_steer_percent(heading_setpoint_deg, effective_speed)
                last_effective_speed = effective_speed
                last_cmd_speed = effective_speed
                move_fn(speed_percent=int(effective_speed), steer_percent=steer)
                time.sleep(self.update_interval_sec)

            return LinearMotionExecutionResult(
                completed=False,
                traveled_cm=min(traveled_cm, distance_cm),
                blocked=False,
            )

        except (OSError, RuntimeError, ConnectionError, ValueError):
            logger.exception("Ошибка в линейном сегменте движения")
            return LinearMotionExecutionResult(
                completed=False,
                traveled_cm=min(traveled_cm, distance_cm),
                blocked=False,
            )

    def _calculate_obstacle_aware_speed(
        self,
        obstacle_cm: float,
        remaining_cm: float,
        max_speed: int,
    ) -> float:
        """Посчитать допустимую скорость по препятствию и остаточной дистанции."""
        if obstacle_cm <= self.min_obstacle_distance_cm:
            return 0.0

        speed: float = float(max_speed)
        if obstacle_cm <= self.deceleration_distance_cm:
            decel_range: float = self.deceleration_distance_cm - self.min_obstacle_distance_cm
            if decel_range <= 0.0:
                return 0.0

            dist_above_min: float = obstacle_cm - self.min_obstacle_distance_cm
            speed *= max(0.0, dist_above_min / decel_range)

        speed *= self._calculate_distance_speed_factor(remaining_cm)
        return max(0.0, speed)

    def _calculate_distance_speed_factor(self, remaining_cm: float) -> float:
        """Коэффициент замедления при приближении к целевой точке."""
        if self.deceleration_distance_cm <= 0.0:
            return self.RAMP_MAX_RATIO

        if remaining_cm >= self.deceleration_distance_cm:
            return self.RAMP_MAX_RATIO

        return max(self.SPEED_FACTOR_MIN, remaining_cm / self.deceleration_distance_cm)

    def _coast_before_stop(
        self,
        *,
        move_fn: MoveCommand,
        heading_setpoint_deg: float,
        last_cmd_speed: float,
    ) -> None:
        """Короткий выбег перед стопом для более плавного завершения сегмента."""
        coast_sp: int = max(
            self.SPEED_PERCENT_MIN,
            min(self.SPEED_PERCENT_MAX, int(last_cmd_speed)),
        )
        if coast_sp <= 0:
            return

        coast_steer: int = self._heading_steer_percent(
            heading_setpoint_deg=heading_setpoint_deg,
            base_speed_percent=float(coast_sp),
        )
        move_fn(speed_percent=coast_sp, steer_percent=coast_steer)
        time.sleep(self.FORWARD_COAST_BEFORE_STOP_SEC)

    def _estimate_traveled_distance(self, speed_percent: float, time_interval: float) -> float:
        """Оценить пройденную дистанцию по скорости и времени."""
        speed_ratio: float = speed_percent / self.ESTIMATE_SPEED_DIVISOR
        distance_per_second: float = self.max_speed_cm_per_sec * speed_ratio
        traveled_distance: float = distance_per_second * time_interval
        return traveled_distance

    def _heading_steer_percent(self, heading_setpoint_deg: float, base_speed_percent: float) -> int:
        """Посчитать подруливание для удержания курса относительно заданного угла."""
        if not self.heading_hold_enabled:
            return 0

        base: float = max(0.0, float(base_speed_percent))
        if base < self.heading_hold_min_speed_percent:
            return 0

        cap_base: float = (
            max(base, self.heading_hold_steer_cap_min_spd_percent)
            if self.heading_hold_steer_cap_min_spd_percent > 0.0
            else base
        )

        steer_cap: int = min(
            self.heading_hold_steer_max,
            int(cap_base * self.heading_hold_steer_speed_ratio),
        )
        if steer_cap <= 0:
            return 0

        err: float = self._angle_error_deg(heading_setpoint_deg, self.gyroscope.get_yaw())
        steer_pid: int = (
            int(round(self.heading_hold_kp * err))
            if abs(err) >= self.heading_hold_deadband_deg
            else 0
        )

        steer_i: int = max(-steer_cap, min(steer_cap, steer_pid + self.heading_hold_steer_trim))
        return -steer_i if self.heading_hold_invert_steer else steer_i

    @staticmethod
    def _angle_error_deg(setpoint: float, current: float) -> float:
        """Нормализовать ошибку угла в диапазон."""
        diff: float = setpoint - current

        while diff > LinearMotionExecutor.TURN_ANGLE_HALF_ROTATION_DEG:
            diff -= LinearMotionExecutor.TURN_ANGLE_FULL_ROTATION_DEG

        while diff < -LinearMotionExecutor.TURN_ANGLE_HALF_ROTATION_DEG:
            diff += LinearMotionExecutor.TURN_ANGLE_FULL_ROTATION_DEG

        return diff
