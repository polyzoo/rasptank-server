from __future__ import annotations

import math
from dataclasses import dataclass
from typing import ClassVar

from src.application.services.l2_models import L2State
from src.application.services.l3_models import (
    L3_STATUS_BLOCKED,
    L3_STATUS_REACHED,
    L3_STATUS_TRACKING,
    GoalTrackingCommand,
    L3Command,
    TargetPoint,
)


@dataclass(frozen=True, slots=True)
class GoalPointController:
    """Рассчитать желаемое движение корпуса к выбранной точке."""

    # Половина полного оборота для нормализации угла в симметричный диапазон.
    HALF_TURN_DEG: ClassVar[float] = 180.0

    # Полный оборот в градусах для циклической нормализации угла.
    FULL_TURN_DEG: ClassVar[float] = 360.0

    # Коэффициент без замедления: препятствие ещё не влияет на скорость.
    FULL_SPEED_RATIO: ClassVar[float] = 1.0

    # Коэффициент полной остановки: скорость полностью подавляется.
    ZERO_SPEED_RATIO: ClassVar[float] = 0.0

    position_tolerance_cm: float
    linear_speed_gain: float
    angular_speed_gain: float
    max_linear_speed_cm_per_sec: float
    max_angular_speed_deg_per_sec: float
    obstacle_stop_distance_cm: float
    obstacle_slowdown_distance_cm: float

    def build_command(self, state: L2State, target: TargetPoint) -> GoalTrackingCommand:
        """Построить команду движения к точке по текущему состоянию корпуса."""
        delta_x_cm: float = target.x_cm - state.x_cm
        delta_y_cm: float = target.y_cm - state.y_cm
        distance_error_cm: float = math.hypot(delta_x_cm, delta_y_cm)
        target_heading_deg: float = math.degrees(math.atan2(delta_y_cm, delta_x_cm))
        heading_error_deg: float = self._normalize_angle_deg(target_heading_deg - state.heading_deg)

        if state.distance_cm is not None and state.distance_cm <= self.obstacle_stop_distance_cm:
            return GoalTrackingCommand(
                command=L3Command(linear_speed_cm_per_sec=0.0, angular_speed_deg_per_sec=0.0),
                distance_error_cm=distance_error_cm,
                heading_error_deg=heading_error_deg,
                target_heading_deg=target_heading_deg,
                status=L3_STATUS_BLOCKED,
            )

        if distance_error_cm <= self.position_tolerance_cm:
            return GoalTrackingCommand(
                command=L3Command(linear_speed_cm_per_sec=0.0, angular_speed_deg_per_sec=0.0),
                distance_error_cm=distance_error_cm,
                heading_error_deg=heading_error_deg,
                target_heading_deg=target_heading_deg,
                status=L3_STATUS_REACHED,
            )

        linear_speed_cm_per_sec: float = min(
            self.max_linear_speed_cm_per_sec,
            distance_error_cm * self.linear_speed_gain,
        )

        heading_slowdown_ratio: float = max(0.0, math.cos(math.radians(heading_error_deg)))
        linear_speed_cm_per_sec *= heading_slowdown_ratio
        linear_speed_cm_per_sec *= self._obstacle_speed_ratio(state.distance_cm)

        angular_speed_deg_per_sec: float = self._clamp(
            heading_error_deg * self.angular_speed_gain,
            -self.max_angular_speed_deg_per_sec,
            self.max_angular_speed_deg_per_sec,
        )

        return GoalTrackingCommand(
            command=L3Command(
                linear_speed_cm_per_sec=linear_speed_cm_per_sec,
                angular_speed_deg_per_sec=angular_speed_deg_per_sec,
            ),
            distance_error_cm=distance_error_cm,
            heading_error_deg=heading_error_deg,
            target_heading_deg=target_heading_deg,
            status=L3_STATUS_TRACKING,
        )

    def _normalize_angle_deg(self, angle_deg: float) -> float:
        """Привести угол к диапазону от -180 до 180 градусов."""
        return ((angle_deg + self.HALF_TURN_DEG) % self.FULL_TURN_DEG) - self.HALF_TURN_DEG

    def _clamp(self, value: float, lower_bound: float, upper_bound: float) -> float:
        """Ограничить значение допустимым диапазоном."""
        return max(lower_bound, min(upper_bound, value))

    def _obstacle_speed_ratio(self, distance_cm: float | None) -> float:
        """Вернуть коэффициент плавного торможения перед препятствием."""
        if distance_cm is None:
            return self.FULL_SPEED_RATIO

        if distance_cm <= self.obstacle_stop_distance_cm:
            return self.ZERO_SPEED_RATIO

        slowdown_range_cm: float = (
            self.obstacle_slowdown_distance_cm - self.obstacle_stop_distance_cm
        )
        if slowdown_range_cm <= 0.0:
            return self.ZERO_SPEED_RATIO

        if distance_cm >= self.obstacle_slowdown_distance_cm:
            return self.FULL_SPEED_RATIO

        return self._clamp(
            (distance_cm - self.obstacle_stop_distance_cm) / slowdown_range_cm,
            self.ZERO_SPEED_RATIO,
            self.FULL_SPEED_RATIO,
        )
