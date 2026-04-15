from __future__ import annotations

import math
from dataclasses import dataclass
from typing import ClassVar


@dataclass(frozen=True, slots=True)
class PoseEstimate:
    """Оценка текущего состояния машинки на плоскости."""

    x_cm: float
    y_cm: float
    heading_deg: float
    linear_speed_cm_per_sec: float
    angular_speed_deg_per_sec: float


class PoseEstimator:
    """Оценка состояния машинки по скорости, угловой скорости и ускорению."""

    # Среднее между углом в начале и в конце шага используется как направление движения за этот шаг.
    AVERAGE_HEADING_DIVISOR: ClassVar[float] = 2.0

    # Перевод из метров в сантиметры для перехода от м/с^2 к см/с^2.
    CENTIMETERS_IN_METER: ClassVar[float] = 100.0

    # Половина полного оборота для нормализации угла в симметричный диапазон.
    HALF_TURN_DEG: ClassVar[float] = 180.0

    # Полный оборот в градусах для циклической нормализации угла.
    FULL_TURN_DEG: ClassVar[float] = 360.0

    def __init__(self) -> None:
        """Подготовить начальное нулевое состояние."""
        self._x_cm: float = 0.0
        self._y_cm: float = 0.0
        self._heading_deg: float = 0.0
        self._linear_speed_cm_per_sec: float = 0.0
        self._angular_speed_deg_per_sec: float = 0.0

    def reset(
        self,
        *,
        x_cm: float = 0.0,
        y_cm: float = 0.0,
        heading_deg: float = 0.0,
        linear_speed_cm_per_sec: float = 0.0,
        angular_speed_deg_per_sec: float = 0.0,
    ) -> None:
        """Сбросить состояние в заданную точку."""
        self._x_cm: float = x_cm
        self._y_cm: float = y_cm
        self._heading_deg: float = self._normalize_angle_deg(heading_deg)
        self._linear_speed_cm_per_sec: float = linear_speed_cm_per_sec
        self._angular_speed_deg_per_sec: float = angular_speed_deg_per_sec

    def snapshot(self) -> PoseEstimate:
        """Вернуть текущую оценку состояния."""
        return PoseEstimate(
            x_cm=self._x_cm,
            y_cm=self._y_cm,
            heading_deg=self._heading_deg,
            linear_speed_cm_per_sec=self._linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=self._angular_speed_deg_per_sec,
        )

    def correct_heading(self, heading_deg: float) -> PoseEstimate:
        """Исправить угол по внешнему измерению без изменения координат."""
        self._heading_deg: float = self._normalize_angle_deg(heading_deg)
        return self.snapshot()

    def update_from_velocity(
        self,
        *,
        linear_speed_cm_per_sec: float,
        angular_speed_deg_per_sec: float,
        dt_sec: float,
    ) -> PoseEstimate:
        """Обновить положение по линейной и угловой скорости."""
        if dt_sec <= 0.0:
            return self.snapshot()

        previous_heading_deg: float = self._heading_deg

        new_heading_deg: float = self._normalize_angle_deg(
            previous_heading_deg + angular_speed_deg_per_sec * dt_sec
        )

        average_heading_rad: float = math.radians(
            (previous_heading_deg + new_heading_deg) / self.AVERAGE_HEADING_DIVISOR
        )

        self._x_cm += linear_speed_cm_per_sec * math.cos(average_heading_rad) * dt_sec
        self._y_cm += linear_speed_cm_per_sec * math.sin(average_heading_rad) * dt_sec
        self._heading_deg: float = new_heading_deg
        self._linear_speed_cm_per_sec: float = linear_speed_cm_per_sec
        self._angular_speed_deg_per_sec: float = angular_speed_deg_per_sec

        return self.snapshot()

    def integrate_longitudinal_acceleration(
        self,
        *,
        longitudinal_acceleration_m_s2: float,
        dt_sec: float,
    ) -> float:
        """Обновить линейную скорость по продольному ускорению."""
        if dt_sec <= 0.0:
            return self._linear_speed_cm_per_sec

        self._linear_speed_cm_per_sec += (
            longitudinal_acceleration_m_s2 * self.CENTIMETERS_IN_METER * dt_sec
        )

        return self._linear_speed_cm_per_sec

    def update_from_imu(
        self,
        *,
        longitudinal_acceleration_m_s2: float,
        angular_speed_deg_per_sec: float,
        dt_sec: float,
    ) -> PoseEstimate:
        """Обновить скорость и положение только по данным IMU."""
        linear_speed_cm_per_sec: float = self.integrate_longitudinal_acceleration(
            longitudinal_acceleration_m_s2=longitudinal_acceleration_m_s2,
            dt_sec=dt_sec,
        )
        return self.update_from_velocity(
            linear_speed_cm_per_sec=linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=angular_speed_deg_per_sec,
            dt_sec=dt_sec,
        )

    def _normalize_angle_deg(self, angle_deg: float) -> float:
        """Привести угол к диапазону от -180 до 180 градусов."""
        return ((angle_deg + self.HALF_TURN_DEG) % self.FULL_TURN_DEG) - self.HALF_TURN_DEG
