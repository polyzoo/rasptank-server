from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class BodyVelocityCommand:
    """Желаемая линейная и угловая скорость корпуса."""

    linear_speed_cm_per_sec: float
    angular_speed_deg_per_sec: float


@dataclass(frozen=True, slots=True)
class L1SensorSnapshot:
    """Снимок данных, полученных от нижнего уровня."""

    yaw_deg: float | None = None
    angular_speed_z_deg_per_sec: float | None = None
    longitudinal_acceleration_m_s2: float | None = None
    accel_x_m_s2: float | None = None
    accel_y_m_s2: float | None = None
    accel_z_m_s2: float | None = None
    distance_cm: float | None = None


@dataclass(frozen=True, slots=True)
class L2State:
    """Текущее состояние нового математического контура."""

    x_cm: float
    y_cm: float
    heading_deg: float
    linear_speed_cm_per_sec: float
    angular_speed_deg_per_sec: float
    left_percent: float
    right_percent: float
    distance_cm: float | None = None
