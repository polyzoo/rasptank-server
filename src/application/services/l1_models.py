from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class L1TrackCommand:
    """Команда для левого и правого борта."""

    left_percent: int
    right_percent: int


@dataclass(frozen=True, slots=True)
class L1SensorState:
    """Измеренные данные нижнего уровня без вычисления положения."""

    angular_speed_z_deg_per_sec: float
    accel_x_m_s2: float
    accel_y_m_s2: float
    accel_z_m_s2: float
    distance_cm: float
