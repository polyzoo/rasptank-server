from __future__ import annotations

from dataclasses import dataclass

StateVector5 = tuple[float, float, float, float, float]


@dataclass(frozen=True, slots=True)
class BodyVelocityCommand:
    """Желаемая линейная и угловая скорость корпуса."""

    linear_speed_cm_per_sec: float
    angular_speed_deg_per_sec: float
    nominal_linear_speed_cm_per_sec: float | None = None
    target_x_cm: float | None = None
    target_y_cm: float | None = None


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
    feedback_delta_u: float | None = None
    feedback_heading_ref_deg: float | None = None
    state_space_gain_k: (
        tuple[tuple[float, float, float, float, float], tuple[float, float, float, float, float]]
        | None
    ) = None
    state_space_real_x: StateVector5 | None = None
    state_space_desired_x: StateVector5 | None = None
    state_space_error_x: StateVector5 | None = None
    state_space_target_ab: tuple[float, float] | None = None
    state_space_control_u: tuple[float, float] | None = None
