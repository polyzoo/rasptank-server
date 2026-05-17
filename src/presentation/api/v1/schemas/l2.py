from __future__ import annotations

from typing import TypeAlias

from pydantic import BaseModel, Field

StateSpaceGainMatrix: TypeAlias = tuple[
    tuple[float, float, float, float, float],
    tuple[float, float, float, float, float],
]


class L2BodyVelocityRequestSchema(BaseModel):
    """API-запрос скорости корпуса для L2."""

    linear_speed_cm_per_sec: float = Field(..., description="Желаемая линейная скорость корпуса.")
    nominal_linear_speed_cm_per_sec: float | None = Field(
        None,
        description="Номинальная скорость v0 для линеаризации МПС.",
    )
    angular_speed_deg_per_sec: float = Field(
        ...,
        description="Желаемая угловая скорость корпуса.",
    )
    target_x_cm: float | None = Field(None, description="Целевая координата X для МПС.")
    target_y_cm: float | None = Field(None, description="Целевая координата Y для МПС.")


class L2ResetStateRequestSchema(BaseModel):
    """API-запрос сброса состояния L2."""

    x_cm: float = 0.0
    y_cm: float = 0.0
    heading_deg: float = 0.0
    linear_speed_cm_per_sec: float = 0.0
    angular_speed_deg_per_sec: float = 0.0


class L2StateResponseSchema(BaseModel):
    """API-ответ состояния L2."""

    x_cm: float
    y_cm: float
    heading_deg: float
    linear_speed_cm_per_sec: float
    angular_speed_deg_per_sec: float
    left_percent: float
    right_percent: float
    distance_cm: float | None = None
    state_space_gain_k: StateSpaceGainMatrix | None = None
    state_space_real_x: tuple[float, float, float, float, float] | None = None
    state_space_desired_x: tuple[float, float, float, float, float] | None = None
    state_space_error_x: tuple[float, float, float, float, float] | None = None
    state_space_target_ab: tuple[float, float] | None = None
    state_space_control_u: tuple[float, float] | None = None


class StateSpaceConfigRequestSchema(BaseModel):
    """API-запрос настройки МПС L2."""

    enabled: bool = Field(True, description="Включить МПС.")
    t_v: float = Field(0.8, description="Постоянная времени T_v.")
    t_w: float = Field(0.55, description="Постоянная времени T_w.")
