from __future__ import annotations

from pydantic import BaseModel, Field


class L2BodyVelocityRequestSchema(BaseModel):
    """Запрос желаемой линейной и угловой скорости корпуса."""

    linear_speed_cm_per_sec: float = Field(..., description="Желаемая линейная скорость корпуса.")
    angular_speed_deg_per_sec: float = Field(
        ...,
        description="Желаемая угловая скорость корпуса.",
    )


class L2ResetStateRequestSchema(BaseModel):
    """Запрос на сброс состояния уровня L2."""

    x_cm: float = 0.0
    y_cm: float = 0.0
    heading_deg: float = 0.0
    linear_speed_cm_per_sec: float = 0.0
    angular_speed_deg_per_sec: float = 0.0


class L2StateResponseSchema(BaseModel):
    """Ответ с текущим состоянием уровня L2."""

    x_cm: float
    y_cm: float
    heading_deg: float
    linear_speed_cm_per_sec: float
    angular_speed_deg_per_sec: float
    left_percent: float
    right_percent: float
    distance_cm: float | None = None


class StateSpaceConfigRequestSchema(BaseModel):
    """Настройка параметров МПС (LQR)."""

    enabled: bool = Field(True, description="Включить МПС.")
    t_v: float = Field(0.8, description="Постоянная времени T_v.")
    t_w: float = Field(0.55, description="Постоянная времени T_w.")
