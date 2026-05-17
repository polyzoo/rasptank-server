from __future__ import annotations

from pydantic import BaseModel, Field


class L1TrackCommandRequestSchema(BaseModel):
    """API-запрос команды бортов L1."""

    left_percent: int = Field(..., ge=-100, le=100, description="Команда левого борта в процентах.")
    right_percent: int = Field(
        ...,
        ge=-100,
        le=100,
        description="Команда правого борта в процентах.",
    )


class L1SensorStateResponseSchema(BaseModel):
    """API-ответ снимка датчиков L1."""

    angular_speed_z_deg_per_sec: float
    accel_x_m_s2: float
    accel_y_m_s2: float
    accel_z_m_s2: float
    distance_cm: float


class L1ActionResponseSchema(BaseModel):
    """API-ответ команды L1."""

    status: str
