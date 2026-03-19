from __future__ import annotations

from typing import Annotated, Literal

from pydantic import BaseModel, Field


class ForwardSegmentSchema(BaseModel):
    """Сегмент движения вперед."""

    action: Literal["forward"] = "forward"
    distance_cm: float = Field(..., ge=0, description="Расстояние (см).")


class BackwardSegmentSchema(BaseModel):
    """Сегмент движения назад."""

    action: Literal["backward"] = "backward"
    distance_cm: float = Field(..., ge=0, description="Расстояние (см).")


class TurnLeftSegmentSchema(BaseModel):
    """Сегмент поворота налево."""

    action: Literal["turn_left"] = "turn_left"
    angle_deg: float = Field(default=90.0, ge=0, description="Угол поворота (градусы).")


class TurnRightSegmentSchema(BaseModel):
    """Сегмент поворота направо."""

    action: Literal["turn_right"] = "turn_right"
    angle_deg: float = Field(default=90.0, ge=0, description="Угол поворота (градусы).")


_SegmentSchema = Annotated[
    ForwardSegmentSchema | BackwardSegmentSchema | TurnLeftSegmentSchema | TurnRightSegmentSchema,
    Field(discriminator="action"),
]


class RouteRequestSchema(BaseModel):
    """Схема запроса для выполнения маршрута."""

    segments: list[_SegmentSchema] = Field(
        ...,
        min_length=1,
        description="Сегменты маршрута в порядке выполнения.",
    )
