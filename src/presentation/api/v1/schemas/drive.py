from typing import Literal, Union, List

from pydantic import BaseModel, Field


class ForwardSegmentSchema(BaseModel):
    """Сегмент движения вперед."""

    action: Literal["forward"] = "forward"
    distance_cm: float = Field(..., ge=0, description="Расстояние (см).")


class TurnLeftSegmentSchema(BaseModel):
    """Сегмент поворота налево."""

    action: Literal["turn_left"] = "turn_left"
    duration_sec: float = Field(..., ge=0, description="Длительность поворота (с).")


class TurnRightSegmentSchema(BaseModel):
    """Сегмент поворота направо."""

    action: Literal["turn_right"] = "turn_right"
    duration_sec: float = Field(..., ge=0, description="Длительность поворота (с).")


class RouteRequestSchema(BaseModel):
    """Схема запроса для выполнения маршрута."""

    segments: List[
        Union[
            ForwardSegmentSchema,
            TurnLeftSegmentSchema,
            TurnRightSegmentSchema,
        ],
    ] = Field(
        ...,
        min_length=1,
        description="Сегменты маршрута в порядке выполнения.",
    )
