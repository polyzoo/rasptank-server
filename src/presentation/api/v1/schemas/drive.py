from typing import Optional

from pydantic import BaseModel, Field


class ForwardRequestSchema(BaseModel):
    """Схема запроса для задания движения вперед."""

    distance_cm: float = Field(
        ...,
        ge=0,
        description="Целевое расстояние в сантиметрах.",
    )
    max_speed_percent: Optional[int] = Field(
        default=None,
        ge=0,
        le=100,
        description="Ограничение скорости (0 – 100). При отсутствии значения — базовая скорость.",
    )
