from typing import Optional

from pydantic import BaseModel, Field


class ForwardRequestSchema(BaseModel):
    """Схема запроса для задания движения машинки вперёд."""

    distance_cm: float = Field(
        ...,
        ge=0,
        description="Целевое расстояние в сантиметрах, которое машинка должна пройти вперёд.",
    )
    max_speed_percent: Optional[int] = Field(
        default=None,
        ge=0,
        le=100,
        description="Ограничение максимальной скорости в процентах (0–100).",
    )
