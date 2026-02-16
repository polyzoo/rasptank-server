from pydantic import BaseModel, Field


class ForwardRequestSchema(BaseModel):
    """Схема запроса для задания движения машинке."""

    distance_cm: float = Field(..., ge=0, description="Целевое расстояние в сантиметрах")
    max_speed_percent: float | None = Field(
        default=None,
        ge=0,
        le=100,
        description="Процент максимальной скорости",
    )
