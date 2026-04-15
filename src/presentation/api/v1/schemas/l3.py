from __future__ import annotations

from pydantic import BaseModel, Field


class TargetPointSchema(BaseModel):
    """Целевая точка на плоскости."""

    x_cm: float
    y_cm: float


class KnownObstacleSchema(BaseModel):
    """Заранее известное круглое препятствие."""

    x_cm: float
    y_cm: float
    radius_cm: float = Field(..., gt=0.0)


class L3GoalRequestSchema(BaseModel):
    """Запрос на движение к одной точке."""

    target: TargetPointSchema
    obstacles: list[KnownObstacleSchema] = Field(default_factory=list)


class L3RouteRequestSchema(BaseModel):
    """Запрос на движение по маршруту из нескольких точек."""

    points: list[TargetPointSchema] = Field(..., min_length=1)
    obstacles: list[KnownObstacleSchema] = Field(default_factory=list)


class L3StateResponseSchema(BaseModel):
    """Ответ с текущим состоянием уровня L3."""

    status: str
    mode: str
    planner_status: str
    target_x_cm: float | None
    target_y_cm: float | None
    active_point_index: int | None
    total_points: int
    distance_error_cm: float | None
    heading_error_deg: float | None
    target_heading_deg: float | None
    linear_speed_cm_per_sec: float
    angular_speed_deg_per_sec: float
    detected_obstacle_x_cm: float | None = None
    detected_obstacle_y_cm: float | None = None
    detected_obstacle_radius_cm: float | None = None
    detected_obstacle_kind: str | None = None
