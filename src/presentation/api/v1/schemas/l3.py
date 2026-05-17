from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field


class TargetPointSchema(BaseModel):
    """API-схема целевой точки."""

    x_cm: float
    y_cm: float


class L3GoalRequestSchema(BaseModel):
    """API-запрос цели L3."""

    model_config = ConfigDict(extra="forbid")

    target: TargetPointSchema


class L3RouteRequestSchema(BaseModel):
    """API-запрос маршрута L3."""

    model_config = ConfigDict(extra="forbid")

    points: list[TargetPointSchema] = Field(..., min_length=1)


class L3StateResponseSchema(BaseModel):
    """API-ответ состояния L3."""

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
    desired_linear_speed_cm_per_sec: float | None = None
    desired_angular_speed_deg_per_sec: float | None = None
    detected_obstacle_x_cm: float | None = None
    detected_obstacle_y_cm: float | None = None
    detected_obstacle_radius_cm: float | None = None
    detected_obstacle_kind: str | None = None
