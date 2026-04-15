from __future__ import annotations

from dataclasses import dataclass
from typing import Final

# Статусы верхнего уровня L3.
L3_STATUS_IDLE: Final[str] = "idle"
L3_STATUS_TRACKING: Final[str] = "tracking"
L3_STATUS_REACHED: Final[str] = "reached"
L3_STATUS_BLOCKED: Final[str] = "blocked"
L3_STATUS_UNREACHABLE: Final[str] = "unreachable"
L3_STATUS_CANCELLED: Final[str] = "cancelled"

# Режимы работы L3.
L3_MODE_IDLE: Final[str] = "idle"
L3_MODE_POINT: Final[str] = "point"
L3_MODE_ROUTE: Final[str] = "route"

# Статусы планировщика пути.
L3_PLANNER_STATUS_IDLE: Final[str] = "idle"
L3_PLANNER_STATUS_EMPTY: Final[str] = "empty"
L3_PLANNER_STATUS_PLANNED: Final[str] = "planned"
L3_PLANNER_STATUS_REPLANNED_DYNAMIC: Final[str] = "replanned_dynamic"
L3_PLANNER_STATUS_IMPOSSIBLE: Final[str] = "impossible"

# Типы препятствий, которые L3 отдаёт наружу.
L3_DETECTED_OBSTACLE_KIND_DYNAMIC: Final[str] = "dynamic"


@dataclass(frozen=True, slots=True)
class TargetPoint:
    """Целевая точка движения на плоскости."""

    x_cm: float
    y_cm: float


@dataclass(frozen=True, slots=True)
class TargetRoute:
    """Маршрут в виде последовательности целевых точек."""

    points: tuple[TargetPoint, ...]


@dataclass(frozen=True, slots=True)
class KnownObstacle:
    """Заранее известное препятствие для планировщика пути."""

    x_cm: float
    y_cm: float
    radius_cm: float


@dataclass(frozen=True, slots=True)
class PlannedRoute:
    """Результат планирования маршрута с учётом препятствий."""

    points: tuple[TargetPoint, ...]
    status: str
    blocking_obstacle_index: int | None = None


@dataclass(frozen=True, slots=True)
class L3Command:
    """Желаемая линейная и угловая скорость для уровня L2."""

    linear_speed_cm_per_sec: float
    angular_speed_deg_per_sec: float


@dataclass(frozen=True, slots=True)
class GoalTrackingCommand:
    """Результат расчёта движения к целевой точке."""

    command: L3Command
    distance_error_cm: float
    heading_error_deg: float
    target_heading_deg: float
    status: str


@dataclass(frozen=True, slots=True)
class L3State:
    """Текущее состояние верхнего уровня нового контура."""

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
