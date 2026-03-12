from __future__ import annotations

from dataclasses import dataclass
from typing import Any, final


@final
class EventType:
    """Типы событий траектории."""

    ROUTE_STARTED: str = "route_started"
    POSITION: str = "position"
    ROUTE_ENDED: str = "route_ended"
    OBSTACLE_CANNOT_BYPASS: str = "obstacle_cannot_bypass"


@final
class RouteEndReason:
    """Причины завершения маршрута."""

    COMPLETE: str = "complete"
    OBSTACLE: str = "obstacle"
    STOP: str = "stop"


@final
@dataclass(slots=True)
class Position:
    """Позиция машинки на плоскости."""

    x_cm: float
    y_cm: float
    heading_deg: float

    def to_dict(self) -> dict[str, Any]:
        """Словарь для сериализации - позиция по осям X и Y и ориентация объекта."""
        return {
            "x_cm": self.x_cm,
            "y_cm": self.y_cm,
            "heading_deg": self.heading_deg,
        }

    def to_event_dict(self, event_type: str) -> dict[str, Any]:
        """Словарь события с позицией - начало движения и позиция объекта."""
        return {
            "type": event_type,
            **self.to_dict(),
        }


@final
@dataclass(slots=True)
class RouteEndedEvent:
    """Событие завершения маршрута."""

    reason: str
    x_cm: float
    y_cm: float
    heading_deg: float

    def to_dict(self) -> dict[str, Any]:
        """Словарь для отправки данных через WebSocket."""
        return {
            "type": EventType.ROUTE_ENDED,
            "reason": self.reason,
            "x_cm": self.x_cm,
            "y_cm": self.y_cm,
            "heading_deg": self.heading_deg,
        }
