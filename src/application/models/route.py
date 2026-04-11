from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, TypeAlias, final

FORWARD_ACTION: str = "forward"
BACKWARD_ACTION: str = "backward"
TURN_LEFT_ACTION: str = "turn_left"
TURN_RIGHT_ACTION: str = "turn_right"


@final
@dataclass(slots=True)
class ForwardSegment:
    """Сегмент движения вперед."""

    action: Literal["forward"] = FORWARD_ACTION
    distance_cm: float = 0.0


@final
@dataclass(slots=True)
class BackwardSegment:
    """Сегмент движения назад."""

    action: Literal["backward"] = BACKWARD_ACTION
    distance_cm: float = 0.0


@final
@dataclass(slots=True)
class TurnLeftSegment:
    """Сегмент поворота налево на месте."""

    action: Literal["turn_left"] = TURN_LEFT_ACTION
    angle_deg: float = 90.0


@final
@dataclass(slots=True)
class TurnRightSegment:
    """Сегмент поворота направо на месте."""

    action: Literal["turn_right"] = TURN_RIGHT_ACTION
    angle_deg: float = 90.0


RouteSegment: TypeAlias = ForwardSegment | BackwardSegment | TurnLeftSegment | TurnRightSegment


@final
@dataclass(slots=True)
class Route:
    """Маршрут из последовательности сегментов движения."""

    segments: list[RouteSegment] = field(default_factory=list)
