from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, TypeAlias, final


@final
@dataclass(slots=True)
class ForwardSegment:
    """Сегмент движения вперед."""

    action: Literal["forward"] = "forward"
    distance_cm: float = 0.0


@final
@dataclass(slots=True)
class BackwardSegment:
    """Сегмент движения назад."""

    action: Literal["backward"] = "backward"
    distance_cm: float = 0.0


@final
@dataclass(slots=True)
class TurnLeftSegment:
    """Сегмент поворота налево на месте."""

    action: Literal["turn_left"] = "turn_left"
    angle_deg: float = 90.0


@final
@dataclass(slots=True)
class TurnRightSegment:
    """Сегмент поворота направо на месте."""

    action: Literal["turn_right"] = "turn_right"
    angle_deg: float = 90.0


RouteSegment: TypeAlias = ForwardSegment | BackwardSegment | TurnLeftSegment | TurnRightSegment


@final
@dataclass(slots=True)
class Route:
    """Маршрут из последовательности сегментов движения."""

    segments: list[RouteSegment] = field(default_factory=list)
