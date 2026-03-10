from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, Union


@dataclass(slots=True)
class ForwardSegment:
    """Сегмент движения вперед."""

    action: Literal["forward"] = "forward"
    distance_cm: float = 0.0


@dataclass(slots=True)
class TurnLeftSegment:
    """Сегмент поворота налево на месте."""

    action: Literal["turn_left"] = "turn_left"
    duration_sec: float = 0.0


@dataclass(slots=True)
class TurnRightSegment:
    """Сегмент поворота направо на месте."""

    action: Literal["turn_right"] = "turn_right"
    duration_sec: float = 0.0


@dataclass(slots=True)
class Route:
    """Маршрут — последовательность сегментов движения."""

    segments: list[
        Union[
            ForwardSegment,
            TurnLeftSegment,
            TurnRightSegment,
        ],
    ] = field(default_factory=list)
