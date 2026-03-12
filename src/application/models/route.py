from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, TypeAlias, final


def _validate_non_negative(value: float, name: str) -> None:
    """Проверяет, что значение не отрицательное."""
    if value < 0:
        raise ValueError(f"{name} не может быть отрицательным: {value}")


@final
@dataclass(slots=True)
class ForwardSegment:
    """Сегмент движения вперед."""

    action: Literal["forward"] = "forward"
    distance_cm: float = 0.0

    def __post_init__(self) -> None:
        """Валидирует расстояние."""
        _validate_non_negative(self.distance_cm, "distance_cm")


@final
@dataclass(slots=True)
class BackwardSegment:
    """Сегмент движения назад."""

    action: Literal["backward"] = "backward"
    distance_cm: float = 0.0

    def __post_init__(self) -> None:
        """Валидирует расстояние."""
        _validate_non_negative(self.distance_cm, "distance_cm")


@final
@dataclass(slots=True)
class TurnLeftSegment:
    """Сегмент поворота налево на месте."""

    action: Literal["turn_left"] = "turn_left"
    duration_sec: float = 0.0

    def __post_init__(self) -> None:
        """Валидирует длительность поворота."""
        _validate_non_negative(self.duration_sec, "duration_sec")


@final
@dataclass(slots=True)
class TurnRightSegment:
    """Сегмент поворота направо на месте."""

    action: Literal["turn_right"] = "turn_right"
    duration_sec: float = 0.0

    def __post_init__(self) -> None:
        """Валидирует длительность поворота."""
        _validate_non_negative(self.duration_sec, "duration_sec")


RouteSegment: TypeAlias = ForwardSegment | BackwardSegment | TurnLeftSegment | TurnRightSegment


@final
@dataclass(slots=True)
class Route:
    """Маршрут из последовательности сегментов движения."""

    segments: list[RouteSegment] = field(default_factory=list)

    def __post_init__(self) -> None:
        """Гарантирует, что маршрут содержит хотя бы один сегмент."""
        if not self.segments:
            raise ValueError("Маршрут должен содержать хотя бы один сегмент")
