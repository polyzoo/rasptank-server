from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from src.application.factories import create_drive_controller as create_app_drive_controller
from src.application.models.route import (
    BackwardSegment,
    ForwardSegment,
    Route,
    TurnLeftSegment,
    TurnRightSegment,
)
from src.config.settings import Settings
from src.application.services.drive_controller import DriveController
from src.infrastructures.motor import MotorController

ENCODING_UTF8: str = "utf-8"
PROMPT_YES_VALUES: tuple[str, ...] = ("y", "yes", "д", "да")
PROMPT_NO_VALUES: tuple[str, ...] = ("n", "no", "н", "нет")
PROMPT_YES_NO_ERROR: str = "Введите y или n"
PROMPT_FLOAT_ERROR: str = "Введите число"
DEFAULT_TURN_ANGLE_DEG: float = 90.0


def create_drive_controller(settings: Settings | None = None) -> DriveController:
    """Создаёт экземпляр контроллера движения."""
    s: Settings = settings or Settings()
    return create_app_drive_controller(s)


def create_motor_controller(settings: Settings | None = None) -> MotorController:
    """Создаёт контроллер моторов без высокоуровневой логики движения."""
    s: Settings = settings or Settings()
    return MotorController(
        tl_left_offset=s.tl_left_offset,
        tl_right_offset=s.tl_right_offset,
    )


def parse_route_from_json(path: Path) -> Route:
    """Загрузка маршрута из JSON-файла."""
    data: dict[str, Any] = json.loads(path.read_text(encoding=ENCODING_UTF8))
    raw_segments: list[dict[str, Any]] = data.get("segments", [])
    segments = [_parse_segment(segment_data) for segment_data in raw_segments]

    if not segments:
        raise ValueError("Маршрут должен содержать хотя бы один сегмент")

    return Route(segments=segments)


def _parse_segment(
    segment_data: dict[str, Any],
) -> ForwardSegment | BackwardSegment | TurnLeftSegment | TurnRightSegment:
    """Преобразует JSON-сегмент в доменную модель маршрута."""
    action: str = str(segment_data.get("action"))

    if action == "forward":
        return ForwardSegment(distance_cm=_parse_non_negative_float(segment_data, "distance_cm"))

    if action == "backward":
        return BackwardSegment(distance_cm=_parse_non_negative_float(segment_data, "distance_cm"))

    if action == "turn_left":
        return TurnLeftSegment(
            angle_deg=_parse_non_negative_float(
                segment_data,
                "angle_deg",
                default=DEFAULT_TURN_ANGLE_DEG,
            ),
        )

    if action == "turn_right":
        return TurnRightSegment(
            angle_deg=_parse_non_negative_float(
                segment_data,
                "angle_deg",
                default=DEFAULT_TURN_ANGLE_DEG,
            ),
        )

    raise ValueError(f"Неизвестный сегмент: {action}")


def _parse_non_negative_float(
    data: dict[str, Any],
    field_name: str,
    *,
    default: float | None = None,
) -> float:
    """Читает неотрицательное число из словаря сегмента."""
    raw_value: Any = data.get(field_name, default)
    value: float = float(raw_value)
    if value < 0:
        raise ValueError(f"{field_name} не может быть отрицательным: {value}")
    return value


def prompt_yes_no(question: str, default: bool = True) -> bool:
    """Запрос у пользователя да или нет."""
    suffix: str = " [Y/n]" if default else " [y/N]"
    while True:
        try:
            answer: str = input(f"{question}{suffix}: ").strip().lower()
        except EOFError:
            return default
        if not answer:
            return default
        if answer in PROMPT_YES_VALUES:
            return True
        if answer in PROMPT_NO_VALUES:
            return False
        print(PROMPT_YES_NO_ERROR)


def prompt_float(question: str, default: float | None = None) -> float:
    """Запрос числа с плавающей точкой."""
    suffix: str = f" [{default}]" if default is not None else ""
    while True:
        try:
            answer: str = input(f"{question}{suffix}: ").strip()
        except EOFError:
            if default is not None:
                return default
            raise
        if not answer and default is not None:
            return default
        try:
            return float(answer)
        except ValueError:
            print(PROMPT_FLOAT_ERROR)
