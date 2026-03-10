from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from src.application.models.route import ForwardSegment, Route, TurnLeftSegment, TurnRightSegment
from src.application.services.drive_controller import DriveController
from src.config.settings import Settings
from src.infrastructures.motor import MotorController
from src.infrastructures.ultrasonic import UltrasonicSensor

ENCODING_UTF8: str = "utf-8"
PROMPT_YES_VALUES: tuple[str, ...] = ("y", "yes", "д", "да")
PROMPT_NO_VALUES: tuple[str, ...] = ("n", "no", "н", "нет")
PROMPT_YES_NO_ERROR: str = "Введите y или n"
PROMT_FLOAT_ERROR: str = "Введите число"


def create_drive_controller(settings: Settings | None = None) -> DriveController:
    """Создаёт экземпляр контроллера движения."""
    s: Settings = settings or Settings()
    ultrasonic_sensor: UltrasonicSensor = UltrasonicSensor()
    motor_controller: MotorController = MotorController(
        tl_left_offset=s.tl_left_offset,
        tl_right_offset=s.tl_right_offset,
    )
    return DriveController(
        motor_controller=motor_controller,
        ultrasonic_sensor=ultrasonic_sensor,
        min_obstacle_distance_cm=s.min_obstacle_distance_cm,
        deceleration_distance_cm=s.deceleration_distance_cm,
        base_speed_percent=s.base_speed_percent,
        turn_speed_percent=s.turn_speed_percent,
        max_speed_cm_per_sec=s.max_speed_cm_per_sec,
        update_interval_sec=s.update_interval_sec,
    )


def parse_route_from_json(path: Path) -> Route:
    """Загрузка маршрута из JSON-файла."""
    data: dict[str, Any] = json.loads(path.read_text(encoding=ENCODING_UTF8))

    segments: list[ForwardSegment | TurnLeftSegment | TurnRightSegment] = []
    for s in data.get("segments", []):
        action: str = s.get("action")

        if action == "forward":
            segments.append(ForwardSegment(distance_cm=float(s["distance_cm"])))

        elif action == "turn_left":
            segments.append(TurnLeftSegment(duration_sec=float(s["duration_sec"])))

        elif action == "turn_right":
            segments.append(TurnRightSegment(duration_sec=float(s["duration_sec"])))

        else:
            raise ValueError(f"Неизвестный сегмент: {action}")

    if not segments:
        raise ValueError("Маршрут должен содержать хотя бы один сегмент")

    return Route(segments=segments)


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
            print(PROMT_FLOAT_ERROR)
