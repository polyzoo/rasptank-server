from __future__ import annotations

import csv
import json
import time
from datetime import UTC, datetime
from pathlib import Path
from statistics import fmean
from typing import TYPE_CHECKING, Any

from src.application.models.route import (
    BackwardSegment,
    ForwardSegment,
    Route,
    TurnLeftSegment,
    TurnRightSegment,
)

if TYPE_CHECKING:
    from src.application.services.drive_controller import DriveController
    from src.config.settings import Settings
    from src.infrastructures.imu import IMUSensor
    from src.infrastructures.motor import MotorController

ENCODING_UTF8: str = "utf-8"
PROMPT_YES_VALUES: tuple[str, ...] = ("y", "yes", "д", "да")
PROMPT_NO_VALUES: tuple[str, ...] = ("n", "no", "н", "нет")
PROMPT_YES_NO_ERROR: str = "Введите y или n"
PROMPT_FLOAT_ERROR: str = "Введите число"
DEFAULT_TURN_ANGLE_DEG: float = 90.0
DEFAULT_RESULTS_DIR_NAME: str = "results"
TIMESTAMP_FORMAT: str = "%Y%m%d_%H%M%S"


def create_drive_controller(settings: "Settings | None" = None) -> "DriveController":
    """Создаёт экземпляр контроллера движения."""
    from src.application.factories import create_drive_controller as create_app_drive_controller
    from src.config.settings import Settings

    s: Settings = settings or Settings()
    return create_app_drive_controller(s)


def create_motor_controller(settings: "Settings | None" = None) -> "MotorController":
    """Создаёт контроллер моторов без высокоуровневой логики движения."""
    from src.config.settings import Settings
    from src.infrastructures.motor import MotorController

    s: Settings = settings or Settings()
    return MotorController(
        tl_left_offset=s.tl_left_offset,
        tl_right_offset=s.tl_right_offset,
    )


def create_imu_sensor(settings: "Settings | None" = None) -> "IMUSensor":
    """Создаёт IMU-сенсор для низкоуровневых аппаратных экспериментов."""
    from src.config.settings import Settings
    from src.infrastructures.imu import IMUSensor

    _ = settings or Settings()
    return IMUSensor()


def create_results_dir(experiment_name: str) -> Path:
    """Создаёт уникальную директорию результатов эксперимента."""
    timestamp: str = datetime.now().strftime(TIMESTAMP_FORMAT)
    results_dir: Path = (
        Path(__file__).resolve().parent
        / DEFAULT_RESULTS_DIR_NAME
        / f"{timestamp}_{experiment_name}"
    )
    results_dir.mkdir(parents=True, exist_ok=True)
    return results_dir


def write_csv_rows(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
    """Сохраняет список словарей в CSV-файл."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding=ENCODING_UTF8, newline="") as file_obj:
        writer = csv.DictWriter(file_obj, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: dict[str, Any]) -> None:
    """Сохраняет JSON с читаемым форматированием."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding=ENCODING_UTF8,
    )


def utc_now_iso() -> str:
    """Текущее время UTC в ISO-формате."""
    return datetime.now(UTC).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def collect_timed_samples(
    duration_sec: float,
    sample_interval_sec: float,
    read_fn: Any,
) -> list[tuple[float, float]]:
    """Собирает значения датчика по таймеру и возвращает пары (t, value)."""
    if duration_sec < 0.0:
        raise ValueError("duration_sec не может быть отрицательным")
    if sample_interval_sec <= 0.0:
        raise ValueError("sample_interval_sec должен быть положительным")

    total_steps: int = int(duration_sec / sample_interval_sec)
    sample_indices: list[int] = list(range(total_steps + 1))
    if (total_steps * sample_interval_sec) < duration_sec:
        sample_indices.append(total_steps + 1)

    start_t: float = time.monotonic()
    samples: list[tuple[float, float]] = []
    for sample_idx in sample_indices:
        target_t: float = start_t + min(duration_sec, sample_idx * sample_interval_sec)
        sleep_sec: float = target_t - time.monotonic()
        if sleep_sec > 0.0:
            time.sleep(sleep_sec)
        elapsed_sec: float = time.monotonic() - start_t
        samples.append((elapsed_sec, float(read_fn())))
    return samples


def mean_sample_value(samples: list[tuple[float, float]]) -> float:
    """Среднее по собранным значениям датчика."""
    if not samples:
        return 0.0
    return float(fmean(value for _, value in samples))

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
