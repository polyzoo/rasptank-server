from __future__ import annotations

import csv
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from tests.common import (
    create_imu_sensor,
    create_motor_controller,
    create_results_dir,
    prompt_yes_no,
    utc_now_iso,
    write_json,
)

DESCRIPTION: str = "Эксперимент 1: поворотные коэффициенты kR/kL по гироскопу"
PROMPT_START: str = "Запустить эксперимент?"
DEFAULT_SPEEDS: tuple[int, ...] = (20, 40, 60)
DEFAULT_SETTLE_SEC: float = 5.0
DEFAULT_MEASURE_SEC: float = 5.0
DEFAULT_PRE_TRIAL_PAUSE_SEC: float = 1.0
DEFAULT_POST_TRIAL_PAUSE_SEC: float = 1.5
CSV_FILE_NAME: str = "turn_trials.csv"
JSON_FILE_NAME: str = "turn_trials.json"


@dataclass(frozen=True, slots=True)
class TurnTrialCommand:
    """Описание одной моторной команды в эксперименте kR/kL."""

    experiment_no: int
    right_track_percent: int
    left_track_percent: int
    label: str


@dataclass(frozen=True, slots=True)
class TurnTrialResult:
    """Структурированный результат одного поворотного опыта."""

    experiment_no: int
    right_track_percent: int
    left_track_percent: int
    angle_window_start_unix_s: float
    angle_window_end_unix_s: float
    angle_window_start_iso: str
    angle_window_end_iso: str
    angle_window_start_rel_s: float
    angle_window_end_rel_s: float
    yaw_start_deg: float
    yaw_end_deg: float
    angle_delta_deg: float


def _print_instructions() -> None:
    print("\n=== " + DESCRIPTION + " ===\n")
    print("Что требуется:")
    print("1. Освободить вокруг машинки место для безопасного вращения.")
    print("2. Поставить машинку на ровную поверхность.")
    print("3. Во время одного опыта подаётся команда только на одну гусеницу.")
    print("4. Первые 5 секунд идут на выход на установившуюся скорость.")
    print("5. Затем за следующие 5 секунд измеряется изменение угла по гироскопу.")
    print()
    print("Результаты будут сохранены в CSV и JSON в tests/results/...")
    print("Поля CSV: experiment_no, right_track_percent, left_track_percent,")
    print("          angle_window_start_*, angle_window_end_*, yaw_start_deg, yaw_end_deg, angle_delta_deg")
    print()


def _build_trial_plan() -> tuple[TurnTrialCommand, ...]:
    """Построить фиксированный план из 12 опытов по ТЗ."""
    commands: list[TurnTrialCommand] = []
    experiment_no: int = 1

    for speed_percent in DEFAULT_SPEEDS:
        commands.append(
            TurnTrialCommand(
                experiment_no=experiment_no,
                right_track_percent=speed_percent,
                left_track_percent=0,
                label=f"right_only_+{speed_percent}",
            ),
        )
        experiment_no += 1
    for speed_percent in DEFAULT_SPEEDS:
        commands.append(
            TurnTrialCommand(
                experiment_no=experiment_no,
                right_track_percent=-speed_percent,
                left_track_percent=0,
                label=f"right_only_-{speed_percent}",
            ),
        )
        experiment_no += 1
    for speed_percent in DEFAULT_SPEEDS:
        commands.append(
            TurnTrialCommand(
                experiment_no=experiment_no,
                right_track_percent=0,
                left_track_percent=speed_percent,
                label=f"left_only_+{speed_percent}",
            ),
        )
        experiment_no += 1
    for speed_percent in DEFAULT_SPEEDS:
        commands.append(
            TurnTrialCommand(
                experiment_no=experiment_no,
                right_track_percent=0,
                left_track_percent=-speed_percent,
                label=f"left_only_-{speed_percent}",
            ),
        )
        experiment_no += 1

    return tuple(commands)


def _run_single_trial(
    *,
    motor: Any,
    imu: Any,
    command: TurnTrialCommand,
    settle_sec: float,
    measure_sec: float,
) -> TurnTrialResult:
    """Выполнить один поворотный опыт и вернуть измеренный угол."""
    motor.stop()
    time.sleep(DEFAULT_PRE_TRIAL_PAUSE_SEC)
    imu.reset_yaw()

    trial_start_t: float = time.monotonic()
    motor.drive_tracks(
        left_speed_percent=command.left_track_percent,
        right_speed_percent=command.right_track_percent,
    )
    time.sleep(settle_sec)

    angle_window_start_unix_s: float = time.time()
    angle_window_start_iso: str = utc_now_iso()
    yaw_start_deg: float = imu.get_yaw()
    angle_window_start_rel_s: float = time.monotonic() - trial_start_t

    time.sleep(measure_sec)

    angle_window_end_unix_s: float = time.time()
    angle_window_end_iso: str = utc_now_iso()
    yaw_end_deg: float = imu.get_yaw()
    angle_window_end_rel_s: float = time.monotonic() - trial_start_t

    motor.stop()
    time.sleep(DEFAULT_POST_TRIAL_PAUSE_SEC)
    return TurnTrialResult(
        experiment_no=command.experiment_no,
        right_track_percent=command.right_track_percent,
        left_track_percent=command.left_track_percent,
        angle_window_start_unix_s=angle_window_start_unix_s,
        angle_window_end_unix_s=angle_window_end_unix_s,
        angle_window_start_iso=angle_window_start_iso,
        angle_window_end_iso=angle_window_end_iso,
        angle_window_start_rel_s=angle_window_start_rel_s,
        angle_window_end_rel_s=angle_window_end_rel_s,
        yaw_start_deg=yaw_start_deg,
        yaw_end_deg=yaw_end_deg,
        angle_delta_deg=yaw_end_deg - yaw_start_deg,
    )


def _write_results(results_dir: Path, rows: list[TurnTrialResult]) -> None:
    """Сохранить CSV и JSON по всем опытам."""
    csv_path: Path = results_dir / CSV_FILE_NAME
    json_path: Path = results_dir / JSON_FILE_NAME

    fieldnames: list[str] = list(asdict(rows[0]).keys()) if rows else []
    with csv_path.open("w", encoding="utf-8", newline="") as file_obj:
        writer = csv.DictWriter(file_obj, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))

    write_json(
        json_path,
        {
            "experiment": "kR_kL_turn_response",
            "created_at": utc_now_iso(),
            "settle_sec": DEFAULT_SETTLE_SEC,
            "measure_sec": DEFAULT_MEASURE_SEC,
            "rows": [asdict(row) for row in rows],
        },
    )


def _print_trial_summary(row: TurnTrialResult) -> None:
    """Вывести на экран сводку опыта в полях, близких к ТЗ."""
    print(
        "  Результат: "
        f"exp={row.experiment_no:02d}, "
        f"UR={row.right_track_percent:+d}%, "
        f"UL={row.left_track_percent:+d}%, "
        f"t_start={row.angle_window_start_iso}, "
        f"t_end={row.angle_window_end_iso}, "
        f"delta_angle={row.angle_delta_deg:.2f}°",
    )


def run() -> int:
    """Запуск эксперимента для оценки kR и kL по гироскопу."""
    from src.config.settings import Settings

    _print_instructions()

    if not prompt_yes_no(PROMPT_START):
        return 0

    settings: Settings = Settings()
    results_dir: Path = create_results_dir("kr_kl_turn")
    trial_plan: tuple[TurnTrialCommand, ...] = _build_trial_plan()
    motor = create_motor_controller(settings)
    imu = create_imu_sensor(settings)
    rows: list[TurnTrialResult] = []

    try:
        imu.start(calibrate=True)
        time.sleep(0.5)
        for command in trial_plan:
            print()
            print(
                f"Опыт #{command.experiment_no:02d}: "
                f"UR={command.right_track_percent:+d}% UL={command.left_track_percent:+d}% "
                f"({command.label})",
            )
            row: TurnTrialResult = _run_single_trial(
                motor=motor,
                imu=imu,
                command=command,
                settle_sec=DEFAULT_SETTLE_SEC,
                measure_sec=DEFAULT_MEASURE_SEC,
            )
            rows.append(row)
            _print_trial_summary(row)
    finally:
        motor.stop()
        motor.destroy()
        imu.stop()
        imu.destroy()

    if not rows:
        print("Опыты не были выполнены.", file=sys.stderr)
        return 1

    _write_results(results_dir=results_dir, rows=rows)
    print()
    print(f"Результаты сохранены в: {results_dir}")
    print(f"CSV: {results_dir / CSV_FILE_NAME}")
    print(f"JSON: {results_dir / JSON_FILE_NAME}")
    return 0


if __name__ == "__main__":
    sys.exit(run())
