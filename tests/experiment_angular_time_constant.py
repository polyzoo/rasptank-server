from __future__ import annotations

import time
from pathlib import Path
from typing import Any

from tests.common import (
    collect_timed_samples,
    create_imu_sensor,
    create_motor_controller,
    create_results_dir,
    mean_sample_value,
    prompt_yes_no,
    utc_now_iso,
    write_csv_rows,
    write_json,
)

DESCRIPTION: str = "Эксперимент 2.2: угловой разгон для оценки Tω"
PROMPT_START: str = "Запустить эксперимент углового разгона?"
PROMPT_READY_TEMPLATE: str = "Машинка готова к угловому разгону на U={u}%?"
DEFAULT_SPEEDS: tuple[int, ...] = (20, 40, 60)
DEFAULT_DURATION_SEC: float = 5.0
DEFAULT_SAMPLE_INTERVAL_SEC: float = 0.1
DEFAULT_BASELINE_DURATION_SEC: float = 0.8
DEFAULT_POST_RUN_PAUSE_SEC: float = 1.5
SUMMARY_CSV_FILE_NAME: str = "angular_time_constant_runs.csv"
SUMMARY_JSON_FILE_NAME: str = "angular_time_constant_runs.json"


def _print_instructions() -> None:
    print("\n=== " + DESCRIPTION + " ===\n")
    print("Что требуется:")
    print("1. Машинка стоит неподвижно на ровной поверхности.")
    print("2. Скрипт сам снимет baseline гироскопа перед каждым разгоном.")
    print("3. Затем подаст противоположные напряжения: UR = -UL = 20%, 40%, 60%.")
    print("4. Угловая скорость пишется каждые 0.1 с в течение 5 секунд.")
    print()
    print("Угловая скорость сохраняется в deg/s.")
    print("CSV каждого прогона: time_s, angular_velocity_deg_s")
    print()


def _write_run_csv(
    *,
    results_dir: Path,
    speed_percent: int,
    rows: list[dict[str, float]],
) -> str:
    """Сохранить CSV одного углового разгона."""
    file_name: str = f"angular_u{speed_percent}.csv"
    write_csv_rows(
        results_dir / file_name,
        fieldnames=["time_s", "angular_velocity_deg_s"],
        rows=rows,
    )
    return file_name


def _collect_angular_run(
    *,
    motor: Any,
    imu: Any,
    speed_percent: int,
) -> tuple[list[dict[str, float]], dict[str, float]]:
    """Собрать baseline и step-response угловой скорости."""
    baseline_samples = collect_timed_samples(
        duration_sec=DEFAULT_BASELINE_DURATION_SEC,
        sample_interval_sec=DEFAULT_SAMPLE_INTERVAL_SEC,
        read_fn=imu.get_gyro_z_deg_per_sec,
    )
    baseline_deg_s: float = mean_sample_value(baseline_samples)
    start_iso: str = utc_now_iso()
    motor.drive_tracks(
        left_speed_percent=-speed_percent,
        right_speed_percent=speed_percent,
    )
    samples = collect_timed_samples(
        duration_sec=DEFAULT_DURATION_SEC,
        sample_interval_sec=DEFAULT_SAMPLE_INTERVAL_SEC,
        read_fn=imu.get_gyro_z_deg_per_sec,
    )
    motor.stop()
    rows: list[dict[str, float]] = [
        {
            "time_s": round(elapsed_sec, 3),
            "angular_velocity_deg_s": round(raw_value - baseline_deg_s, 6),
        }
        for elapsed_sec, raw_value in samples
    ]
    metadata: dict[str, float] = {
        "speed_percent": float(speed_percent),
        "baseline_deg_s": round(baseline_deg_s, 6),
        "sample_count": float(len(rows)),
        "start_iso": start_iso,
    }
    return rows, metadata


def _print_angular_run_summary(
    *,
    speed_percent: int,
    baseline_deg_s: float,
    sample_count: int,
    csv_path: Path,
) -> None:
    """Вывести краткую сводку по угловому разгону."""
    print(
        "  Результат: "
        f"U={speed_percent}%, "
        f"baseline={baseline_deg_s:.5f} deg/s, "
        f"samples={sample_count}, "
        f"csv={csv_path}",
    )


def run() -> int:
    """Запуск углового step-response эксперимента для оценки Tω."""
    from src.config.settings import Settings

    _print_instructions()

    if not prompt_yes_no(PROMPT_START):
        return 0

    settings: Settings = Settings()
    results_dir: Path = create_results_dir("angular_time_constant")
    motor = create_motor_controller(settings)
    imu = create_imu_sensor(settings)
    summary_rows: list[dict[str, object]] = []

    try:
        imu.start(calibrate=True)
        for speed_percent in DEFAULT_SPEEDS:
            print()
            if not prompt_yes_no(PROMPT_READY_TEMPLATE.format(u=speed_percent)):
                continue
            motor.stop()
            rows, metadata = _collect_angular_run(
                motor=motor,
                imu=imu,
                speed_percent=speed_percent,
            )
            csv_file_name: str = _write_run_csv(
                results_dir=results_dir,
                speed_percent=speed_percent,
                rows=rows,
            )
            summary_rows.append(
                {
                    "speed_percent": speed_percent,
                    "left_track_percent": -speed_percent,
                    "right_track_percent": speed_percent,
                    "baseline_deg_s": metadata["baseline_deg_s"],
                    "sample_count": int(metadata["sample_count"]),
                    "csv_file": csv_file_name,
                    "started_at": metadata["start_iso"],
                },
            )
            time.sleep(DEFAULT_POST_RUN_PAUSE_SEC)
            _print_angular_run_summary(
                speed_percent=speed_percent,
                baseline_deg_s=float(metadata["baseline_deg_s"]),
                sample_count=int(metadata["sample_count"]),
                csv_path=results_dir / csv_file_name,
            )
    finally:
        motor.stop()
        motor.destroy()
        imu.stop()
        imu.destroy()

    if not summary_rows:
        print("Не удалось собрать ни одного углового прогона.")
        return 1

    write_csv_rows(
        results_dir / SUMMARY_CSV_FILE_NAME,
        fieldnames=[
            "speed_percent",
            "left_track_percent",
            "right_track_percent",
            "baseline_deg_s",
            "sample_count",
            "csv_file",
            "started_at",
        ],
        rows=summary_rows,
    )
    write_json(
        results_dir / SUMMARY_JSON_FILE_NAME,
        {
            "experiment": "angular_time_constant",
            "created_at": utc_now_iso(),
            "duration_sec": DEFAULT_DURATION_SEC,
            "sample_interval_sec": DEFAULT_SAMPLE_INTERVAL_SEC,
            "baseline_duration_sec": DEFAULT_BASELINE_DURATION_SEC,
            "runs": summary_rows,
        },
    )
    print()
    print(f"Результаты сохранены в: {results_dir}")
    print(f"Сводка CSV: {results_dir / SUMMARY_CSV_FILE_NAME}")
    print(f"Сводка JSON: {results_dir / SUMMARY_JSON_FILE_NAME}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
