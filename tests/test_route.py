from __future__ import annotations

import logging
import os
import sys
from pathlib import Path

from src.application.models.route import Route
from src.application.services.drive_controller import DriveController
from src.config.settings import Settings
from tests.common import create_drive_controller, parse_route_from_json, prompt_yes_no

DESCRIPTION: str = "Выполнение маршрута"
PROMPT_START: str = "Запустить тест?"
PROMPT_RESULT: str = "Машинка выполнила маршрут без рывков и с отклонением ≤ 5 см?"
ERROR_ROUTE_PARSE: str = "Ошибка при разборе маршрута: {exc}"
ERROR_ROUTE_FILE: str = "Ошибка: файл не найден: {path}"
ERROR_ROUTE_CALIBRATION: str = (
    "✗ Тест не пройден. Проверьте калибровку (TL_*, TURN_DURATION_90_DEG_SEC)"
)
USAGE_MESSAGE: str = (
    "Использование: python -m tests.test_route <файл.json> [max_speed_percent] [--start N]\n"
    "  или: RASPTANK_ROUTE_START_SEG=N (тот же эффект, что --start N)\n"
    "  N — индекс сегмента в JSON (0 = с начала), пропускаются 0..N-1."
)


def _diag_mode() -> bool:
    return os.environ.get("RASPTANK_DIAG", "").strip().lower() in ("1", "true", "yes", "on")


def _setup_diag_logging() -> None:
    """Логи IMU и drive_controller в консоль (см. tests/SQUARE_DIAGNOSTIC.md)."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        force=True,
    )
    for name in (
        "src.infrastructures.imu",
        "src.application.services.drive_controller",
    ):
        logging.getLogger(name).setLevel(logging.INFO)


def _parse_start_segment() -> int | None:
    raw: str = os.environ.get("RASPTANK_ROUTE_START_SEG", "").strip()
    if not raw:
        return None
    try:
        return int(raw, 10)
    except ValueError:
        return None


def run(
    route_file: Path,
    max_speed_percent: int | None = None,
    start_segment_index: int = 0,
) -> int:
    """Запуск теста выполнения маршрута."""
    diag: bool = _diag_mode()
    if diag:
        _setup_diag_logging()
        print(
            "RASPTANK_DIAG=1: подробные логи, вопросы Y/n пропущены. "
            "Сохраните вывод: ... 2>&1 | tee square_run.log\n",
        )

    print(f"\n=== {DESCRIPTION} ===\n")
    print(f"Маршрут: {route_file}")

    if not route_file.exists():
        print(ERROR_ROUTE_FILE.format(path=route_file), file=sys.stderr)
        return 1

    try:
        route: Route = parse_route_from_json(route_file)
    except (ValueError, KeyError) as exc:
        print(ERROR_ROUTE_PARSE.format(exc=exc), file=sys.stderr)
        return 1

    n_seg: int = len(route.segments)
    print(f"Сегментов: {n_seg}")
    if start_segment_index > 0:
        if start_segment_index >= n_seg:
            print(
                f"Ошибка: --start / RASPTANK_ROUTE_START_SEG={start_segment_index} "
                f"≥ числа сегментов ({n_seg}).",
                file=sys.stderr,
            )
            return 1
        print(
            f"Старт с сегмента {start_segment_index} (сегменты 0..{start_segment_index - 1} пропущены). "
            "Робот должен стоять так, как после выполнения пропущенной части.",
        )
    if max_speed_percent is not None:
        print(f"Ограничение скорости: {max_speed_percent}%.")
    print()
    print("Установите машинку в начальную точку.")
    print("Маршрут должен быть физически достижим.")
    print()

    if not diag and not prompt_yes_no(PROMPT_START):
        return 0

    settings: Settings = Settings()
    if max_speed_percent is not None:
        settings = settings.model_copy(update={"base_speed_percent": max_speed_percent})
    drive: DriveController = create_drive_controller(settings=settings)
    try:
        drive.execute_route_sync(route=route, start_segment_index=start_segment_index)
        print("Маршрут выполнен. Остановка.")

        if diag:
            print("✓ Прогон завершён (диагностика). Пришлите файл лога, если нужен разбор.")
            return 0

        if prompt_yes_no(PROMPT_RESULT):
            print("✓ Тест пройден.")
            return 0
        print(ERROR_ROUTE_CALIBRATION)
        return 1

    finally:
        drive.destroy()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(USAGE_MESSAGE, file=sys.stderr)
        sys.exit(1)
    route_path: Path = Path(sys.argv[1])
    speed: int | None = None
    start_seg: int = _parse_start_segment() or 0
    i: int = 2
    while i < len(sys.argv):
        arg: str = sys.argv[i]
        if arg == "--start" and i + 1 < len(sys.argv):
            try:
                start_seg = int(sys.argv[i + 1], 10)
            except ValueError:
                print(f"Ожидалось число после --start: {sys.argv[i + 1]}", file=sys.stderr)
                sys.exit(1)
            i += 2
            continue
        try:
            speed = int(arg, 10)
        except ValueError:
            print(f"Неизвестный аргумент: {arg}", file=sys.stderr)
            print(USAGE_MESSAGE, file=sys.stderr)
            sys.exit(1)
        i += 1

    sys.exit(
        run(
            route_file=route_path,
            max_speed_percent=speed,
            start_segment_index=start_seg,
        ),
    )
