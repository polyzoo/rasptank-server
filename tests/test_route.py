from __future__ import annotations

import sys
import time
from pathlib import Path

from src.application.models.route import Route
from src.application.services.drive_controller import DriveController
from tests.common import create_drive_controller, parse_route_from_json, prompt_yes_no

DESCRIPTION: str = "Выполнение маршрута"
PROMPT_START: str = "Запустить тест?"
PROMPT_RESULT: str = "Машинка выполнила маршрут без рывков и с отклонением ≤ 5 см?"
ERROR_ROUTE_PARSE: str = "Ошибка при разборе маршрута: {exc}"
ERROR_ROUTE_FILE: str = "Ошибка: файл не найден: {path}"
ERROR_ROUTE_CALIBRATION: str = (
    "✗ Тест не пройден. Проверьте калибровку (TL_*, TURN_DURATION_90_DEG_SEC)"
)
USAGE_MESSAGE: str = "Использование: python -m tests.test_route <файл.json>"
SLEEP_INTERVAL_SEC: float = 0.2


def run(route_file: Path) -> int:
    """Запуск теста выполнения маршрута."""
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

    print(f"Сегментов: {len(route.segments)}")
    print()
    print("Установите машинку в начальную точку.")
    print("Маршрут должен быть физически достижим.")
    print()

    if not prompt_yes_no(PROMPT_START):
        return 0

    drive: DriveController = create_drive_controller()
    try:
        drive.execute_route(route=route)
        print("Маршрут запущен. Ожидание завершения...")

        while drive.is_moving:
            time.sleep(SLEEP_INTERVAL_SEC)
        print("Остановка.")

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
    sys.exit(run(route_file=Path(sys.argv[1])))
