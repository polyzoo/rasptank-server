from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

EXIT_SUCCESS: int = 0
EXIT_FAILURE: int = 1

# Имена команд CLI
CMD_LIST: str = "list"
CMD_OBSTACLE: str = "obstacle"
CMD_ROUTE: str = "route"
CMD_CALIBRATE_STRAIGHT: str = "calibrate-straight"
CMD_CALIBRATE_TURN: str = "calibrate-turn"
CMD_CALIBRATE_SPEED: str = "calibrate-speed"

# Значения по умолчанию
DEFAULT_OBSTACLE_DISTANCE_CM: float = 200.0
DEFAULT_CALIBRATE_STRAIGHT_DISTANCE_CM: float = 150.0
DEFAULT_CALIBRATE_SPEED_DURATION_SEC: float = 3.0

# Пример файла маршрута для подсказки в help
EXAMPLE_ROUTE_FILE: str = "tests/routes/square_100.json"

# Текст справки для epilog
EPILOG: str = """Команды:
  obstacle                   тест торможения на прямой
  route <файл.json>          тест выполнения маршрута
  calibrate-straight [см]    калибровка прямолинейности
  calibrate-turn [сек]       калибровка поворота на 90°
  calibrate-speed [сек]      калибровка скорости
  list                       список всех тестов
"""

TESTS: dict[str, tuple[str, str]] = {
    CMD_OBSTACLE: ("tests.test_obstacle", "Торможение перед препятствием"),
    CMD_ROUTE: ("tests.test_route", "Выполнение маршрута"),
    CMD_CALIBRATE_STRAIGHT: ("tests.calibrate_straight", "Калибровка прямолинейности"),
    CMD_CALIBRATE_TURN: ("tests.calibrate_turn", "Калибровка поворота на 90°"),
    CMD_CALIBRATE_SPEED: ("tests.calibrate_speed", "Калибровка скорости (см/с)"),
}


def run_list() -> int:
    """Вывод списка тестов."""
    print("\nДоступные тесты и калибровка:\n")

    for cmd, (_, desc) in sorted(TESTS.items()):
        print(f"  {cmd:<22} — {desc}")

    print("\nПримеры:")
    print(f"  python -m tests {CMD_OBSTACLE}")
    print(f"  python -m tests {CMD_ROUTE} {EXAMPLE_ROUTE_FILE}")
    print(f"  python -m tests {CMD_CALIBRATE_STRAIGHT}")
    print(f"  python -m tests {CMD_CALIBRATE_TURN} 0.5")
    print(f"  python -m tests {CMD_CALIBRATE_SPEED} {int(DEFAULT_CALIBRATE_SPEED_DURATION_SEC)}")
    return EXIT_SUCCESS


def main() -> int:
    """Главная функция CLI."""
    parser: argparse.ArgumentParser = argparse.ArgumentParser(
        description="Тесты и калибровка машинки RaspTank",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=EPILOG,
    )

    parser.add_argument(
        "command",
        nargs="?",
        default=CMD_LIST,
        choices=list(TESTS.keys()) + [CMD_LIST],
        help="Команда для выполнения",
    )

    parser.add_argument(
        "args",
        nargs="*",
        help="Аргументы команды (расстояние, путь к файлу, длительность)",
    )

    parsed: argparse.Namespace = parser.parse_args()
    if parsed.command == CMD_LIST:
        return run_list()

    module_name, _ = TESTS[parsed.command]
    args: list[Any] = parsed.args

    try:
        import importlib

        mod: Any = importlib.import_module(module_name)
        run_fn: Any = getattr(mod, "run")

        if parsed.command == CMD_OBSTACLE:
            dist: float = float(args[0]) if args else DEFAULT_OBSTACLE_DISTANCE_CM
            return run_fn(distance_cm=dist)

        if parsed.command == CMD_ROUTE:
            if not args:
                print("Укажите путь к JSON-файлу маршрута", file=sys.stderr)
                return EXIT_FAILURE
            return run_fn(route_file=Path(args[0]))

        if parsed.command == CMD_CALIBRATE_STRAIGHT:
            dist: float = float(args[0]) if args else DEFAULT_CALIBRATE_STRAIGHT_DISTANCE_CM
            return run_fn(distance_cm=dist)

        if parsed.command == CMD_CALIBRATE_TURN:
            from src.config.settings import Settings

            dur: float = float(args[0]) if args else Settings().turn_duration_90_deg_sec
            return run_fn(duration_sec=dur)

        if parsed.command == CMD_CALIBRATE_SPEED:
            dur: float = float(args[0]) if args else DEFAULT_CALIBRATE_SPEED_DURATION_SEC
            return run_fn(duration_sec=dur)

        return EXIT_FAILURE

    except KeyboardInterrupt:
        print("\nПрервано пользователем.", file=sys.stderr)
        return EXIT_FAILURE

    except Exception as exc:
        print(f"Ошибка: {exc}", file=sys.stderr)
        return EXIT_FAILURE


if __name__ == "__main__":
    sys.exit(main())
