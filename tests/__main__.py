from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

EXIT_SUCCESS: int = 0
EXIT_FAILURE: int = 1

# Имена команд CLI
CMD_LIST: str = "list"
CMD_ROUTE: str = "route"
CMD_CALIBRATE_SPEED: str = "calibrate-speed"
CMD_STATIC_VELOCITY: str = "static-velocity"
CMD_TIME_CONSTANT: str = "time-constant"
CMD_KL_KR_TURN: str = "kl-kr-turn"

# Значения по умолчанию
DEFAULT_CALIBRATE_SPEED_DURATION_SEC: float = 3.0

# Пример файла маршрута для подсказки в help
EXAMPLE_ROUTE_FILE: str = "tests/routes/square_40.json"

# Текст справки для epilog
EPILOG: str = """Команды:
  route <файл.json> [%]      тест выполнения маршрута (опц.: ограничение скорости)
  calibrate-speed [сек]      калибровка скорости
  list                       список всех тестов
"""

TESTS: dict[str, tuple[str, str]] = {
    CMD_ROUTE: ("tests.test_route", "Выполнение маршрута"),
    CMD_CALIBRATE_SPEED: ("tests.calibrate_speed", "Калибровка скорости (см/с)"),
    CMD_STATIC_VELOCITY: ("tests.experiment_static_velocity", "Эксперимент v(U)"),
    CMD_TIME_CONSTANT: ("tests.experiment_time_constant", "Эксперимент T"),
    CMD_KL_KR_TURN: ("tests.experiment_kl_kr_turn_in_place", "Эксперимент k_l/k_r"),
}


def run_list() -> int:
    """Вывод списка тестов."""
    print("\nДоступные тесты и калибровка:\n")

    for cmd, (_, desc) in sorted(TESTS.items()):
        print(f"  {cmd:<22} — {desc}")

    print("\nПримеры:")
    print(f"  python -m tests {CMD_ROUTE} {EXAMPLE_ROUTE_FILE}")
    print(f"  python -m tests {CMD_CALIBRATE_SPEED} {int(DEFAULT_CALIBRATE_SPEED_DURATION_SEC)}")
    print(f"  python -m tests {CMD_STATIC_VELOCITY} 50 5")
    print(f"  python -m tests {CMD_TIME_CONSTANT} 60 5")
    print(f"  python -m tests {CMD_KL_KR_TURN}")
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
        help="Аргументы команды (путь к файлу, расстояние, длительность)",
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
        return _run_command(parsed.command, args, run_fn)

    except KeyboardInterrupt:
        print("\nПрервано пользователем.", file=sys.stderr)
        return EXIT_FAILURE

    except Exception as exc:
        print(f"Ошибка: {exc}", file=sys.stderr)
        return EXIT_FAILURE


def _run_command(command: str, args: list[Any], run_fn: Any) -> int:
    """Преобразует CLI-аргументы нужной команды и вызывает её run()."""
    if command == CMD_ROUTE:
        return _run_route_command(args, run_fn)

    if command == CMD_CALIBRATE_SPEED:
        duration_sec: float = float(args[0]) if args else DEFAULT_CALIBRATE_SPEED_DURATION_SEC
        return run_fn(duration_sec=duration_sec)

    if command == CMD_STATIC_VELOCITY:
        speed_percent: int = int(args[0]) if args else 50
        duration_sec: float = float(args[1]) if len(args) > 1 else 5.0
        return run_fn(speed_percent=speed_percent, duration_sec=duration_sec)

    if command == CMD_TIME_CONSTANT:
        speed_percent: int = int(args[0]) if args else 60
        duration_sec: float = float(args[1]) if len(args) > 1 else 5.0
        return run_fn(speed_percent=speed_percent, duration_sec=duration_sec)

    if command == CMD_KL_KR_TURN:
        return run_fn()

    return EXIT_FAILURE


def _run_route_command(args: list[Any], run_fn: Any) -> int:
    """Запускает CLI-команду route."""
    if not args:
        print("Укажите путь к JSON-файлу маршрута", file=sys.stderr)
        return EXIT_FAILURE

    route_path: Path = Path(args[0])
    speed: int | None = int(args[1]) if len(args) > 1 else None
    return run_fn(route_file=route_path, max_speed_percent=speed)


if __name__ == "__main__":
    sys.exit(main())
