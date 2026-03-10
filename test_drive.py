#!/usr/bin/env python3
"""
Скрипт для тестирования машинки без веб-сервера.

Выполняет те же операции, что и REST API:
  forward  — аналог POST /v1/drive/forward.
  stop     — аналог POST /v1/drive/stop (сначала запуск, пауза, затем стоп — для проверки).
"""

import argparse
import sys
import time
from argparse import ArgumentParser, Action, Namespace
from typing import Callable, Dict, Optional

from src.application.services.drive_controller import DriveController
from src.config.settings import Settings
from src.infrastructures.motor import MotorController
from src.infrastructures.ultrasonic import UltrasonicSensor

# Константы валидации и поведения
DISTANCE_MIN_CM: float = 0.0
SPEED_MIN_PERCENT: int = 0
SPEED_MAX_PERCENT: int = 100
DEFAULT_STOP_DELAY_SEC: float = 2.0
TEST_STOP_DISTANCE_CM: float = 100.0

# Коды выхода
EXIT_SUCCESS: int = 0
EXIT_FAILURE: int = 1

Handler = Callable[[Namespace, DriveController], int]


def build_parser() -> ArgumentParser:
    """Создаёт и настраивает парсер аргументов командной строки."""
    parser: ArgumentParser = ArgumentParser(
        description="Тестирование машинки без веб-сервера",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    subparsers: Action = parser.add_subparsers(dest="command", required=True, help="Команда")

    # Команда 'forward'
    p_forward: Action = subparsers.add_parser("forward", help="Движение вперед")
    p_forward.add_argument("distance_cm", type=float, help="Расстояние в сантиметрах")
    p_forward.add_argument(
        "max_speed_percent",
        type=int,
        nargs="?",
        default=None,
        help=f"Максимальная скорость в процентах ({SPEED_MIN_PERCENT} – {SPEED_MAX_PERCENT})",
    )

    # Команда 'stop'
    p_stop: Action = subparsers.add_parser("stop", help="Остановка")
    p_stop.add_argument(
        "--delay",
        type=float,
        default=DEFAULT_STOP_DELAY_SEC,
        help="Пауза в секундах между запуском и остановкой",
    )

    return parser


def create_drive_controller(settings: Settings) -> DriveController:
    """Создаёт и настраивает экземпляр контроллера движения."""
    motor_controller: MotorController = MotorController()
    ultrasonic_sensor: UltrasonicSensor = UltrasonicSensor()
    return DriveController(
        motor_controller=motor_controller,
        ultrasonic_sensor=ultrasonic_sensor,
        min_obstacle_distance_cm=settings.min_obstacle_distance_cm,
        deceleration_distance_cm=settings.deceleration_distance_cm,
        base_speed_percent=settings.base_speed_percent,
        update_interval_sec=settings.update_interval_sec,
    )


def validate_forward_args(args: Namespace) -> bool:
    """Проверяет корректность аргументов команды 'forward'."""
    if args.distance_cm < DISTANCE_MIN_CM:
        print("Ошибка: расстояние должно быть положительным.", file=sys.stderr)
        return False

    speed_percent: Optional[int] = args.max_speed_percent

    if speed_percent is not None and not (SPEED_MIN_PERCENT <= speed_percent <= SPEED_MAX_PERCENT):
        print("Ошибка: скорость должна соответствовать диапазону.", file=sys.stderr)
        return False

    return True


def handle_forward(args: Namespace, drive_controller: DriveController) -> int:
    """Обработчик команды 'forward'."""
    if not validate_forward_args(args):
        return EXIT_FAILURE

    drive_controller.forward_cm(
        distance_cm=args.distance_cm,
        max_speed_percent=args.max_speed_percent,
    )

    speed_percent: Optional[int] = args.max_speed_percent
    speed_str: str = f", max_speed_percent={speed_percent}" if speed_percent is not None else ""
    print(f"forward(distance_cm={args.distance_cm}{speed_str}) — выполнено.")
    return EXIT_SUCCESS


def handle_stop(args: Namespace, drive_controller: DriveController) -> int:
    """Обработчик команды 'stop'."""
    drive_controller.forward_cm(distance_cm=TEST_STOP_DISTANCE_CM)
    print(f"forward({TEST_STOP_DISTANCE_CM}) запущено, ожидание {args.delay} с...")
    time.sleep(args.delay)
    drive_controller.stop()
    print("stop() — выполнено.")
    return EXIT_SUCCESS


def main() -> int:
    """Точка входа скрипта для тестирования машинки без веб-сервера."""
    parser: ArgumentParser = build_parser()
    args: Namespace = parser.parse_args()

    settings: Settings = Settings()
    drive_controller: DriveController = create_drive_controller(settings=settings)

    handlers: Dict[str, Handler] = {
        "forward": handle_forward,
        "stop": handle_stop,
    }

    handler: Optional[Handler] = handlers.get(args.command)
    if handler is None:
        print(f"Неизвестная команда: {args.command}", file=sys.stderr)
        return EXIT_FAILURE

    try:
        return handler(args, drive_controller)
    except KeyboardInterrupt:
        print("\nПрервано пользователем.", file=sys.stderr)
        return EXIT_FAILURE
    finally:
        drive_controller.destroy()


if __name__ == "__main__":
    sys.exit(main())
