from __future__ import annotations

import sys
import time

from src.config.settings import Settings
from tests.common import create_motor_controller, prompt_float, prompt_yes_no

DESCRIPTION: str = "Эксперимент 1.2: разворот на месте для оценки k_l и k_r"
PROMPT_START: str = "Запустить эксперимент?"
PROMPT_SPEEDS: str = "Введите скорости U через пробел (например: 20 40 60 80 100)"
PROMPT_DURATION_SEC: str = "Длительность одного прогона, с"
DEFAULT_SPEEDS: tuple[int, ...] = (20, 40, 60, 80, 100)
DEFAULT_DURATION_SEC: float = 4.0
SLEEP_INTERVAL_SEC: float = 0.02


def _print_instructions() -> None:
    print("\n=== " + DESCRIPTION + " ===\n")
    print("Что требуется:")
    print("1. Оставить вокруг робота свободное место.")
    print("2. Разместить робот на ровной горизонтальной поверхности.")
    print("3. Сравнить левый и правый разворот на одинаковых U.")
    print("4. При необходимости повторить прогон для обратного направления.")
    print()
    print("Этот скрипт запускает моторы напрямую через MotorController.")
    print("Используется разворот на месте: левая и правая гусеницы идут навстречу.")
    print("Цель: получить данные для сравнения k_l и k_r по угловой реакции.")
    print()


def _run_turn(direction: str, speed_percent: int, duration_sec: float) -> float:
    settings = Settings()
    motor = create_motor_controller(settings)
    try:
        print(f"{direction}: U={speed_percent}% на {duration_sec:.1f} с.")
        start: float = time.monotonic()
        if direction == "left":
            motor.turn_left(speed_percent=speed_percent)
        else:
            motor.turn_right(speed_percent=speed_percent)
        while (time.monotonic() - start) < duration_sec:
            time.sleep(SLEEP_INTERVAL_SEC)
        motor.stop()
        return time.monotonic() - start
    finally:
        motor.destroy()


def run() -> int:
    """Запуск эксперимента для сравнения лев/прав симметрии."""
    _print_instructions()

    if not prompt_yes_no(PROMPT_START):
        return 0

    raw_speeds: str = input(f"{PROMPT_SPEEDS} [{' '.join(map(str, DEFAULT_SPEEDS))}]: ").strip()
    if raw_speeds:
        speeds = tuple(int(part) for part in raw_speeds.split())
    else:
        speeds = DEFAULT_SPEEDS

    duration_sec: float = prompt_float(PROMPT_DURATION_SEC, default=DEFAULT_DURATION_SEC)
    if duration_sec <= 0:
        print("Ошибка: длительность должна быть положительной.", file=sys.stderr)
        return 1

    print()
    print("Таблица для записи:")
    print("U (%), direction, t_run (с), final_yaw_deg, комментарий")

    for speed_percent in speeds:
        for direction in ("left", "right"):
            print()
            print(f"Режим {direction.upper()}, U={speed_percent}%")
            elapsed = _run_turn(direction=direction, speed_percent=speed_percent, duration_sec=duration_sec)
            print(f"Фактическая длительность прогона: {elapsed:.2f} с")
            final_yaw = prompt_float("Финальный yaw по гироскопу (deg)", default=0.0)
            comment = input("Комментарий/наблюдение: ").strip()
            print(f"{speed_percent}, {direction}, {elapsed:.2f}, {final_yaw:.2f}, {comment}")

    print()
    print("Что дальше:")
    print("1. Сравнить final_yaw_deg для left и right при одинаковом U.")
    print("2. Если нужно, повторить с другим направлением или скоростью.")
    print("3. По необходимости использовать данные для оценки k_l и k_r.")
    return 0


if __name__ == "__main__":
    sys.exit(run())
