#!/usr/bin/env python3
import sys
from typing import List, Tuple

from src.application.services.drive_controller import DriveController
from src.config.settings import Settings
from src.infrastructures.motor import MotorController
from src.infrastructures.ultrasonic import UltrasonicSensor

DISTANCE_CM: float = 150.0
SPEED_PERCENT: int = 50

COMBINATIONS: List[Tuple[int, int]] = [
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
]


def run_one(m1: int, m2: int) -> None:
    """Один прогон с заданными направлениями."""
    MotorController.M1_DIRECTION = m1
    MotorController.M2_DIRECTION = m2

    motor = MotorController()
    ultrasonic = UltrasonicSensor()
    drive = DriveController(
        motor_controller=motor,
        ultrasonic_sensor=ultrasonic,
        min_obstacle_distance_cm=Settings().min_obstacle_distance_cm,
        deceleration_distance_cm=Settings().deceleration_distance_cm,
        base_speed_percent=Settings().base_speed_percent,
        update_interval_sec=Settings().update_interval_sec,
    )

    try:
        drive.forward_cm_sync(distance_cm=DISTANCE_CM, max_speed_percent=SPEED_PERCENT)
    finally:
        drive.destroy()


def main() -> int:
    print("\n=== Перебор направлений M1, M2 ===\n")
    print(f"Дистанция: {DISTANCE_CM} см, скорость: {SPEED_PERCENT}%")
    print()

    for i, (m1, m2) in enumerate(COMBINATIONS, 1):
        print(f"--- Комбинация {i}/4: M1_DIRECTION={m1}, M2_DIRECTION={m2} ---")
        print("Нажмите Enter для запуска...")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            print("\nВыход.")
            return 0

        run_one(m1, m2)
        print("Готово.")
        print("Введите 'y' если машинка поехала прямо — выход. Иначе Enter для следующей комбинации.")
        try:
            if input().strip().lower() in ("y", "д", "да"):
                print(f"\n✓ Рабочая комбинация: M1_DIRECTION={m1}, M2_DIRECTION={m2}")
                return 0
        except (EOFError, KeyboardInterrupt):
            pass
        print()

    print("Все 4 комбинации проверены.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
