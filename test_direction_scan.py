#!/usr/bin/env python3
from __future__ import annotations
import sys
from typing import Iterator

from src.application.services.drive_controller import DriveController
from src.config.settings import Settings
from src.infrastructures.motor import MotorController
from src.infrastructures.ultrasonic import UltrasonicSensor

DISTANCE_CM: float = 50.0
SPEED_PERCENT: int = 50

CHANNEL_CONFIGS: list[tuple[int, int, int, int]] = [
    (8, 9, 10, 11),
    (9, 8, 10, 11),
    (8, 9, 11, 10),
    (9, 8, 11, 10),
    (10, 11, 8, 9),
    (11, 10, 8, 9),
    (10, 11, 9, 8),
    (11, 10, 9, 8),
]

DIRECTION_CONFIGS: list[tuple[int, int]] = [
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
]


def iter_combinations() -> Iterator[tuple[tuple[int, int, int, int], tuple[int, int], int]]:
    n: int = 0
    for ch in CHANNEL_CONFIGS:
        for d in DIRECTION_CONFIGS:
            n += 1
            yield ch, d, n


def run_one(m1_in1: int, m1_in2: int, m2_in1: int, m2_in2: int, m1_dir: int, m2_dir: int) -> None:
    MotorController.MOTOR_M1_IN1 = m1_in1
    MotorController.MOTOR_M1_IN2 = m1_in2
    MotorController.MOTOR_M2_IN1 = m2_in1
    MotorController.MOTOR_M2_IN2 = m2_in2
    MotorController.M1_DIRECTION = m1_dir
    MotorController.M2_DIRECTION = m2_dir

    motor: MotorController = MotorController()
    ultrasonic: UltrasonicSensor = UltrasonicSensor()
    drive: DriveController = DriveController(
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
    total: int = len(CHANNEL_CONFIGS) * len(DIRECTION_CONFIGS)

    print("\n=== Перебор каналов и направлений ===\n")
    print(f"Дистанция: {DISTANCE_CM} см, скорость: {SPEED_PERCENT}%")
    print(f"Всего комбинаций: {total}")
    print()

    for (m1_in1, m1_in2, m2_in1, m2_in2), (m1_dir, m2_dir), n in iter_combinations():
        progress: str = (
            f"--- {n}/{total}: M1(ch{m1_in1},{m1_in2}) "
            f"M2(ch{m2_in1},{m2_in2}) dir=({m1_dir},{m2_dir}) ---"
        )

        print(progress)
        print("Нажмите Enter для запуска...")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            print("\nВыход.")
            return 0

        run_one(m1_in1, m1_in2, m2_in1, m2_in2, m1_dir, m2_dir)
        print("Готово.")

        print("Введите 'y' если машинка поехала прямо — выход. Иначе Enter для следующей.")
        try:
            if input().strip().lower() in ("y", "д", "да"):
                print(f"\nРабочая комбинация:")
                print(f"  MOTOR_M1_IN1 = {m1_in1}")
                print(f"  MOTOR_M1_IN2 = {m1_in2}")
                print(f"  MOTOR_M2_IN1 = {m2_in1}")
                print(f"  MOTOR_M2_IN2 = {m2_in2}")
                print(f"  M1_DIRECTION = {m1_dir}")
                print(f"  M2_DIRECTION = {m2_dir}")
                return 0
        except (EOFError, KeyboardInterrupt):
            pass
        print()

    print("Все комбинации проверены.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
