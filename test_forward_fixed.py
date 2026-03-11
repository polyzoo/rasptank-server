#!/usr/bin/env python3
from __future__ import annotations

import os
import sys

os.environ.setdefault("GPIOZERO_PIN_FACTORY", "rpigpio")

from src.application.services.drive_controller import DriveController
from src.config.settings import Settings
from src.infrastructures.motor import MotorController
from src.infrastructures.ultrasonic import UltrasonicSensor

DISTANCE_CM: float = 80.0
SPEED_PERCENT: int = 50

# Фиксированные направления (машинка поехала прямо при этой комбинации)
M1_DIRECTION: int = 1
M2_DIRECTION: int = -1


def main() -> int:
    MotorController.M1_DIRECTION = M1_DIRECTION
    MotorController.M2_DIRECTION = M2_DIRECTION

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

    print(f"\nВперёд {DISTANCE_CM} см, {SPEED_PERCENT}%, M1={M1_DIRECTION} M2={M2_DIRECTION}\n")

    try:
        drive.forward_cm_sync(distance_cm=DISTANCE_CM, max_speed_percent=SPEED_PERCENT)
        print("Готово.")
        return 0
    finally:
        drive.destroy()


if __name__ == "__main__":
    sys.exit(main())
