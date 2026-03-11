#!/usr/bin/env python3
"""
Проверка движения назад с фиксированными направлениями M1=1, M2=-1.
"""
from __future__ import annotations

import sys
import time

from src.infrastructures.motor import MotorController

DURATION_SEC: float = 5.0
SPEED_PERCENT: int = 50

# Те же направления, что и для вперёд
M1_DIRECTION: int = 1
M2_DIRECTION: int = -1


def main() -> int:
    MotorController.M1_DIRECTION = M1_DIRECTION
    MotorController.M2_DIRECTION = M2_DIRECTION

    motor = MotorController()
    try:
        print(f"\nНазад {DURATION_SEC} с, {SPEED_PERCENT}%, M1={M1_DIRECTION} M2={M2_DIRECTION}\n")
        motor.move_backward(speed_percent=SPEED_PERCENT)
        time.sleep(DURATION_SEC)
        print("Готово.")
        return 0
    finally:
        motor.stop()
        motor.destroy()


if __name__ == "__main__":
    sys.exit(main())
