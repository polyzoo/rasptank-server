from __future__ import annotations

import sys
import time

from src.application.services.drive_controller import DriveController
from tests.common import create_drive_controller, prompt_yes_no

DEFAULT_DISTANCE_CM: float = 200.0
MIN_OBSTACLE_DISTANCE_CM: float = 20.0
MIN_START_DISTANCE_CM: float = 50.0
SLEEP_INTERVAL_SEC: float = 0.2
DESCRIPTION: str = "Торможение перед препятствием"
PROMPT_START: str = "Запустить тест?"
PROMPT_RESULT: str = (
    "Машинка плавно остановилась на расстоянии "
    f"≥ {MIN_OBSTACLE_DISTANCE_CM} см от препятствия?"
)
ERROR_OBSTACLE_CONFIG: str = (
    "✗ Тест не пройден. Проверьте MIN_OBSTACLE_DISTANCE_CM, DECELERATION_DISTANCE_CM"
)


def run(distance_cm: float = DEFAULT_DISTANCE_CM) -> int:
    """Запуск теста торможения перед препятствием."""
    print(f"\n=== {DESCRIPTION} ===\n")
    print(f"Установите препятствие на пути машинки (≥ {MIN_START_DISTANCE_CM} см от старта).")
    print(f"Машинка поедет вперёд до {distance_cm} см или до препятствия.")
    print(f"Ожидается: плавная остановка на расстоянии ≥ {MIN_OBSTACLE_DISTANCE_CM} см.")
    print()

    if not prompt_yes_no(PROMPT_START):
        return 0

    drive: DriveController = create_drive_controller()
    try:
        drive.forward_cm(distance_cm=distance_cm)
        print("Движение запущено. Ожидание завершения...")

        while drive.is_moving:
            time.sleep(SLEEP_INTERVAL_SEC)
        print("Остановка.")

        if prompt_yes_no(PROMPT_RESULT):
            print("✓ Тест пройден.")
            return 0
        print(ERROR_OBSTACLE_CONFIG)
        return 1

    finally:
        drive.destroy()


if __name__ == "__main__":
    dist: float = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_DISTANCE_CM
    sys.exit(run(distance_cm=dist))
