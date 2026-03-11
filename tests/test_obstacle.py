from __future__ import annotations

import sys

from src.application.services.drive_controller import DriveController
from tests.common import create_drive_controller, prompt_yes_no

DEFAULT_DISTANCE_CM: float = 200.0
MIN_OBSTACLE_DISTANCE_CM: float = 20.0
MIN_START_DISTANCE_CM: float = 50.0
DESCRIPTION: str = "Торможение перед препятствием"
PROMPT_START: str = "Запустить тест?"
PROMPT_RESULT: str = (
    "Машинка плавно остановилась на расстоянии "
    f"≥ {MIN_OBSTACLE_DISTANCE_CM} см от препятствия?"
)
ERROR_OBSTACLE_CONFIG: str = (
    "✗ Тест не пройден. Проверьте MIN_OBSTACLE_DISTANCE_CM, DECELERATION_DISTANCE_CM"
)


def run(distance_cm: float = DEFAULT_DISTANCE_CM, max_speed_percent: int | None = None) -> int:
    """Запуск теста торможения перед препятствием."""
    print(f"\n=== {DESCRIPTION} ===\n")
    print(f"Установите препятствие на пути машинки (≥ {MIN_START_DISTANCE_CM} см от старта).")
    print(f"Машинка поедет вперёд до {distance_cm} см или до препятствия.")
    if max_speed_percent is not None:
        print(f"Ограничение скорости: {max_speed_percent}%.")
    print(f"Ожидается: плавная остановка на расстоянии ≥ {MIN_OBSTACLE_DISTANCE_CM} см.")
    print()

    if not prompt_yes_no(PROMPT_START):
        return 0

    drive: DriveController = create_drive_controller()
    try:
        print("Движение запущено. Ожидание завершения...")

        drive.forward_cm_sync(
            distance_cm=distance_cm,
            max_speed_percent=max_speed_percent,
        )
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
    speed: int | None = int(sys.argv[2]) if len(sys.argv) > 2 else None
    sys.exit(run(distance_cm=dist, max_speed_percent=speed))
