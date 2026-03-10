from __future__ import annotations

import sys
import time

from src.application.services.drive_controller import DriveController
from tests.common import create_drive_controller, prompt_float, prompt_yes_no

DEFAULT_DURATION_SEC: float = 3.0
SLEEP_INTERVAL_SEC: float = 0.1
DESCRIPTION: str = "Калибровка скорости"
PROMPT_START_CALIBRATION: str = "Запустить калибровку?"
PROMPT_DISTANCE_ACTUAL: str = "Реальное пройденное расстояние (см)"
ERROR_DISTANCE_NON_POSITIVE: str = "Ошибка: расстояние должно быть положительным."
MESSAGE_ADD_TO_ENV: str = "Добавьте в .env:"


def run(duration_sec: float = DEFAULT_DURATION_SEC) -> int:
    """Запуск калибровки скорости."""
    print(f"\n=== {DESCRIPTION} ===\n")
    print("Машинка поедет вперёд заданное время на полной скорости.")
    print("Измерьте реальное пройденное расстояние (см).")
    print()

    if not prompt_yes_no(PROMPT_START_CALIBRATION):
        return 0

    drive: DriveController = create_drive_controller()
    try:
        drive.forward_cm(distance_cm=9999.0, max_speed_percent=100)
        print(f"Движение {duration_sec} с. Ожидание...")

        start: float = time.monotonic()
        while drive.is_moving and (time.monotonic() - start) < duration_sec:
            time.sleep(SLEEP_INTERVAL_SEC)

        drive.stop()
        elapsed: float = time.monotonic() - start
        print(f"Остановка. Время: {elapsed:.1f} с")

        actual_distance_cm: float = prompt_float(PROMPT_DISTANCE_ACTUAL)
        if actual_distance_cm <= 0:
            print(ERROR_DISTANCE_NON_POSITIVE, file=sys.stderr)
            return 1

        suggested: float = actual_distance_cm / elapsed
        print(f"\nРекомендация: MAX_SPEED_CM_PER_SEC ≈ {suggested:.1f}")
        print(MESSAGE_ADD_TO_ENV)
        print(f"  MAX_SPEED_CM_PER_SEC={suggested:.1f}")
        return 0

    finally:
        drive.destroy()


if __name__ == "__main__":
    dur: float = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_DURATION_SEC
    sys.exit(run(duration_sec=dur))
