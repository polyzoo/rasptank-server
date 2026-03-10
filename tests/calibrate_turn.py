from __future__ import annotations

import sys
import time

from src.application.services.drive_controller import DriveController
from tests.common import create_drive_controller, prompt_float, prompt_yes_no

DEFAULT_DURATION_SEC: float = 0.5
TARGET_ANGLE_DEG: float = 90.0
ANGLE_TOLERANCE_DEG: float = 5.0
SLEEP_INTERVAL_SEC: float = 0.05
DESCRIPTION: str = "Калибровка поворота на 90°"
PROMPT_START_CALIBRATION: str = "Запустить калибровку?"
MESSAGE_ADD_TO_ENV: str = "Добавьте в .env:"


def run(duration_sec: float = DEFAULT_DURATION_SEC) -> int:
    """Запуск калибровки поворота."""
    print(f"\n=== {DESCRIPTION} ===\n")
    print("Машинка повернёт налево на заданное время.")
    print("Измерьте угол поворота (транспортир или фигура).")
    print()
    print("Текущая длительность берётся из .env (TURN_DURATION_90_DEG_SEC)")
    print(f"Используется: {duration_sec} с")
    print()

    if not prompt_yes_no(PROMPT_START_CALIBRATION):
        return 0

    drive: DriveController = create_drive_controller()
    try:
        drive.motor_controller.turn_left(speed_percent=drive.turn_speed_percent)

        start: float = time.monotonic()
        while (time.monotonic() - start) < duration_sec:
            time.sleep(SLEEP_INTERVAL_SEC)
        drive.motor_controller.stop()

        angle_deg: float = prompt_float(
            question=f"Угол поворота в градусах ({TARGET_ANGLE_DEG:.0f} = идеально)",
            default=TARGET_ANGLE_DEG,
        )
        if (TARGET_ANGLE_DEG - ANGLE_TOLERANCE_DEG) <= angle_deg <= (
            TARGET_ANGLE_DEG + ANGLE_TOLERANCE_DEG
        ):
            print("✓ Поворот ≈ 90°. Калибровка в норме.")
            return 0

        suggested: float = (
            duration_sec * TARGET_ANGLE_DEG / angle_deg if angle_deg else duration_sec
        )
        print(f"\nРекомендация: для 90° используйте duration_sec ≈ {suggested:.2f}")
        print(MESSAGE_ADD_TO_ENV)
        print(f"  TURN_DURATION_90_DEG_SEC={suggested:.2f}")
        print("Или укажите это значение в duration_sec при поворотах в маршруте.")
        return 0

    finally:
        drive.motor_controller.stop()
        drive.destroy()


if __name__ == "__main__":
    dur: float = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_DURATION_SEC
    sys.exit(run(duration_sec=dur))
