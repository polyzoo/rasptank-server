from __future__ import annotations

import sys
import time

from src.application.services.drive_controller import DriveController
from tests.common import create_drive_controller, prompt_float, prompt_yes_no

DEFAULT_DISTANCE_CM: float = 150.0
MAX_ALLOWED_DEVIATION_CM: float = 5.0
SLEEP_INTERVAL_SEC: float = 0.2
DESCRIPTION: str = "Калибровка прямолинейности"
TEXT_LINE_1: str = "Машинка проедет по прямой. Измерьте отклонение от линии."
TEXT_LINE_2: str = "Если уводит вправо — увеличьте TL_LEFT_OFFSET или уменьшите TL_RIGHT_OFFSET."
TEXT_LINE_3: str = "Если уводит влево — наоборот."
TEXT_LINE_4: str = "Текущие значения берутся из .env (TL_LEFT_OFFSET, TL_RIGHT_OFFSET)"
PROMPT_START_CALIBRATION: str = "Запустить калибровку?"
PROMPT_DEVIATION: str = "Отклонение от линии в см (положительное = вправо, отрицательное = влево)"
MESSAGE_OFFSET_PREFIX: str = "Рекомендация: машинка ушла {direction} на {value:.0f} см."
MESSAGE_ADD_TO_ENV: str = "Добавьте в .env:"
OFFSET_STEP: int = 5
MESSAGE_REPEAT_CALIBRATION: str = "Повторите калибровку после изменения."


def run(distance_cm: float = DEFAULT_DISTANCE_CM) -> int:
    """Запуск калибровки прямолинейности."""
    print(f"\n=== {DESCRIPTION} ===\n")
    print(TEXT_LINE_1)
    print(TEXT_LINE_2)
    print(TEXT_LINE_3)
    print()
    print(TEXT_LINE_4)
    print()

    if not prompt_yes_no(PROMPT_START_CALIBRATION):
        return 0

    drive: DriveController = create_drive_controller()
    try:
        drive.forward_cm(distance_cm=distance_cm)

        print(f"Движение на {distance_cm} см. Ожидание...")
        while drive.is_moving:
            time.sleep(SLEEP_INTERVAL_SEC)

        print("Остановка.")

        deviation_cm: float = prompt_float(
            question=PROMPT_DEVIATION,
            default=0.0,
        )
        if abs(deviation_cm) <= MAX_ALLOWED_DEVIATION_CM:
            print(f"✓ Отклонение ≤ {MAX_ALLOWED_DEVIATION_CM} см. Калибровка в норме.")
            return 0

        direction: str = "вправо" if deviation_cm > 0 else "влево"
        print("\n" + MESSAGE_OFFSET_PREFIX.format(direction=direction, value=abs(deviation_cm)))

        print(MESSAGE_ADD_TO_ENV)
        if deviation_cm > 0:
            print(f"  TL_LEFT_OFFSET={OFFSET_STEP}   # или увеличьте на {OFFSET_STEP}")
            print(f"  # или TL_RIGHT_OFFSET={-OFFSET_STEP}")
        else:
            print(f"  TL_RIGHT_OFFSET={OFFSET_STEP}  # или увеличьте на {OFFSET_STEP}")
            print(f"  # или TL_LEFT_OFFSET={-OFFSET_STEP}")

        print(MESSAGE_REPEAT_CALIBRATION)
        return 0

    finally:
        drive.destroy()


if __name__ == "__main__":
    dist: float = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_DISTANCE_CM
    sys.exit(run(distance_cm=dist))
