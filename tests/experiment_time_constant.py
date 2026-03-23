from __future__ import annotations

import sys
import time

from src.config.settings import Settings
from tests.common import create_motor_controller, prompt_float, prompt_yes_no

DESCRIPTION: str = "Эксперимент 2: определение постоянной времени T"
PROMPT_START: str = "Запустить ступеньку управления?"
PROMPT_SPEED_PERCENT: str = "U для ступеньки, %"
PROMPT_DURATION_SEC: str = "Длительность ступеньки, с"
PROMPT_REPEAT_COUNT: str = "Количество повторов"
DEFAULT_SPEED_PERCENT: int = 60
DEFAULT_DURATION_SEC: float = 5.0
DEFAULT_REPEAT_COUNT: int = 5
SLEEP_INTERVAL_SEC: float = 0.02


def _print_instructions() -> None:
    print("\n=== " + DESCRIPTION + " ===\n")
    print("Что требуется:")
    print("1. Платформа стоит неподвижно на ровной горизонтальной поверхности.")
    print("2. Запустить логирование акселерометра с частотой не ниже 50 Гц.")
    print("3. В момент старта резко подать одинаковое U на оба борта.")
    print("4. Логировать 3-5 секунд, пока ускорение не станет близким к нулю.")
    print("5. Повторить эксперимент 5 раз для одного и того же U.")
    print()
    print("Этот скрипт запускает только моторы напрямую через MotorController.")
    print("CSV акселерометра нужно снимать отдельным процессом/скриптом.")
    print("Формат CSV: time_ms, accel_x, accel_y, accel_z")
    print()


def _run_single_step(speed_percent: int, duration_sec: float) -> float:
    settings = Settings()
    motor = create_motor_controller(settings)
    try:
        print(f"Подаём ступеньку U={speed_percent}% на оба борта на {duration_sec:.1f} с.")
        start: float = time.monotonic()
        motor.move_forward(speed_percent=speed_percent)
        while (time.monotonic() - start) < duration_sec:
            time.sleep(SLEEP_INTERVAL_SEC)
        motor.stop()
        return time.monotonic() - start
    finally:
        motor.destroy()


def run() -> int:
    """Запуск эксперимента по оценке постоянной времени T."""
    _print_instructions()

    if not prompt_yes_no(PROMPT_START):
        return 0

    speed_percent: int = int(prompt_float(PROMPT_SPEED_PERCENT, default=DEFAULT_SPEED_PERCENT))
    duration_sec: float = prompt_float(PROMPT_DURATION_SEC, default=DEFAULT_DURATION_SEC)
    repeat_count: int = int(prompt_float(PROMPT_REPEAT_COUNT, default=DEFAULT_REPEAT_COUNT))

    if repeat_count <= 0:
        print("Ошибка: количество повторов должно быть положительным.", file=sys.stderr)
        return 1

    print()
    print("Порядок работы:")
    print("1. Запустите внешний логгер акселерометра.")
    print("2. Затем выполните этот прогон.")
    print("3. После каждого прогона сохраните CSV в отдельный файл.")
    print()

    for run_idx in range(repeat_count):
        print(f"Повтор {run_idx + 1}/{repeat_count}")
        measured = _run_single_step(speed_percent=speed_percent, duration_sec=duration_sec)
        print(f"Остановка через {measured:.2f} с.")
        print("Сохраните этот CSV и потом оцените T по моменту достижения 0.63 vжел.")
        print()

    print("Что дальше:")
    print("1. Отфильтровать accel_x скользящим средним.")
    print("2. Вычесть нулевой дрейф по первым 0.5 с.")
    print("3. Интегрировать ускорение методом трапеций.")
    print("4. Подогнать кривую к vжел и найти момент 0.63 vжел.")
    return 0


if __name__ == "__main__":
    sys.exit(run())
