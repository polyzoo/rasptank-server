from __future__ import annotations

import sys
import time
from dataclasses import dataclass

from src.config.settings import Settings
from tests.common import create_motor_controller, prompt_float, prompt_yes_no

DESCRIPTION: str = "Эксперимент 1: статическая характеристика v(U)"
PROMPT_START: str = "Запустить эксперимент?"
PROMPT_DISTANCE_M: str = "Длина прямой L, м"
PROMPT_VALUES: str = "Введите U в процентах через пробел (например: 20 40 60 80 100)"
PROMPT_REPEAT_COUNT: str = "Количество повторов на каждое U"
PROMPT_T1: str = "Время t1 для U={u}% (с)"
PROMPT_T2: str = "Время t2 для U={u}% (с)"
PROMPT_T3: str = "Время t3 для U={u}% (с)"
RESULT_HEADER: str = "Таблица для записи:"
DEFAULT_DISTANCE_M: float = 5.0
DEFAULT_REPEAT_COUNT: int = 3
DEFAULT_SPEEDS: tuple[int, ...] = (20, 40, 60, 80, 100)
DEFAULT_RUN_TIME_SEC: float = 8.0
SLEEP_INTERVAL_SEC: float = 0.05


@dataclass(frozen=True)
class MeasurementRow:
    """Строка результатов одного режима управления."""

    u_percent: int
    distance_m: float
    t1: float
    t2: float
    t3: float

    @property
    def t_avg(self) -> float:
        return (self.t1 + self.t2 + self.t3) / 3.0

    @property
    def v_ust(self) -> float:
        return self.distance_m / self.t_avg if self.t_avg > 0 else 0.0


def _print_instructions() -> None:
    print("\n=== " + DESCRIPTION + " ===\n")
    print("Что требуется:")
    print("1. Разметить прямую дистанцию длиной минимум 5 м.")
    print("2. Перед стартом оставить небольшой участок разгона.")
    print("3. Для каждого U подать одно и то же значение на оба борта.")
    print("4. Измерить время проезда L три раза и усреднить.")
    print("5. По формуле vуст = L / t_avg получить установившуюся скорость.")
    print()
    print("Запуск идёт напрямую через MotorController, без DriveController.")
    print("После каждого прогона моторы автоматически останавливаются.")
    print()


def _run_single_measurement(speed_percent: int, duration_sec: float) -> float:
    settings = Settings()
    motor = create_motor_controller(settings)
    try:
        print(f"Подаём U={speed_percent}% на оба борта на {duration_sec:.1f} с.")
        start: float = time.monotonic()
        motor.move_forward(speed_percent=speed_percent)
        while (time.monotonic() - start) < duration_sec:
            time.sleep(SLEEP_INTERVAL_SEC)
        motor.stop()
        return time.monotonic() - start
    finally:
        motor.destroy()


def run() -> int:
    """Запуск эксперимента по статической характеристике v(U)."""
    _print_instructions()

    if not prompt_yes_no(PROMPT_START):
        return 0

    distance_m: float = prompt_float(PROMPT_DISTANCE_M, default=DEFAULT_DISTANCE_M)
    if distance_m <= 0:
        print("Ошибка: L должна быть положительной.", file=sys.stderr)
        return 1

    raw_values: str = input(f"{PROMPT_VALUES} [{ ' '.join(map(str, DEFAULT_SPEEDS)) }]: ").strip()
    if raw_values:
        speeds = tuple(int(part) for part in raw_values.split())
    else:
        speeds = DEFAULT_SPEEDS

    repeat_count: int = int(prompt_float(PROMPT_REPEAT_COUNT, default=DEFAULT_REPEAT_COUNT))
    if repeat_count <= 0:
        print("Ошибка: количество повторов должно быть положительным.", file=sys.stderr)
        return 1

    rows: list[MeasurementRow] = []
    print()
    print(RESULT_HEADER)
    print("U (%), L (м), t1, t2, t3 (с), t_avg (с), v_уст (м/с)")

    for u_percent in speeds:
        times: list[float] = []
        for repeat_idx in range(repeat_count):
            print(f"\nРежим U={u_percent}%, повтор {repeat_idx + 1}/{repeat_count}")
            measured_time = _run_single_measurement(
                speed_percent=u_percent,
                duration_sec=DEFAULT_RUN_TIME_SEC,
            )
            print(f"Фактическая длительность управления: {measured_time:.2f} с")
            times.append(prompt_float(PROMPT_T1.format(u=u_percent), default=measured_time))

        while len(times) < 3:
            times.append(times[-1])

        row = MeasurementRow(
            u_percent=u_percent,
            distance_m=distance_m,
            t1=times[0],
            t2=times[1],
            t3=times[2],
        )
        rows.append(row)
        print(f"{row.u_percent}, {row.distance_m:.2f}, {row.t1:.2f}, {row.t2:.2f}, {row.t3:.2f}, {row.t_avg:.2f}, {row.v_ust:.3f}")

    print("\nЧто дальше:")
    print("1. Построить график vжел(U) по столбцу v_уст.")
    print("2. Если нужно, принять k = v_уст / U для линейной аппроксимации.")
    print("3. Для эксперимента с T выбрать одно U из средней области (50-70%).")
    return 0


if __name__ == "__main__":
    sys.exit(run())
