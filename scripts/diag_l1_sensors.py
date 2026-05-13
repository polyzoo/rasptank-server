#!/usr/bin/env python3
"""Проверка датчиков L1 (MPU6050 + HC-SR04) напрямую, без HTTP.

Зачем: ``GET /v1/l1/state`` каждый раз вызывает полное ``read_sensors()``, в том числе
ультразвуковое измерение на GPIO. Эндпоинты ``/v1/l2/state`` и ``/v1/l3/state`` почти
всегда отвечают из памяти и не ждут нового замера УЗ — поэтому они могут «работать»,
а L1 — зависать на HC-SR04 (echo/trigger, общий lock с фоновым контуром).

ВАЖНО: на Raspberry Pi остановите сервер rasptank-server перед запуском — два процесса
не должны одновременно открывать I²C и те же GPIO.

Пример (на машинке, из корня репозитория)::

    python scripts/diag_l1_sensors.py

С таймаутом на одно измерение УЗ (по умолчанию 5 с)::

    python scripts/diag_l1_sensors.py --us-timeout-sec 5

Если датчик вешали на другие GPIO (нумерация BCM)::

    python scripts/diag_l1_sensors.py --us-trigger 17 --us-echo 27

Перебор нескольких пар без правки кода удобнее так::

    python scripts/probe_ultrasonic_pins.py 23 24 17 27

Путь API без reverse proxy: ``/v1/l1/state``, не ``/api/v1/...``.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
from collections.abc import Callable  # noqa: E402
from concurrent.futures import ThreadPoolExecutor  # noqa: E402
from typing import TypeVar  # noqa: E402

from src.application.factories import create_shared_motion_hardware  # noqa: E402
from src.application.services.l1_service import L1Service  # noqa: E402
from src.config.settings import Settings  # noqa: E402
from src.infrastructures.ultrasonic import UltrasonicSensor  # noqa: E402

T = TypeVar("T")


def _run_with_timeout(fn: Callable[[], T], timeout_sec: float) -> T:
    """Выполнить ``fn`` в отдельном потоке с дедлайном.

    Нельзя использовать ``with ThreadPoolExecutor``: при ``TimeoutError`` worker
    часто остаётся внутри gpiozero (ожидание ECHO). Тогда ``__exit__`` вызывает
    ``shutdown(wait=True)`` и процесс зависает навсегда на ``join`` этого потока.
    """
    executor = ThreadPoolExecutor(max_workers=1)
    future = executor.submit(fn)
    try:
        return future.result(timeout=timeout_sec)
    finally:
        # future не done → не ждём зависший GPIO; поток умрёт с процессом скрипта.
        executor.shutdown(wait=future.done())


def _ms_durations(fn: Callable[[], None], repeats: int) -> list[float]:
    rows: list[float] = []
    for _ in range(repeats):
        t0 = time.perf_counter()
        fn()
        rows.append((time.perf_counter() - t0) * 1000.0)
    return rows


def _print_stats(label: str, ms: list[float]) -> None:
    if not ms:
        print(f"{label}: (no samples)")
        return
    print(
        f"{label}: n={len(ms)}  mean={statistics.mean(ms):.1f} ms  "
        f"min={min(ms):.1f} ms  max={max(ms):.1f} ms",
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Замер времени чтения IMU, HC-SR04 и полного цикла L1.read_sensors.",
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Выполнить калибровку IMU при старте (~1 с и дольше).",
    )
    parser.add_argument(
        "--imu-warmup-sec",
        type=float,
        default=0.25,
        help="Пауза после старта IMU.",
    )
    parser.add_argument("--repeat", type=int, default=5, help="Сколько раз повторить каждый замер.")
    parser.add_argument(
        "--us-timeout-sec",
        type=float,
        default=5.0,
        help="Таймаут на одно измерение HC-SR04 (сек); при превышении — ошибка по строке.",
    )
    parser.add_argument(
        "--us-trigger",
        type=int,
        default=UltrasonicSensor.TRIGGER_PIN,
        metavar="BCM",
        help="BCM номер TRIG HC-SR04 (по умолчанию как в ultrasonic.py).",
    )
    parser.add_argument(
        "--us-echo",
        type=int,
        default=UltrasonicSensor.ECHO_PIN,
        metavar="BCM",
        help="BCM номер ECHO HC-SR04 (по умолчанию как в ultrasonic.py).",
    )
    args = parser.parse_args()
    if args.repeat < 1:
        print("--repeat must be >= 1", file=sys.stderr)
        return 2

    UltrasonicSensor.TRIGGER_PIN = args.us_trigger
    UltrasonicSensor.ECHO_PIN = args.us_echo
    print(f"HC-SR04 (BCM): trigger={args.us_trigger} echo={args.us_echo}\n")

    settings = Settings()
    hardware = create_shared_motion_hardware(settings)
    l1 = L1Service(
        motor_controller=hardware.motor_controller,
        gyroscope=hardware.gyroscope,
        ultrasonic_sensor=hardware.ultrasonic_sensor,
        head_servo=hardware.head_servo,
    )

    print("Старт IMU...")
    hardware.gyroscope.start(calibrate=args.calibrate)
    time.sleep(max(0.0, args.imu_warmup_sec))

    imu_ms = _ms_durations(
        lambda: (
            hardware.gyroscope.get_acceleration_xyz_m_s2(),
            hardware.gyroscope.get_angular_speed_z_deg_per_sec(),
        ),
        args.repeat,
    )
    _print_stats("IMU (accel + ω_z из кэша потока)", imu_ms)

    us_ms: list[float] = []
    us_values: list[float] = []
    print(f"HC-SR04 ({args.repeat} измерений, таймаут {args.us_timeout_sec} с каждое)...")
    for i in range(args.repeat):
        t0 = time.perf_counter()
        try:
            dist = _run_with_timeout(
                hardware.ultrasonic_sensor.measure_distance_cm,
                args.us_timeout_sec,
            )
        except TimeoutError:
            elapsed = (time.perf_counter() - t0) * 1000.0
            print(
                f"  [{i + 1}/{args.repeat}] TIMEOUT после {elapsed:.0f} ms — нет импульса ECHO за "
                f"{args.us_timeout_sec} с. Проверьте: TRIGGER={args.us_trigger} "
                f"ECHO={args.us_echo}, делитель 5 V→3.3 V на ECHO, "
                "общую землю; для gpiozero часто нужен pin factory pigpio "
                "(см. предупреждение PWMSoftwareFallback).",
                file=sys.stderr,
            )
            l1.destroy()
            return 1
        except OSError as exc:
            print(f"  [{i + 1}/{args.repeat}] OSError: {exc}", file=sys.stderr)
            l1.destroy()
            return 1
        us_ms.append((time.perf_counter() - t0) * 1000.0)
        us_values.append(dist)
        print(f"  [{i + 1}/{args.repeat}] {us_ms[-1]:.1f} ms  distance_cm={dist:.1f}")

    _print_stats("HC-SR04 (итого)", us_ms)
    if us_values:
        print(f"  последние distance_cm: min={min(us_values):.1f} max={max(us_values):.1f}")

    full_ms: list[float] = []
    print("Полный L1.read_sensors() (IMU getters + один HC-SR04)...")
    for i in range(args.repeat):
        t0 = time.perf_counter()
        try:
            state = _run_with_timeout(l1.read_sensors, max(args.us_timeout_sec * 2, 10.0))
        except TimeoutError:
            print(
                f"  [{i + 1}/{args.repeat}] TIMEOUT full read_sensors — смотрите УЗ выше",
                file=sys.stderr,
            )
            l1.destroy()
            return 1
        full_ms.append((time.perf_counter() - t0) * 1000.0)
        print(
            f"  [{i + 1}/{args.repeat}] {full_ms[-1]:.1f} ms | "
            f"ω_z={state.angular_speed_z_deg_per_sec:.3f} °/s  dist={state.distance_cm:.1f} cm",
        )

    _print_stats("L1.read_sensors (итого)", full_ms)

    l1.destroy()
    print("Готово.")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
