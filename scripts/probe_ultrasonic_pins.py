#!/usr/bin/env python3
"""Быстрая проверка HC-SR04: какая пара пинов BCM (trigger, echo) даёт ответ.

В сервере пины зашиты в ``src/infrastructures/ultrasonic.py`` (по умолчанию 23 и 24).
Если датчик перепаяли — переберите кандидатов этим скриптом **без правки кода**.

Остановите rasptank-server; один процесс на GPIO.

**Почему каждая пара — отдельный процесс:** после неудачного замера gpiozero может
долго держать нити и резервировать пины. Тогда следующий замер даёт
``GPIOPinInUse``. Дочерний процесс по таймауту **убивается** родителем — ядро
освобождает GPIO для следующей пары.

Запуск из корня репозитория::

    python scripts/probe_ultrasonic_pins.py
    python scripts/probe_ultrasonic_pins.py 23 24 17 27 5 6
    python scripts/probe_ultrasonic_pins.py --timeout 3 20 21

Один замер в текущем процессе (отладка; при зависании — только ``kill -9``)::

    python scripts/probe_ultrasonic_pins.py --no-subprocess 23 24

Нумерация **BCM** (команда на Pi: ``pinout``).
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from src.infrastructures.ultrasonic import UltrasonicSensor

# Типовые пары для ручного перебора (не исчерпывающий список — допишите свои).
_DEFAULT_PAIRS: tuple[tuple[int, int], ...] = (
    (23, 24),
    (24, 23),
    (17, 27),
    (27, 17),
    (5, 6),
    (6, 5),
    (16, 12),
)

# Запас к ``--timeout``: gpiozero при отсутствии ECHO может ждать дольше номинала HC-SR04.
_PARENT_KILL_SLACK_SEC: float = 8.0


def _parse_pairs(strings: list[str]) -> list[tuple[int, int]]:
    if len(strings) % 2 != 0:
        print("Аргументы парами: BCM_trigger BCM_echo ...", file=sys.stderr)
        raise SystemExit(2)
    pairs: list[tuple[int, int]] = []
    for i in range(0, len(strings), 2):
        pairs.append((int(strings[i]), int(strings[i + 1])))
    return pairs


def _run_one_pair_same_process(trig: int, echo: int) -> tuple[str, int]:
    """Один замер в главном потоке (без вложенного ThreadPoolExecutor)."""
    UltrasonicSensor.TRIGGER_PIN = trig
    UltrasonicSensor.ECHO_PIN = echo
    sensor = UltrasonicSensor()
    t0 = time.perf_counter()
    try:
        dist_cm = sensor.measure_distance_cm()
    except OSError as exc:
        ms = (time.perf_counter() - t0) * 1000.0
        return f"  trigger={trig:2d} echo={echo:2d}  OSError ({ms:.0f} ms): {exc}", 1
    finally:
        sensor.destroy()

    ms = (time.perf_counter() - t0) * 1000.0
    fb = UltrasonicSensor.FALLBACK_DISTANCE_CM
    if dist_cm >= fb - 1.0:
        return (
            f"  trigger={trig:2d} echo={echo:2d}  {ms:.0f} ms  distance={dist_cm:.1f} cm "
            f"(≈ fallback {fb} — скорее ошибка/нет эха)",
            1,
        )
    return (
        f"  trigger={trig:2d} echo={echo:2d}  {ms:.0f} ms  distance={dist_cm:.1f} cm  <<< OK",
        0,
    )


def _subprocess_probe_main(trig: int, echo: int) -> None:
    """Точка входа дочернего процесса (ровно одна пара)."""
    line, code = _run_one_pair_same_process(trig, echo)
    print(line, flush=True)
    # Без os._exit: при редких зависаниях в фоне всё же лучше не рисковать join при shutdown.
    os._exit(code)


def main() -> int:
    parser = argparse.ArgumentParser(description="Проба пинов HC-SR04 (BCM).")
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.5,
        help=(
            "Макс. секунд на одну пару (родитель убивает дочерний процесс через "
            "timeout + небольшой запас)."
        ),
    )
    parser.add_argument(
        "--no-subprocess",
        action="store_true",
        help="Все пары в одном процессе (может дать GPIOPinInUse после таймаута).",
    )
    parser.add_argument(
        "--subprocess-probe",
        nargs=2,
        type=int,
        metavar=("TRIG", "ECHO"),
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "pins",
        nargs="*",
        type=int,
        metavar="N",
        help="Пары trigger echo; если не указано — встроенный короткий список кандидатов.",
    )
    args = parser.parse_args()

    if args.subprocess_probe is not None:
        trig, echo = args.subprocess_probe
        _subprocess_probe_main(trig, echo)

    pairs: list[tuple[int, int]] = (
        _parse_pairs([str(p) for p in args.pins]) if args.pins else list(_DEFAULT_PAIRS)
    )

    print(
        "Проверка HC-SR04. Код по умолчанию: "
        f"TRIGGER={UltrasonicSensor.TRIGGER_PIN} ECHO={UltrasonicSensor.ECHO_PIN} (BCM).",
    )
    mode = "один процесс (--no-subprocess)" if args.no_subprocess else "отдельный процесс на пару"
    print(f"Режим: {mode}. Лимит времени на пару: {args.timeout + _PARENT_KILL_SLACK_SEC:.1f} с.\n")

    any_ok = False
    parent_limit = args.timeout + _PARENT_KILL_SLACK_SEC

    for trig, echo in pairs:
        if args.no_subprocess:
            line, code = _run_one_pair_same_process(trig, echo)
            print(line)
            if code == 0:
                any_ok = True
            continue

        cmd: list[str] = [
            sys.executable,
            str(Path(__file__).resolve()),
            "--subprocess-probe",
            str(trig),
            str(echo),
        ]
        t0 = time.perf_counter()
        try:
            proc = subprocess.run(
                cmd,
                cwd=str(_REPO_ROOT),
                timeout=parent_limit,
                check=False,
                text=True,
            )
        except subprocess.TimeoutExpired:
            ms = (time.perf_counter() - t0) * 1000.0
            print(
                f"  trigger={trig:2d} echo={echo:2d}  KILLED после {ms:.0f} ms "
                f"(>{parent_limit:.1f} с) — нет ответа / зависание GPIO; смотрите проводку, "
                "ECHO 5 V→3.3 V, pigpio.",
            )
            continue

        if proc.returncode == 0:
            any_ok = True

    if any_ok:
        print("\nЕсть хотя бы одна пара с похожим на правду расстоянием — сравните с метром.")
        print("Чтобы сервер использовал эти пины, выставьте их в ultrasonic.py (или добавьте env).")
    else:
        print(
            "\nНи одна пара не дала уверенного ответа: проводка, делитель на ECHO, земля, pigpio.",
        )
    return 0 if any_ok else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
