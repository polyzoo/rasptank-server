#!/usr/bin/env python3
"""Автоподбор коэффициентов L2 для прямой по гироскопу.

Скрипт автоматически делает несколько прогонов на разных процентах L1,
оценивает дрейф (mean ωz) и подбирает разницу max-скоростей бортов
в кинематике L2.

Результат печатается как готовые строки для `.env`:
- LEFT_TRACK_MAX_SPEED_CM_PER_SEC
- RIGHT_TRACK_MAX_SPEED_CM_PER_SEC

По умолчанию скрипт не требует ручного ввода: достаточно запустить его на
машинке рядом со свободным прямым участком пола.
"""

from __future__ import annotations

import argparse
import json
import signal
import statistics
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from math import radians
from typing import Any


@dataclass(frozen=True, slots=True)
class SampleStats:
    """Статистика по серии замеров во время движения."""

    n: int
    mean_wz_deg_s: float
    stdev_wz_deg_s: float


@dataclass(frozen=True, slots=True)
class Trial:
    """Один прогон на фиксированном проценте и его статистика."""

    track_percent: int
    repeat_index: int
    stats: SampleStats
    distance_cm: float | None = None
    run_duration_sec: float = 0.0


def _post_json(url: str, payload: dict[str, Any], timeout_sec: float) -> None:
    req = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=timeout_sec):
        pass


def _get_json(url: str, timeout_sec: float) -> dict[str, Any]:
    req = urllib.request.Request(url, method="GET")
    with urllib.request.urlopen(req, timeout=timeout_sec) as resp:
        raw = resp.read()
    return json.loads(raw.decode("utf-8"))


def _measure_mean_wz(
    *,
    tracks_url: str,
    stop_url: str,
    state_url: str,
    track_percent: int,
    settle_sec: float,
    sample_sec: float,
    sample_interval_sec: float,
    timeout_sec: float,
) -> SampleStats:
    _post_json(
        tracks_url,
        {"left_percent": track_percent, "right_percent": track_percent},
        timeout_sec,
    )
    time.sleep(settle_sec)

    values: list[float] = []
    deadline = time.monotonic() + sample_sec
    while time.monotonic() < deadline:
        st = _get_json(state_url, timeout_sec)
        values.append(float(st["angular_speed_z_deg_per_sec"]))
        time.sleep(sample_interval_sec)

    _post_json(stop_url, {}, timeout_sec)
    time.sleep(0.2)

    if not values:
        return SampleStats(n=0, mean_wz_deg_s=0.0, stdev_wz_deg_s=0.0)
    return SampleStats(
        n=len(values),
        mean_wz_deg_s=float(statistics.mean(values)),
        stdev_wz_deg_s=float(statistics.pstdev(values)) if len(values) > 1 else 0.0,
    )


def _parse_percent_list(raw: str) -> list[int]:
    values: list[int] = []
    for part in raw.split(","):
        stripped: str = part.strip()
        if not stripped:
            continue
        val = int(stripped)
        if val <= 0 or val > 100:
            msg = "all values in --track-percents must be in (0, 100]"
            raise ValueError(msg)
        values.append(val)
    if not values:
        raise ValueError("--track-percents must contain at least one integer value")
    return values


def _estimate_delta_right_minus_left_cm_s(
    *,
    trials: list[Trial],
    track_width_cm: float,
) -> float:
    """Оценить (Vmax_right - Vmax_left) по МНК через модель omega = p * delta / B."""
    sum_p_omega: float = 0.0
    sum_p2: float = 0.0
    for trial in trials:
        p: float = trial.track_percent / 100.0
        omega_rad_s: float = radians(trial.stats.mean_wz_deg_s)
        sum_p_omega += p * omega_rad_s
        sum_p2 += p * p
    if sum_p2 <= 1e-12:
        return 0.0
    return track_width_cm * sum_p_omega / sum_p2


def _estimate_track_max_speeds_from_distance(
    *,
    trials: list[Trial],
    track_width_cm: float,
) -> tuple[float, float] | None:
    """Оценить абсолютные Vmax левого/правого борта по дистанции и gyro."""
    left_values: list[float] = []
    right_values: list[float] = []

    for trial in trials:
        if trial.distance_cm is None or trial.run_duration_sec <= 0.0:
            continue

        p: float = trial.track_percent / 100.0
        if p <= 0.0:
            continue

        linear_speed_cm_s: float = trial.distance_cm / trial.run_duration_sec
        omega_rad_s: float = radians(trial.stats.mean_wz_deg_s)
        half_width_cm: float = track_width_cm / 2.0
        right_speed_cm_s: float = linear_speed_cm_s + omega_rad_s * half_width_cm
        left_speed_cm_s: float = linear_speed_cm_s - omega_rad_s * half_width_cm

        left_values.append(left_speed_cm_s / p)
        right_values.append(right_speed_cm_s / p)

    if not left_values or not right_values:
        return None

    return float(statistics.mean(left_values)), float(statistics.mean(right_values))


def _read_optional_distance_cm() -> float | None:
    raw = input("Measured center distance, cm (blank to skip): ").strip()
    if not raw:
        return None
    return float(raw.replace(",", "."))


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Автоподбор LEFT/RIGHT_TRACK_MAX_SPEED_CM_PER_SEC по серии прогонов "
            "на прямой (равные L/R на L1)."
        ),
    )
    parser.add_argument("--base-url", default="http://127.0.0.1:8010")
    parser.add_argument(
        "--track-percents",
        default="20,30,40",
        help="Список процентов L1 через запятую, например 20,30,40.",
    )
    parser.add_argument(
        "--repeats",
        type=int,
        default=2,
        help="Количество повторов на каждый процент.",
    )
    parser.add_argument(
        "--track-width-cm",
        type=float,
        default=14.0,
        help="Текущий TRACK_WIDTH_CM (по умолчанию 14.0).",
    )
    parser.add_argument(
        "--avg-max-cm-s",
        type=float,
        default=30.0,
        help=(
            "Средний масштаб max-скорости борта для пары LEFT/RIGHT. "
            "Без измерения дистанции скрипт подбирает разницу бортов вокруг этого значения."
        ),
    )
    parser.add_argument(
        "--ask-distance",
        action="store_true",
        help=(
            "После каждого прогона спросить фактически пройденное расстояние центра машинки. "
            "Если заполнить, скрипт оценит абсолютные LEFT/RIGHT max speed."
        ),
    )
    parser.add_argument(
        "--no-pause",
        action="store_true",
        help="Совместимость со старым режимом: сейчас пауз нет по умолчанию.",
    )
    parser.add_argument(
        "--pause",
        action="store_true",
        help="Ждать Enter между прогонами.",
    )
    parser.add_argument("--settle-sec", type=float, default=0.8)
    parser.add_argument("--sample-sec", type=float, default=3.0)
    parser.add_argument("--sample-interval-sec", type=float, default=0.1)
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    parser.add_argument(
        "--safety-scale",
        type=float,
        default=1.0,
        help="0..1: доля коррекции (1.0 = полный расчёт, 0.5 = мягче).",
    )
    args = parser.parse_args()

    try:
        track_percents: list[int] = _parse_percent_list(args.track_percents)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if args.repeats < 1:
        print("--repeats must be >= 1.", file=sys.stderr)
        return 2
    if args.track_width_cm <= 0.0:
        print("track-width-cm must be > 0.", file=sys.stderr)
        return 2
    if args.avg_max_cm_s <= 0.0:
        print("--avg-max-cm-s must be > 0.", file=sys.stderr)
        return 2
    safety_scale = max(0.0, min(1.0, float(args.safety_scale)))

    base = args.base_url.rstrip("/")
    tracks_url = f"{base}/v1/l1/tracks"
    stop_url = f"{base}/v1/l1/stop"
    state_url = f"{base}/v1/l1/state"

    def safe_stop() -> None:
        try:
            _post_json(stop_url, {}, args.timeout_sec)
        except (OSError, urllib.error.URLError, urllib.error.HTTPError):
            pass

    def sigint_handler(_sig: int, _frame: Any) -> None:
        safe_stop()
        raise SystemExit(130)

    signal.signal(signal.SIGINT, sigint_handler)

    trials: list[Trial] = []
    try:
        for track_percent in track_percents:
            for repeat_index in range(1, args.repeats + 1):
                stats = _measure_mean_wz(
                    tracks_url=tracks_url,
                    stop_url=stop_url,
                    state_url=state_url,
                    track_percent=track_percent,
                    settle_sec=args.settle_sec,
                    sample_sec=args.sample_sec,
                    sample_interval_sec=args.sample_interval_sec,
                    timeout_sec=args.timeout_sec,
                )
                print(
                    f"trial p={track_percent:2d}% rep={repeat_index}/{args.repeats} "
                    f"mean_wz={stats.mean_wz_deg_s:+.4f} deg/s "
                    f"stdev={stats.stdev_wz_deg_s:.4f} n={stats.n}",
                )
                trials.append(
                    Trial(
                        track_percent=track_percent,
                        repeat_index=repeat_index,
                        stats=stats,
                        distance_cm=_read_optional_distance_cm() if args.ask_distance else None,
                        run_duration_sec=args.settle_sec + args.sample_sec,
                    )
                )
                if args.pause and not args.ask_distance:
                    input("Press Enter for next run...")
                input()
    except (OSError, urllib.error.URLError, urllib.error.HTTPError) as exc:
        safe_stop()
        print(f"HTTP error: {exc}", file=sys.stderr)
        return 1
    finally:
        safe_stop()

    if not trials:
        print("No trials collected.", file=sys.stderr)
        return 1

    delta_v = _estimate_delta_right_minus_left_cm_s(
        trials=trials,
        track_width_cm=args.track_width_cm,
    )
    recommended_delta = delta_v * safety_scale
    distance_based_speeds = _estimate_track_max_speeds_from_distance(
        trials=trials,
        track_width_cm=args.track_width_cm,
    )
    if distance_based_speeds is None:
        avg_v = args.avg_max_cm_s
    else:
        avg_v = statistics.mean(distance_based_speeds)

    new_left = max(1e-3, avg_v - recommended_delta / 2.0)
    new_right = max(1e-3, avg_v + recommended_delta / 2.0)

    if distance_based_speeds is not None:
        measured_left, measured_right = distance_based_speeds
        measured_delta = measured_right - measured_left
        new_left = max(1e-3, avg_v - measured_delta * safety_scale / 2.0)
        new_right = max(1e-3, avg_v + measured_delta * safety_scale / 2.0)

    mean_of_means = float(statistics.mean(t.stats.mean_wz_deg_s for t in trials))
    stdev_of_means = (
        float(statistics.pstdev(t.stats.mean_wz_deg_s for t in trials)) if len(trials) > 1 else 0.0
    )

    print()
    print("=== L2 coefficients calibration (multi-run) ===")
    print(f"trials={len(trials)} mean(mean_wz)={mean_of_means:+.5f} deg/s")
    print(f"stdev(mean_wz)={stdev_of_means:.5f} deg/s")
    print(
        f"estimated delta (right-left) max speed = {delta_v:+.5f} cm/s",
    )
    print()
    print("Current:")
    print("  LEFT_TRACK_MAX_SPEED_CM_PER_SEC=auto_from_avg_minus_delta")
    print("  RIGHT_TRACK_MAX_SPEED_CM_PER_SEC=auto_from_avg_plus_delta")
    print(f"  AVG_MAX_CM_S={avg_v:.6f}")
    if distance_based_speeds is not None:
        measured_left, measured_right = distance_based_speeds
        print()
        print("Distance-based estimate:")
        print(f"  measured LEFT max = {measured_left:.6f} cm/s")
        print(f"  measured RIGHT max = {measured_right:.6f} cm/s")
        print(f"  measured delta right-left = {measured_right - measured_left:+.6f} cm/s")
    print()
    print("Recommended (.env):")
    print(f"  LEFT_TRACK_MAX_SPEED_CM_PER_SEC={new_left:.6f}")
    print(f"  RIGHT_TRACK_MAX_SPEED_CM_PER_SEC={new_right:.6f}")
    print()
    print(
        "Если увод остался — повторите калибровку и усредните рекомендации. "
        "Для более мягкой коррекции задайте --safety-scale 0.5.",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
