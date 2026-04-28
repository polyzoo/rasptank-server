#!/usr/bin/env python3
"""Проверка прямолинейного движения на разных скоростях.

Скрипт запускает серию прогонов и для каждого оценивает среднюю угловую скорость
`angular_speed_z_deg_per_sec` (через `/v1/l1/state`). Чем ближе среднее к нулю,
тем меньше увод.

Режимы:
- l2: отправка команд через `/v1/l2/cmd-vel` (рекомендуется после калибровки L2)
- l1: отправка сырых команд через `/v1/l1/tracks`
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
from typing import Any


@dataclass(frozen=True, slots=True)
class RunResult:
    """Результат одного прогона на фиксированной скорости."""

    speed: float
    repeat_index: int
    n: int
    mean_wz_deg_s: float
    stdev_wz_deg_s: float


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
        return json.loads(resp.read().decode("utf-8"))


def _parse_speeds(raw: str) -> list[float]:
    values: list[float] = []
    for part in raw.split(","):
        text = part.strip()
        if not text:
            continue
        values.append(float(text))
    if not values:
        raise ValueError("--speeds must contain at least one value")
    return values


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Серия прогонов на прямой с оценкой увода по mean(ω_z).",
    )
    parser.add_argument("--base-url", default="http://127.0.0.1:8010")
    parser.add_argument(
        "--mode",
        choices=("l2", "l1"),
        default="l2",
        help="l2: /v1/l2/cmd-vel, l1: /v1/l1/tracks",
    )
    parser.add_argument(
        "--speeds",
        default="12,15,20",
        help=(
            "Скорости через запятую: в режиме l2 это cm/s, в режиме l1 это проценты "
            "одинаково для L/R."
        ),
    )
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--settle-sec", type=float, default=0.8)
    parser.add_argument("--sample-sec", type=float, default=2.5)
    parser.add_argument("--sample-interval-sec", type=float, default=0.1)
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    parser.add_argument(
        "--pass-threshold-deg-s",
        type=float,
        default=2.0,
        help="Порог PASS по |mean_wz| в deg/s.",
    )
    args = parser.parse_args()

    try:
        speeds = _parse_speeds(args.speeds)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if args.repeats < 1:
        print("--repeats must be >= 1", file=sys.stderr)
        return 2

    base = args.base_url.rstrip("/")
    l1_state_url = f"{base}/v1/l1/state"
    l1_tracks_url = f"{base}/v1/l1/tracks"
    l1_stop_url = f"{base}/v1/l1/stop"
    l2_cmd_vel_url = f"{base}/v1/l2/cmd-vel"
    l2_stop_url = f"{base}/v1/l2/stop"

    def safe_stop() -> None:
        try:
            _post_json(l1_stop_url, {}, args.timeout_sec)
        except (OSError, urllib.error.URLError, urllib.error.HTTPError):
            pass
        try:
            _post_json(l2_stop_url, {}, args.timeout_sec)
        except (OSError, urllib.error.URLError, urllib.error.HTTPError):
            pass

    def _sigint_handler(_sig: int, _frame: Any) -> None:
        safe_stop()
        raise SystemExit(130)

    signal.signal(signal.SIGINT, _sigint_handler)

    results: list[RunResult] = []
    try:
        for speed in speeds:
            for rep in range(1, args.repeats + 1):
                if args.mode == "l2":
                    _post_json(
                        l2_cmd_vel_url,
                        {
                            "linear_speed_cm_per_sec": float(speed),
                            "angular_speed_deg_per_sec": 0.0,
                        },
                        args.timeout_sec,
                    )
                else:
                    pct = int(round(speed))
                    _post_json(
                        l1_tracks_url,
                        {"left_percent": pct, "right_percent": pct},
                        args.timeout_sec,
                    )

                time.sleep(args.settle_sec)
                samples: list[float] = []
                deadline = time.monotonic() + args.sample_sec
                while time.monotonic() < deadline:
                    st = _get_json(l1_state_url, args.timeout_sec)
                    samples.append(float(st["angular_speed_z_deg_per_sec"]))
                    time.sleep(args.sample_interval_sec)

                safe_stop()
                time.sleep(0.2)

                if not samples:
                    result = RunResult(
                        speed=speed,
                        repeat_index=rep,
                        n=0,
                        mean_wz_deg_s=0.0,
                        stdev_wz_deg_s=0.0,
                    )
                else:
                    result = RunResult(
                        speed=speed,
                        repeat_index=rep,
                        n=len(samples),
                        mean_wz_deg_s=float(statistics.mean(samples)),
                        stdev_wz_deg_s=float(statistics.pstdev(samples))
                        if len(samples) > 1
                        else 0.0,
                    )
                results.append(result)
                print(
                    f"run speed={speed:g} rep={rep}/{args.repeats} "
                    f"mean_wz={result.mean_wz_deg_s:+.4f} "
                    f"stdev={result.stdev_wz_deg_s:.4f} n={result.n}",
                )
    except (OSError, urllib.error.URLError, urllib.error.HTTPError) as exc:
        safe_stop()
        print(f"HTTP error: {exc}", file=sys.stderr)
        return 1
    finally:
        safe_stop()

    if not results:
        print("No runs completed.", file=sys.stderr)
        return 1

    print()
    print("=== Summary by speed ===")
    overall_pass = True
    for speed in speeds:
        bucket = [r for r in results if r.speed == speed]
        means = [r.mean_wz_deg_s for r in bucket]
        mean_of_means = float(statistics.mean(means))
        stdev_of_means = float(statistics.pstdev(means)) if len(means) > 1 else 0.0
        passed = abs(mean_of_means) <= args.pass_threshold_deg_s
        overall_pass = overall_pass and passed
        status = "PASS" if passed else "FAIL"
        print(
            f"speed={speed:g}: mean(mean_wz)={mean_of_means:+.4f} "
            f"stdev={stdev_of_means:.4f} -> {status}",
        )

    print()
    overall = "PASS" if overall_pass else "FAIL"
    print(f"OVERALL: {overall} (threshold |mean_wz| <= {args.pass_threshold_deg_s:.3f} deg/s)")
    return 0 if overall_pass else 3


if __name__ == "__main__":
    raise SystemExit(main())
