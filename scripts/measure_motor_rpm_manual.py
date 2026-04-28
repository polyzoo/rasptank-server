#!/usr/bin/env python3
"""Measure motor/track output RPM manually without the FastAPI server.

The script runs one track at a fixed percent. Put a visible mark on the output
shaft, sprocket, or wheel, count a fixed number of revolutions, then press Enter.
The script computes RPM from elapsed time.
"""

from __future__ import annotations

import argparse
import signal
import statistics
import sys
import time
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.config.settings import Settings  # noqa: E402
from src.infrastructures.motor import MotorController  # noqa: E402


def _track_command(side: str, percent: int) -> tuple[int, int]:
    if side == "left":
        return percent, 0
    if side == "right":
        return 0, percent
    return percent, percent


def _mean_or_zero(values: list[float]) -> float:
    return float(statistics.mean(values)) if values else 0.0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Manual RPM measurement for RaspTank motors/tracks.",
    )
    parser.add_argument("--side", choices=("left", "right", "both"), default="left")
    parser.add_argument("--percent", type=int, default=50)
    parser.add_argument(
        "--revolutions",
        type=float,
        default=10.0,
        help="How many output revolutions you will count before pressing Enter.",
    )
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--settle-sec", type=float, default=1.0)
    parser.add_argument("--between-samples-sec", type=float, default=1.0)
    parser.add_argument(
        "--reverse",
        action="store_true",
        help="Run the selected side backward.",
    )
    args = parser.parse_args()

    if args.revolutions <= 0.0:
        print("--revolutions must be > 0", file=sys.stderr)
        return 2
    if args.samples < 1:
        print("--samples must be >= 1", file=sys.stderr)
        return 2

    settings = Settings()
    motor = MotorController(
        tl_left_offset=settings.tl_left_offset,
        tl_right_offset=settings.tl_right_offset,
        m1_direction=settings.m1_direction,
        m2_direction=settings.m2_direction,
    )

    signed_percent = -abs(args.percent) if args.reverse else abs(args.percent)
    left_percent, right_percent = _track_command(args.side, signed_percent)

    def safe_stop() -> None:
        try:
            motor.stop()
        except Exception:
            pass

    def sigint_handler(_sig: int, _frame: Any) -> None:
        safe_stop()
        raise SystemExit(130)

    signal.signal(signal.SIGINT, sigint_handler)

    rpm_values: list[float] = []
    try:
        print()
        print("Manual RPM measurement")
        print(f"side={args.side} percent={signed_percent:+d}%")
        print(f"count {args.revolutions:g} output revolutions, then press Enter")
        print("Put the robot on a stand so the track/wheel can rotate freely.")
        print()

        for sample_index in range(1, args.samples + 1):
            input(f"Sample {sample_index}/{args.samples}: press Enter to start motor...")
            motor.set_tracks(left_speed_percent=left_percent, right_speed_percent=right_percent)
            time.sleep(args.settle_sec)

            input(
                f"Now start counting {args.revolutions:g} revolutions. "
                "Press Enter exactly when the count is done..."
            )
            start = time.monotonic()
            input("Counting... press Enter now when done.")
            elapsed = time.monotonic() - start

            motor.stop()
            rpm = (args.revolutions / elapsed) * 60.0 if elapsed > 0.0 else 0.0
            rpm_values.append(rpm)
            print(f"elapsed={elapsed:.3f}s rpm={rpm:.2f}")
            time.sleep(args.between_samples_sec)
    finally:
        safe_stop()
        motor.destroy()

    mean_rpm = _mean_or_zero(rpm_values)
    stdev_rpm = float(statistics.pstdev(rpm_values)) if len(rpm_values) > 1 else 0.0

    print()
    print("=== SUMMARY ===")
    print(f"side={args.side}")
    print(f"percent={signed_percent:+d}%")
    print(f"mean_rpm={mean_rpm:.2f}")
    print(f"stdev_rpm={stdev_rpm:.2f}")
    print(f"samples={len(rpm_values)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
