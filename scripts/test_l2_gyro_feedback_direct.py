#!/usr/bin/env python3
"""Проверить gyro-feedback для прямого хода без поднятого FastAPI-сервера.

Скрипт запускается прямо на Raspberry Pi. Он напрямую управляет моторами через
MotorController, читает MPU6050 через IMUSensor и пробует удерживать wz около 0.
Код не меняет .env и не встраивает регулятор в L2: это только полевой тест идеи.
"""

from __future__ import annotations

import argparse
import signal
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.config.settings import Settings  # noqa: E402
from src.infrastructures.imu import IMUSensor  # noqa: E402
from src.infrastructures.motor import MotorController  # noqa: E402


@dataclass(frozen=True, slots=True)
class RunResult:
    """Итог одного тестового прогона."""

    kp: float
    sign: int
    n: int
    mean_wz_deg_s: float
    stdev_wz_deg_s: float
    mean_abs_wz_deg_s: float
    mean_correction_percent: float
    score: float


@dataclass(frozen=True, slots=True)
class FeedbackParams:
    """Параметры одного движения с feedback-регулятором."""

    base_percent: float
    desired_wz_deg_s: float
    kp: float
    sign: int
    duration_sec: float
    interval_sec: float
    max_correction_percent: float


def _parse_float_list(raw: str) -> list[float]:
    values: list[float] = []
    for part in raw.split(","):
        text = part.strip()
        if text:
            values.append(float(text))
    if not values:
        raise ValueError("list must contain at least one value")
    return values


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _run_feedback_trial(
    *,
    motor: MotorController,
    imu: IMUSensor,
    params: FeedbackParams,
) -> RunResult:
    samples: list[float] = []
    corrections: list[float] = []
    deadline: float = time.monotonic() + params.duration_sec

    while time.monotonic() < deadline:
        wz_deg_s: float = imu.get_angular_speed_z_deg_per_sec()
        error_deg_s: float = params.desired_wz_deg_s - wz_deg_s
        correction_percent: float = params.sign * params.kp * error_deg_s
        correction_percent = _clamp(
            correction_percent,
            -params.max_correction_percent,
            params.max_correction_percent,
        )

        left_percent: int = int(
            round(_clamp(params.base_percent - correction_percent, -100.0, 100.0))
        )
        right_percent: int = int(
            round(_clamp(params.base_percent + correction_percent, -100.0, 100.0))
        )
        motor.set_tracks(left_speed_percent=left_percent, right_speed_percent=right_percent)

        samples.append(wz_deg_s)
        corrections.append(correction_percent)
        time.sleep(params.interval_sec)

    motor.stop()
    time.sleep(0.5)

    if not samples:
        return RunResult(
            kp=params.kp,
            sign=params.sign,
            n=0,
            mean_wz_deg_s=0.0,
            stdev_wz_deg_s=0.0,
            mean_abs_wz_deg_s=0.0,
            mean_correction_percent=0.0,
            score=float("inf"),
        )

    mean_wz: float = float(statistics.mean(samples))
    stdev_wz: float = float(statistics.pstdev(samples)) if len(samples) > 1 else 0.0
    mean_abs_wz: float = float(statistics.mean(abs(value) for value in samples))
    mean_correction: float = float(statistics.mean(corrections)) if corrections else 0.0
    score: float = abs(mean_wz) + 0.25 * stdev_wz + 0.25 * mean_abs_wz

    return RunResult(
        kp=params.kp,
        sign=params.sign,
        n=len(samples),
        mean_wz_deg_s=mean_wz,
        stdev_wz_deg_s=stdev_wz,
        mean_abs_wz_deg_s=mean_abs_wz,
        mean_correction_percent=mean_correction,
        score=score,
    )


def _return_backward(
    *,
    motor: MotorController,
    imu: IMUSensor,
    forward_params: FeedbackParams,
    return_percent: float,
    return_duration_sec: float,
) -> None:
    if return_duration_sec <= 0.0 or return_percent <= 0.0:
        return

    _run_feedback_trial(
        motor=motor,
        imu=imu,
        params=FeedbackParams(
            base_percent=-abs(return_percent),
            desired_wz_deg_s=forward_params.desired_wz_deg_s,
            kp=forward_params.kp,
            sign=forward_params.sign,
            duration_sec=return_duration_sec,
            interval_sec=forward_params.interval_sec,
            max_correction_percent=forward_params.max_correction_percent,
        ),
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Direct gyro-feedback test for straight driving, without FastAPI.",
    )
    parser.add_argument("--base-percent", type=float, default=30.0)
    parser.add_argument("--desired-wz-deg-s", type=float, default=0.0)
    parser.add_argument("--kp-values", default="0,0.3,0.5,0.7,1.0,1.3")
    parser.add_argument("--duration-sec", type=float, default=3.0)
    parser.add_argument("--interval-sec", type=float, default=0.10)
    parser.add_argument("--between-runs-sec", type=float, default=1.0)
    parser.add_argument(
        "--return-back",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="After each forward run, drive backward to save room space.",
    )
    parser.add_argument(
        "--return-percent",
        type=float,
        default=None,
        help="Backward percent. Defaults to --base-percent.",
    )
    parser.add_argument(
        "--return-duration-sec",
        type=float,
        default=None,
        help="Backward duration. Defaults to --duration-sec.",
    )
    parser.add_argument("--start-delay-sec", type=float, default=3.0)
    parser.add_argument("--max-correction-percent", type=float, default=18.0)
    parser.add_argument(
        "--calibrate-imu",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Calibrate IMU at start. Keep the robot still during start-delay and calibration.",
    )
    args = parser.parse_args()

    kp_values: list[float] = _parse_float_list(args.kp_values)
    return_percent: float = (
        abs(args.base_percent) if args.return_percent is None else abs(args.return_percent)
    )
    return_duration_sec: float = (
        args.duration_sec if args.return_duration_sec is None else args.return_duration_sec
    )
    settings = Settings()

    motor = MotorController(
        tl_left_offset=settings.tl_left_offset,
        tl_right_offset=settings.tl_right_offset,
        m1_direction=settings.m1_direction,
        m2_direction=settings.m2_direction,
    )
    imu = IMUSensor(
        gyro_yaw_integration_deadband_deg_per_sec=settings.gyro_yaw_integration_deadband_deg_per_sec,
        mpu6050_dlpf_cfg=settings.imu_mpu6050_dlpf_cfg,
        mpu6050_smplrt_div=settings.imu_mpu6050_smplrt_div,
        accel_ema_alpha=settings.imu_accel_ema_alpha,
        gyro_ema_alpha=settings.imu_gyro_ema_alpha,
        ekf_enabled=settings.imu_ekf_enabled,
        ekf_q_angle=settings.imu_ekf_q_angle,
        ekf_q_bias=settings.imu_ekf_q_bias,
        ekf_r_accel=settings.imu_ekf_r_accel,
        ekf_accel_gate=settings.imu_ekf_accel_gate,
    )

    def safe_stop() -> None:
        try:
            motor.stop()
        except Exception:
            pass

    def sigint_handler(_sig: int, _frame: Any) -> None:
        safe_stop()
        raise SystemExit(130)

    signal.signal(signal.SIGINT, sigint_handler)

    results: list[RunResult] = []
    try:
        print(
            f"Starting in {args.start_delay_sec:.1f}s. Keep robot still for IMU calibration.",
            flush=True,
        )
        time.sleep(args.start_delay_sec)
        imu.start(calibrate=args.calibrate_imu)
        time.sleep(0.5)

        for kp in kp_values:
            signs = (1,) if kp == 0.0 else (1, -1)
            for sign in signs:
                print(
                    f"\nrun kp={kp:.3f} sign={sign:+d} base={args.base_percent:.1f}%",
                    flush=True,
                )
                forward_params = FeedbackParams(
                    base_percent=args.base_percent,
                    desired_wz_deg_s=args.desired_wz_deg_s,
                    kp=kp,
                    sign=sign,
                    duration_sec=args.duration_sec,
                    interval_sec=args.interval_sec,
                    max_correction_percent=args.max_correction_percent,
                )
                result = _run_feedback_trial(
                    motor=motor,
                    imu=imu,
                    params=forward_params,
                )
                results.append(result)
                print(
                    f"mean_wz={result.mean_wz_deg_s:+.3f} deg/s "
                    f"stdev={result.stdev_wz_deg_s:.3f} "
                    f"mean_abs={result.mean_abs_wz_deg_s:.3f} "
                    f"mean_corr={result.mean_correction_percent:+.2f}% "
                    f"score={result.score:.3f} n={result.n}",
                    flush=True,
                )
                if args.return_back:
                    print(
                        f"return back {return_percent:.1f}% for {return_duration_sec:.1f}s",
                        flush=True,
                    )
                    _return_backward(
                        motor=motor,
                        imu=imu,
                        forward_params=forward_params,
                        return_percent=return_percent,
                        return_duration_sec=return_duration_sec,
                    )
                time.sleep(args.between_runs_sec)
    finally:
        safe_stop()
        imu.destroy()
        motor.destroy()

    if not results:
        print("No results.")
        return 1

    best = min(results, key=lambda item: item.score)
    print("\n=== BEST ===")
    print(
        f"kp={best.kp:.3f} sign={best.sign:+d} "
        f"mean_wz={best.mean_wz_deg_s:+.3f} deg/s "
        f"stdev={best.stdev_wz_deg_s:.3f} "
        f"mean_abs={best.mean_abs_wz_deg_s:.3f} "
        f"mean_corr={best.mean_correction_percent:+.2f}% "
        f"score={best.score:.3f}",
    )
    print()
    print("If BEST kp is 0.000, feedback did not improve this run.")
    print("If BEST kp > 0, these kp/sign values are candidates for L2 closed-loop control.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
