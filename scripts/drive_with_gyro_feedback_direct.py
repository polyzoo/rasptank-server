#!/usr/bin/env python3
"""Двигать машинку напрямую с gyro-feedback без FastAPI-сервера.

Пример:
    python3 scripts/drive_with_gyro_feedback_direct.py --percent 30 --duration-sec 5

Скрипт запускается на Raspberry Pi, напрямую управляет MotorController и читает
IMUSensor. По умолчанию удерживает wz=0, то есть едет прямо.
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
from src.infrastructures.imu import IMUSensor  # noqa: E402
from src.infrastructures.motor import MotorController  # noqa: E402


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _normalize_angle_deg(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0


def _drive_with_feedback(
    *,
    motor: MotorController,
    imu: IMUSensor,
    percent: float,
    desired_wz_deg_s: float,
    duration_sec: float,
    interval_sec: float,
    kp: float,
    heading_kp: float,
    ki: float,
    sign: int,
    trim_percent: float,
    integral_limit: float,
    max_correction_percent: float,
    print_every: int,
    hold_heading: bool,
) -> tuple[list[float], list[float]]:
    samples: list[float] = []
    corrections: list[float] = []
    deadline: float = time.monotonic() + duration_sec
    step: int = 0
    integral_error: float = 0.0
    previous_time: float = time.monotonic()
    target_heading_deg: float = imu.get_yaw()

    while time.monotonic() < deadline:
        now: float = time.monotonic()
        dt_sec: float = max(0.0, now - previous_time)
        previous_time = now

        wz_deg_s: float = imu.get_angular_speed_z_deg_per_sec()
        yaw_error_deg: float = 0.0
        if hold_heading and desired_wz_deg_s == 0.0:
            yaw_error_deg = _normalize_angle_deg(target_heading_deg - imu.get_yaw())

        wz_error_deg_s: float = desired_wz_deg_s - wz_deg_s
        combined_error: float = wz_error_deg_s + heading_kp * yaw_error_deg
        integral_error = _clamp(
            integral_error + combined_error * dt_sec,
            -integral_limit,
            integral_limit,
        )
        correction_percent: float = trim_percent + sign * (
            kp * wz_error_deg_s + heading_kp * yaw_error_deg + ki * integral_error
        )
        correction_percent = _clamp(
            correction_percent,
            -max_correction_percent,
            max_correction_percent,
        )

        left_percent: int = int(round(_clamp(percent - correction_percent, -100.0, 100.0)))
        right_percent: int = int(round(_clamp(percent + correction_percent, -100.0, 100.0)))
        motor.set_tracks(left_speed_percent=left_percent, right_speed_percent=right_percent)

        samples.append(wz_deg_s)
        corrections.append(correction_percent)

        if print_every > 0 and step % print_every == 0:
            print(
                f"wz={wz_deg_s:+.2f} deg/s "
                f"yaw_err={yaw_error_deg:+.2f} deg "
                f"corr={correction_percent:+.2f}% "
                f"int={integral_error:+.2f} "
                f"L={left_percent:+d}% R={right_percent:+d}%",
                flush=True,
            )

        step += 1
        time.sleep(interval_sec)

    motor.stop()
    return samples, corrections


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Drive directly with gyro-feedback, without FastAPI.",
    )
    parser.add_argument(
        "--percent",
        type=float,
        default=30.0,
        help="Base signed track percent. Positive moves forward, negative moves backward.",
    )
    parser.add_argument("--duration-sec", type=float, default=3.0)
    parser.add_argument(
        "--desired-wz-deg-s",
        type=float,
        default=0.0,
        help="Target yaw rate. 0 means straight motion.",
    )
    parser.add_argument("--kp", type=float, default=0.6)
    parser.add_argument(
        "--heading-kp",
        type=float,
        default=0.45,
        help="Heading-hold gain in percent per degree of yaw error.",
    )
    parser.add_argument(
        "--ki",
        type=float,
        default=0.0,
        help="Integral gain. Useful when constant track asymmetry remains.",
    )
    parser.add_argument("--sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument(
        "--trim-percent",
        type=float,
        default=0.0,
        help="Constant correction: positive lowers left percent and raises right percent.",
    )
    parser.add_argument(
        "--integral-limit",
        type=float,
        default=30.0,
        help="Clamp for accumulated yaw-rate error in deg.",
    )
    parser.add_argument("--max-correction-percent", type=float, default=25.0)
    parser.add_argument(
        "--heading-hold",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Hold initial yaw while desired-wz-deg-s is 0.",
    )
    parser.add_argument("--interval-sec", type=float, default=0.10)
    parser.add_argument("--start-delay-sec", type=float, default=3.0)
    parser.add_argument("--print-every", type=int, default=5)
    parser.add_argument(
        "--return-back",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="After the main motion, drive back with negative percent.",
    )
    parser.add_argument(
        "--return-percent",
        type=float,
        default=None,
        help="Return percent magnitude. Defaults to abs(--percent).",
    )
    parser.add_argument(
        "--return-duration-sec",
        type=float,
        default=None,
        help="Return duration. Defaults to --duration-sec.",
    )
    parser.add_argument(
        "--calibrate-imu",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Calibrate IMU before motion. Keep robot still during startup.",
    )
    args = parser.parse_args()

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

    all_samples: list[float] = []
    all_corrections: list[float] = []
    try:
        print(
            f"Starting in {args.start_delay_sec:.1f}s. Keep robot still for IMU calibration.",
            flush=True,
        )
        time.sleep(args.start_delay_sec)
        imu.start(calibrate=args.calibrate_imu)
        time.sleep(0.5)

        print(
            f"drive percent={args.percent:+.1f}% duration={args.duration_sec:.1f}s "
            f"target_wz={args.desired_wz_deg_s:+.1f} kp={args.kp:.3f} "
            f"heading_kp={args.heading_kp:.3f} ki={args.ki:.3f} "
            f"trim={args.trim_percent:+.1f}% sign={args.sign:+d}",
            flush=True,
        )
        samples, corrections = _drive_with_feedback(
            motor=motor,
            imu=imu,
            percent=args.percent,
            desired_wz_deg_s=args.desired_wz_deg_s,
            duration_sec=args.duration_sec,
            interval_sec=args.interval_sec,
            kp=args.kp,
            heading_kp=args.heading_kp,
            ki=args.ki,
            sign=args.sign,
            trim_percent=args.trim_percent,
            integral_limit=args.integral_limit,
            max_correction_percent=args.max_correction_percent,
            print_every=args.print_every,
            hold_heading=args.heading_hold,
        )
        all_samples.extend(samples)
        all_corrections.extend(corrections)

        if args.return_back:
            return_percent = (
                abs(args.percent) if args.return_percent is None else abs(args.return_percent)
            )
            return_duration = (
                args.duration_sec
                if args.return_duration_sec is None
                else args.return_duration_sec
            )
            print(
                f"return percent={-return_percent:+.1f}% duration={return_duration:.1f}s",
                flush=True,
            )
            samples, corrections = _drive_with_feedback(
                motor=motor,
                imu=imu,
                percent=-return_percent,
                desired_wz_deg_s=args.desired_wz_deg_s,
                duration_sec=return_duration,
                interval_sec=args.interval_sec,
                kp=args.kp,
                heading_kp=args.heading_kp,
                ki=args.ki,
                sign=args.sign,
                trim_percent=args.trim_percent,
                integral_limit=args.integral_limit,
                max_correction_percent=args.max_correction_percent,
                print_every=args.print_every,
                hold_heading=args.heading_hold,
            )
            all_samples.extend(samples)
            all_corrections.extend(corrections)
    finally:
        safe_stop()
        imu.destroy()
        motor.destroy()

    if all_samples:
        mean_wz = float(statistics.mean(all_samples))
        stdev_wz = float(statistics.pstdev(all_samples)) if len(all_samples) > 1 else 0.0
        mean_abs_wz = float(statistics.mean(abs(value) for value in all_samples))
        mean_corr = float(statistics.mean(all_corrections)) if all_corrections else 0.0
        print()
        print("=== SUMMARY ===")
        print(f"mean_wz={mean_wz:+.3f} deg/s")
        print(f"stdev_wz={stdev_wz:.3f} deg/s")
        print(f"mean_abs_wz={mean_abs_wz:.3f} deg/s")
        print(f"mean_correction={mean_corr:+.2f}%")
        print(f"samples={len(all_samples)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
