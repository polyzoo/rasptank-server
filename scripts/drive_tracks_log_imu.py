#!/usr/bin/env python3
"""Запустить гусеницы на заданных процентах и логировать IMU каждые 0.1 с.

Пример на Raspberry Pi из корня репозитория:

    python3 scripts/drive_tracks_log_imu.py --left-percent 30 --right-percent 30 --duration-sec 5

Проценты задаются со знаком в диапазоне -100..100:
- положительные значения обычно едут вперед;
- отрицательные значения обычно едут назад;
- разные знаки разворачивают машинку на месте.

Перед запуском остановите FastAPI-сервер, чтобы два процесса не открывали одни и те же
I2C/PCA9685/MPU6050 ресурсы одновременно.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.config.settings import Settings  # noqa: E402
from src.infrastructures.imu import IMUSensor  # noqa: E402
from src.infrastructures.motor import MotorController  # noqa: E402


@dataclass(frozen=True, slots=True)
class ImuSample:
    t_sec: float
    yaw_deg: float
    gyro_z_deg_s: float


@dataclass(frozen=True, slots=True)
class ExponentialFit:
    """Модель y(t) = steady + amplitude * exp(-t / tau)."""

    steady: float
    amplitude: float
    tau_sec: float
    rmse: float
    r2: float
    points: int

    @property
    def initial(self) -> float:
        return self.steady + self.amplitude


def _clamp_percent(value: float) -> int:
    """Ограничить команду гусеницы диапазоном MotorController.set_tracks()."""
    return int(round(max(-100.0, min(100.0, value))))


def _create_motor(settings: Settings) -> MotorController:
    return MotorController(
        tl_left_offset=settings.tl_left_offset,
        tl_right_offset=settings.tl_right_offset,
        m1_direction=settings.m1_direction,
        m2_direction=settings.m2_direction,
    )


def _create_imu(settings: Settings) -> IMUSensor:
    return IMUSensor(
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


def _linear_fit_for_tau(
    *,
    times_sec: list[float],
    values: list[float],
    tau_sec: float,
) -> tuple[float, float, float]:
    """Вернуть (steady, amplitude, sse) для фиксированного tau."""
    n = len(times_sec)
    xs = [math.exp(-t / tau_sec) for t in times_sec]
    sum_x = sum(xs)
    sum_y = sum(values)
    sum_xx = sum(x * x for x in xs)
    sum_xy = sum(x * y for x, y in zip(xs, values, strict=True))

    det = n * sum_xx - sum_x * sum_x
    if abs(det) <= 1e-12:
        steady = sum_y / n
        amplitude = 0.0
    else:
        steady = (sum_y * sum_xx - sum_x * sum_xy) / det
        amplitude = (n * sum_xy - sum_x * sum_y) / det

    sse = sum(
        (value - (steady + amplitude * math.exp(-t / tau_sec))) ** 2
        for t, value in zip(times_sec, values, strict=True)
    )
    return steady, amplitude, sse


def _fit_exponential_response(samples: list[ImuSample]) -> ExponentialFit | None:
    """Подобрать экспоненту к gyro_z методом сеточного поиска по tau и МНК по A/C."""
    if len(samples) < 4:
        return None

    t0 = samples[0].t_sec
    times_sec = [sample.t_sec - t0 for sample in samples]
    values = [sample.gyro_z_deg_s for sample in samples]
    duration_sec = times_sec[-1] - times_sec[0]
    if duration_sec <= 0.0:
        return None

    # Ищем tau в широком диапазоне относительно длины прогона. Для каждого tau
    # steady и amplitude находятся обычным МНК, поэтому SciPy не нужна.
    min_tau = max(0.02, duration_sec / 200.0)
    max_tau = max(min_tau * 1.1, duration_sec * 5.0)
    steps = 240

    best: tuple[float, float, float, float] | None = None
    log_min = math.log(min_tau)
    log_max = math.log(max_tau)
    for index in range(steps):
        fraction = index / (steps - 1)
        tau_sec = math.exp(log_min + (log_max - log_min) * fraction)
        steady, amplitude, sse = _linear_fit_for_tau(
            times_sec=times_sec,
            values=values,
            tau_sec=tau_sec,
        )
        if best is None or sse < best[3]:
            best = (steady, amplitude, tau_sec, sse)

    if best is None:
        return None

    steady, amplitude, tau_sec, sse = best
    mean_value = sum(values) / len(values)
    total_sse = sum((value - mean_value) ** 2 for value in values)
    rmse = math.sqrt(sse / len(values))
    r2 = 1.0 - sse / total_sse if total_sse > 1e-12 else 1.0
    return ExponentialFit(
        steady=steady,
        amplitude=amplitude,
        tau_sec=tau_sec,
        rmse=rmse,
        r2=r2,
        points=len(values),
    )


def _print_exponential_fit(samples: list[ImuSample]) -> None:
    fit = _fit_exponential_response(samples)
    if fit is None:
        print("exponential_fit: not enough points", flush=True)
        return

    t0 = samples[0].t_sec
    yaw0 = samples[0].yaw_deg
    print(
        "exponential_fit_gyro_z: "
        "gyro_z_deg_s(t) = "
        f"{fit.steady:.3f} + ({fit.amplitude:.3f}) * exp(-(t - {t0:.3f}) / {fit.tau_sec:.3f})",
        flush=True,
    )
    print(
        "exponential_fit_yaw_from_gyro: "
        "yaw_deg(t) = "
        f"{yaw0:.3f} + {fit.steady:.3f} * dt + "
        f"({fit.amplitude:.3f}) * {fit.tau_sec:.3f} * (1 - exp(-dt / {fit.tau_sec:.3f})), "
        f"dt = t - {t0:.3f}",
        flush=True,
    )
    print(
        "exponential_fit_stats: "
        f"points={fit.points} initial_gyro_z_deg_s={fit.initial:.3f} "
        f"steady_gyro_z_deg_s={fit.steady:.3f} tau_sec={fit.tau_sec:.3f} "
        f"rmse_deg_s={fit.rmse:.3f} r2={fit.r2:.3f}",
        flush=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run left/right tracks with signed percent commands and print gyro/accelerometer "
            "data every interval."
        ),
    )
    parser.add_argument(
        "--left-percent",
        type=float,
        required=True,
        help="Процент напряжения/мощности левой гусеницы: -100..100.",
    )
    parser.add_argument(
        "--right-percent",
        type=float,
        required=True,
        help="Процент напряжения/мощности правой гусеницы: -100..100.",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        required=True,
        help="Время работы моторов в секундах.",
    )
    parser.add_argument(
        "--interval-sec",
        type=float,
        default=0.10,
        help="Интервал вывода данных IMU в секундах.",
    )
    parser.add_argument(
        "--start-delay-sec",
        type=float,
        default=1.0,
        help="Пауза перед стартом, чтобы поставить машинку на пол.",
    )
    parser.add_argument(
        "--calibrate-imu",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Калибровать IMU перед стартом. Во время калибровки машинка должна стоять.",
    )
    parser.add_argument(
        "--fit-exponential",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="После прогона приблизить gyro_z экспонентой первого порядка и вывести параметры.",
    )
    args = parser.parse_args()

    if args.duration_sec <= 0.0:
        print("--duration-sec must be > 0", file=sys.stderr)
        return 2
    if args.interval_sec <= 0.0:
        print("--interval-sec must be > 0", file=sys.stderr)
        return 2
    if args.start_delay_sec < 0.0:
        print("--start-delay-sec must be >= 0", file=sys.stderr)
        return 2

    left_percent = _clamp_percent(args.left_percent)
    right_percent = _clamp_percent(args.right_percent)

    settings = Settings()
    motor = _create_motor(settings)
    imu = _create_imu(settings)

    def safe_stop() -> None:
        try:
            motor.stop()
        except Exception:
            pass

    def sigint_handler(_sig: int, _frame: Any) -> None:
        safe_stop()
        raise SystemExit(130)

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        print(
            f"Starting in {args.start_delay_sec:.1f}s. "
            "Keep robot still for IMU calibration.",
            flush=True,
        )
        time.sleep(args.start_delay_sec)

        imu.start(calibrate=args.calibrate_imu)
        time.sleep(max(0.2, args.interval_sec))

        print(
            f"tracks: left={left_percent:+d}% right={right_percent:+d}% "
            f"duration={args.duration_sec:.2f}s interval={args.interval_sec:.2f}s",
            flush=True,
        )
        print("t_sec,yaw_deg,gyro_z_deg_s,accel_x_m_s2,accel_y_m_s2,accel_z_m_s2", flush=True)

        start = time.monotonic()
        deadline = start + args.duration_sec
        next_sample = start
        samples: list[ImuSample] = []
        motor.set_tracks(left_speed_percent=left_percent, right_speed_percent=right_percent)

        while True:
            now = time.monotonic()
            if now >= deadline:
                break

            if now >= next_sample:
                ax_m_s2, ay_m_s2, az_m_s2 = imu.get_acceleration_xyz_m_s2()
                t_sec = now - start
                yaw_deg = imu.get_yaw()
                gyro_z_deg_s = imu.get_angular_speed_z_deg_per_sec()
                samples.append(
                    ImuSample(
                        t_sec=t_sec,
                        yaw_deg=yaw_deg,
                        gyro_z_deg_s=gyro_z_deg_s,
                    ),
                )
                print(
                    f"{t_sec:.3f},"
                    f"{yaw_deg:.3f},"
                    f"{gyro_z_deg_s:.3f},"
                    f"{ax_m_s2:.3f},"
                    f"{ay_m_s2:.3f},"
                    f"{az_m_s2:.3f}",
                    flush=True,
                )
                next_sample += args.interval_sec

            time.sleep(min(0.01, max(0.0, next_sample - time.monotonic())))

    finally:
        safe_stop()
        imu.destroy()
        motor.destroy()

    print("Done. Motors stopped.", flush=True)
    if args.fit_exponential:
        _print_exponential_fit(samples)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
