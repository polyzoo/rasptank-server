from __future__ import annotations

import pytest

from src.application.services.pose_estimator import PoseEstimate, PoseEstimator


def test_reset_and_snapshot_store_state() -> None:
    """Снимок состояния возвращает значения после явного сброса."""
    estimator: PoseEstimator = PoseEstimator()

    estimator.reset(
        x_cm=10.0,
        y_cm=5.0,
        heading_deg=190.0,
        linear_speed_cm_per_sec=12.0,
        angular_speed_deg_per_sec=3.0,
    )
    snapshot: PoseEstimate = estimator.snapshot()

    assert snapshot.x_cm == 10.0
    assert snapshot.y_cm == 5.0
    assert snapshot.heading_deg == -170.0
    assert snapshot.linear_speed_cm_per_sec == 12.0
    assert snapshot.angular_speed_deg_per_sec == 3.0


def test_update_from_velocity_moves_robot_forward() -> None:
    """Обновление по скорости меняет координату по текущему направлению."""
    estimator: PoseEstimator = PoseEstimator()

    snapshot: PoseEstimate = estimator.update_from_velocity(
        linear_speed_cm_per_sec=10.0,
        angular_speed_deg_per_sec=0.0,
        dt_sec=2.0,
    )

    assert snapshot.x_cm == pytest.approx(20.0)
    assert snapshot.y_cm == pytest.approx(0.0)
    assert snapshot.heading_deg == pytest.approx(0.0)


def test_update_from_velocity_respects_heading_change() -> None:
    """При повороте изменение положения считается по среднему направлению."""
    estimator: PoseEstimator = PoseEstimator()

    snapshot: PoseEstimate = estimator.update_from_velocity(
        linear_speed_cm_per_sec=10.0,
        angular_speed_deg_per_sec=90.0,
        dt_sec=1.0,
    )

    assert snapshot.x_cm == pytest.approx(7.0710678, rel=1e-6)
    assert snapshot.y_cm == pytest.approx(7.0710678, rel=1e-6)
    assert snapshot.heading_deg == pytest.approx(90.0)


def test_integrate_longitudinal_acceleration_changes_linear_speed() -> None:
    """Продольное ускорение обновляет линейную скорость в сантиметрах в секунду."""
    estimator: PoseEstimator = PoseEstimator()

    linear_speed_cm_per_sec: float = estimator.integrate_longitudinal_acceleration(
        longitudinal_acceleration_m_s2=0.5,
        dt_sec=2.0,
    )

    assert linear_speed_cm_per_sec == pytest.approx(100.0)


def test_update_from_imu_uses_acceleration_and_angular_speed() -> None:
    """Обновление по IMU одновременно меняет скорость, положение и угол."""
    estimator: PoseEstimator = PoseEstimator()

    snapshot: PoseEstimate = estimator.update_from_imu(
        longitudinal_acceleration_m_s2=0.5,
        angular_speed_deg_per_sec=30.0,
        dt_sec=1.0,
    )

    assert snapshot.linear_speed_cm_per_sec == pytest.approx(50.0)
    assert snapshot.heading_deg == pytest.approx(30.0)
    assert snapshot.x_cm > 0.0
    assert snapshot.y_cm > 0.0


def test_update_methods_return_current_state_for_non_positive_dt() -> None:
    """При нулевом шаге времени состояние не меняется."""
    estimator: PoseEstimator = PoseEstimator()
    estimator.reset(x_cm=3.0, y_cm=4.0, heading_deg=5.0, linear_speed_cm_per_sec=6.0)

    velocity_snapshot: PoseEstimate = estimator.update_from_velocity(
        linear_speed_cm_per_sec=100.0,
        angular_speed_deg_per_sec=50.0,
        dt_sec=0.0,
    )
    linear_speed_cm_per_sec: float = estimator.integrate_longitudinal_acceleration(
        longitudinal_acceleration_m_s2=1.0,
        dt_sec=0.0,
    )

    assert velocity_snapshot.x_cm == pytest.approx(3.0)
    assert velocity_snapshot.y_cm == pytest.approx(4.0)
    assert velocity_snapshot.heading_deg == pytest.approx(5.0)
    assert linear_speed_cm_per_sec == pytest.approx(6.0)
