from __future__ import annotations

import pytest

from src.application.services.kinematics import (
    ChassisVelocity,
    DifferentialDriveKinematics,
    TrackCommand,
)


def _kinematics() -> DifferentialDriveKinematics:
    """Создать кинематику с простыми симметричными параметрами."""
    return DifferentialDriveKinematics(
        track_width_cm=20.0,
        left_track_max_speed_cm_per_sec=40.0,
        right_track_max_speed_cm_per_sec=40.0,
    )


def test_to_track_command_for_straight_motion() -> None:
    """Движение прямо даёт одинаковые команды обоим бортам."""
    kinematics: DifferentialDriveKinematics = _kinematics()

    command: TrackCommand = kinematics.to_track_command(
        linear_speed_cm_per_sec=20.0,
        angular_speed_deg_per_sec=0.0,
    )

    assert command.left_percent == pytest.approx(50.0)
    assert command.right_percent == pytest.approx(50.0)


def test_to_track_command_for_rotation_changes_sides_in_opposite_directions() -> None:
    """Поворот на месте даёт разные знаки командам бортов."""
    kinematics: DifferentialDriveKinematics = _kinematics()

    command: TrackCommand = kinematics.to_track_command(
        linear_speed_cm_per_sec=0.0,
        angular_speed_deg_per_sec=90.0,
    )

    assert command.left_percent < 0.0
    assert command.right_percent > 0.0
    assert abs(command.left_percent) == pytest.approx(abs(command.right_percent))


def test_to_track_command_normalizes_over_limit_and_keeps_ratio() -> None:
    """Команды за пределами диапазона нормализуются с сохранением соотношения."""
    kinematics: DifferentialDriveKinematics = _kinematics()

    command: TrackCommand = kinematics.to_track_command(
        linear_speed_cm_per_sec=60.0,
        angular_speed_deg_per_sec=180.0,
    )

    assert max(abs(command.left_percent), abs(command.right_percent)) == pytest.approx(100.0)
    assert command.right_percent > command.left_percent


def test_to_chassis_velocity_restores_linear_and_angular_components() -> None:
    """Обратное преобразование даёт скорость корпуса по командам бортов."""
    kinematics: DifferentialDriveKinematics = _kinematics()

    velocity: ChassisVelocity = kinematics.to_chassis_velocity(
        left_percent=25.0,
        right_percent=75.0,
    )

    assert velocity.linear_speed_cm_per_sec == pytest.approx(20.0)
    assert velocity.angular_speed_deg_per_sec == pytest.approx(57.2957795, rel=1e-6)
