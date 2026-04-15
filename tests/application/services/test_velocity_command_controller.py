from __future__ import annotations

from src.application.services.kinematics import DifferentialDriveKinematics
from src.application.services.velocity_command_controller import (
    AppliedVelocityCommand,
    VelocityCommandController,
)


class FakeMotor:
    """Заглушка моторов с сохранением команд по бортам."""

    def __init__(self) -> None:
        """Подготовить хранилище последней команды."""
        self.commands: list[tuple[int, int]] = []

    def set_tracks(self, left_speed_percent: int, right_speed_percent: int) -> None:
        """Сохранить команду для обоих бортов."""
        self.commands.append((left_speed_percent, right_speed_percent))


def test_apply_command_sends_calculated_track_command_to_motor() -> None:
    """Контроллер скорости считает команды бортов и передаёт их в моторы."""
    motor: FakeMotor = FakeMotor()
    controller: VelocityCommandController = VelocityCommandController(
        motor_controller=motor,  # type: ignore[arg-type]
        kinematics=DifferentialDriveKinematics(
            track_width_cm=20.0,
            left_track_max_speed_cm_per_sec=40.0,
            right_track_max_speed_cm_per_sec=40.0,
        ),
    )

    result: AppliedVelocityCommand = controller.apply_command(
        linear_speed_cm_per_sec=20.0,
        angular_speed_deg_per_sec=0.0,
    )

    assert motor.commands == [(50, 50)]
    assert result.left_percent == 50.0
    assert result.right_percent == 50.0


def test_stop_sends_zero_command_to_both_tracks() -> None:
    """Остановка передаёт нулевую команду обоим бортам."""
    motor: FakeMotor = FakeMotor()
    controller: VelocityCommandController = VelocityCommandController(
        motor_controller=motor,  # type: ignore[arg-type]
        kinematics=DifferentialDriveKinematics(
            track_width_cm=20.0,
            left_track_max_speed_cm_per_sec=40.0,
            right_track_max_speed_cm_per_sec=40.0,
        ),
    )

    controller.stop()

    assert motor.commands == [(0, 0)]
