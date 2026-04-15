from __future__ import annotations

from dataclasses import dataclass

from src.application.protocols import MotorControllerProtocol
from src.application.services.kinematics import DifferentialDriveKinematics, TrackCommand


@dataclass(frozen=True, slots=True)
class AppliedVelocityCommand:
    """Результат применения желаемой скорости корпуса."""

    linear_speed_cm_per_sec: float
    angular_speed_deg_per_sec: float
    left_percent: float
    right_percent: float


class VelocityCommandController:
    """Преобразовать скорость корпуса в команды бортов и передать их в моторы."""

    def __init__(
        self,
        motor_controller: MotorControllerProtocol,
        kinematics: DifferentialDriveKinematics,
    ) -> None:
        """Сохранить зависимости контроллера команд."""
        self._motor_controller: MotorControllerProtocol = motor_controller
        self._kinematics: DifferentialDriveKinematics = kinematics

    def apply_command(
        self,
        *,
        linear_speed_cm_per_sec: float,
        angular_speed_deg_per_sec: float,
    ) -> AppliedVelocityCommand:
        """Вычислить команды бортов и передать их в нижний уровень."""
        track_command: TrackCommand = self._kinematics.to_track_command(
            linear_speed_cm_per_sec=linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=angular_speed_deg_per_sec,
        )

        self._motor_controller.set_tracks(
            left_speed_percent=int(round(track_command.left_percent)),
            right_speed_percent=int(round(track_command.right_percent)),
        )

        return AppliedVelocityCommand(
            linear_speed_cm_per_sec=linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=angular_speed_deg_per_sec,
            left_percent=track_command.left_percent,
            right_percent=track_command.right_percent,
        )

    def stop(self) -> None:
        """Остановить оба борта."""
        self._motor_controller.set_tracks(left_speed_percent=0, right_speed_percent=0)
