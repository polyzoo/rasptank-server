from __future__ import annotations

from src.application.services.kinematics import (
    ChassisVelocity,
    DifferentialDriveKinematics,
    TrackCommand,
)
from src.application.services.l2_models import BodyVelocityCommand, L1SensorSnapshot, L2State
from src.application.services.pose_estimator import PoseEstimate, PoseEstimator
from src.application.services.velocity_command_controller import (
    AppliedVelocityCommand,
    VelocityCommandController,
)


class L2Service:
    """Изолированный математический контур нового решения."""

    def __init__(
        self,
        kinematics: DifferentialDriveKinematics,
        pose_estimator: PoseEstimator,
        velocity_controller: VelocityCommandController,
    ) -> None:
        """Сохранить составные части уровня L2."""
        self._kinematics: DifferentialDriveKinematics = kinematics
        self._pose_estimator: PoseEstimator = pose_estimator
        self._velocity_controller: VelocityCommandController = velocity_controller
        self._last_track_command: TrackCommand = TrackCommand(left_percent=0.0, right_percent=0.0)
        self._last_distance_cm: float | None = None

    def apply_body_velocity(self, command: BodyVelocityCommand) -> L2State:
        """Принять желаемое движение корпуса и передать команды в нижний уровень."""
        applied: AppliedVelocityCommand = self._velocity_controller.apply_command(
            linear_speed_cm_per_sec=command.linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=command.angular_speed_deg_per_sec,
        )

        self._last_track_command = TrackCommand(
            left_percent=applied.left_percent,
            right_percent=applied.right_percent,
        )

        return self.get_state()

    def stop(self) -> L2State:
        """Остановить движение нового контура."""
        self._velocity_controller.stop()
        self._last_track_command = TrackCommand(left_percent=0.0, right_percent=0.0)
        return self.get_state()

    def reset_state(
        self,
        *,
        x_cm: float = 0.0,
        y_cm: float = 0.0,
        heading_deg: float = 0.0,
        linear_speed_cm_per_sec: float = 0.0,
        angular_speed_deg_per_sec: float = 0.0,
    ) -> L2State:
        """Сбросить оценку состояния нового контура."""
        self._pose_estimator.reset(
            x_cm=x_cm,
            y_cm=y_cm,
            heading_deg=heading_deg,
            linear_speed_cm_per_sec=linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=angular_speed_deg_per_sec,
        )
        return self.get_state()

    def update_from_l1(self, snapshot: L1SensorSnapshot, dt_sec: float) -> L2State:
        """Обновить состояние по данным нижнего уровня и последней команде бортов."""
        inferred_velocity: ChassisVelocity = self._kinematics.to_chassis_velocity(
            left_percent=self._last_track_command.left_percent,
            right_percent=self._last_track_command.right_percent,
        )

        angular_speed_deg_per_sec: float = (
            snapshot.angular_speed_z_deg_per_sec
            if snapshot.angular_speed_z_deg_per_sec is not None
            else inferred_velocity.angular_speed_deg_per_sec
        )

        if snapshot.longitudinal_acceleration_m_s2 is not None:
            self._pose_estimator.integrate_longitudinal_acceleration(
                longitudinal_acceleration_m_s2=snapshot.longitudinal_acceleration_m_s2,
                dt_sec=dt_sec,
            )
            current_linear_speed_cm_per_sec: float = (
                self._pose_estimator.snapshot().linear_speed_cm_per_sec
            )

        else:
            current_linear_speed_cm_per_sec: float = inferred_velocity.linear_speed_cm_per_sec

        self._pose_estimator.update_from_velocity(
            linear_speed_cm_per_sec=current_linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=angular_speed_deg_per_sec,
            dt_sec=dt_sec,
        )

        if snapshot.yaw_deg is not None:
            self._pose_estimator.correct_heading(snapshot.yaw_deg)

        self._last_distance_cm: float = snapshot.distance_cm
        return self.get_state()

    def get_state(self) -> L2State:
        """Вернуть текущее состояние нового контура."""
        pose: PoseEstimate = self._pose_estimator.snapshot()
        return L2State(
            x_cm=pose.x_cm,
            y_cm=pose.y_cm,
            heading_deg=pose.heading_deg,
            linear_speed_cm_per_sec=pose.linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=pose.angular_speed_deg_per_sec,
            left_percent=self._last_track_command.left_percent,
            right_percent=self._last_track_command.right_percent,
            distance_cm=self._last_distance_cm,
        )
