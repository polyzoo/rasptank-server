from __future__ import annotations

from typing import ClassVar

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

    CENTIMETERS_IN_METER: ClassVar[float] = 100.0
    DEFAULT_ACCEL_SPEED_BLEND_ALPHA: ClassVar[float] = 0.2
    DEFAULT_ACCEL_STATIONARY_THRESHOLD_M_S2: ClassVar[float] = 0.25
    DEFAULT_GYRO_STATIONARY_THRESHOLD_DEG_PER_SEC: ClassVar[float] = 2.0
    DEFAULT_ACCEL_BIAS_LEARNING_RATE: ClassVar[float] = 0.02
    DEFAULT_ACCEL_SPEED_LIMIT_FACTOR: ClassVar[float] = 1.2

    def __init__(
        self,
        kinematics: DifferentialDriveKinematics,
        pose_estimator: PoseEstimator,
        velocity_controller: VelocityCommandController,
        *,
        accel_speed_fusion_enabled: bool = False,
        accel_speed_blend_alpha: float = DEFAULT_ACCEL_SPEED_BLEND_ALPHA,
        accel_stationary_threshold_m_s2: float = DEFAULT_ACCEL_STATIONARY_THRESHOLD_M_S2,
        gyro_stationary_threshold_deg_per_sec: float = (
            DEFAULT_GYRO_STATIONARY_THRESHOLD_DEG_PER_SEC
        ),
        accel_bias_learning_rate: float = DEFAULT_ACCEL_BIAS_LEARNING_RATE,
        accel_speed_limit_factor: float = DEFAULT_ACCEL_SPEED_LIMIT_FACTOR,
    ) -> None:
        """Сохранить составные части уровня L2."""
        self._kinematics: DifferentialDriveKinematics = kinematics
        self._pose_estimator: PoseEstimator = pose_estimator
        self._velocity_controller: VelocityCommandController = velocity_controller
        self._last_track_command: TrackCommand = TrackCommand(left_percent=0.0, right_percent=0.0)
        self._last_distance_cm: float | None = None
        self._accel_speed_fusion_enabled: bool = accel_speed_fusion_enabled
        self._accel_speed_blend_alpha: float = max(0.0, min(1.0, accel_speed_blend_alpha))
        self._accel_stationary_threshold_m_s2: float = max(0.0, accel_stationary_threshold_m_s2)
        self._gyro_stationary_threshold_deg_per_sec: float = max(
            0.0,
            gyro_stationary_threshold_deg_per_sec,
        )
        self._accel_bias_learning_rate: float = max(0.0, min(1.0, accel_bias_learning_rate))
        self._accel_speed_limit_factor: float = max(0.1, accel_speed_limit_factor)
        self._accel_longitudinal_bias_m_s2: float = 0.0
        self._accel_integrated_speed_cm_per_sec: float = 0.0

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
        self._accel_integrated_speed_cm_per_sec = 0.0
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
        self._accel_integrated_speed_cm_per_sec = linear_speed_cm_per_sec
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

        current_linear_speed_cm_per_sec: float = self._estimate_linear_speed_cm_per_sec(
            snapshot=snapshot,
            inferred_linear_speed_cm_per_sec=inferred_velocity.linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=angular_speed_deg_per_sec,
            dt_sec=dt_sec,
        )

        self._pose_estimator.update_from_velocity(
            linear_speed_cm_per_sec=current_linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=angular_speed_deg_per_sec,
            dt_sec=dt_sec,
        )

        if snapshot.yaw_deg is not None:
            self._pose_estimator.correct_heading(snapshot.yaw_deg)

        self._last_distance_cm: float = snapshot.distance_cm
        return self.get_state()

    def _estimate_linear_speed_cm_per_sec(
        self,
        *,
        snapshot: L1SensorSnapshot,
        inferred_linear_speed_cm_per_sec: float,
        angular_speed_deg_per_sec: float,
        dt_sec: float,
    ) -> float:
        """Оценить линейную скорость: кинематика + безопасная accel-поддержка."""
        if (
            not self._accel_speed_fusion_enabled
            or snapshot.longitudinal_acceleration_m_s2 is None
            or dt_sec <= 0.0
        ):
            self._accel_integrated_speed_cm_per_sec = inferred_linear_speed_cm_per_sec
            return inferred_linear_speed_cm_per_sec

        measured_accel_m_s2: float = snapshot.longitudinal_acceleration_m_s2
        accel_without_bias_m_s2: float = measured_accel_m_s2 - self._accel_longitudinal_bias_m_s2

        if self._is_stationary(
            inferred_linear_speed_cm_per_sec=inferred_linear_speed_cm_per_sec,
            accel_without_bias_m_s2=accel_without_bias_m_s2,
            angular_speed_deg_per_sec=angular_speed_deg_per_sec,
        ):
            # В покое дообучаем bias и не даём интегралу разгоняться шумом.
            self._accel_longitudinal_bias_m_s2 = (
                (1.0 - self._accel_bias_learning_rate) * self._accel_longitudinal_bias_m_s2
                + self._accel_bias_learning_rate * measured_accel_m_s2
            )
            self._accel_integrated_speed_cm_per_sec = 0.0
            return inferred_linear_speed_cm_per_sec

        self._accel_integrated_speed_cm_per_sec += (
            accel_without_bias_m_s2 * self.CENTIMETERS_IN_METER * dt_sec
        )
        max_speed_cm_per_sec: float = max(
            self._kinematics.left_track_max_speed_cm_per_sec,
            self._kinematics.right_track_max_speed_cm_per_sec,
        )
        speed_limit_cm_per_sec: float = max_speed_cm_per_sec * self._accel_speed_limit_factor
        self._accel_integrated_speed_cm_per_sec = max(
            -speed_limit_cm_per_sec,
            min(speed_limit_cm_per_sec, self._accel_integrated_speed_cm_per_sec),
        )

        alpha: float = self._accel_speed_blend_alpha
        return (
            1.0 - alpha
        ) * inferred_linear_speed_cm_per_sec + alpha * self._accel_integrated_speed_cm_per_sec

    def _is_stationary(
        self,
        *,
        inferred_linear_speed_cm_per_sec: float,
        accel_without_bias_m_s2: float,
        angular_speed_deg_per_sec: float,
    ) -> bool:
        """Эвристика покоя для bias-обучения и защиты интегратора скорости."""
        return (
            abs(inferred_linear_speed_cm_per_sec) < 1.0
            and abs(accel_without_bias_m_s2) <= self._accel_stationary_threshold_m_s2
            and abs(angular_speed_deg_per_sec) <= self._gyro_stationary_threshold_deg_per_sec
        )

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
