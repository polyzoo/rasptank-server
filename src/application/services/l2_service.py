from __future__ import annotations

import math
from typing import ClassVar

from src.application.services.kinematics import (
    ChassisVelocity,
    DifferentialDriveKinematics,
    TrackCommand,
)
from src.application.services.l2_feedback_controller import L2FeedbackController
from src.application.services.l2_models import BodyVelocityCommand, L1SensorSnapshot, L2State
from src.application.services.l2_state_space_controller import L2StateSpaceController
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
    DEFAULT_STATE_SPACE_MAX_LINEAR_CORR_CM_PER_SEC: ClassVar[float] = 20.0
    DEFAULT_STATE_SPACE_MAX_ANGULAR_CORR_DEG_PER_SEC: ClassVar[float] = 90.0
    DEFAULT_STATE_SPACE_MAX_TRACK_DELTA_PERCENT: ClassVar[float] = 5.0
    DEFAULT_STATE_SPACE_MIN_MOVING_TRACK_PERCENT: ClassVar[float] = 15.0

    def __init__(
        self,
        kinematics: DifferentialDriveKinematics,
        pose_estimator: PoseEstimator,
        velocity_controller: VelocityCommandController,
        *,
        feedback_controller: L2FeedbackController | None = None,
        accel_speed_fusion_enabled: bool = False,
        accel_speed_blend_alpha: float = DEFAULT_ACCEL_SPEED_BLEND_ALPHA,
        accel_stationary_threshold_m_s2: float = DEFAULT_ACCEL_STATIONARY_THRESHOLD_M_S2,
        gyro_stationary_threshold_deg_per_sec: float = (
            DEFAULT_GYRO_STATIONARY_THRESHOLD_DEG_PER_SEC
        ),
        accel_bias_learning_rate: float = DEFAULT_ACCEL_BIAS_LEARNING_RATE,
        accel_speed_limit_factor: float = DEFAULT_ACCEL_SPEED_LIMIT_FACTOR,
        state_space_max_linear_corr_cm_per_sec: float = (
            DEFAULT_STATE_SPACE_MAX_LINEAR_CORR_CM_PER_SEC
        ),
        state_space_max_angular_corr_deg_per_sec: float = (
            DEFAULT_STATE_SPACE_MAX_ANGULAR_CORR_DEG_PER_SEC
        ),
        state_space_max_track_delta_percent: float = DEFAULT_STATE_SPACE_MAX_TRACK_DELTA_PERCENT,
        state_space_min_moving_track_percent: float = (
            DEFAULT_STATE_SPACE_MIN_MOVING_TRACK_PERCENT
        ),
    ) -> None:
        """Сохранить составные части уровня L2."""
        self._kinematics: DifferentialDriveKinematics = kinematics
        self._pose_estimator: PoseEstimator = pose_estimator
        self._velocity_controller: VelocityCommandController = velocity_controller
        self._feedback_controller: L2FeedbackController | None = feedback_controller
        self._last_track_command: TrackCommand = TrackCommand(left_percent=0.0, right_percent=0.0)
        self._last_distance_cm: float | None = None
        self._last_delta_u: float | None = None
        self._last_omega_gyro_deg_per_sec: float = 0.0
        self._last_body_velocity_command: BodyVelocityCommand | None = None
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
        self._state_space_controller: L2StateSpaceController | None = None
        self._use_state_space: bool = False
        self._mps_heading_ref_deg: float | None = None
        self._state_space_max_linear_corr_cm_per_sec: float = max(
            0.0,
            state_space_max_linear_corr_cm_per_sec,
        )
        self._state_space_max_angular_corr_deg_per_sec: float = max(
            0.0,
            state_space_max_angular_corr_deg_per_sec,
        )
        self._state_space_max_track_delta_percent: float = max(
            0.0,
            state_space_max_track_delta_percent,
        )
        self._state_space_min_moving_track_percent: float = max(
            0.0,
            state_space_min_moving_track_percent,
        )

    def enable_state_space_control(self, controller: L2StateSpaceController) -> None:
        """Активировать управление на базе МПС (LQR)."""
        self._state_space_controller = controller
        self._use_state_space = True

    def disable_state_space_control(self) -> None:
        """Отключить управление на базе МПС (LQR)."""
        self._use_state_space = False

    def configure_state_space(self, enabled: bool, t_v: float, t_w: float) -> None:
        """Динамически настроить параметры МПС."""
        if not enabled:
            self.disable_state_space_control()
            return

        if self._state_space_controller is None:
            from src.application.services.l2_state_space_controller import L2StateSpaceController

            self._state_space_controller = L2StateSpaceController(t_v=t_v, t_w=t_w)
        else:
            self._state_space_controller.t_v = max(0.01, t_v)
            self._state_space_controller.t_w = max(0.01, t_w)
            self._state_space_controller.reset_gains()

        self.enable_state_space_control(self._state_space_controller)

    def apply_body_velocity(self, command: BodyVelocityCommand) -> L2State:
        """Принять желаемое движение корпуса и передать команды в нижний уровень."""
        self._last_body_velocity_command = command

        if self._use_state_space and self._state_space_controller is not None:
            return self._apply_body_velocity_state_space(command)

        if self._feedback_controller is None:
            return self._apply_body_velocity_open_loop(command)

        return self._apply_body_velocity_closed_loop(command)

    def _apply_body_velocity_open_loop(self, command: BodyVelocityCommand) -> L2State:
        """Разомкнутое преобразование: кинематика → моторы без обратной связи."""
        applied: AppliedVelocityCommand = self._velocity_controller.apply_command(
            linear_speed_cm_per_sec=command.linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=command.angular_speed_deg_per_sec,
        )

        self._last_track_command = TrackCommand(
            left_percent=applied.left_percent,
            right_percent=applied.right_percent,
        )
        self._last_delta_u = None

        return self.get_state()

    def _apply_body_velocity_state_space(self, command: BodyVelocityCommand) -> L2State:
        """Замкнутое преобразование: кинематика + LQR контроллер (МПС)."""
        controller = self._state_space_controller
        pose: PoseEstimate = self._pose_estimator.snapshot()

        if self._mps_heading_ref_deg is None:
            self._mps_heading_ref_deg = pose.heading_deg

        v_des = command.linear_speed_cm_per_sec
        omega_des = command.angular_speed_deg_per_sec

        # Упрощенные ошибки состояния: пока удерживаем только курс и скорости.
        x_err, y_err = 0.0, 0.0

        # Ошибка по курсу: actual - reference, в радианах для state-space модели.
        theta_err_deg = self._normalize_angle_deg(pose.heading_deg - self._mps_heading_ref_deg)
        theta_err_rad = math.radians(theta_err_deg)

        # Ошибка по скоростям: actual - reference.
        v_err = pose.linear_speed_cm_per_sec - v_des
        omega_err_rad_per_sec = math.radians(self._last_omega_gyro_deg_per_sec - omega_des)

        v_cmd_corr, omega_cmd_corr_rad_per_sec = controller.compute_control(
            x_err=x_err,
            y_err=y_err,
            theta_err_rad=theta_err_rad,
            v_err=v_err,
            omega_err_rad_per_sec=omega_err_rad_per_sec,
            v0=v_des if abs(v_des) > 1.0 else 1.0,  # Защита от 0 в знаменателе
        )

        v_cmd_corr = self._clamp(
            v_cmd_corr,
            -self._state_space_max_linear_corr_cm_per_sec,
            self._state_space_max_linear_corr_cm_per_sec,
        )
        omega_cmd_corr = self._clamp(
            math.degrees(omega_cmd_corr_rad_per_sec),
            -self._state_space_max_angular_corr_deg_per_sec,
            self._state_space_max_angular_corr_deg_per_sec,
        )

        final_v_cmd = v_des + v_cmd_corr
        final_omega_cmd = omega_des + omega_cmd_corr

        base_command: TrackCommand = self._velocity_controller.compute_command(
            linear_speed_cm_per_sec=final_v_cmd,
            angular_speed_deg_per_sec=final_omega_cmd,
        )

        normalized_command: TrackCommand = self._kinematics._normalize_track_command(base_command)
        applied_command: TrackCommand = self._limit_track_delta(normalized_command)
        self._velocity_controller.send_track_command(applied_command)

        self._last_track_command = applied_command
        self._last_delta_u = v_cmd_corr  # Логируем коррекцию линейной скорости

        return self.get_state()

    def _apply_body_velocity_closed_loop(self, command: BodyVelocityCommand) -> L2State:
        """Замкнутое преобразование: кинематика → коррекция ΔU → моторы."""
        feedback: L2FeedbackController = self._feedback_controller  # type: ignore[assignment]
        pose: PoseEstimate = self._pose_estimator.snapshot()

        if feedback.heading_ref_deg is None:
            feedback.begin_motion(pose.heading_deg)

        base_command: TrackCommand = self._velocity_controller.compute_command(
            linear_speed_cm_per_sec=command.linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=command.angular_speed_deg_per_sec,
        )

        delta_u: float = feedback.compute_correction(
            omega_des_deg_per_sec=command.angular_speed_deg_per_sec,
            omega_gyro_deg_per_sec=self._last_omega_gyro_deg_per_sec,
            theta_hat_deg=pose.heading_deg,
            v_des_cm_per_sec=command.linear_speed_cm_per_sec,
            dt_sec=0.0,
        )
        self._last_delta_u = delta_u

        corrected_command: TrackCommand = TrackCommand(
            left_percent=base_command.left_percent - delta_u,
            right_percent=base_command.right_percent + delta_u,
        )

        normalized_command: TrackCommand = self._kinematics._normalize_track_command(
            corrected_command
        )
        self._velocity_controller.send_track_command(normalized_command)

        self._last_track_command = normalized_command

        return self.get_state()

    def stop(self) -> L2State:
        """Остановить движение нового контура."""
        self._velocity_controller.stop()
        self._last_track_command = TrackCommand(left_percent=0.0, right_percent=0.0)
        self._accel_integrated_speed_cm_per_sec = 0.0
        self._last_delta_u = None
        self._last_body_velocity_command = None
        self._mps_heading_ref_deg = None
        if self._feedback_controller is not None:
            self._feedback_controller.reset()
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
        self._last_delta_u = None
        self._last_body_velocity_command = None
        self._mps_heading_ref_deg = None
        if self._feedback_controller is not None:
            self._feedback_controller.reset()
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

        if snapshot.angular_speed_z_deg_per_sec is not None:
            self._last_omega_gyro_deg_per_sec = snapshot.angular_speed_z_deg_per_sec

        if self._use_state_space and self._last_body_velocity_command is not None:
            return self._apply_body_velocity_state_space(self._last_body_velocity_command)

        return self.get_state()

    def _normalize_angle_deg(self, angle_deg: float) -> float:
        """Нормализовать угол в диапазон [-180, 180)."""
        half_turn, full_turn = 180.0, 360.0
        return ((angle_deg + half_turn) % full_turn) - half_turn

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        """Ограничить значение указанными пределами."""
        return max(lower, min(upper, value))

    def _limit_track_delta(self, target: TrackCommand) -> TrackCommand:
        """Ограничить скачок команд гусениц между соседними шагами МПС."""
        max_delta = self._state_space_max_track_delta_percent
        if max_delta <= 0.0:
            return target

        limited = TrackCommand(
            left_percent=self._slew_value(
                current=self._last_track_command.left_percent,
                target=target.left_percent,
                max_delta=max_delta,
            ),
            right_percent=self._slew_value(
                current=self._last_track_command.right_percent,
                target=target.right_percent,
                max_delta=max_delta,
            ),
        )
        return self._apply_min_moving_track_percent(limited, target)

    def _slew_value(self, *, current: float, target: float, max_delta: float) -> float:
        """Сдвинуть значение к цели не быстрее заданного шага."""
        return current + self._clamp(target - current, -max_delta, max_delta)

    def _apply_min_moving_track_percent(
        self,
        limited: TrackCommand,
        target: TrackCommand,
    ) -> TrackCommand:
        """Поднять ненулевые команды выше deadzone моторов."""
        min_percent = self._state_space_min_moving_track_percent
        if min_percent <= 0.0:
            return limited

        return TrackCommand(
            left_percent=self._lift_track_percent(
                value=limited.left_percent,
                target=target.left_percent,
                min_percent=min_percent,
            ),
            right_percent=self._lift_track_percent(
                value=limited.right_percent,
                target=target.right_percent,
                min_percent=min_percent,
            ),
        )

    def _lift_track_percent(self, *, value: float, target: float, min_percent: float) -> float:
        """Сохранить знак команды, но не давать ненулевому борту оставаться ниже deadzone."""
        if abs(target) <= 0.0 or abs(value) >= min_percent:
            return value
        sign = 1.0 if target > 0.0 else -1.0
        return sign * min(abs(target), min_percent)

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

        feedback_heading_ref_deg: float | None = None
        if self._feedback_controller is not None:
            feedback_heading_ref_deg = self._feedback_controller.heading_ref_deg

        state_space_gain_k = None
        state_space_error_x = None
        state_space_control_u = None
        if self._use_state_space and self._state_space_controller is not None:
            state_space_gain_k = self._state_space_controller.gain_matrix
            state_space_error_x = self._state_space_controller.last_error_state
            state_space_control_u = self._state_space_controller.last_control_u

        return L2State(
            x_cm=pose.x_cm,
            y_cm=pose.y_cm,
            heading_deg=pose.heading_deg,
            linear_speed_cm_per_sec=pose.linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=pose.angular_speed_deg_per_sec,
            left_percent=self._last_track_command.left_percent,
            right_percent=self._last_track_command.right_percent,
            distance_cm=self._last_distance_cm,
            feedback_delta_u=self._last_delta_u,
            feedback_heading_ref_deg=feedback_heading_ref_deg,
            state_space_gain_k=state_space_gain_k,
            state_space_error_x=state_space_error_x,
            state_space_control_u=state_space_control_u,
        )
