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
    """Уровень L2 для оценки позы и управления скоростью корпуса."""

    CENTIMETERS_IN_METER: ClassVar[float] = 100.0
    DEFAULT_ACCEL_SPEED_BLEND_ALPHA: ClassVar[float] = 0.05
    DEFAULT_ACCEL_STATIONARY_THRESHOLD_M_S2: ClassVar[float] = 0.30
    DEFAULT_GYRO_STATIONARY_THRESHOLD_DEG_PER_SEC: ClassVar[float] = 2.5
    DEFAULT_ACCEL_BIAS_LEARNING_RATE: ClassVar[float] = 0.03
    DEFAULT_ACCEL_SPEED_LIMIT_FACTOR: ClassVar[float] = 1.1
    DEFAULT_STATE_SPACE_MAX_LINEAR_CORR_CM_PER_SEC: ClassVar[float] = 20.0
    DEFAULT_STATE_SPACE_MAX_ANGULAR_CORR_DEG_PER_SEC: ClassVar[float] = 90.0
    DEFAULT_STATE_SPACE_MAX_TRACK_DELTA_PERCENT: ClassVar[float] = 5.0
    DEFAULT_STATE_SPACE_MIN_MOVING_TRACK_PERCENT: ClassVar[float] = 15.0
    DEFAULT_STATE_SPACE_TARGET_TOLERANCE_CM: ClassVar[float] = 2.0
    DEFAULT_STATE_SPACE_REVERSE_TURN_TOLERANCE_DEG: ClassVar[float] = 1.0
    STATE_SPACE_TURN_EPSILON_DEG_PER_SEC: ClassVar[float] = 1e-6
    STATE_SPACE_LINE_EPSILON_CM: ClassVar[float] = 1e-6

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
        state_space_turn_in_place_heading_error_deg: float = 5.0,
        state_space_turn_in_place_angular_gain: float = 2.0,
        state_space_turn_in_place_max_angular_speed_deg_per_sec: float = 120.0,
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
        self._last_state_space_target_ab: tuple[float, float] | None = None
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
        self._state_space_turn_in_place_heading_error_deg: float = max(
            0.0,
            state_space_turn_in_place_heading_error_deg,
        )
        self._state_space_turn_in_place_angular_gain: float = max(
            0.0,
            state_space_turn_in_place_angular_gain,
        )
        self._state_space_turn_in_place_max_angular_speed_deg_per_sec: float = max(
            0.0,
            state_space_turn_in_place_max_angular_speed_deg_per_sec,
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
        """Принять скорость корпуса и передать команды в L1."""
        self._last_body_velocity_command = command

        if self._use_state_space and self._state_space_controller is not None:
            turn_command: BodyVelocityCommand | None = self._state_space_turn_command(command)
            if turn_command is None:
                return self._apply_body_velocity_state_space(command)
            return self._apply_body_velocity_without_state_space(turn_command)

        return self._apply_body_velocity_without_state_space(command)

    def _apply_body_velocity_without_state_space(self, command: BodyVelocityCommand) -> L2State:
        """Применить команду без МПС."""
        self._mps_heading_ref_deg = None
        self._last_state_space_target_ab = None

        if self._feedback_controller is None:
            return self._apply_body_velocity_open_loop(command)

        return self._apply_body_velocity_closed_loop(command)

    def _apply_body_velocity_open_loop(self, command: BodyVelocityCommand) -> L2State:
        """Применить команду через прямую кинематику."""
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
        """Применить команду через МПС."""
        controller = self._state_space_controller
        pose: PoseEstimate = self._pose_estimator.snapshot()

        if self._mps_heading_ref_deg is None:
            self._mps_heading_ref_deg = pose.heading_deg

        v_des = command.linear_speed_cm_per_sec
        target_is_set = command.target_x_cm is not None and command.target_y_cm is not None
        omega_des = 0.0 if target_is_set else command.angular_speed_deg_per_sec
        v0 = self._state_space_nominal_speed(command)
        omega_real_deg_per_sec = self._last_omega_gyro_deg_per_sec

        real_state = (
            pose.x_cm,
            pose.y_cm,
            math.radians(pose.heading_deg),
            pose.linear_speed_cm_per_sec,
            math.radians(omega_real_deg_per_sec),
        )

        target_ab: tuple[float, float] | None = None
        if target_is_set:
            if self._state_space_target_reached(
                pose=pose,
                target_x_cm=command.target_x_cm,  # type: ignore[arg-type]
                target_y_cm=command.target_y_cm,  # type: ignore[arg-type]
            ):
                self._velocity_controller.stop()
                self._last_track_command = TrackCommand(left_percent=0.0, right_percent=0.0)
                self._last_body_velocity_command = None
                self._last_delta_u = None
                return self.get_state()

            desired_x_cm, desired_y_cm, theta_des_deg, a_cm, b_cm = (
                self._state_space_desired_line_state(
                    pose=pose,
                    target_x_cm=command.target_x_cm,  # type: ignore[arg-type]
                    target_y_cm=command.target_y_cm,  # type: ignore[arg-type]
                )
            )
            target_ab = (a_cm, b_cm)
            desired_state = (
                desired_x_cm,
                desired_y_cm,
                math.radians(theta_des_deg),
                v_des,
                0.0,
            )
        else:
            desired_state = (
                pose.x_cm,
                pose.y_cm,
                math.radians(self._mps_heading_ref_deg),
                v_des,
                math.radians(omega_des),
            )

        x_err = desired_state[0] - real_state[0]
        y_err = desired_state[1] - real_state[1]
        theta_err_rad = math.radians(
            self._normalize_angle_deg(math.degrees(desired_state[2]) - math.degrees(real_state[2]))
        )
        v_err = desired_state[3] - real_state[3]
        omega_err_rad_per_sec = desired_state[4] - real_state[4]

        v_cmd_corr, omega_cmd_corr_rad_per_sec = controller.compute_control(
            x_err=x_err,
            y_err=y_err,
            theta_err_rad=theta_err_rad,
            v_err=v_err,
            omega_err_rad_per_sec=omega_err_rad_per_sec,
            v0=v0,
            real_state=real_state,
            desired_state=desired_state,
        )
        self._last_state_space_target_ab = target_ab

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

        final_v_cmd = self._cap_linear_speed_to_desired(
            desired_linear_speed_cm_per_sec=v_des,
            corrected_linear_speed_cm_per_sec=v_des + v_cmd_corr,
        )
        final_omega_cmd = omega_des + omega_cmd_corr

        base_command: TrackCommand = self._velocity_controller.compute_command(
            linear_speed_cm_per_sec=final_v_cmd,
            angular_speed_deg_per_sec=final_omega_cmd,
        )

        normalized_command: TrackCommand = self._kinematics._normalize_track_command(base_command)
        applied_command: TrackCommand = self._limit_track_delta(normalized_command)
        self._velocity_controller.send_track_command(applied_command)

        self._last_track_command = applied_command
        self._last_delta_u = v_cmd_corr  # Коррекция линейной скорости для диагностики.

        return self.get_state()

    def _apply_body_velocity_closed_loop(self, command: BodyVelocityCommand) -> L2State:
        """Применить команду через обратную связь L2."""
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
        """Остановить уровень L2."""
        self._velocity_controller.stop()
        self._last_track_command = TrackCommand(left_percent=0.0, right_percent=0.0)
        self._accel_integrated_speed_cm_per_sec = 0.0
        self._last_delta_u = None
        self._last_body_velocity_command = None
        self._mps_heading_ref_deg = None
        self._last_state_space_target_ab = None
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
        """Сбросить состояние L2."""
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
        self._last_state_space_target_ab = None
        if self._feedback_controller is not None:
            self._feedback_controller.reset()
        return self.get_state()

    def update_from_l1(self, snapshot: L1SensorSnapshot, dt_sec: float) -> L2State:
        """Обновить состояние по данным L1 и последней команде бортов."""
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
            turn_command: BodyVelocityCommand | None = self._state_space_turn_command(
                self._last_body_velocity_command
            )
            if turn_command is None:
                return self._apply_body_velocity_state_space(self._last_body_velocity_command)
            return self._apply_body_velocity_without_state_space(turn_command)

        return self.get_state()

    def _state_space_turn_command(self, command: BodyVelocityCommand) -> BodyVelocityCommand | None:
        """Вернуть команду разворота на месте, если МПС ещё нельзя вести вперёд."""
        target_is_set = command.target_x_cm is not None and command.target_y_cm is not None
        if (
            not target_is_set
            and abs(command.angular_speed_deg_per_sec) > self.STATE_SPACE_TURN_EPSILON_DEG_PER_SEC
        ):
            return BodyVelocityCommand(
                linear_speed_cm_per_sec=0.0,
                angular_speed_deg_per_sec=command.angular_speed_deg_per_sec,
                nominal_linear_speed_cm_per_sec=command.nominal_linear_speed_cm_per_sec,
            )

        heading_error_deg: float | None = self._state_space_target_heading_error_deg(command)
        if heading_error_deg is None:
            return None

        if self._state_space_command_target_reached(command):
            return None

        turn_tolerance_deg: float = self._state_space_turn_tolerance_deg(command)
        if abs(heading_error_deg) <= turn_tolerance_deg:
            return None

        angular_speed_deg_per_sec: float = self._clamp(
            heading_error_deg * self._state_space_turn_in_place_angular_gain,
            -self._state_space_turn_in_place_max_angular_speed_deg_per_sec,
            self._state_space_turn_in_place_max_angular_speed_deg_per_sec,
        )
        angular_speed_deg_per_sec = self._apply_min_state_space_turn_speed(
            angular_speed_deg_per_sec
        )
        return BodyVelocityCommand(
            linear_speed_cm_per_sec=0.0,
            angular_speed_deg_per_sec=angular_speed_deg_per_sec,
            nominal_linear_speed_cm_per_sec=command.nominal_linear_speed_cm_per_sec,
        )

    def _state_space_target_heading_error_deg(
        self,
        command: BodyVelocityCommand,
    ) -> float | None:
        """Посчитать ошибку курса до целевой точки команды МПС."""
        if command.target_x_cm is None or command.target_y_cm is None:
            return None

        pose: PoseEstimate = self._pose_estimator.snapshot()
        delta_x_cm: float = command.target_x_cm - pose.x_cm
        delta_y_cm: float = command.target_y_cm - pose.y_cm
        if math.hypot(delta_x_cm, delta_y_cm) <= self.STATE_SPACE_LINE_EPSILON_CM:
            return None

        target_heading_deg: float = math.degrees(math.atan2(delta_y_cm, delta_x_cm))
        return self._normalize_angle_deg(target_heading_deg - pose.heading_deg)

    def _state_space_command_target_reached(self, command: BodyVelocityCommand) -> bool:
        """Проверить, что команда с целью уже дошла до конечной точки."""
        if command.target_x_cm is None or command.target_y_cm is None:
            return False

        pose: PoseEstimate = self._pose_estimator.snapshot()
        return self._state_space_target_reached(
            pose=pose,
            target_x_cm=command.target_x_cm,
            target_y_cm=command.target_y_cm,
        )

    def _state_space_turn_tolerance_deg(self, command: BodyVelocityCommand) -> float:
        """Вернуть допуск входа в МПС для текущей цели."""
        if command.target_x_cm is None or command.target_y_cm is None:
            return self._state_space_turn_in_place_heading_error_deg

        target_heading_deg = math.degrees(math.atan2(command.target_y_cm, command.target_x_cm))
        if abs(self._normalize_angle_deg(target_heading_deg)) > 90.0:
            return min(
                self._state_space_turn_in_place_heading_error_deg,
                self.DEFAULT_STATE_SPACE_REVERSE_TURN_TOLERANCE_DEG,
            )

        return self._state_space_turn_in_place_heading_error_deg

    def _apply_min_state_space_turn_speed(self, angular_speed_deg_per_sec: float) -> float:
        """Не давать предварительному развороту МПС попасть в мёртвую зону моторов."""
        if abs(angular_speed_deg_per_sec) <= self.STATE_SPACE_TURN_EPSILON_DEG_PER_SEC:
            return 0.0

        min_track_speed_cm_per_sec: float = (
            self._kinematics.left_track_max_speed_cm_per_sec
            * self._state_space_min_moving_track_percent
            / 100.0
        )
        min_angular_speed_deg_per_sec: float = math.degrees(
            (2.0 * min_track_speed_cm_per_sec) / self._kinematics.track_width_cm
        )
        min_angular_speed_deg_per_sec = min(
            min_angular_speed_deg_per_sec,
            self._state_space_turn_in_place_max_angular_speed_deg_per_sec,
        )
        if abs(angular_speed_deg_per_sec) >= min_angular_speed_deg_per_sec:
            return angular_speed_deg_per_sec

        sign: float = 1.0 if angular_speed_deg_per_sec > 0.0 else -1.0
        return sign * min_angular_speed_deg_per_sec

    def _normalize_angle_deg(self, angle_deg: float) -> float:
        """Нормализовать угол в диапазон [-180, 180)."""
        half_turn, full_turn = 180.0, 360.0
        return ((angle_deg + half_turn) % full_turn) - half_turn

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        """Ограничить значение указанными пределами."""
        return max(lower, min(upper, value))

    def _cap_linear_speed_to_desired(
        self,
        *,
        desired_linear_speed_cm_per_sec: float,
        corrected_linear_speed_cm_per_sec: float,
    ) -> float:
        """Не давать МПС разгонять корпус быстрее заданной пользователем скорости."""
        if desired_linear_speed_cm_per_sec > 0.0:
            return self._clamp(
                corrected_linear_speed_cm_per_sec,
                0.0,
                desired_linear_speed_cm_per_sec,
            )
        if desired_linear_speed_cm_per_sec < 0.0:
            return self._clamp(
                corrected_linear_speed_cm_per_sec,
                desired_linear_speed_cm_per_sec,
                0.0,
            )
        return 0.0

    def _state_space_nominal_speed(self, command: BodyVelocityCommand) -> float:
        """Выбрать скорость v0 для линеаризации МПС."""
        speed = (
            command.nominal_linear_speed_cm_per_sec
            if command.nominal_linear_speed_cm_per_sec is not None
            else command.linear_speed_cm_per_sec
        )
        return speed if abs(speed) > 1.0 else 1.0

    def _state_space_target_reached(
        self,
        *,
        pose: PoseEstimate,
        target_x_cm: float,
        target_y_cm: float,
    ) -> bool:
        """Проверить достижение конечной точки команды МПС по прогрессу вдоль прямой."""
        target_distance_cm = math.hypot(target_x_cm, target_y_cm)
        if target_distance_cm <= self.STATE_SPACE_LINE_EPSILON_CM:
            return True

        progress_cm = (pose.x_cm * target_x_cm + pose.y_cm * target_y_cm) / target_distance_cm
        return progress_cm >= target_distance_cm - self.DEFAULT_STATE_SPACE_TARGET_TOLERANCE_CM

    def _state_space_desired_line_state(
        self,
        *,
        pose: PoseEstimate,
        target_x_cm: float,
        target_y_cm: float,
    ) -> tuple[float, float, float, float, float]:
        """Рассчитать желаемое состояние МПС для прямой к цели."""
        b_cm = target_x_cm
        a_cm = target_y_cm
        theta_des_rad = math.atan2(a_cm, b_cm)
        theta_des_deg = math.degrees(theta_des_rad)
        target_distance_cm = math.hypot(target_x_cm, target_y_cm)
        if target_distance_cm <= self.STATE_SPACE_LINE_EPSILON_CM:
            return pose.x_cm, pose.y_cm, theta_des_deg, a_cm, b_cm

        unit_x = target_x_cm / target_distance_cm
        unit_y = target_y_cm / target_distance_cm
        progress_cm = pose.x_cm * unit_x + pose.y_cm * unit_y
        desired_x_cm = unit_x * progress_cm
        desired_y_cm = unit_y * progress_cm
        return desired_x_cm, desired_y_cm, theta_des_deg, a_cm, b_cm

    def _limit_track_delta(self, target: TrackCommand) -> TrackCommand:
        """Ограничить скачок команд бортов между соседними шагами МПС."""
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
        """Поднять ненулевые команды выше минимального порога моторов."""
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
        """Сохранить знак команды и учесть минимальный порог движения."""
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
        """Оценить линейную скорость по кинематике и акселерометру."""
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
            # В покое обновляем смещение акселерометра и подавляем накопление шума.
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
        """Проверить состояние покоя для смещения акселерометра."""
        return (
            abs(inferred_linear_speed_cm_per_sec) < 1.0
            and abs(accel_without_bias_m_s2) <= self._accel_stationary_threshold_m_s2
            and abs(angular_speed_deg_per_sec) <= self._gyro_stationary_threshold_deg_per_sec
        )

    def get_state(self) -> L2State:
        """Вернуть состояние L2."""
        pose: PoseEstimate = self._pose_estimator.snapshot()

        feedback_heading_ref_deg: float | None = None
        if self._feedback_controller is not None:
            feedback_heading_ref_deg = self._feedback_controller.heading_ref_deg

        state_space_gain_k = None
        state_space_real_x = None
        state_space_desired_x = None
        state_space_error_x = None
        state_space_target_ab = None
        state_space_control_u = None
        if self._use_state_space and self._state_space_controller is not None:
            state_space_gain_k = self._state_space_controller.gain_matrix
            state_space_real_x = getattr(self._state_space_controller, "last_real_state", None)
            state_space_desired_x = getattr(
                self._state_space_controller,
                "last_desired_state",
                None,
            )
            state_space_error_x = self._state_space_controller.last_error_state
            state_space_target_ab = getattr(self, "_last_state_space_target_ab", None)
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
            state_space_real_x=state_space_real_x,
            state_space_desired_x=state_space_desired_x,
            state_space_error_x=state_space_error_x,
            state_space_target_ab=state_space_target_ab,
            state_space_control_u=state_space_control_u,
        )
