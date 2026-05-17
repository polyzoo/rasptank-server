from __future__ import annotations

from unittest.mock import MagicMock

import pytest
from pydantic import ValidationError

from src.application.factories import (
    SharedMotionHardware,
    create_differential_drive_kinematics,
    create_goal_point_controller,
    create_l1_service,
    create_l2_service,
    create_l3_service,
    create_path_planner,
    create_pose_estimator,
    create_velocity_command_controller,
)
from src.application.services.goal_point_controller import GoalPointController
from src.application.services.l1_service import L1Service
from src.application.services.l2_feedback_controller import L2FeedbackController
from src.application.services.l2_models import BodyVelocityCommand
from src.application.services.l2_service import L2Service
from src.application.services.l3_service import L3Service
from src.application.services.path_planner import PathPlanner
from src.application.services.pose_estimator import PoseEstimator
from src.application.services.velocity_command_controller import VelocityCommandController
from src.config.settings import Settings


class FakeMotor:
    """Заглушка моторов для фабрик L1-L3."""

    def __init__(self) -> None:
        """Подготовить историю команд."""
        self.track_commands: list[tuple[int, int]] = []

    def set_tracks(self, left_speed_percent: int, right_speed_percent: int) -> None:
        """Сохранить команду левого и правого борта."""
        self.track_commands.append((left_speed_percent, right_speed_percent))


def test_create_differential_drive_kinematics_uses_settings() -> None:
    """Фабрика кинематики переносит коэффициенты из настроек."""
    settings: Settings = Settings(
        TRACK_WIDTH_CM=17.0,
        LEFT_TRACK_MAX_SPEED_CM_PER_SEC=30.0,
        RIGHT_TRACK_MAX_SPEED_CM_PER_SEC=31.0,
    )

    kinematics = create_differential_drive_kinematics(settings)

    assert kinematics.track_width_cm == 17.0
    assert kinematics.left_track_max_speed_cm_per_sec == 30.0
    assert kinematics.right_track_max_speed_cm_per_sec == 31.0


def test_create_pose_estimator_returns_estimator() -> None:
    """Фабрика оценщика создаёт отдельный объект состояния."""
    estimator = create_pose_estimator()

    assert isinstance(estimator, PoseEstimator)
    assert estimator.snapshot().x_cm == 0.0


def test_create_velocity_command_controller_builds_controller() -> None:
    """Фабрика контроллера команд собирает его поверх моторов и кинематики."""
    settings: Settings = Settings()
    motor: FakeMotor = FakeMotor()

    controller = create_velocity_command_controller(settings, motor)  # type: ignore[arg-type]

    assert isinstance(controller, VelocityCommandController)
    result = controller.apply_command(linear_speed_cm_per_sec=0.0, angular_speed_deg_per_sec=0.0)
    assert motor.track_commands == [(0, 0)]
    assert result.left_percent == 0.0
    assert result.right_percent == 0.0


def test_create_l2_service_builds_isolated_math_layer() -> None:
    """Фабрика L2 собирает сервис с внутренними дефолтами алгоритма."""
    settings: Settings = Settings()
    motor: FakeMotor = FakeMotor()

    service = create_l2_service(settings, motor)  # type: ignore[arg-type]

    assert isinstance(service, L2Service)
    assert service._state_space_max_track_delta_percent == (
        L2Service.DEFAULT_STATE_SPACE_MAX_TRACK_DELTA_PERCENT
    )
    assert service._state_space_min_moving_track_percent == (
        L2Service.DEFAULT_STATE_SPACE_MIN_MOVING_TRACK_PERCENT
    )
    assert service._state_space_turn_in_place_heading_error_deg == 5.0
    assert service._state_space_turn_in_place_angular_gain == 2.0
    assert service._state_space_turn_in_place_max_angular_speed_deg_per_sec == 120.0
    state = service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=0.0, angular_speed_deg_per_sec=0.0)
    )
    assert state.left_percent == 0.0
    assert motor.track_commands == [(0, 0)]


def test_create_l2_service_with_feedback_controller() -> None:
    """Фабрика L2 собирает сервис с контроллером обратной связи, если он включен."""
    settings: Settings = Settings(L2_FEEDBACK_ENABLED=True)
    motor: FakeMotor = FakeMotor()

    service = create_l2_service(settings, motor)  # type: ignore[arg-type]

    assert isinstance(service, L2Service)
    assert service._feedback_controller is not None
    assert service._feedback_controller._k_omega == L2FeedbackController.DEFAULT_K_OMEGA


def test_create_l1_service_builds_clean_hardware_layer() -> None:
    """Фабрика L1 собирает сервис уровня L1."""
    settings: Settings = Settings()

    service = create_l1_service(settings)

    assert isinstance(service, L1Service)


def test_create_goal_point_controller_uses_controller_defaults() -> None:
    """Фабрика контроллера цели использует внутренние дефолты L3."""
    settings: Settings = Settings()

    controller = create_goal_point_controller(settings)

    assert isinstance(controller, GoalPointController)
    assert controller.position_tolerance_cm == 5.0
    assert controller.linear_speed_gain == 1.0
    assert controller.angular_speed_gain == 2.0
    assert controller.max_linear_speed_cm_per_sec == 20.0
    assert controller.max_angular_speed_deg_per_sec == 120.0
    assert controller.obstacle_stop_distance_cm == 45.0
    assert controller.obstacle_slowdown_distance_cm == 65.0


def test_create_path_planner_uses_planner_defaults() -> None:
    """Фабрика планировщика использует внутренние дефолты L3."""
    settings: Settings = Settings()

    planner = create_path_planner(settings)

    assert isinstance(planner, PathPlanner)
    assert planner.obstacle_clearance_cm == 5.0
    assert planner.max_detour_offset_cm == 40.0
    assert planner.max_waypoints == 24


def test_create_l3_service_builds_isolated_navigation_layer() -> None:
    """Фабрика L3 собирает сервис уровня L3 поверх L2."""
    settings: Settings = Settings()
    motor: FakeMotor = FakeMotor()
    l2_service = create_l2_service(settings, motor)  # type: ignore[arg-type]

    service = create_l3_service(settings, l2_service)

    assert isinstance(service, L3Service)
    assert service.get_state().status == "idle"
    assert service.get_state().mode == "idle"
    assert service.get_state().planner_status == "idle"
    assert service.unknown_obstacle_radius_cm == 8.0


def test_shared_motion_hardware_destroy_calls_components() -> None:
    """SharedMotionHardware.destroy делегирует всем устройствам."""
    motor: MagicMock = MagicMock()
    gyro: MagicMock = MagicMock()
    ultrasonic: MagicMock = MagicMock()
    head: MagicMock = MagicMock()
    hardware: SharedMotionHardware = SharedMotionHardware(
        motor_controller=motor,
        gyroscope=gyro,
        ultrasonic_sensor=ultrasonic,
        head_servo=head,
    )

    hardware.destroy()

    motor.destroy.assert_called_once()
    ultrasonic.destroy.assert_called_once()
    gyro.destroy.assert_called_once()
    head.destroy.assert_called_once()


def test_settings_rejects_invalid_motor_direction() -> None:
    """M1_DIRECTION и M2_DIRECTION принимают только ±1."""
    with pytest.raises(ValidationError):
        Settings(M1_DIRECTION=2)


def test_settings_defaults_are_safe_for_l2_hardware_trials() -> None:
    """Дефолты настроек включают штатное движение L2 через МПС."""
    settings = Settings()

    assert settings.m1_direction == 1
    assert settings.m2_direction == -1
    assert settings.head_servo_home_angle_deg == 90.0
    assert settings.update_interval_sec == 0.2
    assert settings.l2_feedback_enabled is False
    assert settings.l2_state_space_enabled is True
    assert settings.l2_state_space_t_v == 0.8
    assert settings.l2_state_space_t_w == 0.55
