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
from src.application.services.l2_models import BodyVelocityCommand
from src.application.services.l2_service import L2Service
from src.application.services.l3_service import L3Service
from src.application.services.path_planner import PathPlanner
from src.application.services.pose_estimator import PoseEstimator
from src.application.services.velocity_command_controller import VelocityCommandController
from src.config.settings import Settings


class FakeMotor:
    """Заглушка моторов для фабрик нового контура."""

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
    """Фабрика L2 собирает единый изолированный сервис нового контура."""
    settings: Settings = Settings()
    motor: FakeMotor = FakeMotor()

    service = create_l2_service(settings, motor)  # type: ignore[arg-type]

    assert isinstance(service, L2Service)
    state = service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=0.0, angular_speed_deg_per_sec=0.0)
    )
    assert state.left_percent == 0.0
    assert motor.track_commands == [(0, 0)]


def test_create_l1_service_builds_clean_hardware_layer() -> None:
    """Фабрика L1 собирает чистый сервис нового нижнего уровня."""
    settings: Settings = Settings()

    service = create_l1_service(settings)

    assert isinstance(service, L1Service)


def test_create_goal_point_controller_uses_settings() -> None:
    """Фабрика контроллера цели переносит коэффициенты L3 из настроек."""
    settings: Settings = Settings(
        L3_POSITION_TOLERANCE_CM=4.0,
        L3_LINEAR_SPEED_GAIN=1.5,
        L3_ANGULAR_SPEED_GAIN=3.0,
        L3_MAX_LINEAR_SPEED_CM_PER_SEC=18.0,
        L3_MAX_ANGULAR_SPEED_DEG_PER_SEC=150.0,
        L3_OBSTACLE_STOP_DISTANCE_CM=22.0,
        L3_OBSTACLE_SLOWDOWN_DISTANCE_CM=44.0,
    )

    controller = create_goal_point_controller(settings)

    assert isinstance(controller, GoalPointController)
    assert controller.position_tolerance_cm == 4.0
    assert controller.linear_speed_gain == 1.5
    assert controller.angular_speed_gain == 3.0
    assert controller.max_linear_speed_cm_per_sec == 18.0
    assert controller.max_angular_speed_deg_per_sec == 150.0
    assert controller.obstacle_stop_distance_cm == 22.0
    assert controller.obstacle_slowdown_distance_cm == 44.0


def test_create_path_planner_uses_settings() -> None:
    """Фабрика планировщика переносит параметры обхода препятствий из настроек."""
    settings: Settings = Settings(
        L3_PLANNER_OBSTACLE_CLEARANCE_CM=6.0,
        L3_PLANNER_MAX_DETOUR_OFFSET_CM=35.0,
        L3_PLANNER_MAX_WAYPOINTS=12,
    )

    planner = create_path_planner(settings)

    assert isinstance(planner, PathPlanner)
    assert planner.obstacle_clearance_cm == 6.0
    assert planner.max_detour_offset_cm == 35.0
    assert planner.max_waypoints == 12


def test_create_l3_service_builds_isolated_navigation_layer() -> None:
    """Фабрика L3 собирает верхний уровень нового контура поверх L2."""
    settings: Settings = Settings()
    motor: FakeMotor = FakeMotor()
    l2_service = create_l2_service(settings, motor)  # type: ignore[arg-type]

    service = create_l3_service(settings, l2_service)

    assert isinstance(service, L3Service)
    assert service.get_state().status == "idle"
    assert service.get_state().mode == "idle"
    assert service.get_state().planner_status == "idle"
    assert service.unknown_obstacle_radius_cm == settings.l3_unknown_obstacle_radius_cm


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
