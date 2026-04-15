from __future__ import annotations

from src.application.factories import (
    create_differential_drive_kinematics,
    create_l1_service,
    create_l2_service,
    create_pose_estimator,
    create_velocity_command_controller,
)
from src.application.services.l1_service import L1Service
from src.application.services.l2_models import BodyVelocityCommand
from src.application.services.l2_service import L2Service
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
