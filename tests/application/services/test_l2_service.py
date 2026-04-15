from __future__ import annotations

import pytest

from src.application.services.kinematics import DifferentialDriveKinematics
from src.application.services.l2_models import BodyVelocityCommand, L1SensorSnapshot
from src.application.services.l2_service import L2Service
from src.application.services.pose_estimator import PoseEstimator
from src.application.services.velocity_command_controller import VelocityCommandController


class FakeMotor:
    """Заглушка моторов с сохранением последних команд."""

    def __init__(self) -> None:
        """Подготовить список команд."""
        self.commands: list[tuple[int, int]] = []

    def set_tracks(self, left_speed_percent: int, right_speed_percent: int) -> None:
        """Сохранить команду для обоих бортов."""
        self.commands.append((left_speed_percent, right_speed_percent))


def _service() -> tuple[L2Service, FakeMotor]:
    """Создать изолированный сервис L2 с простыми параметрами."""
    motor: FakeMotor = FakeMotor()
    kinematics = DifferentialDriveKinematics(
        track_width_cm=20.0,
        left_track_max_speed_cm_per_sec=40.0,
        right_track_max_speed_cm_per_sec=40.0,
    )
    service = L2Service(
        kinematics=kinematics,
        pose_estimator=PoseEstimator(),
        velocity_controller=VelocityCommandController(
            motor_controller=motor,  # type: ignore[arg-type]
            kinematics=kinematics,
        ),
    )
    return service, motor


def test_apply_body_velocity_passes_commands_to_l1() -> None:
    """Желаемая скорость корпуса превращается в команды бортов."""
    service, motor = _service()

    state = service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=20.0, angular_speed_deg_per_sec=0.0)
    )

    assert motor.commands == [(50, 50)]
    assert state.left_percent == pytest.approx(50.0)
    assert state.right_percent == pytest.approx(50.0)


def test_update_from_l1_uses_last_command_and_sensor_data() -> None:
    """Сервис обновляет состояние по последней команде и данным нижнего уровня."""
    service, _ = _service()
    service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=20.0, angular_speed_deg_per_sec=0.0)
    )

    state = service.update_from_l1(
        L1SensorSnapshot(
            yaw_deg=5.0,
            angular_speed_z_deg_per_sec=10.0,
            distance_cm=42.0,
        ),
        dt_sec=1.0,
    )

    assert state.x_cm > 0.0
    assert state.heading_deg == pytest.approx(5.0)
    assert state.angular_speed_deg_per_sec == pytest.approx(10.0)
    assert state.distance_cm == pytest.approx(42.0)


def test_update_from_l1_can_integrate_longitudinal_acceleration() -> None:
    """Продольное ускорение изменяет скорость и положение в оценщике."""
    service, _ = _service()

    state = service.update_from_l1(
        L1SensorSnapshot(
            longitudinal_acceleration_m_s2=0.5,
            angular_speed_z_deg_per_sec=0.0,
        ),
        dt_sec=2.0,
    )

    assert state.linear_speed_cm_per_sec == pytest.approx(100.0)
    assert state.x_cm == pytest.approx(200.0)


def test_reset_state_and_stop_keep_service_isolated() -> None:
    """Сервис умеет отдельно сбрасывать состояние и останавливать борта."""
    service, motor = _service()
    service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=20.0, angular_speed_deg_per_sec=0.0)
    )

    reset_state = service.reset_state(x_cm=10.0, y_cm=20.0, heading_deg=30.0)
    stopped_state = service.stop()

    assert reset_state.x_cm == pytest.approx(10.0)
    assert reset_state.y_cm == pytest.approx(20.0)
    assert reset_state.heading_deg == pytest.approx(30.0)
    assert motor.commands[-1] == (0, 0)
    assert stopped_state.left_percent == pytest.approx(0.0)
    assert stopped_state.right_percent == pytest.approx(0.0)
