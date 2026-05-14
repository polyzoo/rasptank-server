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


def _service_with_accel_fusion(alpha: float = 1.0) -> tuple[L2Service, FakeMotor]:
    """Создать L2 с включённой accel-коррекцией скорости."""
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
        accel_speed_fusion_enabled=True,
        accel_speed_blend_alpha=alpha,
        accel_speed_limit_factor=10.0,
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
    service, _ = _service_with_accel_fusion(alpha=1.0)

    state = service.update_from_l1(
        L1SensorSnapshot(
            longitudinal_acceleration_m_s2=0.5,
            angular_speed_z_deg_per_sec=0.0,
        ),
        dt_sec=2.0,
    )

    assert state.linear_speed_cm_per_sec == pytest.approx(100.0)
    assert state.x_cm == pytest.approx(200.0)


def test_update_from_l1_without_accel_fusion_uses_kinematic_speed_only() -> None:
    """По умолчанию L2 не разгоняет v_hat по акселю: берём скорость из кинематики."""
    service, _ = _service()
    service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=20.0, angular_speed_deg_per_sec=0.0)
    )
    state = service.update_from_l1(
        L1SensorSnapshot(
            longitudinal_acceleration_m_s2=2.0,
            angular_speed_z_deg_per_sec=0.0,
        ),
        dt_sec=1.0,
    )
    assert state.linear_speed_cm_per_sec == pytest.approx(20.0)


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


def test_update_from_l1_stationary_learns_accel_bias() -> None:
    """В покое сервис дообучает bias акселерометра и не разгоняет интегратор."""
    service, _ = _service_with_accel_fusion(alpha=1.0)

    state = service.update_from_l1(
        L1SensorSnapshot(
            longitudinal_acceleration_m_s2=0.2,
            angular_speed_z_deg_per_sec=0.0,
        ),
        dt_sec=1.0,
    )

    assert state.linear_speed_cm_per_sec == pytest.approx(0.0)
    assert service._accel_longitudinal_bias_m_s2 == pytest.approx(0.004)
    assert service._accel_integrated_speed_cm_per_sec == pytest.approx(0.0)


def _service_with_feedback(**fb_kwargs: object) -> tuple[L2Service, FakeMotor]:
    """Создать L2 с включённой обратной связью по гироскопу."""
    from src.application.services.l2_feedback_controller import L2FeedbackController

    motor: FakeMotor = FakeMotor()
    kinematics = DifferentialDriveKinematics(
        track_width_cm=20.0,
        left_track_max_speed_cm_per_sec=40.0,
        right_track_max_speed_cm_per_sec=40.0,
    )
    defaults: dict[str, object] = {
        "k_omega": 1.0,
        "k_theta": 0.5,
        "k_i": 0.0,
        "i_max": 10.0,
        "u_max_corr": 20.0,
        "u_trim": 0.0,
        "k_omega_turn": 0.5,
        "u_max_turn": 15.0,
    }
    defaults.update(fb_kwargs)
    feedback_controller = L2FeedbackController(**defaults)  # type: ignore[arg-type]
    service = L2Service(
        kinematics=kinematics,
        pose_estimator=PoseEstimator(),
        velocity_controller=VelocityCommandController(
            motor_controller=motor,  # type: ignore[arg-type]
            kinematics=kinematics,
        ),
        feedback_controller=feedback_controller,
    )
    return service, motor


def test_feedback_modifies_motor_commands() -> None:
    """С обратной связью и ω_gyro≠0 команды бортов отличаются от базовых."""
    service, motor = _service_with_feedback(k_omega=2.0, k_theta=0.0, k_i=0.0)

    service.update_from_l1(
        L1SensorSnapshot(angular_speed_z_deg_per_sec=5.0),
        dt_sec=0.1,
    )

    state = service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=20.0, angular_speed_deg_per_sec=0.0)
    )

    assert state.left_percent != pytest.approx(state.right_percent)
    assert state.feedback_delta_u is not None
    assert state.feedback_delta_u != pytest.approx(0.0)


def test_feedback_heading_ref_auto_initialized() -> None:
    """Начальный курс θ_ref запоминается автоматически при первом apply_body_velocity."""
    service, _ = _service_with_feedback()

    state = service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=10.0, angular_speed_deg_per_sec=0.0)
    )

    assert state.feedback_heading_ref_deg is not None
    assert state.feedback_heading_ref_deg == pytest.approx(0.0)


def test_feedback_no_error_gives_equal_tracks() -> None:
    """Без ошибок обратная связь не меняет команды бортов."""
    service, motor = _service_with_feedback(k_omega=1.0, k_theta=1.0, k_i=0.0, u_trim=0.0)

    state = service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=20.0, angular_speed_deg_per_sec=0.0)
    )

    assert state.left_percent == pytest.approx(state.right_percent)
    assert state.feedback_delta_u == pytest.approx(0.0)


def test_feedback_reset_on_stop() -> None:
    """stop() сбрасывает feedback controller."""
    service, _ = _service_with_feedback()

    service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=10.0, angular_speed_deg_per_sec=0.0)
    )
    state = service.stop()

    assert state.feedback_delta_u is None
    assert state.feedback_heading_ref_deg is None


def test_feedback_reset_on_reset_state() -> None:
    """reset_state() сбрасывает feedback controller."""
    service, _ = _service_with_feedback()

    service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=10.0, angular_speed_deg_per_sec=0.0)
    )
    state = service.reset_state()

    assert state.feedback_delta_u is None
    assert state.feedback_heading_ref_deg is None


def test_without_feedback_state_has_none_delta_u() -> None:
    """Без feedback controller диагностические поля равны None."""
    service, _ = _service()

    state = service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=20.0, angular_speed_deg_per_sec=0.0)
    )

    assert state.feedback_delta_u is None
    assert state.feedback_heading_ref_deg is None


def test_configure_state_space() -> None:
    service, _ = _service()
    service.configure_state_space(enabled=True, t_v=1.0, t_w=2.0)
    assert service._use_state_space is True
    assert service._state_space_controller is not None
    assert service._state_space_controller.t_v == 1.0
    assert service._state_space_controller.t_w == 2.0

    service.configure_state_space(enabled=True, t_v=3.0, t_w=4.0)
    assert service._state_space_controller.t_v == 3.0
    assert service._state_space_controller.t_w == 4.0
    assert service._state_space_controller._K is None

    service.configure_state_space(enabled=False, t_v=3.0, t_w=4.0)
    assert service._use_state_space is False


def test_apply_body_velocity_state_space(monkeypatch) -> None:
    service, motor = _service()

    # Mock the controller entirely so we don't rely on scipy/numpy availability

    class MockL2StateSpaceController:
        def __init__(self, t_v, t_w):
            self.t_v = t_v
            self.t_w = t_w
            self._K = None
            self.gain_matrix = None
            self.last_error_state = None
            self.last_control_u = None

        def compute_control(self, **kwargs):
            return (1.0, 2.0)

        def reset_gains(self):
            self._K = None

    monkeypatch.setattr(
        "src.application.services.l2_state_space_controller.L2StateSpaceController",
        MockL2StateSpaceController,
    )
    service._state_space_max_track_delta_percent = 0.0

    service.configure_state_space(enabled=True, t_v=1.0, t_w=1.0)

    state = service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=20.0, angular_speed_deg_per_sec=0.0)
    )

    assert state is not None
    assert service._mps_heading_ref_deg == 0.0


def test_state_space_uses_actual_minus_reference_errors(monkeypatch) -> None:
    service, motor = _service()
    captured: dict[str, float] = {}

    class MockL2StateSpaceController:
        def __init__(self, t_v, t_w):
            self.t_v = t_v
            self.t_w = t_w
            self._K = None
            self.gain_matrix = None
            self.last_error_state = None
            self.last_control_u = None

        def compute_control(self, **kwargs):
            captured.update(kwargs)
            return (2.0, 0.1)

        def reset_gains(self):
            self._K = None

    monkeypatch.setattr(
        "src.application.services.l2_state_space_controller.L2StateSpaceController",
        MockL2StateSpaceController,
    )
    service._state_space_max_track_delta_percent = 0.0

    service.configure_state_space(enabled=True, t_v=1.0, t_w=1.0)
    service.reset_state(heading_deg=10.0, linear_speed_cm_per_sec=5.0)
    service.update_from_l1(L1SensorSnapshot(angular_speed_z_deg_per_sec=4.0), dt_sec=0.1)
    state = service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=20.0, angular_speed_deg_per_sec=6.0)
    )

    assert captured["theta_err_rad"] == pytest.approx(0.0)
    assert captured["v_err"] == pytest.approx(-20.0)
    assert captured["omega_err_rad_per_sec"] == pytest.approx(-0.034906585, rel=1e-6)
    assert state.left_percent < state.right_percent
    assert motor.commands[-1][0] < motor.commands[-1][1]


def test_state_space_recomputes_after_l1_update(monkeypatch) -> None:
    service, motor = _service()
    calls: list[dict[str, float]] = []

    class MockL2StateSpaceController:
        def __init__(self, t_v, t_w):
            self.t_v = t_v
            self.t_w = t_w
            self._K = None
            self.gain_matrix = None
            self.last_error_state = None
            self.last_control_u = None

        def compute_control(self, **kwargs):
            calls.append(kwargs)
            return (0.0, 0.0)

        def reset_gains(self):
            self._K = None

    monkeypatch.setattr(
        "src.application.services.l2_state_space_controller.L2StateSpaceController",
        MockL2StateSpaceController,
    )
    service._state_space_max_track_delta_percent = 0.0

    service.configure_state_space(enabled=True, t_v=1.0, t_w=1.0)
    service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=20.0, angular_speed_deg_per_sec=0.0)
    )
    service.update_from_l1(L1SensorSnapshot(angular_speed_z_deg_per_sec=5.0), dt_sec=0.1)

    assert len(calls) == 2
    assert calls[1]["omega_err_rad_per_sec"] == pytest.approx(0.0872664626)
    assert motor.commands[-1] == (50, 50)


def test_state_space_limits_track_command_jumps(monkeypatch) -> None:
    service, motor = _service()

    class MockL2StateSpaceController:
        def __init__(self, t_v, t_w):
            self.t_v = t_v
            self.t_w = t_w
            self._K = None
            self.gain_matrix = None
            self.last_error_state = None
            self.last_control_u = None

        def compute_control(self, **kwargs):
            return (0.0, 0.0)

        def reset_gains(self):
            self._K = None

    monkeypatch.setattr(
        "src.application.services.l2_state_space_controller.L2StateSpaceController",
        MockL2StateSpaceController,
    )

    service.configure_state_space(enabled=True, t_v=1.0, t_w=1.0)
    state = service.apply_body_velocity(
        BodyVelocityCommand(linear_speed_cm_per_sec=20.0, angular_speed_deg_per_sec=0.0)
    )

    assert motor.commands[-1] == (5, 5)
    assert state.left_percent == pytest.approx(5.0)
    assert state.right_percent == pytest.approx(5.0)
