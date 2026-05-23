import math
from dataclasses import dataclass

import pytest

try:
    import numpy as np  # noqa: F401
    import scipy  # noqa: F401

    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

from src.application.services.l2_state_space_controller import L2StateSpaceController


@dataclass(slots=True)
class FirstOrderPlantState:
    """Минимальная модель робота для проверки знаков LQR без железа."""

    theta_rad: float = 0.0
    linear_speed_cm_per_sec: float = 0.0
    angular_speed_rad_per_sec: float = 0.0


def test_l2_state_space_controller_init():
    controller = L2StateSpaceController(t_v=0.8, t_w=0.55)
    assert controller.t_v == 0.8
    assert controller.t_w == 0.55
    if SCIPY_AVAILABLE:
        assert controller.Q is not None
        assert controller.R is not None
        assert controller.Q.diagonal().tolist() == [10.0, 15.0, 8.0, 1.0, 1.0]


def test_compute_gains():
    if not SCIPY_AVAILABLE:
        pytest.skip("scipy is not available")

    controller = L2StateSpaceController(t_v=0.8, t_w=0.55)
    K = controller.compute_gains(v0=18.0)
    assert K is not None
    assert K.shape == (2, 5)
    assert controller._K is not None
    assert controller._last_v0 == 18.0


def test_compute_control():
    if not SCIPY_AVAILABLE:
        pytest.skip("scipy is not available")

    controller = L2StateSpaceController(t_v=0.8, t_w=0.55)
    v_cmd, omega_cmd = controller.compute_control(
        x_err=1.0,
        y_err=2.0,
        theta_err_rad=0.1,
        v_err=0.0,
        omega_err_rad_per_sec=0.0,
        v0=20.0,
    )

    assert isinstance(v_cmd, float)
    assert isinstance(omega_cmd, float)
    assert controller.gain_matrix is not None
    assert controller.last_error_state == (1.0, 2.0, 0.1, 0.0, 0.0)
    assert controller.last_control_u == (v_cmd, omega_cmd)


def test_compute_control_stores_real_and_desired_states() -> None:
    if not SCIPY_AVAILABLE:
        pytest.skip("scipy is not available")

    controller = L2StateSpaceController(t_v=0.8, t_w=0.55)
    real_state = (1.0, 2.0, 0.1, 3.0, 0.2)
    desired_state = (10.0, 20.0, 0.3, 12.0, 0.0)

    controller.compute_control(
        x_err=9.0,
        y_err=18.0,
        theta_err_rad=0.2,
        v_err=9.0,
        omega_err_rad_per_sec=-0.2,
        v0=12.0,
        real_state=real_state,
        desired_state=desired_state,
    )

    assert controller.last_real_state == real_state
    assert controller.last_desired_state == desired_state


def test_control_signs_reduce_speed_and_omega_errors() -> None:
    if not SCIPY_AVAILABLE:
        pytest.skip("scipy is not available")

    controller = L2StateSpaceController(t_v=0.8, t_w=0.55)

    v_corr, _ = controller.compute_control(
        x_err=0.0,
        y_err=0.0,
        theta_err_rad=0.0,
        v_err=5.0,
        omega_err_rad_per_sec=0.0,
        v0=10.0,
    )
    _, omega_corr = controller.compute_control(
        x_err=0.0,
        y_err=0.0,
        theta_err_rad=0.0,
        v_err=0.0,
        omega_err_rad_per_sec=-0.3,
        v0=10.0,
    )

    assert v_corr > 0.0
    assert omega_corr < 0.0


def test_gain_matrix_recomputes_only_when_nominal_speed_changes_enough() -> None:
    if not SCIPY_AVAILABLE:
        pytest.skip("scipy is not available")

    controller = L2StateSpaceController(t_v=0.8, t_w=0.55)

    controller.compute_control(
        x_err=0.0,
        y_err=0.0,
        theta_err_rad=0.0,
        v_err=0.0,
        omega_err_rad_per_sec=0.0,
        v0=10.0,
    )
    assert controller._last_v0 == pytest.approx(10.0)

    controller.compute_control(
        x_err=0.0,
        y_err=0.0,
        theta_err_rad=0.0,
        v_err=0.0,
        omega_err_rad_per_sec=0.0,
        v0=10.4,
    )
    assert controller._last_v0 == pytest.approx(10.0)

    controller.compute_control(
        x_err=0.0,
        y_err=0.0,
        theta_err_rad=0.0,
        v_err=0.0,
        omega_err_rad_per_sec=0.0,
        v0=10.6,
    )
    assert controller._last_v0 == pytest.approx(10.6)


def test_lqr_stabilizes_first_order_speed_and_heading_model() -> None:
    if not SCIPY_AVAILABLE:
        pytest.skip("scipy is not available")

    controller = L2StateSpaceController(t_v=0.8, t_w=0.55)
    state = FirstOrderPlantState(
        theta_rad=0.2,
        linear_speed_cm_per_sec=0.0,
        angular_speed_rad_per_sec=0.4,
    )
    desired_v = 10.0
    desired_omega = 0.0
    dt_sec = 0.02

    for _ in range(250):
        v_corr, omega_corr = controller.compute_control(
            x_err=0.0,
            y_err=0.0,
            theta_err_rad=-state.theta_rad,
            v_err=desired_v - state.linear_speed_cm_per_sec,
            omega_err_rad_per_sec=desired_omega - state.angular_speed_rad_per_sec,
            v0=desired_v,
        )
        v_cmd = desired_v + max(-20.0, min(20.0, v_corr))
        omega_cmd = desired_omega + max(-math.radians(90.0), min(math.radians(90.0), omega_corr))

        state.linear_speed_cm_per_sec += (
            (v_cmd - state.linear_speed_cm_per_sec) / controller.t_v
        ) * dt_sec
        state.angular_speed_rad_per_sec += (
            (omega_cmd - state.angular_speed_rad_per_sec) / controller.t_w
        ) * dt_sec
        state.theta_rad += state.angular_speed_rad_per_sec * dt_sec

    assert state.linear_speed_cm_per_sec == pytest.approx(desired_v, abs=0.4)
    assert state.angular_speed_rad_per_sec == pytest.approx(0.0, abs=0.04)
    assert state.theta_rad == pytest.approx(0.0, abs=0.08)


def test_diagnostic_properties_are_none_before_first_control() -> None:
    controller = L2StateSpaceController(t_v=0.8, t_w=0.55)

    assert controller.gain_matrix is None
    assert controller.last_real_state is None
    assert controller.last_desired_state is None
    assert controller.last_error_state is None
    assert controller.last_control_u is None


def test_reset_gains_clears_gain_matrix_only() -> None:
    if not SCIPY_AVAILABLE:
        pytest.skip("scipy is not available")

    controller = L2StateSpaceController(t_v=0.8, t_w=0.55)
    controller.compute_control(
        x_err=0.0,
        y_err=0.0,
        theta_err_rad=0.0,
        v_err=1.0,
        omega_err_rad_per_sec=0.0,
        v0=10.0,
    )

    assert controller.gain_matrix is not None
    assert controller.last_error_state is not None
    assert controller.last_control_u is not None

    controller.reset_gains()

    assert controller.gain_matrix is None
    assert controller.last_error_state is not None
    assert controller.last_control_u is not None


def test_compute_gains_raises_without_scipy(monkeypatch):
    import src.application.services.l2_state_space_controller as l2_ssc

    monkeypatch.setattr(l2_ssc, "SCIPY_AVAILABLE", False)
    controller = l2_ssc.L2StateSpaceController()
    with pytest.raises(RuntimeError):
        controller.compute_gains(10.0)


def test_compute_control_without_scipy(monkeypatch):
    import src.application.services.l2_state_space_controller as l2_ssc

    monkeypatch.setattr(l2_ssc, "SCIPY_AVAILABLE", False)
    controller = l2_ssc.L2StateSpaceController()
    v, w = controller.compute_control(
        x_err=0,
        y_err=0,
        theta_err_rad=0,
        v_err=0,
        omega_err_rad_per_sec=0,
        v0=10,
    )
    assert v == 0.0
    assert w == 0.0
