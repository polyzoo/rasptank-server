import pytest

try:
    import numpy as np  # noqa: F401
    import scipy  # noqa: F401

    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

from src.application.services.l2_state_space_controller import L2StateSpaceController


def test_l2_state_space_controller_init():
    controller = L2StateSpaceController(t_v=0.8, t_w=0.55)
    assert controller.t_v == 0.8
    assert controller.t_w == 0.55
    if SCIPY_AVAILABLE:
        assert controller.Q is not None
        assert controller.R is not None


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


def test_diagnostic_properties_are_none_before_first_control() -> None:
    controller = L2StateSpaceController(t_v=0.8, t_w=0.55)

    assert controller.gain_matrix is None
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
