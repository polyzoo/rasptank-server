from __future__ import annotations

import pytest

from src.application.services.l2_feedback_controller import L2FeedbackController


def _controller(**kwargs: object) -> L2FeedbackController:
    """Создать контроллер с дефолтными параметрами для тестов."""
    defaults: dict[str, object] = {
        "k_omega": 1.0,
        "k_theta": 0.5,
        "k_i": 0.1,
        "i_max": 10.0,
        "u_max_corr": 20.0,
        "u_trim": 0.0,
        "k_omega_turn": 0.5,
        "u_max_turn": 15.0,
    }
    defaults.update(kwargs)
    return L2FeedbackController(**defaults)  # type: ignore[arg-type]


class TestLinearCorrection:
    """Тесты коррекции для прямолинейного движения."""

    def test_no_error_gives_zero_correction(self) -> None:
        """Без ошибок коррекция нулевая."""
        ctrl = _controller(u_trim=0.0)
        ctrl.begin_motion(heading_deg=0.0)

        delta_u = ctrl.compute_correction(
            omega_des_deg_per_sec=0.0,
            omega_gyro_deg_per_sec=0.0,
            theta_hat_deg=0.0,
            v_des_cm_per_sec=10.0,
            dt_sec=0.1,
        )

        assert delta_u == pytest.approx(0.0)

    def test_omega_error_produces_positive_correction(self) -> None:
        """Ненулевая ω_gyro при ω_des=0 даёт коррекцию."""
        ctrl = _controller(k_omega=2.0, k_theta=0.0, k_i=0.0, u_trim=0.0)
        ctrl.begin_motion(heading_deg=0.0)

        delta_u = ctrl.compute_correction(
            omega_des_deg_per_sec=0.0,
            omega_gyro_deg_per_sec=5.0,
            theta_hat_deg=0.0,
            v_des_cm_per_sec=10.0,
            dt_sec=0.0,
        )

        assert delta_u == pytest.approx(-10.0)

    def test_theta_error_produces_correction(self) -> None:
        """Ошибка угла к начальному курсу создаёт коррекцию."""
        ctrl = _controller(k_omega=0.0, k_theta=2.0, k_i=0.0, u_trim=0.0)
        ctrl.begin_motion(heading_deg=0.0)

        delta_u = ctrl.compute_correction(
            omega_des_deg_per_sec=0.0,
            omega_gyro_deg_per_sec=0.0,
            theta_hat_deg=10.0,
            v_des_cm_per_sec=10.0,
            dt_sec=0.0,
        )

        assert delta_u == pytest.approx(-20.0)

    def test_u_trim_adds_constant_offset(self) -> None:
        """U_trim добавляет постоянную поправку."""
        ctrl = _controller(k_omega=0.0, k_theta=0.0, k_i=0.0, u_trim=5.0)
        ctrl.begin_motion(heading_deg=0.0)

        delta_u = ctrl.compute_correction(
            omega_des_deg_per_sec=0.0,
            omega_gyro_deg_per_sec=0.0,
            theta_hat_deg=0.0,
            v_des_cm_per_sec=10.0,
            dt_sec=0.0,
        )

        assert delta_u == pytest.approx(5.0)

    def test_saturation_clamps_correction(self) -> None:
        """Сатурация ограничивает коррекцию."""
        ctrl = _controller(k_omega=100.0, k_theta=0.0, k_i=0.0, u_max_corr=5.0)
        ctrl.begin_motion(heading_deg=0.0)

        delta_u = ctrl.compute_correction(
            omega_des_deg_per_sec=0.0,
            omega_gyro_deg_per_sec=10.0,
            theta_hat_deg=0.0,
            v_des_cm_per_sec=10.0,
            dt_sec=0.0,
        )

        assert delta_u == pytest.approx(-5.0)

    def test_integrator_accumulates_error(self) -> None:
        """Интегратор накапливает ошибку при повторных вызовах."""
        ctrl = _controller(k_omega=0.0, k_theta=0.0, k_i=1.0)
        ctrl.begin_motion(heading_deg=0.0)

        ctrl.compute_correction(
            omega_des_deg_per_sec=0.0,
            omega_gyro_deg_per_sec=-10.0,
            theta_hat_deg=0.0,
            v_des_cm_per_sec=10.0,
            dt_sec=1.0,
        )

        assert ctrl.integrator == pytest.approx(10.0)

    def test_integrator_clamped_by_i_max(self) -> None:
        """Интегратор ограничен лимитом I_max."""
        ctrl = _controller(k_omega=0.0, k_theta=0.0, k_i=1.0, i_max=5.0)
        ctrl.begin_motion(heading_deg=0.0)

        for _ in range(100):
            ctrl.compute_correction(
                omega_des_deg_per_sec=0.0,
                omega_gyro_deg_per_sec=-10.0,
                theta_hat_deg=0.0,
                v_des_cm_per_sec=10.0,
                dt_sec=1.0,
            )

        assert ctrl.integrator == pytest.approx(5.0)

    def test_no_heading_ref_gives_zero_theta_error(self) -> None:
        """Без begin_motion ошибка угла считается нулевой."""
        ctrl = _controller(k_omega=0.0, k_theta=2.0, k_i=0.0)

        delta_u = ctrl.compute_correction(
            omega_des_deg_per_sec=0.0,
            omega_gyro_deg_per_sec=0.0,
            theta_hat_deg=10.0,
            v_des_cm_per_sec=10.0,
            dt_sec=0.0,
        )

        assert delta_u == pytest.approx(0.0)


class TestTurnCorrection:
    """Тесты коррекции для поворота на месте."""

    def test_turn_mode_activated_when_v_des_zero(self) -> None:
        """При v_des≈0 и ω_des≠0 используется режим поворота."""
        ctrl = _controller(k_omega=0.0, k_omega_turn=2.0, u_max_turn=25.0)
        ctrl.begin_motion(heading_deg=0.0)

        delta_u = ctrl.compute_correction(
            omega_des_deg_per_sec=30.0,
            omega_gyro_deg_per_sec=20.0,
            theta_hat_deg=10.0,
            v_des_cm_per_sec=0.0,
            dt_sec=0.1,
        )

        assert delta_u == pytest.approx(20.0)

    def test_turn_saturation_clamps(self) -> None:
        """Сатурация поворота ограничивает коррекцию."""
        ctrl = _controller(k_omega_turn=100.0, u_max_turn=5.0)

        delta_u = ctrl.compute_correction(
            omega_des_deg_per_sec=30.0,
            omega_gyro_deg_per_sec=0.0,
            theta_hat_deg=0.0,
            v_des_cm_per_sec=0.0,
            dt_sec=0.0,
        )

        assert delta_u == pytest.approx(5.0)

    def test_turn_does_not_use_theta_error(self) -> None:
        """В режиме поворота ошибка угла не используется."""
        ctrl = _controller(k_omega_turn=0.0, k_theta=100.0)
        ctrl.begin_motion(heading_deg=0.0)

        delta_u = ctrl.compute_correction(
            omega_des_deg_per_sec=30.0,
            omega_gyro_deg_per_sec=30.0,
            theta_hat_deg=45.0,
            v_des_cm_per_sec=0.0,
            dt_sec=0.1,
        )

        assert delta_u == pytest.approx(0.0)


class TestAngleWrapping:
    """Тесты обёртки угла."""

    def test_wrap_across_180_boundary(self) -> None:
        """Обёртка корректно работает через границу ±180°."""
        ctrl = _controller(k_omega=0.0, k_theta=1.0, k_i=0.0)
        ctrl.begin_motion(heading_deg=170.0)

        delta_u = ctrl.compute_correction(
            omega_des_deg_per_sec=0.0,
            omega_gyro_deg_per_sec=0.0,
            theta_hat_deg=-170.0,
            v_des_cm_per_sec=10.0,
            dt_sec=0.0,
        )

        assert delta_u == pytest.approx(-20.0)


class TestReset:
    """Тесты сброса состояния."""

    def test_reset_clears_heading_ref(self) -> None:
        """reset() очищает запомненный курс."""
        ctrl = _controller()
        ctrl.begin_motion(heading_deg=45.0)
        ctrl.reset()

        assert ctrl.heading_ref_deg is None

    def test_reset_clears_integrator(self) -> None:
        """reset() обнуляет интегратор."""
        ctrl = _controller(k_i=1.0)
        ctrl.begin_motion(heading_deg=0.0)

        ctrl.compute_correction(
            omega_des_deg_per_sec=0.0,
            omega_gyro_deg_per_sec=-10.0,
            theta_hat_deg=0.0,
            v_des_cm_per_sec=10.0,
            dt_sec=1.0,
        )
        ctrl.reset()

        assert ctrl.integrator == pytest.approx(0.0)

    def test_begin_motion_resets_integrator(self) -> None:
        """begin_motion() обнуляет интегратор."""
        ctrl = _controller(k_i=1.0)
        ctrl.begin_motion(heading_deg=0.0)

        ctrl.compute_correction(
            omega_des_deg_per_sec=0.0,
            omega_gyro_deg_per_sec=-10.0,
            theta_hat_deg=0.0,
            v_des_cm_per_sec=10.0,
            dt_sec=1.0,
        )
        ctrl.begin_motion(heading_deg=90.0)

        assert ctrl.integrator == pytest.approx(0.0)
        assert ctrl.heading_ref_deg == pytest.approx(90.0)
