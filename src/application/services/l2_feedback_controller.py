from __future__ import annotations

from typing import ClassVar


class L2FeedbackController:
    """Обратная связь по гироскопу для уровня L2.

    Вычисляет коррекцию ΔU, которая добавляется к базовым командам бортов:

        U_L = U_L⁰ − ΔU
        U_R = U_R⁰ + ΔU

    В режиме прямолинейного движения (v_des ≠ 0):

        ΔU = sat(U_trim + K_ω·e_ω + K_θ·e_θ + K_I·I)

    В режиме поворота на месте (v_des ≈ 0, ω_des ≠ 0):

        ΔU = sat(K_ω_turn · e_ω)
    """

    # Половина полного оборота для нормализации угла в симметричный диапазон.
    HALF_TURN_DEG: ClassVar[float] = 180.0

    # Полный оборот в градусах для циклической нормализации угла.
    FULL_TURN_DEG: ClassVar[float] = 360.0

    # Порог линейной скорости, ниже которого движение считается поворотом на месте.
    LINEAR_SPEED_TURN_THRESHOLD_CM_PER_SEC: ClassVar[float] = 0.5

    def __init__(
        self,
        *,
        k_omega: float,
        k_theta: float,
        k_i: float,
        i_max: float,
        u_max_corr: float,
        u_trim: float,
        k_omega_turn: float,
        u_max_turn: float,
    ) -> None:
        """Сохранить коэффициенты обратной связи."""
        self._k_omega: float = k_omega
        self._k_theta: float = k_theta
        self._k_i: float = k_i
        self._i_max: float = i_max
        self._u_max_corr: float = u_max_corr
        self._u_trim: float = u_trim
        self._k_omega_turn: float = k_omega_turn
        self._u_max_turn: float = u_max_turn

        self._heading_ref_deg: float | None = None
        self._integrator: float = 0.0

    @property
    def heading_ref_deg(self) -> float | None:
        """Вернуть запомненный начальный курс θ_ref."""
        return self._heading_ref_deg

    @property
    def integrator(self) -> float:
        """Вернуть текущее значение интегратора."""
        return self._integrator

    def begin_motion(self, heading_deg: float) -> None:
        """Запомнить начальный курс и сбросить интегратор.

        Вызывается при начале нового движения вперёд или назад.
        """
        self._heading_ref_deg = heading_deg
        self._integrator = 0.0

    def compute_correction(
        self,
        *,
        omega_des_deg_per_sec: float,
        omega_gyro_deg_per_sec: float,
        theta_hat_deg: float,
        v_des_cm_per_sec: float,
        dt_sec: float,
    ) -> float:
        """Вычислить коррекцию ΔU по текущим измерениям и желаемому движению.

        Для прямолинейного движения (|v_des| > порога):
            e_ω = ω_des − ω_gyro
            e_θ = wrap(θ_ref − θ̂)
            ΔU = sat(U_trim + K_ω·e_ω + K_θ·e_θ + K_I·I)

        Для поворота на месте (|v_des| ≤ порога, |ω_des| > 0):
            e_ω = ω_des − ω_gyro
            ΔU = sat(K_ω_turn · e_ω)
        """
        is_turning_in_place: bool = (
            abs(v_des_cm_per_sec) <= self.LINEAR_SPEED_TURN_THRESHOLD_CM_PER_SEC
            and abs(omega_des_deg_per_sec) > 0.0
        )

        if is_turning_in_place:
            return self._compute_turn_correction(
                omega_des_deg_per_sec=omega_des_deg_per_sec,
                omega_gyro_deg_per_sec=omega_gyro_deg_per_sec,
            )

        return self._compute_linear_correction(
            omega_des_deg_per_sec=omega_des_deg_per_sec,
            omega_gyro_deg_per_sec=omega_gyro_deg_per_sec,
            theta_hat_deg=theta_hat_deg,
            dt_sec=dt_sec,
        )

    def reset(self) -> None:
        """Сбросить интегратор и начальный курс."""
        self._heading_ref_deg = None
        self._integrator = 0.0

    def _compute_linear_correction(
        self,
        *,
        omega_des_deg_per_sec: float,
        omega_gyro_deg_per_sec: float,
        theta_hat_deg: float,
        dt_sec: float,
    ) -> float:
        """Коррекция для прямолинейного движения: PID по курсу."""
        e_omega: float = omega_des_deg_per_sec - omega_gyro_deg_per_sec

        e_theta: float = 0.0
        if self._heading_ref_deg is not None:
            e_theta = self._wrap_angle_deg(self._heading_ref_deg - theta_hat_deg)

        if dt_sec > 0.0:
            combined_error: float = e_omega + self._k_theta * e_theta
            self._integrator += combined_error * dt_sec
            self._integrator = self._clamp(self._integrator, -self._i_max, self._i_max)

        raw_correction: float = (
            self._u_trim
            + self._k_omega * e_omega
            + self._k_theta * e_theta
            + self._k_i * self._integrator
        )

        return self._clamp(raw_correction, -self._u_max_corr, self._u_max_corr)

    def _compute_turn_correction(
        self,
        *,
        omega_des_deg_per_sec: float,
        omega_gyro_deg_per_sec: float,
    ) -> float:
        """Коррекция для поворота на месте: P-регулятор по ω."""
        e_omega: float = omega_des_deg_per_sec - omega_gyro_deg_per_sec
        raw_correction: float = self._k_omega_turn * e_omega
        return self._clamp(raw_correction, -self._u_max_turn, self._u_max_turn)

    def _wrap_angle_deg(self, angle_deg: float) -> float:
        """Привести угол к диапазону от −180° до 180°."""
        return ((angle_deg + self.HALF_TURN_DEG) % self.FULL_TURN_DEG) - self.HALF_TURN_DEG

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        """Ограничить значение допустимым диапазоном."""
        return max(lower, min(upper, value))
