from __future__ import annotations

import logging

try:
    import numpy as np
    import scipy.linalg

    SCIPY_AVAILABLE = True
except ImportError:  # pragma: no cover
    SCIPY_AVAILABLE = False

logger = logging.getLogger(__name__)


class L2StateSpaceController:
    """Контроллер на базе Модели Пространства Состояний (МПС) и LQR.

    Вычисляет оптимальные команды управления [v_cmd, omega_cmd] на основе
    линеаризованной модели дифференциального привода с учетом инерции (T_v, T_w).
    """

    def __init__(
        self,
        t_v: float = 0.8,
        t_w: float = 0.55,
        q_diag: list[float] | None = None,
        r_diag: list[float] | None = None,
    ) -> None:
        """Инициализировать параметры МПС.

        Args:
            t_v: Постоянная времени линейной скорости (сек).
            t_w: Постоянная времени угловой скорости (сек).
            q_diag: Диагональ матрицы штрафов за ошибку состояний (размер 5).
            r_diag: Диагональ матрицы штрафов за управление (размер 2).
        """
        self.t_v = max(0.01, t_v)
        self.t_w = max(0.01, t_w)

        # Штрафы по умолчанию для Q = diag([x, y, theta, v, omega])
        self.Q = np.diag(q_diag or [10.0, 10.0, 5.0, 1.0, 1.0]) if SCIPY_AVAILABLE else None

        # Штрафы по умолчанию для R = diag([v_cmd, omega_cmd])
        self.R = np.diag(r_diag or [1.0, 1.0]) if SCIPY_AVAILABLE else None

        self._K: np.ndarray | None = None
        self._last_v0: float | None = None

        if not SCIPY_AVAILABLE:
            logger.warning("Библиотеки numpy и scipy не установлены. LQR не будет работать!")

    def compute_gains(self, v0: float) -> np.ndarray:
        """Рассчитать матрицу коэффициентов K с помощью LQR для заданной скорости v0."""
        if not SCIPY_AVAILABLE:
            raise RuntimeError("Для работы МПС необходимы numpy и scipy (pip install numpy scipy)")

        # Линеаризованная матрица A
        A = np.array(
            [
                [0, 0, 0, 1, 0],
                [0, 0, v0, 0, 0],
                [0, 0, 0, 0, 1],
                [0, 0, 0, -1.0 / self.t_v, 0],
                [0, 0, 0, 0, -1.0 / self.t_w],
            ],
            dtype=float,
        )

        # Матрица управления B
        B = np.array(
            [[0, 0], [0, 0], [0, 0], [1.0 / self.t_v, 0], [0, 1.0 / self.t_w]], dtype=float
        )

        # Решение алгебраического уравнения Риккати (Continuous-time ARE)
        P = scipy.linalg.solve_continuous_are(A, B, self.Q, self.R)

        # Вычисление оптимальной матрицы K = R^-1 B^T P
        self._K = np.linalg.inv(self.R) @ B.T @ P
        self._last_v0 = v0

        return self._K

    def compute_control(
        self,
        *,
        x_err: float,
        y_err: float,
        theta_err: float,
        v_err: float,
        omega_err: float,
        v0: float,
    ) -> tuple[float, float]:
        """Вычислить команды v_cmd и omega_cmd на основе ошибки состояния."""
        if not SCIPY_AVAILABLE or self.Q is None or self.R is None:
            return 0.0, 0.0

        # Пересчитываем матрицу K, если базовая скорость сильно изменилась
        if self._K is None or self._last_v0 is None or abs(self._last_v0 - v0) > 0.5:
            self.compute_gains(v0)

        # Вектор ошибки состояний
        error_state = np.array([x_err, y_err, theta_err, v_err, omega_err])

        # Управляющее воздействие: u = -K * e
        u = -self._K @ error_state

        v_cmd = float(u[0])
        omega_cmd = float(u[1])

        return v_cmd, omega_cmd
