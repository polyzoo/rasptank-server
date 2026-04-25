from __future__ import annotations

from math import atan2, degrees, pi, sqrt


class EkfImu:
    """Упрощённый фильтр: yaw по гироскопу Z + ZUPT, roll/pitch по гравитации.

    Углы гироскопа в вызовах :meth:`predict` — радианы/сек.
    Линейное ускорение в :meth:`update` — м/с².
    """

    def __init__(
        self,
        q_angle: float = 0.001,
        q_bias: float = 0.0001,
        r_accel: float = 0.5,
        accel_gate: float = 0.3,
        g: float = 9.80665,
    ) -> None:
        """Инициализировать ковариации и пороги ZUPT.

        Args:
            q_angle: Дисперсия процесса по углу yaw (рад²/с).
            q_bias: Дисперсия процесса по bias ω_z (рад²/с³).
            r_accel: Зарезервировано (в :meth:`update` не используется).
            accel_gate: Доля от ``g``: при большем отклонении |‖a‖−g| roll/pitch не обновляются.
            g: Ожидаемая норма ускорения покоя (м/с²).
        """
        self.q_angle: float = q_angle
        self.q_bias: float = q_bias
        self._r_accel: float = r_accel
        self.accel_gate: float = accel_gate
        self.g: float = g

        self._yaw: float = 0.0
        self._gz_bias: float = 0.0
        self._P_yaw: float = 0.1
        self._P_bias: float = 0.01
        self._P_cross: float = 0.0

        self._home_roll: float = 0.0
        self._home_pitch: float = 0.0

        self._roll: float = 0.0
        self._pitch: float = 0.0

        self._zupt_enter: float = 0.015
        self._zupt_exit: float = 0.04
        self._zupt_accel_enter: float = 0.12
        self._zupt_accel_exit: float = 0.30
        self._zupt_confirm: int = 5
        self._zupt_count: int = 0
        self._stationary: bool = False

        self._r_zupt: float = 0.0005

    def reset(self) -> None:
        """Сбросить оценки углов, bias и флаги стационарности (без смены home)."""
        self._yaw = 0.0
        self._gz_bias = 0.0
        self._P_yaw = 0.1
        self._P_bias = 0.01
        self._P_cross = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._stationary = False
        self._zupt_count = 0

    def init_from_calibration(
        self,
        roll: float,
        pitch: float,
        yaw: float,
        gyro_bias: tuple[float, float, float],
    ) -> None:
        """Задать «домашние» углы после калибровки и сузить доверие к оценке.

        Args:
            roll: Опорный roll (рад) — среднее из акселя при покое.
            pitch: Опорный pitch (рад).
            yaw: Текущий yaw (рад), обычно 0 после выставления нуля.
            gyro_bias: Кортеж bias гироскопа (совместимость API). Для Z — внутренний bias фильтра.
        """
        self._home_roll = roll
        self._home_pitch = pitch
        self._yaw = yaw
        _ = gyro_bias
        self._gz_bias = 0.0
        self._P_yaw = 0.001
        self._P_bias = 0.0001
        self._P_cross = 0.0
        self._stationary = False
        self._zupt_count = 0

    @property
    def stationary(self) -> bool:
        """True, если фильтр считает платформу неподвижной (ZUPT активен)."""
        return self._stationary

    def predict(self, gx: float, gy: float, gz: float, dt: float) -> None:
        """Шаг предсказания по гироскопу (рад/с) и шагу времени ``dt`` (с).

        Args:
            gx: Угловая скорость вокруг X (рад/с).
            gy: Угловая скорость вокруг Y (рад/с).
            gz: Угловая скорость вокруг Z (рад/с).
            dt: Интервал интегрирования; при ``dt <= 0`` метод ничего не делает.
        """
        if dt <= 0:
            return

        wz = gz - self._gz_bias

        abs_wz = abs(wz)
        if self._stationary:
            if abs_wz > self._zupt_exit:
                self._stationary = False
                self._zupt_count = 0
        else:
            if abs_wz < self._zupt_enter:
                self._zupt_count += 1
                if self._zupt_count >= self._zupt_confirm:
                    self._stationary = True
            else:
                self._zupt_count = 0

        if self._stationary:
            S = self._P_bias + self._r_zupt
            if S > 1e-12:
                K_yaw = self._P_cross / S
                K_bias = self._P_bias / S

                self._yaw += K_yaw * wz
                self._gz_bias += K_bias * wz

                p11 = self._P_yaw
                p12 = self._P_cross
                p22 = self._P_bias
                self._P_yaw = p11 - K_yaw * p12
                self._P_cross = p12 - K_yaw * p22
                self._P_bias = p22 - K_bias * p22

                if self._P_yaw < 1e-10:
                    self._P_yaw = 1e-10
                if self._P_bias < 1e-10:
                    self._P_bias = 1e-10

            self._P_yaw += self.q_angle * dt * 0.01
            self._P_bias += self.q_bias * dt

            self._yaw = (self._yaw + pi) % (2 * pi) - pi
            return

        self._yaw += wz * dt

        self._yaw = (self._yaw + pi) % (2 * pi) - pi

        p11 = self._P_yaw
        p12 = self._P_cross
        p22 = self._P_bias

        self._P_yaw = p11 + (-dt) * p12 + (-dt) * (p12 + (-dt) * p22) + self.q_angle * dt
        self._P_cross = p12 + (-dt) * p22
        self._P_bias = p22 + self.q_bias * dt

    def update(self, ax: float, ay: float, az: float) -> None:
        """Обновить roll/pitch по акселерометру и логику выхода из ZUPT по ‖a‖.

        При сильном отклонении нормы ускорения от ``g`` измерение игнорируется.
        """
        a_mag = sqrt(ax * ax + ay * ay + az * az)
        a_dev = abs(a_mag - self.g)

        if self._stationary and a_dev > self._zupt_accel_exit:
            self._stationary = False
            self._zupt_count = 0
        elif not self._stationary and a_dev < self._zupt_accel_enter:
            self._zupt_count = min(self._zupt_count + 1, self._zupt_confirm)

        if a_dev > self.accel_gate * self.g:
            return

        raw_roll = atan2(ay, az)
        raw_pitch = atan2(-ax, sqrt(ay * ay + az * az))

        self._roll = raw_roll - self._home_roll
        self._pitch = raw_pitch - self._home_pitch

        self._roll = (self._roll + pi) % (2 * pi) - pi
        self._pitch = (self._pitch + pi) % (2 * pi) - pi

    @property
    def roll(self) -> float:
        """Оценка roll относительно калибровочного home (рад)."""
        return self._roll

    @property
    def pitch(self) -> float:
        """Оценка pitch относительно калибровочного home (рад)."""
        return self._pitch

    @property
    def yaw(self) -> float:
        """Оценка yaw (рад), непрерывная в пределах (-π, π]."""
        return self._yaw

    @property
    def gyro_bias(self) -> tuple[float, float, float]:
        """Оценённый bias гироскопа; X/Y всегда 0, Z — bias по ω_z (рад/с)."""
        return 0.0, 0.0, self._gz_bias

    def get_euler_deg(self) -> tuple[float, float, float]:
        """Roll, pitch, yaw в градусах (для отладки и логов)."""
        return degrees(self._roll), degrees(self._pitch), degrees(self._yaw)
