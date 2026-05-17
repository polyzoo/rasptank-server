from __future__ import annotations

from math import atan2, degrees, pi, sqrt


class EkfImu:
    """Фильтр IMU для yaw, roll, pitch и ZUPT."""

    def __init__(
        self,
        q_angle: float = 0.001,
        q_bias: float = 0.0001,
        r_accel: float = 0.5,
        accel_gate: float = 0.3,
        g: float = 9.80665,
    ) -> None:
        """Подготовить ковариации и пороги ZUPT."""
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
        """Сбросить оценки фильтра без смены home."""
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
        """Задать home-углы после калибровки."""
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
        """Вернуть признак покоя по ZUPT."""
        return self._stationary

    def predict(self, gx: float, gy: float, gz: float, dt: float) -> None:
        """Выполнить шаг предсказания по гироскопу."""
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
        """Обновить roll, pitch и состояние ZUPT по акселерометру."""
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
        """Вернуть roll относительно home."""
        return self._roll

    @property
    def pitch(self) -> float:
        """Вернуть pitch относительно home."""
        return self._pitch

    @property
    def yaw(self) -> float:
        """Вернуть yaw фильтра."""
        return self._yaw

    @property
    def gyro_bias(self) -> tuple[float, float, float]:
        """Вернуть оценку смещения гироскопа."""
        return 0.0, 0.0, self._gz_bias

    def get_euler_deg(self) -> tuple[float, float, float]:
        """Вернуть углы Эйлера в градусах."""
        return degrees(self._roll), degrees(self._pitch), degrees(self._yaw)
