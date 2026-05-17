from __future__ import annotations

from abc import abstractmethod
from typing import Protocol


class MotorControllerProtocol(Protocol):
    """Контракт управления моторами."""

    @abstractmethod
    def set_tracks(self, left_speed_percent: int, right_speed_percent: int) -> None:
        """Задать скорости левого и правого борта в процентах."""

    @abstractmethod
    def stop(self) -> None:
        """Остановить оба мотора."""

    @abstractmethod
    def destroy(self) -> None:
        """Освободить ресурсы."""


class UltrasonicSensorProtocol(Protocol):
    """Контракт ультразвукового датчика расстояния."""

    @abstractmethod
    def measure_distance_cm(self) -> float:
        """Измерить расстояние до препятствия."""

    @abstractmethod
    def destroy(self) -> None:
        """Освободить ресурсы."""


class GyroscopeProtocol(Protocol):
    """Контракт гироскопа."""

    @abstractmethod
    def start(self, calibrate: bool = True) -> None:
        """Запустить отслеживание угла."""

    @abstractmethod
    def get_yaw(self) -> float:
        """Вернуть текущий угол."""

    @abstractmethod
    def get_angular_speed_z_deg_per_sec(self) -> float:
        """Вернуть угловую скорость вокруг вертикальной оси."""

    @abstractmethod
    def get_acceleration_xyz_m_s2(self) -> tuple[float, float, float]:
        """Вернуть ускорения по трём осям."""

    @abstractmethod
    def stop(self) -> None:
        """Остановить фоновое отслеживание."""

    @abstractmethod
    def reset_yaw(self) -> None:
        """Сбросить угол."""

    @abstractmethod
    def destroy(self) -> None:
        """Освободить ресурсы."""


class HeadServoProtocol(Protocol):
    """Контракт сервопривода головы."""

    @abstractmethod
    def set_angle(self, angle_deg: float) -> None:
        """Задать угол головы."""

    @abstractmethod
    def destroy(self) -> None:
        """Освободить ресурсы."""
