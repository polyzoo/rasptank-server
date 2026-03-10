from abc import abstractmethod
from typing import Optional, Protocol


class MotorControllerProtocol(Protocol):
    """Протокол низкоуровневого управления двигателями."""

    @abstractmethod
    def move_forward(self, speed_percent: int) -> None:
        """Движение вперед с заданной скоростью."""

    @abstractmethod
    def stop(self) -> None:
        """Немедленная остановка обоих двигателей."""

    @abstractmethod
    def destroy(self) -> None:
        """Освобождение ресурсов."""


class UltrasonicSensorProtocol(Protocol):
    """Протокол ультразвукового датчика расстояния HC-SR04."""

    @abstractmethod
    def measure_distance_cm(self) -> float:
        """Измерение расстояния до препятствия."""

    @abstractmethod
    def destroy(self) -> None:
        """Освобождение ресурсов."""


class DriveControllerProtocol(Protocol):
    """Протокол высокоуровневого управления движением."""

    @abstractmethod
    def forward_cm(self, distance_cm: float, max_speed_percent: Optional[int] = None) -> None:
        """Движение вперед на заданное расстояние."""

    @abstractmethod
    def stop(self) -> None:
        """Немедленная остановка движения."""

    @abstractmethod
    def destroy(self) -> None:
        """Освобождение ресурсов."""
