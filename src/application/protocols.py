from abc import abstractmethod
from typing import Protocol, Optional


class MotorControllerProtocol(Protocol):
    """Протокол для абстракции управления двигателями машинки."""

    @abstractmethod
    def move_forward(self, speed_percent: int) -> None:
        """Движение машинки вперед с заданной скоростью."""

    @abstractmethod
    def stop(self) -> None:
        """Немедленная остановка обоих двигателей."""


class UltrasonicSensorProtocol(Protocol):
    """Протокол для работы с ультразвуковым датчиком расстояния машинки."""

    @abstractmethod
    def measure_distance_cm(self) -> float:
        """Выполнение измерения и получение расстояния до препятствия в сантиметрах."""


class DriveControllerProtocol(Protocol):
    """Протокол для управления движением машинки на высоком уровне."""

    @abstractmethod
    def forward_cm(self, distance_cm: float, max_speed_percent: Optional[int] = None) -> None:
        """Движение машинки вперёд на заданное расстояние с ограничением скорости."""

    @abstractmethod
    def stop(self) -> None:
        """Немедленная остановка движения машинки."""
