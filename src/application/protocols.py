from abc import abstractmethod
from typing import Protocol


class HealthCheckProtocol(Protocol):
    """Протокол для проверки готовности сервера к работе."""

    @abstractmethod
    def check(self) -> None:
        """Проверка готовности сервера с машинкой к работе."""


class DriveControllerProtocol(Protocol):
    """Протокол для управления движением машинки."""

    @abstractmethod
    def forward_cm(self, distance_cm: float, max_speed_percent: float | None = None) -> None:
        """Движение машинки вперёд на заданное расстояние с ограничением скорости."""

    @abstractmethod
    def stop(self) -> None:
        """Немедленная остановка машинки."""
