from abc import abstractmethod
from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from src.application.models.route import Route


class MotorControllerProtocol(Protocol):
    """Протокол низкоуровневого управления двигателями."""

    @abstractmethod
    def move_forward(self, speed_percent: int) -> None:
        """Движение вперед с заданной скоростью."""

    @abstractmethod
    def move_backward(self, speed_percent: int) -> None:
        """Движение назад с заданной скоростью."""

    @abstractmethod
    def turn_left(self, speed_percent: int) -> None:
        """Поворот налево на месте."""

    @abstractmethod
    def turn_right(self, speed_percent: int) -> None:
        """Поворот направо на месте."""

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
    def forward_cm(self, distance_cm: float, max_speed_percent: int | None = None) -> None:
        """Движение вперед на заданное расстояние."""

    @abstractmethod
    def execute_route(self, route: Route) -> None:
        """Выполнение маршрута с учетом препятствий."""

    @abstractmethod
    def stop(self) -> None:
        """Немедленная остановка движения."""

    @abstractmethod
    def destroy(self) -> None:
        """Освобождение ресурсов."""
