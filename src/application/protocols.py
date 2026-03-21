from __future__ import annotations

from abc import abstractmethod
from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from src.application.models.route import Route


class MotorControllerProtocol(Protocol):
    """Протокол управления двигателями."""

    @abstractmethod
    def move_forward(self, speed_percent: int, steer_percent: int = 0) -> None:
        """Движение вперед. steer_percent: дифференциал для удержания курса (−100…100)."""

    @abstractmethod
    def move_backward(self, speed_percent: int, steer_percent: int = 0) -> None:
        """Движение назад. steer_percent — как у move_forward."""

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
    """Протокол ультразвукового датчика расстояния."""

    @abstractmethod
    def measure_distance_cm(self) -> float:
        """Измерение расстояния до препятствия."""

    @abstractmethod
    def destroy(self) -> None:
        """Освобождение ресурсов."""


class GyroscopeProtocol(Protocol):
    """Протокол гироскопа."""

    @abstractmethod
    def start(self, calibrate: bool = True) -> None:
        """Запуск отслеживания угла."""

    @abstractmethod
    def get_yaw(self) -> float:
        """Возврат текущего угла."""

    @abstractmethod
    def reset_yaw(self) -> None:
        """Сброс угла."""

    @abstractmethod
    def destroy(self) -> None:
        """Освобождение ресурсов."""


class DriveControllerProtocol(Protocol):
    """Протокол высокоуровневого управления движением."""

    @abstractmethod
    def forward_cm_sync(self, distance_cm: float, max_speed_percent: int | None = None) -> None:
        """Движение вперёд на заданное расстояние в текущем потоке."""

    @abstractmethod
    def execute_route(self, route: Route, *, start_segment_index: int = 0) -> None:
        """Выполнение маршрута в фоновом потоке."""

    @abstractmethod
    def execute_route_sync(self, route: Route, *, start_segment_index: int = 0) -> None:
        """Выполнение маршрута в текущем потоке."""

    @abstractmethod
    def stop(self) -> None:
        """Немедленная остановка движения."""

    @abstractmethod
    def destroy(self) -> None:
        """Освобождение ресурсов."""
