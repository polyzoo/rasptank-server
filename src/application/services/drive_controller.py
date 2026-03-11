import logging
import time
from threading import Event, Thread
from typing import Optional, final

from src.application.protocols import (
    DriveControllerProtocol,
    MotorControllerProtocol,
    UltrasonicSensorProtocol,
)

logger: logging.Logger = logging.getLogger(__name__)


@final
class DriveController(DriveControllerProtocol):
    """Контроллер движения с учетом препятствий и целевой дистанции."""

    # Линейная скорость при 100% мощности (см/с), для оценки пройденного пути
    MAX_SPEED_CM_PER_SEC: float = 30.0

    def __init__(
        self,
        motor_controller: MotorControllerProtocol,
        ultrasonic_sensor: UltrasonicSensorProtocol,
        min_obstacle_distance_cm: float = 20.0,
        deceleration_distance_cm: float = 50.0,
        base_speed_percent: int = 60,
        update_interval_sec: float = 0.1,
    ) -> None:
        """Инициализация контроллера движения.

        Args:
            motor_controller: Низкоуровневое управление моторами.
            ultrasonic_sensor: Датчик расстояния до препятствий.
            min_obstacle_distance_cm: Дистанция остановки (см). Ниже — скорость 0.
            deceleration_distance_cm: Зона торможения по препятствию (см).
            base_speed_percent: Базовая скорость (%), если 'max_speed' не задан.
            update_interval_sec: Интервал опроса датчика и пересчета скорости (с).
        """
        self.motor_controller: MotorControllerProtocol = motor_controller
        self.ultrasonic_sensor: UltrasonicSensorProtocol = ultrasonic_sensor
        self.min_obstacle_distance_cm: float = min_obstacle_distance_cm
        self.deceleration_distance_cm: float = deceleration_distance_cm
        self.base_speed_percent: int = base_speed_percent
        self.update_interval_sec: float = update_interval_sec

        self._is_moving: bool = False
        self._target_distance_cm: Optional[float] = None
        self._traveled_distance_cm: float = 0.0
        self._max_speed_percent: Optional[int] = None
        self._movement_thread: Optional[Thread] = None
        self._stop_event: Event = Event()

    def forward_cm(self, distance_cm: float, max_speed_percent: Optional[int] = None) -> None:
        """Запуск движения вперед на заданное расстояние.

        Не блокирует: движение выполняется в daemon-потоке. При уже идущем движении
        предыдущее останавливается и запускается новое.

        Args:
            distance_cm: Целевое расстояние в сантиметрах.
            max_speed_percent: Ограничение скорости (0 – 100).
        """
        if self._is_moving:
            self.stop()

        self._target_distance_cm: float = distance_cm
        self._traveled_distance_cm: float = 0.0
        self._max_speed_percent: int = max_speed_percent or self.base_speed_percent
        self._is_moving: bool = True
        self._stop_event.clear()

        logger.debug(
            "forward_cm: distance=%.1f см, max_speed=%d%%, запуск потока",
            distance_cm,
            self._max_speed_percent,
        )
        self._movement_thread: Thread = Thread(target=self._autonomous_movement, daemon=True)
        self._movement_thread.start()

    def forward_cm_sync(
        self,
        distance_cm: float,
        max_speed_percent: Optional[int] = None,
        max_duration_sec: Optional[float] = None,
    ) -> None:
        """Движение вперёд в текущем потоке (без отдельного потока).

        Для отладки: весь путь выполняется линейно, логи идут по порядку.

        Args:
            distance_cm: Целевое расстояние в сантиметрах.
            max_speed_percent: Ограничение скорости (0 – 100).
            max_duration_sec: Макс. время (с); по истечении — остановка (для теста stop).
        """
        if self._is_moving:
            self.stop()

        self._target_distance_cm = distance_cm
        self._traveled_distance_cm = 0.0
        self._max_speed_percent = max_speed_percent or self.base_speed_percent
        self._is_moving = True
        self._stop_event.clear()
        self._movement_thread = None

        logger.debug(
            "forward_cm_sync: distance=%.1f см, max_speed=%d%%, max_duration=%s с",
            distance_cm,
            self._max_speed_percent,
            max_duration_sec,
        )

        try:
            self._run_autonomous_movement(max_duration_sec=max_duration_sec)
        finally:
            self.motor_controller.stop()
            self._is_moving = False

    def stop(self) -> None:
        """Немедленная остановка движения.

        Останавливает моторы и ждет завершения потока автономного движения (до 1 с).
        """
        logger.debug("DriveController.stop: _is_moving=False, stop_event.set(), motor.stop()")
        self._is_moving: bool = False
        self._stop_event.set()
        self.motor_controller.stop()

        if self._movement_thread and self._movement_thread.is_alive():
            self._movement_thread.join(timeout=1.0)

        self._movement_thread: Optional[Thread] = None

    def destroy(self) -> None:
        """Освобождение ресурсов: остановка движения, моторов и датчика."""
        self.stop()
        self.motor_controller.destroy()
        self.ultrasonic_sensor.destroy()

    def _calculate_speed(self, obstacle_distance_cm: float, remaining_distance_cm: float) -> float:
        """Скорость с учетом препятствий и оставшейся дистанции.

        По препятствию: 0 при 'distance' <= 'min', линейный рост до 'max' в зоне торможения.
        По цели: дополнительное снижение в зоне торможения (не ниже 10%).

        Returns:
            Скорость в процентах (0–100).
        """
        if obstacle_distance_cm <= self.min_obstacle_distance_cm:
            return 0.0

        speed: float = self._max_speed_percent

        if obstacle_distance_cm <= self.deceleration_distance_cm:
            decel_range: float = self.deceleration_distance_cm - self.min_obstacle_distance_cm
            distance_above_min: float = obstacle_distance_cm - self.min_obstacle_distance_cm
            speed *= max(0.0, min(1.0, distance_above_min / decel_range))

        if remaining_distance_cm and remaining_distance_cm < self.deceleration_distance_cm:
            speed *= max(0.1, min(1.0, remaining_distance_cm / self.deceleration_distance_cm))

        return max(0.0, speed)

    def _estimate_traveled_distance(self, speed_percent: float, time_interval: float) -> float:
        """Оценка пройденного расстояния за интервал.

        Модель: при 100% скорость 'MAX_SPEED_CM_PER_SEC' см/с. Не учитывает
        проскальзывание и уклон.

        Returns:
            Оценочное расстояние в сантиметрах.
        """
        return self.MAX_SPEED_CM_PER_SEC * (speed_percent / 100.0) * time_interval

    def _run_autonomous_movement(
        self,
        max_duration_sec: Optional[float] = None,
    ) -> None:
        """Цикл движения. Вызывается из потока или синхронно."""
        start_time: float = time.monotonic()
        try:
            while self._is_moving and not self._stop_event.is_set():
                if max_duration_sec is not None:
                    elapsed: float = time.monotonic() - start_time
                    if elapsed >= max_duration_sec:
                        logger.debug("forward_cm_sync: достигнут max_duration=%.1f с", max_duration_sec)
                        break
                obstacle_distance_cm: float = self.ultrasonic_sensor.measure_distance_cm()

                remaining_distance: float = (
                    self._target_distance_cm - self._traveled_distance_cm
                )
                if remaining_distance <= 0:
                    logger.debug("Цель достигнута: remaining=%.1f", remaining_distance)
                    break

                current_speed: float = self._calculate_speed(
                    obstacle_distance_cm=obstacle_distance_cm,
                    remaining_distance_cm=remaining_distance,
                )

                logger.debug(
                    "obstacle=%.1f см, remaining=%.1f см, speed=%.1f%%",
                    obstacle_distance_cm,
                    remaining_distance,
                    current_speed,
                )

                if current_speed <= 0.0:
                    logger.debug("speed=0 (препятствие или цель), stop")
                    self.motor_controller.stop()
                    time.sleep(self.update_interval_sec)
                    continue

                self.motor_controller.move_forward(speed_percent=int(current_speed))
                self._traveled_distance_cm += self._estimate_traveled_distance(
                    speed_percent=current_speed,
                    time_interval=self.update_interval_sec,
                )

                time.sleep(self.update_interval_sec)

        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.exception("Ошибка в автономном движении: %s", exc)

        finally:
            logger.debug("_run_autonomous_movement: завершение, stop моторов")
            self.motor_controller.stop()
            self._is_moving = False

    def _autonomous_movement(self) -> None:
        """Цикл движения в daemon-потоке. Обёртка над _run_autonomous_movement."""
        try:
            self._run_autonomous_movement(max_duration_sec=None)
        finally:
            self.motor_controller.stop()
            self._is_moving = False
