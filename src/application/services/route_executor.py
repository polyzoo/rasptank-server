from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, final

from src.application.models.event import (
    EventType,
    Position,
    RouteEndedEvent,
    RouteEndReason,
)
from src.application.models.route import (
    BackwardSegment,
    ForwardSegment,
    RouteSegment,
    TurnLeftSegment,
    TurnRightSegment,
)
from src.application.protocols import (
    MotorControllerProtocol,
    UltrasonicSensorProtocol,
)

logger: logging.Logger = logging.getLogger(__name__)


@final
@dataclass(frozen=True)
class RouteExecutorConfig:
    """Конфигурация исполнителя маршрута.

    Attributes:
        min_obstacle_distance_cm: Дистанция остановки перед препятствием (см).
        deceleration_distance_cm: Зона плавного торможения (см).
        base_speed_percent: Базовая скорость (%).
        turn_speed_percent: Скорость поворотов (%).
        max_speed_cm_per_sec: Линейная скорость при 100% для оценки пути.
        update_interval_sec: Интервал опроса датчика (с).
        obstacle_cannot_bypass_timeout_sec: Ожидание перед препятствием до уведомления (с).
        turn_duration_90_deg_sec: Длительность поворота на 90° (с).
    """

    min_obstacle_distance_cm: float
    deceleration_distance_cm: float
    base_speed_percent: int
    turn_speed_percent: int
    max_speed_cm_per_sec: float
    update_interval_sec: float
    obstacle_cannot_bypass_timeout_sec: float
    turn_duration_90_deg_sec: float


@final
class RouteExecutor:
    """Выполнение сегментов маршрута с учётом препятствий и трекингом позиции."""

    # Допустимый диапазон процентов скорости
    SPEED_PERCENT_MIN: int = 0
    SPEED_PERCENT_MAX: int = 100

    # Диапазон коэффициента замедления по цели/препятствию
    SPEED_FACTOR_MIN: float = 0.1
    SPEED_FACTOR_MAX: float = 1.0

    # Диапазон коэффициента замедления только по препятствию
    OBSTACLE_SPEED_FACTOR_MIN: float = 0.0
    OBSTACLE_SPEED_FACTOR_MAX: float = 1.0

    # Делитель для перевода процентов скорости в долю от 'max_speed_cm_per_sec'
    ESTIMATE_SPEED_DIVISOR: float = 100.0

    # Минимальный интервал проверки препятствий при повороте (с)
    TURN_CHECK_INTERVAL_MIN_SEC: float = 0.1

    # Делитель для вычисления интервала проверки от длительности поворота
    TURN_CHECK_INTERVAL_DIVISOR: float = 10.0

    # Базовый угол поворота для расчёта 'deg_per_sec' (90°)
    TURN_ANGLE_90_DEG: float = 90.0

    # Запасной 'deg_per_sec', если 'turn_duration_90_deg_sec' не задан
    TURN_DEG_PER_SEC_FALLBACK: float = 180.0

    # Порог оставшейся дистанции, ниже которого сегмент считается завершённым
    REMAINING_DISTANCE_THRESHOLD_CM: float = 0.0

    # Порог скорости, ниже которого считаем, что нужно остановиться
    SPEED_STOP_THRESHOLD: float = 0.0

    # Начальные координаты позиции (X, Y) и ориентации
    INITIAL_POSITION_CM: float = 0.0

    # Начальное значение пройденного пути в сегменте
    INITIAL_TRAVELED_CM: float = 0.0

    # Начальное значение прошедшего времени в сегменте (с)
    INITIAL_ELAPSED_SEC: float = 0.0

    # Начальное значение накопленного ожидания препятствия (с)
    INITIAL_WAITED_SEC: float = 0.0

    def __init__(
        self,
        motor_controller: MotorControllerProtocol,
        ultrasonic_sensor: UltrasonicSensorProtocol,
        config: RouteExecutorConfig,
        on_obstacle_cannot_bypass: Callable[[], None] | None = None,
        on_trajectory_event: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        """Инициализация исполнителя маршрута.

        Args:
            motor_controller: Низкоуровневое управление моторами.
            ultrasonic_sensor: Датчик расстояния до препятствий.
            config: Параметры движения и торможения.
            on_obstacle_cannot_bypass: Callback при невозможности объезда.
            on_trajectory_event: Callback при событии траектории.
        """
        self._motor: MotorControllerProtocol = motor_controller
        self._sensor: UltrasonicSensorProtocol = ultrasonic_sensor
        self._config: RouteExecutorConfig = config
        self._on_obstacle_cannot_bypass: Callable[[], None] | None = on_obstacle_cannot_bypass
        self._on_trajectory_event: Callable[[dict[str, Any]], None] | None = on_trajectory_event

        self._pos_x_cm: float = self.INITIAL_POSITION_CM
        self._pos_y_cm: float = self.INITIAL_POSITION_CM
        self._pos_heading_deg: float = self.INITIAL_POSITION_CM

    def execute(
        self,
        segments: list[RouteSegment],
        should_continue: Callable[[], bool],
    ) -> None:
        """Выполнить сегменты маршрута.

        Блокирует до завершения, остановки по should_continue или препятствию.
        Отправляет route_started, position, route_ended при наличии callback.

        Args:
            segments: Сегменты маршрута в порядке выполнения.
            should_continue: Callable, возвращающий False для досрочной остановки.
        """
        self._reset_position()
        self._emit(
            event=Position(
                x_cm=self._pos_x_cm,
                y_cm=self._pos_y_cm,
                heading_deg=self._pos_heading_deg,
            ).to_event_dict(EventType.ROUTE_STARTED),
        )

        for segment in segments:
            if not should_continue():
                self._emit_route_ended(RouteEndReason.STOP)
                break

            if isinstance(segment, ForwardSegment):
                if not self._run_forward_segment(
                    distance_cm=segment.distance_cm,
                    speed_percent=self._config.base_speed_percent,
                    should_continue=should_continue,
                ):
                    self._emit_route_ended(RouteEndReason.OBSTACLE)
                    break

            elif isinstance(segment, BackwardSegment):
                if not self._run_backward_segment(
                    distance_cm=segment.distance_cm,
                    speed_percent=self._config.base_speed_percent,
                    should_continue=should_continue,
                ):
                    self._emit_route_ended(RouteEndReason.STOP)
                    break

            elif isinstance(segment, TurnLeftSegment):
                self._run_turn_segment(True, segment.duration_sec, should_continue)

            elif isinstance(segment, TurnRightSegment):
                self._run_turn_segment(False, segment.duration_sec, should_continue)

        else:
            self._emit_route_ended(RouteEndReason.COMPLETE)

    def run_forward_segment(
        self,
        distance_cm: float,
        speed_percent: int,
        should_continue: Callable[[], bool],
    ) -> bool:
        """Выполнить один сегмент «вперёд».

        Для калибровки и тестов. Отправляет 'route_started', 'position', 'route_ended'
        при наличии callback.

        Args:
            distance_cm: Целевое расстояние (см).
            speed_percent: Скорость (%).
            should_continue: Callable для досрочной остановки.

        Returns:
            True — сегмент пройден полностью; False — остановка по препятствию или should_continue.
        """
        self._reset_position()
        self._emit(
            event=Position(
                x_cm=self._pos_x_cm,
                y_cm=self._pos_y_cm,
                heading_deg=self._pos_heading_deg,
            ).to_event_dict(EventType.ROUTE_STARTED),
        )

        completed: bool = self._run_forward_segment(distance_cm, speed_percent, should_continue)
        reason: RouteEndReason = RouteEndReason.COMPLETE if completed else RouteEndReason.STOP
        self._emit_route_ended(reason)

        return completed

    def _reset_position(self) -> None:
        """Сбросить позицию в начальную."""
        self._pos_x_cm: float = self.INITIAL_POSITION_CM
        self._pos_y_cm: float = self.INITIAL_POSITION_CM
        self._pos_heading_deg: float = self.INITIAL_POSITION_CM

    def _emit(self, event: dict[str, Any]) -> None:
        """Отправить событие о траектории в callback."""
        if self._on_trajectory_event:
            try:
                self._on_trajectory_event(event)
            except Exception as exc:
                logger.exception("Ошибка при отправке события траектории: %s", exc)

    def _emit_position(self) -> None:
        """Отправить событие о текущей позицией."""
        self._emit(
            event=Position(
                x_cm=self._pos_x_cm,
                y_cm=self._pos_y_cm,
                heading_deg=self._pos_heading_deg,
            ).to_event_dict(EventType.POSITION),
        )

    def _emit_route_ended(self, reason: str) -> None:
        """Отправить событие об окончании маршрута."""
        self._emit(
            event=RouteEndedEvent(
                reason=reason,
                x_cm=self._pos_x_cm,
                y_cm=self._pos_y_cm,
                heading_deg=self._pos_heading_deg,
            ).to_dict(),
        )

    def _run_forward_segment(
        self,
        distance_cm: float,
        speed_percent: int,
        should_continue: Callable[[], bool],
    ) -> bool:
        """Сегмент «вперёд» с учётом препятствий и целевой дистанции.

        Returns:
            True — сегмент пройден; False — остановка по препятствию или 'should_continue'.
        """
        traveled_cm: float = self.INITIAL_TRAVELED_CM
        clamped_speed: int = max(self.SPEED_PERCENT_MIN, min(self.SPEED_PERCENT_MAX, speed_percent))

        try:
            while should_continue():
                obstacle_cm: float = self._sensor.measure_distance_cm()

                remaining_distance: float = distance_cm - traveled_cm
                if remaining_distance <= self.REMAINING_DISTANCE_THRESHOLD_CM:
                    return True

                current_speed: float = self._calculate_speed(
                    obstacle_distance_cm=obstacle_cm,
                    remaining_distance_cm=remaining_distance,
                    max_speed_percent=clamped_speed,
                )
                if current_speed <= self.SPEED_STOP_THRESHOLD:
                    self._motor.stop()
                    if not self._wait_for_obstacle_clear_or_timeout(should_continue):
                        return False
                    continue

                self._motor.move_forward(speed_percent=int(current_speed))
                step_cm: float = self._estimate_traveled_distance(
                    speed_percent=current_speed,
                    time_interval=self._config.update_interval_sec,
                )

                traveled_cm += step_cm
                self._update_position_forward(step_cm)
                self._emit_position()
                time.sleep(self._config.update_interval_sec)

            return False

        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.exception("Ошибка в сегменте движения вперёд: %s", exc)
            return False

    def _run_backward_segment(
        self,
        distance_cm: float,
        speed_percent: int,
        should_continue: Callable[[], bool],
    ) -> bool:
        """Сегмент «назад» на заданную дистанцию. Препятствия не проверяются.

        Returns:
            True — сегмент пройден; False — остановка по should_continue.
        """
        traveled_cm: float = self.INITIAL_TRAVELED_CM
        clamped_speed: int = max(self.SPEED_PERCENT_MIN, min(self.SPEED_PERCENT_MAX, speed_percent))

        try:
            while should_continue():
                remaining_distance: float = distance_cm - traveled_cm
                if remaining_distance <= self.REMAINING_DISTANCE_THRESHOLD_CM:
                    return True

                speed_factor: float = (
                    self.SPEED_FACTOR_MAX
                    if remaining_distance >= self._config.deceleration_distance_cm
                    else max(
                        self.SPEED_FACTOR_MIN,
                        min(
                            self.SPEED_FACTOR_MAX,
                            remaining_distance / self._config.deceleration_distance_cm,
                        ),
                    )
                )

                current_speed: float = clamped_speed * speed_factor
                self._motor.move_backward(speed_percent=int(current_speed))
                step_cm: float = self._estimate_traveled_distance(
                    speed_percent=current_speed,
                    time_interval=self._config.update_interval_sec,
                )

                traveled_cm += step_cm
                self._update_position_backward(step_cm)
                self._emit_position()
                time.sleep(self._config.update_interval_sec)

            return False

        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.exception("Ошибка в сегменте движения назад: %s", exc)
            return False

    def _run_turn_segment(
        self,
        turn_left: bool,
        duration_sec: float,
        should_continue: Callable[[], bool],
    ) -> None:
        """Сегмент поворота на месте с проверкой препятствий."""
        elapsed: float = self.INITIAL_ELAPSED_SEC

        check_interval: float = min(
            self.TURN_CHECK_INTERVAL_MIN_SEC,
            duration_sec / self.TURN_CHECK_INTERVAL_DIVISOR,
        )

        deg_per_sec: float = (
            self.TURN_ANGLE_90_DEG / self._config.turn_duration_90_deg_sec
            if self._config.turn_duration_90_deg_sec
            else self.TURN_DEG_PER_SEC_FALLBACK
        )

        while elapsed < duration_sec and should_continue():
            obstacle_cm: float = self._sensor.measure_distance_cm()
            if obstacle_cm <= self._config.min_obstacle_distance_cm:
                self._motor.stop()
                return

            if turn_left:
                self._motor.turn_left(speed_percent=self._config.turn_speed_percent)
                self._pos_heading_deg += deg_per_sec * check_interval
            else:
                self._motor.turn_right(speed_percent=self._config.turn_speed_percent)
                self._pos_heading_deg -= deg_per_sec * check_interval

            self._emit_position()
            time.sleep(check_interval)
            elapsed += check_interval

        self._motor.stop()

    def _calculate_speed(
        self,
        obstacle_distance_cm: float,
        remaining_distance_cm: float,
        max_speed_percent: int,
    ) -> float:
        """Скорость с учётом препятствий и оставшейся дистанции.

        Returns:
            Скорость в процентах (0–100). 0 при препятствии в зоне остановки.
        """
        if obstacle_distance_cm <= self._config.min_obstacle_distance_cm:
            return self.SPEED_STOP_THRESHOLD

        speed: float = float(max_speed_percent)

        if obstacle_distance_cm <= self._config.deceleration_distance_cm:
            decel_range: float = (
                self._config.deceleration_distance_cm
                - self._config.min_obstacle_distance_cm
            )

            distance_above_min: float = (
                obstacle_distance_cm - self._config.min_obstacle_distance_cm
            )

            speed *= max(
                self.OBSTACLE_SPEED_FACTOR_MIN,
                min(
                    self.OBSTACLE_SPEED_FACTOR_MAX,
                    distance_above_min / decel_range,
                ),
            )

        if (
            remaining_distance_cm
            and remaining_distance_cm < self._config.deceleration_distance_cm
        ):
            speed *= max(
                self.SPEED_FACTOR_MIN,
                min(
                    self.SPEED_FACTOR_MAX,
                    remaining_distance_cm / self._config.deceleration_distance_cm,
                ),
            )

        return max(self.OBSTACLE_SPEED_FACTOR_MIN, speed)

    def _wait_for_obstacle_clear_or_timeout(self, should_continue: Callable[[], bool]) -> bool:
        """Ожидание освобождения пути или таймаута.

        При таймауте вызывает on_obstacle_cannot_bypass.

        Returns:
            True — путь свободен; False — таймаут или should_continue вернул False.
        """
        waited_sec: float = self.INITIAL_WAITED_SEC
        while waited_sec < self._config.obstacle_cannot_bypass_timeout_sec:
            if not should_continue():
                return False

            obstacle_cm: float = self._sensor.measure_distance_cm()
            if obstacle_cm > self._config.min_obstacle_distance_cm:
                return True

            time.sleep(self._config.update_interval_sec)
            waited_sec += self._config.update_interval_sec

        if self._on_obstacle_cannot_bypass:
            try:
                self._on_obstacle_cannot_bypass()
            except Exception as exc:
                logger.exception("Ошибка при ожидании освобождения пути: %s", exc)

        return False

    def _update_position_forward(self, distance_cm: float) -> None:
        """Обновить позицию после движения вперёд."""
        rad: float = math.radians(self._pos_heading_deg)
        self._pos_x_cm -= distance_cm * math.sin(rad)
        self._pos_y_cm += distance_cm * math.cos(rad)

    def _update_position_backward(self, distance_cm: float) -> None:
        """Обновить позицию после движения назад."""
        rad: float = math.radians(self._pos_heading_deg)
        self._pos_x_cm += distance_cm * math.sin(rad)
        self._pos_y_cm -= distance_cm * math.cos(rad)

    def _estimate_traveled_distance(self, speed_percent: float, time_interval: float) -> float:
        """Оценка пройденного расстояния за интервал (см)."""
        speed_ratio: float = speed_percent / self.ESTIMATE_SPEED_DIVISOR
        return self._config.max_speed_cm_per_sec * speed_ratio * time_interval
