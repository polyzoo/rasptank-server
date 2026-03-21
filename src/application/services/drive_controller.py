from __future__ import annotations

import logging
import os
import time
from threading import Event, Thread
from typing import final

from src.application.models.route import (
    BackwardSegment,
    ForwardSegment,
    Route,
    RouteSegment,
    TurnLeftSegment,
    TurnRightSegment,
)
from src.application.protocols import (
    DriveControllerProtocol,
    GyroscopeProtocol,
    MotorControllerProtocol,
    UltrasonicSensorProtocol,
)

logger: logging.Logger = logging.getLogger(__name__)


def _route_diag_enabled() -> bool:
    """Подробные логи маршрута (сегменты, yaw, причина остановки поворота)."""
    return os.environ.get("RASPTANK_DIAG", "").strip().lower() in ("1", "true", "yes", "on")


@final
class DriveController(DriveControllerProtocol):
    """Контроллер движения с учетом препятствий и целевой дистанции."""

    # Ограничения и коэффициенты для пересчёта скорости
    SPEED_PERCENT_MIN: int = 0
    SPEED_PERCENT_MAX: int = 100
    SPEED_FACTOR_MIN: float = 0.1
    SPEED_FACTOR_MAX: float = 1.0

    # Ограничения для расчёта скорости по препятствию
    OBSTACLE_SPEED_FACTOR_MIN: float = 0.0
    OBSTACLE_SPEED_FACTOR_MAX: float = 1.0

    # Ограничения для навигации
    ESTIMATE_SPEED_DIVISOR: float = 100.0
    TURN_CHECK_INTERVAL_SEC: float = 0.05
    TURN_TIMEOUT_PER_DEG: float = 0.05
    TURN_TIMEOUT_MIN: float = 1.0

    # Ожидание завершения потока
    STOP_JOIN_TIMEOUT_SEC: float = 1.0

    # Краткий выбег без дифференциала перед стопом (снимает рывок «поворота» в конце прямого)
    FORWARD_COAST_BEFORE_STOP_SEC: float = 0.08

    @staticmethod
    def _angle_error_deg(setpoint: float, current: float) -> float:
        """Ошибка курса (setpoint − current) в градусах, нормализованная в [−180, 180]."""
        diff: float = setpoint - current
        while diff > 180.0:
            diff -= 360.0
        while diff < -180.0:
            diff += 360.0
        return diff

    def __init__(
        self,
        motor_controller: MotorControllerProtocol,
        ultrasonic_sensor: UltrasonicSensorProtocol,
        gyroscope: GyroscopeProtocol,
        min_obstacle_distance_cm: float = 20.0,
        deceleration_distance_cm: float = 50.0,
        base_speed_percent: int = 60,
        turn_speed_percent: int = 50,
        turn_slowdown_remaining_deg: float = 18.0,
        turn_creep_speed_percent: int = 26,
        turn_angle_trim_deg: float = 0.0,
        last_turn_angle_trim_deg: float = 0.0,
        max_speed_cm_per_sec: float = 30.0,
        update_interval_sec: float = 0.1,
        heading_hold_enabled: bool = True,
        heading_hold_kp: float = 2.8,
        heading_hold_steer_max: int = 45,
        heading_hold_deadband_deg: float = 0.4,
        heading_hold_steer_speed_ratio: float = 0.55,
        heading_hold_min_speed_percent: float = 0.0,
        heading_hold_steer_trim: int = 0,
        heading_hold_invert_steer: bool = False,
        forward_soft_start_sec: float = 0.35,
    ) -> None:
        """Инициализация контроллера движения."""
        self.motor_controller: MotorControllerProtocol = motor_controller
        self.ultrasonic_sensor: UltrasonicSensorProtocol = ultrasonic_sensor
        self.gyroscope: GyroscopeProtocol = gyroscope
        
        self.min_obstacle_distance_cm: float = min_obstacle_distance_cm
        self.deceleration_distance_cm: float = deceleration_distance_cm
        self.base_speed_percent: int = base_speed_percent
        self.turn_speed_percent: int = turn_speed_percent
        self.turn_slowdown_remaining_deg: float = turn_slowdown_remaining_deg
        self.turn_creep_speed_percent: int = turn_creep_speed_percent
        self.turn_angle_trim_deg: float = turn_angle_trim_deg
        self.last_turn_angle_trim_deg: float = last_turn_angle_trim_deg
        self.max_speed_cm_per_sec: float = max_speed_cm_per_sec
        self.update_interval_sec: float = update_interval_sec
        self.heading_hold_enabled: bool = heading_hold_enabled
        self.heading_hold_kp: float = heading_hold_kp
        self.heading_hold_steer_max: int = heading_hold_steer_max
        self.heading_hold_deadband_deg: float = heading_hold_deadband_deg
        self.heading_hold_steer_speed_ratio: float = heading_hold_steer_speed_ratio
        self.heading_hold_min_speed_percent: float = heading_hold_min_speed_percent
        self.heading_hold_steer_trim: int = heading_hold_steer_trim
        self.heading_hold_invert_steer: bool = heading_hold_invert_steer
        self.forward_soft_start_sec: float = forward_soft_start_sec

        self._is_moving: bool = False
        self._is_route_running: bool = False
        self._movement_thread: Thread | None = None
        self._stop_event: Event = Event()

    def forward_cm_sync(self, distance_cm: float, max_speed_percent: int | None = None) -> None:
        """Запуск движения вперед на заданное расстояние в текущем потоке."""
        if self._is_moving:
            self.stop()

        self._is_moving: bool = True
        self._stop_event.clear()

        try:
            self._run_forward_segment(distance_cm, max_speed_percent or self.base_speed_percent)
        finally:
            self.motor_controller.stop()
            self._is_moving: bool = False

    def execute_route(self, route: Route) -> None:
        """Запуск выполнения маршрута в фоновом потоке."""
        if self._is_moving or self._is_route_running:
            self.stop()

        self._is_moving: bool = True
        self._is_route_running: bool = True
        self._stop_event.clear()

        self._movement_thread: Thread = Thread(
            target=self._execute_route_loop,
            args=(route.segments,),
            daemon=True,
        )
        self._movement_thread.start()

    def execute_route_sync(self, route: Route) -> None:
        """Запуск выполнения маршрута в текущем потоке."""
        if self._is_moving or self._is_route_running:
            self.stop()

        self._is_moving: bool = True
        self._is_route_running: bool = True
        self._stop_event.clear()

        try:
            self._execute_route_segments(route.segments)
        finally:
            self.motor_controller.stop()
            self._is_moving: bool = False
            self._is_route_running: bool = False

    def stop(self) -> None:
        """Немедленная остановка движения."""
        self._is_moving: bool = False
        self._is_route_running: bool = False
        self._stop_event.set()
        self.motor_controller.stop()

        if self._movement_thread and self._movement_thread.is_alive():
            self._movement_thread.join(timeout=self.STOP_JOIN_TIMEOUT_SEC)

        self._movement_thread: Thread | None = None

    def destroy(self) -> None:
        """Освобождение ресурсов."""
        self.stop()
        self.motor_controller.destroy()
        self.ultrasonic_sensor.destroy()
        self.gyroscope.destroy()

    def _execute_route_loop(self, segments: list[RouteSegment]) -> None:
        """Цикл выполнения маршрута в daemon-потоке."""
        try:
            self._execute_route_segments(segments)
        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.exception("Ошибка при выполнении маршрута: %s", exc)
        finally:
            self.motor_controller.stop()
            self._is_moving: bool = False
            self._is_route_running: bool = False

    def _execute_route_segments(self, segments: list[RouteSegment]) -> None:
        """Последовательное выполнение сегментов маршрута."""
        self.gyroscope.start(calibrate=True)
        n_seg: int = len(segments)

        for idx, segment in enumerate(segments):
            if not self._is_moving or self._stop_event.is_set():
                break

            if isinstance(segment, ForwardSegment):
                if _route_diag_enabled():
                    logger.info(
                        "route seg %d: forward distance_cm=%.1f",
                        idx,
                        segment.distance_cm,
                    )
                if not self._run_forward_segment(segment.distance_cm, self.base_speed_percent):
                    if _route_diag_enabled():
                        logger.info("route seg %d: forward прерван (препятствие/стоп)", idx)
                    break

            elif isinstance(segment, BackwardSegment):
                if _route_diag_enabled():
                    logger.info(
                        "route seg %d: backward distance_cm=%.1f",
                        idx,
                        segment.distance_cm,
                    )
                if not self._run_backward_segment(segment.distance_cm, self.base_speed_percent):
                    if _route_diag_enabled():
                        logger.info("route seg %d: backward прерван", idx)
                    break

            elif isinstance(segment, TurnLeftSegment):
                last_extra: float = (
                    self.last_turn_angle_trim_deg if idx == n_seg - 1 else 0.0
                )
                eff_angle = max(
                    1.0,
                    min(179.0, segment.angle_deg + self.turn_angle_trim_deg + last_extra),
                )
                t_out: float = max(
                    self.TURN_TIMEOUT_MIN,
                    eff_angle * self.TURN_TIMEOUT_PER_DEG,
                )
                if _route_diag_enabled():
                    logger.info(
                        "route seg %d: turn_left angle_deg=%.1f effective=%.1f "
                        "(trim %+.1f last %+.1f) timeout_sec=%.2f",
                        idx,
                        segment.angle_deg,
                        eff_angle,
                        self.turn_angle_trim_deg,
                        last_extra,
                        t_out,
                    )
                self._run_turn_segment(
                    turn_left=True,
                    target_angle=eff_angle,
                    timeout_sec=t_out,
                    segment_index=idx,
                )

            elif isinstance(segment, TurnRightSegment):
                last_extra = (
                    self.last_turn_angle_trim_deg if idx == n_seg - 1 else 0.0
                )
                eff_angle = max(
                    1.0,
                    min(179.0, segment.angle_deg + self.turn_angle_trim_deg + last_extra),
                )
                t_out = max(
                    self.TURN_TIMEOUT_MIN,
                    eff_angle * self.TURN_TIMEOUT_PER_DEG,
                )
                if _route_diag_enabled():
                    logger.info(
                        "route seg %d: turn_right angle_deg=%.1f effective=%.1f "
                        "(trim %+.1f last %+.1f) timeout_sec=%.2f",
                        idx,
                        segment.angle_deg,
                        eff_angle,
                        self.turn_angle_trim_deg,
                        last_extra,
                        t_out,
                    )
                self._run_turn_segment(
                    turn_left=False,
                    target_angle=eff_angle,
                    timeout_sec=t_out,
                    segment_index=idx,
                )

    def _heading_steer_percent(self, heading_setpoint_deg: float, base_speed_percent: float) -> int:
        """Дифференциал колёс для возврата курса к setpoint (положительный — доворот против часовой).

        При малой базовой скорости большой steer заставляет одно колесо уйти в 0%% ШИМ — робот крутится на месте.
        Поэтому |steer| ограничивается долей от текущей скорости.
        """
        if not self.heading_hold_enabled:
            return 0

        base: float = max(0.0, float(base_speed_percent))
        if base < self.heading_hold_min_speed_percent:
            return 0

        steer_cap: int = min(
            self.heading_hold_steer_max,
            int(base * self.heading_hold_steer_speed_ratio),
        )
        if steer_cap <= 0:
            return 0

        err: float = self._angle_error_deg(heading_setpoint_deg, self.gyroscope.get_yaw())
        steer_pid: int = 0
        if abs(err) >= self.heading_hold_deadband_deg:
            steer_pid = int(round(self.heading_hold_kp * err))

        steer_i: int = steer_pid + self.heading_hold_steer_trim
        steer_i = max(-steer_cap, min(steer_cap, steer_i))
        if self.heading_hold_invert_steer:
            steer_i = -steer_i
        return steer_i

    def _run_forward_segment(self, distance_cm: float, speed_percent: int) -> bool:
        """Запуск сегмента «вперед» с учетом препятствий."""
        traveled_cm: float = 0.0
        last_effective_speed: float = 0.0
        last_cmd_speed: float = 0.0
        last_time: float = time.monotonic()
        heading_setpoint_deg: float = self.gyroscope.get_yaw()
        segment_start_t: float = time.monotonic()
        if _route_diag_enabled():
            logger.info(
                "forward: start heading_setpoint_deg=%.2f distance_cm=%.1f",
                heading_setpoint_deg,
                distance_cm,
            )

        clamped_speed: int = max(self.SPEED_PERCENT_MIN, min(self.SPEED_PERCENT_MAX, speed_percent))
        try:
            while self._is_moving and not self._stop_event.is_set():
                obstacle_cm: float = self.ultrasonic_sensor.measure_distance_cm()

                now: float = time.monotonic()
                dt: float = now - last_time
                last_time: float = now

                traveled_cm += self._estimate_traveled_distance(last_effective_speed, dt)

                remaining_dist: float = distance_cm - traveled_cm
                if remaining_dist <= 0:
                    coast_sp: int = max(
                        self.SPEED_PERCENT_MIN,
                        min(self.SPEED_PERCENT_MAX, int(last_cmd_speed)),
                    )
                    if coast_sp > 0:
                        self.motor_controller.move_forward(
                            speed_percent=coast_sp,
                            steer_percent=0,
                        )
                        time.sleep(self.FORWARD_COAST_BEFORE_STOP_SEC)
                    self.motor_controller.stop()
                    if _route_diag_enabled():
                        logger.info(
                            "forward: done yaw_deg=%.2f traveled_cm~%.1f",
                            self.gyroscope.get_yaw(),
                            traveled_cm,
                        )
                    return True

                current_speed: float = self._calculate_speed(
                    obstacle_cm=obstacle_cm,
                    remaining_cm=remaining_dist,
                    max_speed=clamped_speed,
                )

                if current_speed <= 0.0:
                    self.motor_controller.stop()
                    time.sleep(self.update_interval_sec)
                    if _route_diag_enabled():
                        logger.info(
                            "forward: stopped (obstacle) yaw_deg=%.2f traveled_cm~%.1f",
                            self.gyroscope.get_yaw(),
                            traveled_cm,
                        )
                    return False

                elapsed_seg: float = time.monotonic() - segment_start_t
                if self.forward_soft_start_sec > 0.0:
                    ramp: float = min(
                        1.0,
                        elapsed_seg / self.forward_soft_start_sec,
                    )
                else:
                    ramp = 1.0
                effective_speed: float = current_speed * ramp

                steer: int = self._heading_steer_percent(heading_setpoint_deg, effective_speed)
                last_effective_speed = effective_speed
                last_cmd_speed = effective_speed
                self.motor_controller.move_forward(
                    speed_percent=int(effective_speed),
                    steer_percent=steer,
                )
                time.sleep(self.update_interval_sec)

            return False

        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.exception("Ошибка в сегменте движения вперед: %s", exc)
            return False

    def _run_backward_segment(self, distance_cm: float, speed_percent: int) -> bool:
        """Запуск сегмента «назад»."""
        traveled_cm: float = 0.0
        current_speed: float = 0.0
        last_cmd_speed: float = 0.0
        last_time: float = time.monotonic()
        heading_setpoint_deg: float = self.gyroscope.get_yaw()

        clamped_speed: int = max(self.SPEED_PERCENT_MIN, min(self.SPEED_PERCENT_MAX, speed_percent))
        try:
            while self._is_moving and not self._stop_event.is_set():
                now: float = time.monotonic()
                dt: float = now - last_time
                last_time: float = now

                traveled_cm += self._estimate_traveled_distance(current_speed, dt)

                remaining_dist: float = distance_cm - traveled_cm
                if remaining_dist <= 0:
                    coast_sp = max(
                        self.SPEED_PERCENT_MIN,
                        min(self.SPEED_PERCENT_MAX, int(last_cmd_speed)),
                    )
                    if coast_sp > 0:
                        self.motor_controller.move_backward(
                            speed_percent=coast_sp,
                            steer_percent=0,
                        )
                        time.sleep(self.FORWARD_COAST_BEFORE_STOP_SEC)
                    self.motor_controller.stop()
                    return True

                speed_factor: float = (
                    self.SPEED_FACTOR_MAX
                    if remaining_dist >= self.deceleration_distance_cm
                    else max(self.SPEED_FACTOR_MIN, remaining_dist / self.deceleration_distance_cm)
                )

                current_speed: float = clamped_speed * speed_factor
                steer: int = self._heading_steer_percent(heading_setpoint_deg, current_speed)
                last_cmd_speed = current_speed
                self.motor_controller.move_backward(
                    speed_percent=int(current_speed),
                    steer_percent=steer,
                )
                time.sleep(self.update_interval_sec)

            return False

        except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
            logger.exception("Ошибка в сегменте движения назад: %s", exc)
            return False

    def _turn_slowdown_zone_deg(self, target_angle: float) -> float:
        """Сколько последних градусов дуги идти на creep.

        Для коротких поворотов (30–40°) фиксированные 18° «съедали» почти всю дугу — почти весь
        поворот на низкой скорости, хуже попадание в угол. Ужимаем зону пропорционально цели.
        """
        if self.turn_slowdown_remaining_deg <= 0.0:
            return 0.0
        proportional: float = max(5.0, target_angle * 0.40)
        return min(self.turn_slowdown_remaining_deg, proportional)

    def _turn_motor_speed_percent(self, target_angle: float, current_yaw_abs: float) -> int:
        """Полная скорость до конца дуги, затем creep — меньше проскальзывание, ближе реальный угол."""
        zone: float = self._turn_slowdown_zone_deg(target_angle)
        if zone <= 0.0:
            return self.turn_speed_percent
        remaining: float = target_angle - current_yaw_abs
        if remaining > zone:
            return self.turn_speed_percent
        creep: int = max(
            15,
            min(self.turn_speed_percent, self.turn_creep_speed_percent),
        )
        return creep

    def _run_turn_segment(
        self,
        turn_left: bool,
        target_angle: float,
        timeout_sec: float,
        segment_index: int = -1,
    ) -> None:
        """Запуск сегмента поворота на месте с проверкой препятствий."""
        self.gyroscope.reset_yaw()
        start_time: float = time.monotonic()
        stop_reason: str = "stop_event"

        while self._is_moving and not self._stop_event.is_set():
            current_yaw: float = self.gyroscope.get_yaw()

            if abs(current_yaw) >= target_angle:
                stop_reason = "angle_reached"
                break

            if (time.monotonic() - start_time) >= timeout_sec:
                stop_reason = "timeout"
                break

            obstacle_cm: float = self.ultrasonic_sensor.measure_distance_cm()
            if obstacle_cm <= self.min_obstacle_distance_cm:
                stop_reason = "obstacle"
                break

            yaw_abs: float = abs(current_yaw)
            sp: int = self._turn_motor_speed_percent(target_angle, yaw_abs)

            if turn_left:
                self.motor_controller.turn_left(speed_percent=sp)
            else:
                self.motor_controller.turn_right(speed_percent=sp)

            time.sleep(self.TURN_CHECK_INTERVAL_SEC)

        self.motor_controller.stop()

        if _route_diag_enabled():
            elapsed: float = time.monotonic() - start_time
            final_yaw: float = self.gyroscope.get_yaw()
            direction: str = "left" if turn_left else "right"
            logger.info(
                "turn_%s seg=%d: target_deg=%.1f final_yaw_deg=%.2f elapsed=%.2fs reason=%s "
                "(timeout was %.2fs; если reason=timeout и |yaw|<<цели — проверьте IMU/I2C)",
                direction,
                segment_index,
                target_angle,
                final_yaw,
                elapsed,
                stop_reason,
                timeout_sec,
            )

    def _calculate_speed(self, obstacle_cm: float, remaining_cm: float, max_speed: int) -> float:
        """Расчет скорости с учетом препятствий и цели."""
        if obstacle_cm <= self.min_obstacle_distance_cm:
            return 0.0

        speed: float = float(max_speed)

        if obstacle_cm <= self.deceleration_distance_cm:
            decel_range: float = self.deceleration_distance_cm - self.min_obstacle_distance_cm
            dist_above_min: float = obstacle_cm - self.min_obstacle_distance_cm
            speed *= max(self.OBSTACLE_SPEED_FACTOR_MIN, dist_above_min / decel_range)

        if remaining_cm < self.deceleration_distance_cm:
            speed *= max(self.SPEED_FACTOR_MIN, remaining_cm / self.deceleration_distance_cm)

        return max(self.OBSTACLE_SPEED_FACTOR_MIN, speed)

    def _estimate_traveled_distance(self, speed_percent: float, time_interval: float) -> float:
        """Оценка пройденного пути."""
        speed_ratio: float = speed_percent / self.ESTIMATE_SPEED_DIVISOR
        return self.max_speed_cm_per_sec * speed_ratio * time_interval
