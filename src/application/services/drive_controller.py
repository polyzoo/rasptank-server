from __future__ import annotations

from typing import final

from src.application.models.route import Route
from src.application.protocols import (
    DriveControllerProtocol,
    GyroscopeProtocol,
    MotorControllerProtocol,
    UltrasonicSensorProtocol,
)
from src.application.services.motion_config import MotionConfig
from src.application.services.linear_motion_executor import LinearMotionExecutor
from src.application.services.motion_lifecycle import MotionLifecycle
from src.application.services.route_executor import RouteExecutor
from src.application.services.route_runner import RouteRunner
from src.application.services.turn_executor import TurnExecutor


@final
class DriveController(DriveControllerProtocol):
    """Контроллер движения, который управляет потоком, состоянием и исполнителями."""

    def __init__(
        self,
        motor_controller: MotorControllerProtocol,
        ultrasonic_sensor: UltrasonicSensorProtocol,
        gyroscope: GyroscopeProtocol,
        config: MotionConfig,
    ) -> None:
        """Инициализация контроллера движения."""
        self.motor_controller: MotorControllerProtocol = motor_controller
        self.ultrasonic_sensor: UltrasonicSensorProtocol = ultrasonic_sensor
        self.gyroscope: GyroscopeProtocol = gyroscope
        self.config: MotionConfig = config
        self.lifecycle: MotionLifecycle = MotionLifecycle()

        self._linear_motion: LinearMotionExecutor = LinearMotionExecutor(
            motor_controller=motor_controller,
            ultrasonic_sensor=ultrasonic_sensor,
            gyroscope=gyroscope,
            min_obstacle_distance_cm=config.min_obstacle_distance_cm,
            deceleration_distance_cm=config.deceleration_distance_cm,
            max_speed_cm_per_sec=config.max_speed_cm_per_sec,
            update_interval_sec=config.update_interval_sec,
            forward_soft_start_sec=config.forward_soft_start_sec,
            heading_hold_enabled=config.heading_hold_enabled,
            heading_hold_kp=config.heading_hold_kp,
            heading_hold_steer_max=config.heading_hold_steer_max,
            heading_hold_deadband_deg=config.heading_hold_deadband_deg,
            heading_hold_steer_speed_ratio=config.heading_hold_steer_speed_ratio,
            heading_hold_min_speed_percent=config.heading_hold_min_speed_percent,
            heading_hold_steer_cap_min_spd_percent=config.heading_hold_steer_cap_min_spd_percent,
            heading_hold_steer_trim=config.heading_hold_steer_trim,
            heading_hold_invert_steer=config.heading_hold_invert_steer,
            lifecycle=self.lifecycle,
        )

        self._turn_executor: TurnExecutor = TurnExecutor(
            motor_controller=motor_controller,
            ultrasonic_sensor=ultrasonic_sensor,
            gyroscope=gyroscope,
            min_obstacle_distance_cm=config.min_obstacle_distance_cm,
            turn_speed_percent=config.turn_speed_percent,
            turn_slowdown_remaining_deg=config.turn_slowdown_remaining_deg,
            turn_creep_speed_percent=config.turn_creep_speed_percent,
            turn_angle_trim_deg=config.turn_angle_trim_deg,
            last_turn_angle_trim_deg=config.last_turn_angle_trim_deg,
            turn_check_interval_sec=config.turn_check_interval_sec,
            turn_obstacle_check_interval_sec=config.turn_obstacle_check_interval_sec,
            turn_timeout_per_deg=config.turn_timeout_per_deg,
            turn_timeout_min=config.turn_timeout_min,
            lifecycle=self.lifecycle,
        )

        self._route_runner: RouteRunner = RouteRunner(
            linear_motion=self._linear_motion,
            turn_executor=self._turn_executor,
            move_forward=motor_controller.move_forward,
            move_backward=motor_controller.move_backward,
            gyroscope=gyroscope,
            lifecycle=self.lifecycle,
            base_speed_percent=config.base_speed_percent,
        )

        self._route_executor: RouteExecutor = RouteExecutor(
            route_runner=self._route_runner,
            motor_controller=motor_controller,
            lifecycle=self.lifecycle,
        )

    def forward_cm_sync(self, distance_cm: float, max_speed_percent: int | None = None) -> None:
        """Проехать заданную дистанцию в текущем потоке."""
        self._stop_motion()
        self.lifecycle.set_running(is_route_running=False)

        try:
            speed_percent: int = (
                self.config.base_speed_percent
                if max_speed_percent is None
                else max_speed_percent
            )
            self._linear_motion.run(distance_cm, speed_percent, self.motor_controller.move_forward)
        finally:
            self.motor_controller.stop()
            self.lifecycle.set_stopped()

    def execute_route(self, route: Route) -> None:
        """Запустить выполнение маршрута в фоновом потоке."""
        self._stop_motion()
        self.lifecycle.set_running(is_route_running=True)
        self._route_executor.execute(route)

    def execute_route_sync(self, route: Route) -> None:
        """Запустить выполнение маршрута в текущем потоке."""
        self._stop_motion()
        self.lifecycle.set_running(is_route_running=True)

        try:
            self._route_executor.execute_sync(route)
        finally:
            self.motor_controller.stop()
            self.lifecycle.set_stopped()

    def stop(self) -> None:
        """Остановить текущее движение."""
        self._route_executor.stop()
        self._stop_motion()

    def destroy(self) -> None:
        """Освободить все ресурсы, связанные с движением."""
        self.stop()
        self.motor_controller.destroy()
        self.ultrasonic_sensor.destroy()
        self.gyroscope.destroy()

    def _stop_motion(self) -> None:
        """Остановить движение и дождаться фонового потока при необходимости."""
        self.lifecycle.stop_and_join(self.motor_controller.stop, timeout_sec=1.0)
