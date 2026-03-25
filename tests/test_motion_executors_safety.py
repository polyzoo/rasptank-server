from __future__ import annotations

import unittest
from unittest.mock import Mock

from src.application.services.linear_motion_executor import LinearMotionExecutor
from src.application.services.motion_lifecycle import MotionLifecycle
from src.application.services.turn_executor import TurnExecutor


class MotionExecutorsSafetyTests(unittest.TestCase):
    """Safety-регрессии для low-level executors движения."""

    def test_linear_motion_executor_stops_motor_on_exception(self) -> None:
        """Исключение внутри линейного движения не должно оставлять моторы активными."""
        lifecycle = MotionLifecycle()
        lifecycle.set_running(is_route_running=False)
        motor = Mock()
        sensor = Mock()
        gyro = Mock()
        sensor.measure_distance_cm.side_effect = RuntimeError("sensor boom")
        gyro.get_yaw.return_value = 0.0

        executor = LinearMotionExecutor(
            motor_controller=motor,
            ultrasonic_sensor=sensor,
            gyroscope=gyro,
            min_obstacle_distance_cm=20.0,
            deceleration_distance_cm=40.0,
            max_speed_cm_per_sec=30.0,
            update_interval_sec=0.0,
            forward_soft_start_sec=0.0,
            heading_hold_enabled=False,
            heading_hold_kp=0.0,
            heading_hold_steer_max=0,
            heading_hold_deadband_deg=0.0,
            heading_hold_steer_speed_ratio=0.0,
            heading_hold_min_speed_percent=0.0,
            heading_hold_steer_cap_min_spd_percent=0.0,
            heading_hold_steer_trim=0,
            heading_hold_invert_steer=False,
            lifecycle=lifecycle,
        )

        result = executor.run_with_result(
            distance_cm=20.0,
            speed_percent=50,
            move_fn=motor.move_forward,
            obstacle_aware=True,
        )

        self.assertFalse(result.completed)
        motor.stop.assert_called_once()

    def test_turn_executor_stops_motor_on_exception(self) -> None:
        """Исключение во время поворота должно гарантированно делать stop в finally."""
        lifecycle = MotionLifecycle()
        lifecycle.set_running(is_route_running=False)
        motor = Mock()
        sensor = Mock()
        gyro = Mock()
        gyro.get_yaw.side_effect = RuntimeError("gyro boom")

        executor = TurnExecutor(
            motor_controller=motor,
            ultrasonic_sensor=sensor,
            gyroscope=gyro,
            min_obstacle_distance_cm=20.0,
            turn_speed_percent=50,
            turn_slowdown_remaining_deg=8.0,
            turn_creep_speed_percent=30,
            turn_angle_trim_deg=0.0,
            last_turn_angle_trim_deg=0.0,
            turn_check_interval_sec=0.0,
            turn_obstacle_check_interval_sec=0.1,
            turn_timeout_per_deg=0.05,
            turn_timeout_min=0.1,
            lifecycle=lifecycle,
        )

        with self.assertRaises(RuntimeError):
            executor.run_relative(
                target_angle_deg=45.0,
                turn_left=True,
                stop_on_front_obstacle=False,
            )

        motor.stop.assert_called_once()


if __name__ == "__main__":
    unittest.main()
