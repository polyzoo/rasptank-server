from __future__ import annotations

import unittest
from unittest.mock import Mock, call, patch

from src.application.services.drive_controller import (
    AvoidanceContext,
    AvoidanceResult,
    AvoidanceSide,
    DriveController,
    LinearMoveResult,
    SideScanResult,
    TurnResult,
)


class DummyMotorController:
    """Простой тестовый контроллер моторов."""

    def move_forward(self, speed_percent: int, steer_percent: int = 0) -> None:
        return

    def move_backward(self, speed_percent: int, steer_percent: int = 0) -> None:
        return

    def turn_left(self, speed_percent: int) -> None:
        return

    def turn_right(self, speed_percent: int) -> None:
        return

    def stop(self) -> None:
        return

    def destroy(self) -> None:
        return


class DummyUltrasonicSensor:
    """Простой тестовый ультразвуковой датчик."""

    def measure_distance_cm(self) -> float:
        return 999.0

    def destroy(self) -> None:
        return


class DummyGyroscope:
    """Простой тестовый гироскоп."""

    def start(self, calibrate: bool = True) -> None:
        return

    def get_yaw(self) -> float:
        return 0.0

    def reset_yaw(self) -> None:
        return

    def stop(self) -> None:
        return

    def destroy(self) -> None:
        return


class DriveControllerObstacleAvoidanceTests(unittest.TestCase):
    """Тесты пошагового обхода препятствий."""

    def setUp(self) -> None:
        self.controller: DriveController = DriveController(
            motor_controller=DummyMotorController(),
            ultrasonic_sensor=DummyUltrasonicSensor(),
            gyroscope=DummyGyroscope(),
            min_obstacle_distance_cm=20.0,
            deceleration_distance_cm=50.0,
            base_speed_percent=60,
            turn_speed_percent=50,
            max_speed_cm_per_sec=30.0,
            update_interval_sec=0.01,
            avoidance_scan_angle_deg=45.0,
            avoidance_side_step_cm=12.0,
            avoidance_forward_step_cm=15.0,
            avoidance_rejoin_step_cm=12.0,
            avoidance_max_attempts=24,
            avoidance_confirm_readings=3,
            avoidance_min_side_clearance_cm=25.0,
            avoidance_max_lateral_offset_cm=60.0,
            avoidance_max_bypass_distance_cm=200.0,
        )
        self.controller._is_moving = True
        self.controller._stop_event.clear()

    def _make_side_scan_result(
        self,
        side: AvoidanceSide,
        *,
        clearance_cm: float | None,
        heading_restored: bool = True,
        scan_completed: bool = True,
        used_partial_scan: bool = False,
        rejection_reason: str | None = None,
        turned_angle_deg: float = 45.0,
        limited_confidence: bool | None = None,
        turn_stop_reason: str | None = None,
        scan_useful: bool | None = None,
    ) -> SideScanResult:
        """Собирает side scan result для тестов выбора направления обхода."""
        return SideScanResult(
            side=side,
            target_angle_deg=45.0,
            turned_angle_deg=turned_angle_deg,
            clearance_cm=clearance_cm,
            heading_restored=heading_restored,
            scan_completed=scan_completed,
            used_partial_scan=used_partial_scan,
            scan_useful=clearance_cm is not None if scan_useful is None else scan_useful,
            limited_confidence=(
                (used_partial_scan or not scan_completed)
                if limited_confidence is None
                else limited_confidence
            ),
            turn_stop_reason=(
                "target_reached"
                if turn_stop_reason is None and scan_completed
                else "timeout" if turn_stop_reason is None else turn_stop_reason
            ),
            rejection_reason=rejection_reason,
        )

    def test_select_avoidance_side_prefers_more_open_direction(self) -> None:
        """Выбирается сторона с большим запасом по расстоянию."""
        with patch.object(
            self.controller,
            "_scan_side_observation",
            side_effect=[
                self._make_side_scan_result(AvoidanceSide.LEFT, clearance_cm=65.0),
                self._make_side_scan_result(AvoidanceSide.RIGHT, clearance_cm=30.0),
            ],
        ) as scan_mock:
            result: AvoidanceSide | None = self.controller._select_avoidance_side()

        self.assertEqual(AvoidanceSide.LEFT, result)
        self.assertEqual(
            [call(AvoidanceSide.LEFT), call(AvoidanceSide.RIGHT)],
            scan_mock.call_args_list,
        )

    def test_select_avoidance_side_returns_none_when_both_sides_tight(self) -> None:
        """Обход не начинается, если безопасной стороны нет."""
        with patch.object(
            self.controller,
            "_scan_side_observation",
            side_effect=[
                self._make_side_scan_result(AvoidanceSide.LEFT, clearance_cm=21.0),
                self._make_side_scan_result(AvoidanceSide.RIGHT, clearance_cm=22.0),
            ],
        ):
            result: AvoidanceSide | None = self.controller._select_avoidance_side()

        self.assertIsNone(result)

    def test_select_avoidance_side_uses_other_side_when_one_restore_fails(self) -> None:
        """Отказ restore heading на одной стороне не должен отменять обход целиком."""
        with patch.object(
            self.controller,
            "_scan_side_observation",
            side_effect=[
                self._make_side_scan_result(
                    AvoidanceSide.LEFT,
                    clearance_cm=None,
                    heading_restored=False,
                    rejection_reason="restore failed",
                ),
                self._make_side_scan_result(AvoidanceSide.RIGHT, clearance_cm=41.0),
            ],
        ):
            result: AvoidanceSide | None = self.controller._select_avoidance_side()

        self.assertEqual(AvoidanceSide.RIGHT, result)

    def test_select_avoidance_side_accepts_borderline_valid_side(self) -> None:
        """Пограничное шумное измерение не должно преждевременно запрещать обход."""
        with patch.object(
            self.controller,
            "_scan_side_observation",
            side_effect=[
                self._make_side_scan_result(AvoidanceSide.LEFT, clearance_cm=24.0),
                self._make_side_scan_result(AvoidanceSide.RIGHT, clearance_cm=22.0),
            ],
        ):
            result: AvoidanceSide | None = self.controller._select_avoidance_side()

        self.assertEqual(AvoidanceSide.LEFT, result)

    def test_select_avoidance_side_uses_best_inconclusive_partial_scan(self) -> None:
        """Неполный scan около 30° не должен сразу приводить к выводу `обход невозможен`."""
        with patch.object(
            self.controller,
            "_scan_side_observation",
            side_effect=[
                self._make_side_scan_result(
                    AvoidanceSide.LEFT,
                    clearance_cm=18.0,
                    scan_completed=False,
                    used_partial_scan=True,
                    turned_angle_deg=30.0,
                    limited_confidence=True,
                    turn_stop_reason="timeout",
                ),
                self._make_side_scan_result(
                    AvoidanceSide.RIGHT,
                    clearance_cm=16.0,
                    scan_completed=False,
                    used_partial_scan=True,
                    turned_angle_deg=29.0,
                    limited_confidence=True,
                    turn_stop_reason="timeout",
                ),
            ],
        ):
            result: AvoidanceSide | None = self.controller._select_avoidance_side()

        self.assertEqual(AvoidanceSide.LEFT, result)

    def test_scan_side_clearance_returns_none_when_heading_restore_fails(self) -> None:
        """После неудачного возврата курса side scan аварийно завершается."""
        with (
            patch.object(
                self.controller,
                "_turn_relative",
                side_effect=[
                    TurnResult(completed=True, angle_deg=45.0),
                    TurnResult(completed=False, angle_deg=30.0),
                ],
            ) as turn_mock,
            patch.object(self.controller, "_measure_front_distance", return_value=55.0),
        ):
            result: float | None = self.controller._scan_side_clearance(AvoidanceSide.LEFT)

        self.assertIsNone(result)
        self.assertEqual(2, turn_mock.call_count)

    def test_scan_side_clearance_uses_effective_partial_scan(self) -> None:
        """Частичный поворот около 30° должен считаться advisory scan, а не пустым."""
        with (
            patch.object(
                self.controller,
                "_turn_relative",
                side_effect=[
                    TurnResult(completed=False, angle_deg=28.0, stop_reason="timeout"),
                    TurnResult(completed=True, angle_deg=28.0, stop_reason="target_reached"),
                ],
            ) as turn_mock,
            patch.object(self.controller, "_measure_front_distance", return_value=52.0),
        ):
            result: float | None = self.controller._scan_side_clearance(AvoidanceSide.LEFT)

        self.assertEqual(52.0, result)
        self.assertEqual(2, turn_mock.call_count)

    def test_run_linear_motion_confirms_front_block_before_reporting_blocked(self) -> None:
        """Одиночный шумный low reading не должен переводить в obstacle avoidance."""
        self.controller.update_interval_sec = 0.0
        self.controller.motor_controller = Mock(spec=DummyMotorController)
        self.controller.motor_controller.move_forward.side_effect = (
            lambda speed_percent, steer_percent=0: setattr(self.controller, "_is_moving", False)
        )
        self.controller.ultrasonic_sensor = Mock(spec=DummyUltrasonicSensor)
        self.controller.ultrasonic_sensor.measure_distance_cm.side_effect = [10.0, 100.0, 100.0, 100.0]

        result: LinearMoveResult = self.controller._run_linear_motion(
            distance_cm=20.0,
            speed_percent=60,
            move_forward=True,
            obstacle_aware=True,
        )

        self.assertFalse(result.blocked)
        self.controller.motor_controller.move_forward.assert_called_once()

    def test_measure_front_distance_uses_minimum_sample_count(self) -> None:
        """Даже при misconfig внутри FSM остаётся минимум три чтения для фильтрации."""
        self.controller.avoidance_confirm_readings = 1
        self.controller.ultrasonic_sensor = Mock(spec=DummyUltrasonicSensor)
        self.controller.ultrasonic_sensor.measure_distance_cm.side_effect = [45.0, 10.0, 55.0]

        result: float = self.controller._measure_front_distance()

        self.assertEqual(45.0, result)
        self.assertEqual(3, self.controller.ultrasonic_sensor.measure_distance_cm.call_count)

    def test_front_clear_confirmation_requires_margin_twice(self) -> None:
        """Одного «почти свободно» недостаточно для выхода из обхода."""
        self.controller.update_interval_sec = 0.0

        with patch.object(
            self.controller,
            "_measure_front_distance",
            side_effect=[24.0, 21.0],
        ) as distance_mock:
            result: bool = self.controller._is_front_clear_confirmed()

        self.assertFalse(result)
        self.assertEqual(2, distance_mock.call_count)

    def test_obstacle_avoidance_handles_longer_obstacle_in_multiple_steps(self) -> None:
        """Обход продолжает смещаться и постепенно возвращается на маршрут."""
        with (
            patch.object(self.controller, "_select_avoidance_side", return_value=AvoidanceSide.LEFT),
            patch.object(
                self.controller,
                "_perform_side_step",
                side_effect=[
                    LinearMoveResult(completed=True, traveled_cm=12.0),
                    LinearMoveResult(completed=True, traveled_cm=12.0),
                ],
            ) as side_step_mock,
            patch.object(
                self.controller,
                "_measure_front_distance",
                side_effect=[10.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0],
            ) as distance_mock,
            patch.object(
                self.controller,
                "_perform_forward_step",
                side_effect=[
                    LinearMoveResult(completed=True, traveled_cm=15.0),
                    LinearMoveResult(completed=True, traveled_cm=15.0),
                ],
            ) as forward_step_mock,
            patch.object(
                self.controller,
                "_attempt_rejoin_step",
                side_effect=[
                    LinearMoveResult(completed=True, traveled_cm=12.0),
                    LinearMoveResult(completed=True, traveled_cm=12.0),
                ],
            ) as rejoin_mock,
        ):
            result: AvoidanceResult = self.controller._run_obstacle_avoidance(
                remaining_cm=80.0,
                speed_percent=60,
            )

        self.assertTrue(result.completed)
        self.assertEqual(30.0, result.forward_progress_cm)
        self.assertEqual(2, side_step_mock.call_count)
        self.assertEqual(2, forward_step_mock.call_count)
        self.assertEqual(2, rejoin_mock.call_count)
        self.assertEqual(7, distance_mock.call_count)

    def test_obstacle_avoidance_stops_when_attempt_limit_is_reached(self) -> None:
        """Автомат безопасно останавливается, если шагов обхода слишком много."""
        self.controller.avoidance_max_attempts = 2

        with (
            patch.object(self.controller, "_select_avoidance_side", return_value=AvoidanceSide.RIGHT),
            patch.object(
                self.controller,
                "_perform_side_step",
                return_value=LinearMoveResult(completed=True, traveled_cm=12.0),
            ) as side_step_mock,
            patch.object(self.controller, "_measure_front_distance", return_value=10.0),
        ):
            result: AvoidanceResult = self.controller._run_obstacle_avoidance(
                remaining_cm=100.0,
                speed_percent=60,
            )

        self.assertFalse(result.completed)
        self.assertEqual(0.0, result.forward_progress_cm)
        self.assertEqual(2, side_step_mock.call_count)

    def test_obstacle_avoidance_stops_when_side_step_does_not_restore_heading(self) -> None:
        """После неудачного восстановления курса автомат аварийно завершается."""
        with (
            patch.object(self.controller, "_select_avoidance_side", return_value=AvoidanceSide.LEFT),
            patch.object(
                self.controller,
                "_perform_side_step",
                return_value=LinearMoveResult(
                    completed=False,
                    traveled_cm=12.0,
                    heading_restored=False,
                ),
            ) as side_step_mock,
            patch.object(self.controller, "_measure_front_distance") as distance_mock,
        ):
            result: AvoidanceResult = self.controller._run_obstacle_avoidance(
                remaining_cm=100.0,
                speed_percent=60,
            )

        self.assertFalse(result.completed)
        self.assertEqual(0.0, result.forward_progress_cm)
        side_step_mock.assert_called_once()
        distance_mock.assert_not_called()

    def test_obstacle_avoidance_stops_on_incomplete_unblocked_side_step(self) -> None:
        """Abort линейного шага не должен маскироваться под валидный partial side step."""
        with (
            patch.object(self.controller, "_select_avoidance_side", return_value=AvoidanceSide.LEFT),
            patch.object(
                self.controller,
                "_perform_side_step",
                return_value=LinearMoveResult(
                    completed=False,
                    traveled_cm=6.0,
                    blocked=False,
                    heading_restored=True,
                ),
            ) as side_step_mock,
            patch.object(self.controller, "_measure_front_distance") as distance_mock,
        ):
            result: AvoidanceResult = self.controller._run_obstacle_avoidance(
                remaining_cm=100.0,
                speed_percent=60,
            )

        self.assertFalse(result.completed)
        self.assertEqual(0.0, result.forward_progress_cm)
        side_step_mock.assert_called_once()
        distance_mock.assert_not_called()

    def test_obstacle_avoidance_rejoins_before_finishing_segment(self) -> None:
        """Даже при добранном forward progress робот должен сначала убрать боковое смещение."""
        with (
            patch.object(self.controller, "_select_avoidance_side", return_value=AvoidanceSide.LEFT),
            patch.object(
                self.controller,
                "_perform_side_step",
                return_value=LinearMoveResult(completed=True, traveled_cm=12.0),
            ) as side_step_mock,
            patch.object(self.controller, "_is_front_clear_confirmed", return_value=True),
            patch.object(
                self.controller,
                "_perform_forward_step",
                return_value=LinearMoveResult(completed=True, traveled_cm=10.0),
            ) as forward_step_mock,
            patch.object(
                self.controller,
                "_attempt_rejoin_step",
                return_value=LinearMoveResult(completed=True, traveled_cm=12.0),
            ) as rejoin_mock,
        ):
            result: AvoidanceResult = self.controller._run_obstacle_avoidance(
                remaining_cm=10.0,
                speed_percent=60,
            )

        self.assertTrue(result.completed)
        self.assertEqual(10.0, result.forward_progress_cm)
        side_step_mock.assert_called_once()
        forward_step_mock.assert_called_once()
        rejoin_mock.assert_called_once()

    def test_obstacle_avoidance_stops_on_incomplete_unblocked_rejoin(self) -> None:
        """Abort во время возврата на маршрут не должен тихо продолжать FSM."""
        with (
            patch.object(self.controller, "_select_avoidance_side", return_value=AvoidanceSide.LEFT),
            patch.object(
                self.controller,
                "_perform_side_step",
                return_value=LinearMoveResult(completed=True, traveled_cm=12.0),
            ),
            patch.object(self.controller, "_is_front_clear_confirmed", return_value=True),
            patch.object(
                self.controller,
                "_perform_forward_step",
                return_value=LinearMoveResult(completed=True, traveled_cm=8.0),
            ),
            patch.object(
                self.controller,
                "_attempt_rejoin_step",
                return_value=LinearMoveResult(
                    completed=False,
                    traveled_cm=6.0,
                    blocked=False,
                    heading_restored=True,
                ),
            ) as rejoin_mock,
        ):
            result: AvoidanceResult = self.controller._run_obstacle_avoidance(
                remaining_cm=40.0,
                speed_percent=60,
            )

        self.assertFalse(result.completed)
        self.assertEqual(8.0, result.forward_progress_cm)
        rejoin_mock.assert_called_once()

    def test_avoidance_limit_stops_at_exact_lateral_offset_boundary(self) -> None:
        """Боковой лимит должен быть жёстким, без дополнительного лишнего шага."""
        context = AvoidanceContext(
            lateral_offset_cm=self.controller.avoidance_max_lateral_offset_cm,
        )

        result: bool = self.controller._has_exceeded_avoidance_limits(context)

        self.assertTrue(result)

    def test_avoidance_limit_stops_at_exact_bypass_distance_boundary(self) -> None:
        """Лимит суммарной длины обхода должен срабатывать прямо на границе."""
        context = AvoidanceContext(
            bypass_distance_cm=self.controller.avoidance_max_bypass_distance_cm,
        )

        result: bool = self.controller._has_exceeded_avoidance_limits(context)

        self.assertTrue(result)

    def test_forward_segment_resumes_after_successful_avoidance(self) -> None:
        """После обхода контроллер продолжает исходный forward-сегмент."""
        with (
            patch.object(
                self.controller,
                "_run_linear_motion",
                side_effect=[
                    LinearMoveResult(completed=False, traveled_cm=30.0, blocked=True),
                    LinearMoveResult(completed=True, traveled_cm=20.0),
                ],
            ) as linear_mock,
            patch.object(
                self.controller,
                "_run_obstacle_avoidance",
                return_value=AvoidanceResult(completed=True, forward_progress_cm=10.0),
            ) as avoidance_mock,
        ):
            result: bool = self.controller._run_forward_segment(distance_cm=60.0, speed_percent=60)

        self.assertTrue(result)
        self.assertEqual(
            [
                call(
                    distance_cm=60.0,
                    speed_percent=60,
                    move_forward=True,
                    obstacle_aware=True,
                ),
                call(
                    distance_cm=20.0,
                    speed_percent=60,
                    move_forward=True,
                    obstacle_aware=True,
                ),
            ],
            linear_mock.call_args_list,
        )
        avoidance_mock.assert_called_once_with(remaining_cm=30.0, speed_percent=60)


if __name__ == "__main__":
    unittest.main()
