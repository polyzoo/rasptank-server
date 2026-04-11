from __future__ import annotations

from collections.abc import Callable

from src.application.models.route import (
    BACKWARD_ACTION,
    FORWARD_ACTION,
    TURN_LEFT_ACTION,
    TURN_RIGHT_ACTION,
    BackwardSegment,
    ForwardSegment,
    RouteSegment,
    TurnLeftSegment,
    TurnRightSegment,
)
from src.application.protocols import GyroscopeProtocol
from src.application.services.linear_motion_executor import LinearMotionExecutor, MoveCommand
from src.application.services.motion_lifecycle import MotionLifecycle
from src.application.services.turn_executor import TurnExecutor

ForwardSegmentRunner = Callable[[float, int], bool]
BackwardSegmentRunner = Callable[[float, int], bool]
TurnCompletedCallback = Callable[[float, bool], None]


class RouteRunner:
    """Исполнитель последовательного выполнения сегментов маршрута."""

    def __init__(
        self,
        linear_motion: LinearMotionExecutor,
        turn_executor: TurnExecutor,
        move_forward: MoveCommand,
        move_backward: MoveCommand,
        gyroscope: GyroscopeProtocol,
        lifecycle: MotionLifecycle,
        base_speed_percent: int,
        forward_segment_runner: ForwardSegmentRunner | None = None,
        backward_segment_runner: BackwardSegmentRunner | None = None,
        turn_completed_callback: TurnCompletedCallback | None = None,
    ) -> None:
        """Инициализация исполнителя маршрута."""
        self._linear_motion: LinearMotionExecutor = linear_motion
        self._turn_executor: TurnExecutor = turn_executor
        self._move_forward: MoveCommand = move_forward
        self._move_backward: MoveCommand = move_backward
        self._gyroscope: GyroscopeProtocol = gyroscope
        self._lifecycle: MotionLifecycle = lifecycle
        self.base_speed_percent: int = base_speed_percent
        self._forward_segment_runner: ForwardSegmentRunner | None = forward_segment_runner
        self._backward_segment_runner: BackwardSegmentRunner | None = backward_segment_runner
        self._turn_completed_callback: TurnCompletedCallback | None = turn_completed_callback

    def run(self, segments: list[RouteSegment]) -> None:
        """Выполнить все сегменты маршрута по порядку."""
        self._gyroscope.start(calibrate=True)
        try:
            self._run_segments(segments)
        finally:
            self._gyroscope.stop()

    def _run_segments(self, segments: list[RouteSegment]) -> None:
        """Пройти маршрут по сегментам."""
        total_segments: int = len(segments)
        for idx, segment in enumerate(segments):
            if not self._lifecycle.should_keep_running():
                break

            if segment.action == FORWARD_ACTION:
                if not self._run_forward_segment(segment):
                    break

            elif segment.action == BACKWARD_ACTION:
                if not self._run_backward_segment(segment):
                    break

            elif segment.action == TURN_LEFT_ACTION:
                self._run_turn_left_segment(segment, idx, total_segments)

            elif segment.action == TURN_RIGHT_ACTION:
                self._run_turn_right_segment(segment, idx, total_segments)

            else:
                break

    def _run_forward_segment(self, segment: ForwardSegment) -> bool:
        """Выполнить прямой сегмент маршрута."""
        if self._forward_segment_runner is not None:
            return self._forward_segment_runner(segment.distance_cm, self.base_speed_percent)

        return self._linear_motion.run(
            distance_cm=segment.distance_cm,
            speed_percent=self.base_speed_percent,
            move_fn=self._move_forward,
            obstacle_aware=True,
        )

    def _run_backward_segment(self, segment: BackwardSegment) -> bool:
        """Выполнить обратный сегмент маршрута."""
        if self._backward_segment_runner is not None:
            return self._backward_segment_runner(segment.distance_cm, self.base_speed_percent)

        return self._linear_motion.run(
            distance_cm=segment.distance_cm,
            speed_percent=self.base_speed_percent,
            move_fn=self._move_backward,
            obstacle_aware=False,
        )

    def _run_turn_left_segment(
        self,
        segment: TurnLeftSegment,
        segment_index: int,
        total_segments: int,
    ) -> None:
        """Выполнить поворот налево."""
        result = self._turn_executor.run(
            requested_angle_deg=segment.angle_deg,
            turn_left=True,
            segment_index=segment_index,
            total_segments=total_segments,
        )
        if self._turn_completed_callback is not None:
            self._turn_completed_callback(result.angle_deg, True)

    def _run_turn_right_segment(
        self,
        segment: TurnRightSegment,
        segment_index: int,
        total_segments: int,
    ) -> None:
        """Выполнить поворот направо."""
        result = self._turn_executor.run(
            requested_angle_deg=segment.angle_deg,
            turn_left=False,
            segment_index=segment_index,
            total_segments=total_segments,
        )
        if self._turn_completed_callback is not None:
            self._turn_completed_callback(result.angle_deg, False)
