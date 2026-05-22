from __future__ import annotations

from src.application.services.goal_point_controller import GoalPointController
from src.application.services.l2_models import BodyVelocityCommand, L2State
from src.application.services.l3_models import (
    L3_PLANNER_STATUS_IMPOSSIBLE,
    L3State,
    Obstacle,
    PlannedRoute,
    TargetPoint,
    TargetRoute,
)
from src.application.services.l3_service import L3Service
from src.application.services.path_planner import PathPlanner


class FakeL2Service:
    """Заглушка L2 для проверки логики верхнего уровня."""

    def __init__(self, initial_state: L2State) -> None:
        """Сохранить начальное состояние и историю вызовов."""
        self.state: L2State = initial_state
        self.applied_commands: list[BodyVelocityCommand] = []
        self.stop_calls: int = 0

    def get_state(self) -> L2State:
        """Вернуть текущее состояние."""
        return self.state

    def apply_body_velocity(self, command: BodyVelocityCommand) -> L2State:
        """Сохранить команду и отразить её в текущем состоянии."""
        self.applied_commands.append(command)
        self.state = L2State(
            x_cm=self.state.x_cm,
            y_cm=self.state.y_cm,
            heading_deg=self.state.heading_deg,
            linear_speed_cm_per_sec=command.linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=command.angular_speed_deg_per_sec,
            left_percent=self.state.left_percent,
            right_percent=self.state.right_percent,
            distance_cm=self.state.distance_cm,
        )
        return self.state

    def stop(self) -> L2State:
        """Зафиксировать остановку L2."""
        self.stop_calls += 1
        self.state = L2State(
            x_cm=self.state.x_cm,
            y_cm=self.state.y_cm,
            heading_deg=self.state.heading_deg,
            linear_speed_cm_per_sec=0.0,
            angular_speed_deg_per_sec=0.0,
            left_percent=self.state.left_percent,
            right_percent=self.state.right_percent,
            distance_cm=self.state.distance_cm,
        )
        return self.state


def _controller() -> GoalPointController:
    """Создать контроллер цели для сервиса L3."""
    return GoalPointController(
        position_tolerance_cm=5.0,
        linear_speed_gain=1.0,
        angular_speed_gain=2.0,
        max_linear_speed_cm_per_sec=20.0,
        max_angular_speed_deg_per_sec=120.0,
        obstacle_stop_distance_cm=20.0,
        obstacle_slowdown_distance_cm=40.0,
    )


def _planner(*, max_detour_offset_cm: float = 40.0) -> PathPlanner:
    """Создать планировщик пути для L3."""
    return PathPlanner(
        obstacle_clearance_cm=5.0,
        max_detour_offset_cm=max_detour_offset_cm,
        max_waypoints=24,
    )


def _service(
    l2_service: FakeL2Service,
    *,
    max_detour_offset_cm: float = 40.0,
) -> L3Service:
    """Собрать сервис L3 с единым радиусом неизвестного препятствия для тестов."""
    return L3Service(
        _controller(),
        _planner(max_detour_offset_cm=max_detour_offset_cm),
        l2_service,  # type: ignore[arg-type]
        unknown_obstacle_radius_cm=8.0,
    )


def _state(**overrides: float | None) -> L2State:
    """Собрать состояние L2 для тестов L3."""
    return L2State(
        x_cm=overrides.get("x_cm", 0.0) or 0.0,
        y_cm=overrides.get("y_cm", 0.0) or 0.0,
        heading_deg=overrides.get("heading_deg", 0.0) or 0.0,
        linear_speed_cm_per_sec=overrides.get("linear_speed_cm_per_sec", 0.0) or 0.0,
        angular_speed_deg_per_sec=overrides.get("angular_speed_deg_per_sec", 0.0) or 0.0,
        left_percent=overrides.get("left_percent", 0.0) or 0.0,
        right_percent=overrides.get("right_percent", 0.0) or 0.0,
        distance_cm=overrides.get("distance_cm"),
    )


def test_step_returns_idle_without_target() -> None:
    """Без цели L3 остаётся в состоянии idle."""
    service = _service(FakeL2Service(_state()))

    state: L3State = service.step()

    assert state.status == "idle"
    assert state.mode == "idle"
    assert state.target_x_cm is None
    assert state.target_y_cm is None


def test_step_tracks_target_through_l2() -> None:
    """При цели впереди L3 запрашивает движение через L2."""
    l2_service = FakeL2Service(_state())
    service = _service(l2_service)
    service.set_target_point(TargetPoint(x_cm=50.0, y_cm=0.0))

    state: L3State = service.step()

    assert state.status == "tracking"
    assert state.mode == "point"
    assert state.planner_status == "planned"
    assert state.active_point_index == 0
    assert state.total_points == 1
    assert len(l2_service.applied_commands) == 1
    assert l2_service.applied_commands[0].linear_speed_cm_per_sec == 20.0
    assert l2_service.applied_commands[0].angular_speed_deg_per_sec == 0.0
    assert l2_service.applied_commands[0].target_x_cm == 50.0
    assert l2_service.applied_commands[0].target_y_cm == 0.0


def test_step_marks_target_reached_and_clears_it() -> None:
    """Когда цель достигнута, L3 останавливает движение и очищает цель."""
    l2_service = FakeL2Service(_state(x_cm=9.0, y_cm=9.0))
    service = _service(l2_service)
    service.set_target_point(TargetPoint(x_cm=10.0, y_cm=10.0))

    state: L3State = service.step()

    assert state.status == "reached"
    assert state.mode == "idle"
    assert state.target_x_cm is None
    assert state.target_y_cm is None
    assert l2_service.stop_calls == 1


def test_step_marks_target_reached_when_robot_passes_target_progress() -> None:
    """Если робот проскочил цель вдоль отрезка, L3 не должен разворачиваться назад."""
    l2_service = FakeL2Service(_state())
    service = _service(l2_service)
    service.set_target_point(TargetPoint(x_cm=100.0, y_cm=0.0))
    l2_service.state = _state(x_cm=106.0, y_cm=0.0, heading_deg=0.0)

    state: L3State = service.step()

    assert state.status == "reached"
    assert state.mode == "idle"
    assert l2_service.stop_calls == 1
    assert len(l2_service.applied_commands) == 0


def test_step_marks_target_blocked_and_keeps_it() -> None:
    """Если впереди неизвестное препятствие, L3 перестраивает маршрут и сохраняет цель."""
    l2_service = FakeL2Service(_state(distance_cm=10.0))
    service = _service(l2_service)
    service.set_target_point(TargetPoint(x_cm=50.0, y_cm=0.0))

    state: L3State = service.step()

    assert state.status == "tracking"
    assert state.mode == "point"
    assert state.planner_status == "replanned_dynamic"
    assert state.target_x_cm != 50.0
    assert state.target_y_cm != 0.0
    assert state.detected_obstacle_x_cm is not None
    assert state.detected_obstacle_y_cm == 0.0
    assert state.detected_obstacle_radius_cm == 8.0
    assert state.detected_obstacle_kind == "dynamic"
    assert l2_service.stop_calls == 1


def test_cancel_stops_l2_and_resets_target() -> None:
    """Отмена цели переводит L3 в состояние cancelled."""
    l2_service = FakeL2Service(_state(linear_speed_cm_per_sec=5.0))
    service = _service(l2_service)
    service.set_target_point(TargetPoint(x_cm=50.0, y_cm=0.0))

    state: L3State = service.cancel()

    assert state.status == "cancelled"
    assert state.mode == "idle"
    assert state.target_x_cm is None
    assert state.target_y_cm is None
    assert state.detected_obstacle_x_cm is None
    assert l2_service.stop_calls == 1


def test_step_advances_to_next_route_point() -> None:
    """При достижении промежуточной точки L3 переключается на следующую точку маршрута."""
    l2_service = FakeL2Service(_state(x_cm=10.0, y_cm=10.0))
    service = _service(l2_service)
    service.set_route(
        TargetRoute(
            points=(
                TargetPoint(x_cm=10.0, y_cm=10.0),
                TargetPoint(x_cm=50.0, y_cm=0.0),
            )
        )
    )

    state: L3State = service.step()

    assert state.status == "tracking"
    assert state.mode == "route"
    assert state.active_point_index == 1
    assert state.total_points == 2
    assert state.target_x_cm == 50.0
    assert state.target_y_cm == 0.0
    assert l2_service.stop_calls == 1


def test_set_route_with_empty_points_cancels_motion() -> None:
    """Пустой маршрут сразу приводит к отмене движения."""
    l2_service = FakeL2Service(_state(linear_speed_cm_per_sec=3.0))
    service = _service(l2_service)

    state: L3State = service.set_route(TargetRoute(points=()))

    assert state.status == "cancelled"
    assert state.mode == "idle"
    assert l2_service.stop_calls == 1


def test_step_returns_idle_when_active_point_index_out_of_range() -> None:
    """Защитная ветка: некорректный индекс активной точки переводит сервис в idle."""
    l2_service = FakeL2Service(_state())
    service = _service(l2_service)
    service.set_target_point(TargetPoint(x_cm=50.0, y_cm=0.0))
    service._active_point_index = 10  # type: ignore[attr-defined]

    state: L3State = service.step()

    assert state.status == "idle"
    assert state.mode == "idle"


def test_advance_route_returns_false_without_active_point() -> None:
    """Защитная ветка: без активной точки маршрут не продвигается."""
    service = _service(FakeL2Service(_state()))

    assert service._advance_route() is False  # type: ignore[attr-defined]


def test_current_segment_start_point_returns_none_without_active_point() -> None:
    """Защитная ветка: без активной точки старт отрезка неизвестен."""
    service = _service(FakeL2Service(_state()))

    assert service._current_segment_start_point() is None  # type: ignore[attr-defined]


def test_current_segment_start_point_returns_previous_route_point() -> None:
    """Для не первой точки маршрута стартом считается предыдущая точка."""
    service = _service(FakeL2Service(_state()))
    first_point = TargetPoint(x_cm=10.0, y_cm=0.0)
    second_point = TargetPoint(x_cm=20.0, y_cm=0.0)
    service._route_points = (first_point, second_point)  # type: ignore[attr-defined]
    service._active_point_index = 1  # type: ignore[attr-defined]

    assert service._current_segment_start_point() == first_point  # type: ignore[attr-defined]


def test_current_segment_start_point_returns_none_for_missing_previous_point() -> None:
    """Защитная ветка: битый индекс не должен давать фиктивный старт отрезка."""
    service = _service(FakeL2Service(_state()))
    service._route_points = (TargetPoint(x_cm=10.0, y_cm=0.0),)  # type: ignore[attr-defined]
    service._active_point_index = 2  # type: ignore[attr-defined]

    assert service._current_segment_start_point() is None  # type: ignore[attr-defined]


def test_current_target_reached_by_progress_returns_false_without_segment_start() -> None:
    """Без старта отрезка защита от перелета не срабатывает."""
    service = _service(FakeL2Service(_state()))

    assert (  # type: ignore[attr-defined]
        service._current_target_reached_by_progress(
            current_state=_state(x_cm=10.0),
            current_target=TargetPoint(x_cm=10.0, y_cm=0.0),
        )
        is False
    )


def test_current_target_reached_by_progress_returns_false_for_zero_segment() -> None:
    """Нулевой отрезок нельзя считать достигнутым по прогрессу."""
    service = _service(FakeL2Service(_state()))
    point = TargetPoint(x_cm=10.0, y_cm=0.0)
    service._route_start_point = point  # type: ignore[attr-defined]
    service._route_points = (point,)  # type: ignore[attr-defined]
    service._active_point_index = 0  # type: ignore[attr-defined]

    assert (  # type: ignore[attr-defined]
        service._current_target_reached_by_progress(
            current_state=_state(x_cm=10.0),
            current_target=point,
        )
        is False
    )


def test_mark_active_goal_reached_returns_without_current_target() -> None:
    """Защитная ветка: без активной точки достигнутую цель удалять не нужно."""
    service = _service(FakeL2Service(_state()))
    service._remaining_goal_points = (TargetPoint(x_cm=50.0, y_cm=0.0),)  # type: ignore[attr-defined]

    service._mark_active_goal_reached()  # type: ignore[attr-defined]

    assert service._remaining_goal_points == (TargetPoint(x_cm=50.0, y_cm=0.0),)  # type: ignore[attr-defined]


def test_step_keeps_tracking_detour_when_same_unknown_obstacle_is_seen_again() -> None:
    """Повторно увиденное препятствие не останавливает поворот к боковой точке обхода."""
    l2_service = FakeL2Service(_state(distance_cm=10.0))
    service = _service(l2_service)
    service.set_target_point(TargetPoint(x_cm=50.0, y_cm=0.0))
    first_step: L3State = service.step()
    assert first_step.status == "tracking"

    state: L3State = service.step()

    assert state.status == "tracking"
    assert state.mode == "point"
    assert state.target_x_cm == first_step.target_x_cm
    assert state.target_y_cm == first_step.target_y_cm
    assert state.detected_obstacle_x_cm == first_step.detected_obstacle_x_cm
    assert state.detected_obstacle_kind == "dynamic"
    assert l2_service.stop_calls == 1
    assert len(l2_service.applied_commands) == 1
    assert l2_service.applied_commands[0].angular_speed_deg_per_sec != 0.0


def test_step_returns_blocked_when_obstacle_cannot_be_inferred() -> None:
    """Если blocked пришёл без дальности, L3 остаётся в blocked без обхода."""
    l2_service = FakeL2Service(_state(distance_cm=10.0))
    service = _service(l2_service)
    service.set_target_point(TargetPoint(x_cm=50.0, y_cm=0.0))
    service._handle_unknown_obstacle = lambda **kwargs: None  # type: ignore[method-assign]

    state: L3State = service.step()

    assert state.status == "blocked"
    assert state.desired_linear_speed_cm_per_sec == 0.0
    assert state.desired_angular_speed_deg_per_sec == 0.0


def test_step_tries_left_bypass_when_right_bypass_blocks() -> None:
    """Если правый квадратный обход блокируется, L3 пробует левую сторону."""
    l2_service = FakeL2Service(_state(distance_cm=10.0))
    service = _service(l2_service)
    service.set_target_point(TargetPoint(x_cm=100.0, y_cm=0.0))
    right_state: L3State = service.step()
    assert right_state.target_y_cm == -40.0

    l2_service.state = _state(distance_cm=10.0, heading_deg=-90.0)
    left_state: L3State = service.step()

    assert left_state.status == "tracking"
    assert left_state.target_y_cm == 40.0
    assert left_state.planner_status == "replanned_dynamic"


def test_step_marks_goal_unreachable_when_unknown_obstacle_cannot_be_bypassed() -> None:
    """Если обе стороны обхода заблокированы, L3 сообщает unreachable."""
    l2_service = FakeL2Service(_state(distance_cm=10.0))
    service = _service(l2_service)
    service.set_target_point(TargetPoint(x_cm=100.0, y_cm=0.0))
    service.step()
    l2_service.state = _state(distance_cm=10.0, heading_deg=-90.0)
    service.step()
    l2_service.state = _state(distance_cm=10.0, heading_deg=90.0)

    state: L3State = service.step()

    assert state.status == "unreachable"
    assert state.mode == "idle"
    assert state.planner_status == "impossible"
    assert state.detected_obstacle_x_cm is not None
    assert state.detected_obstacle_kind == "dynamic"


def test_handle_unknown_obstacle_marks_unreachable_when_bypass_points_missing() -> None:
    """Если квадратный обход нельзя построить, L3 сообщает unreachable."""
    service = _service(FakeL2Service(_state(distance_cm=10.0)))
    service.set_target_point(TargetPoint(x_cm=50.0, y_cm=0.0))
    service._build_reactive_bypass_points = lambda **kwargs: None  # type: ignore[method-assign]

    state = service._handle_unknown_obstacle(  # type: ignore[attr-defined]
        current_state=_state(distance_cm=10.0),
        current_target=TargetPoint(x_cm=50.0, y_cm=0.0),
        mode="point",
    )

    assert state is not None
    assert state.status == "unreachable"


def test_handle_unknown_obstacle_returns_none_without_distance() -> None:
    """Без измеренной дальности L3 не может оценить новое препятствие."""
    service = _service(FakeL2Service(_state(distance_cm=None)))

    replanned_state = service._handle_unknown_obstacle(  # type: ignore[attr-defined]
        current_state=_state(distance_cm=None),
        current_target=TargetPoint(x_cm=50.0, y_cm=0.0),
        mode="point",
    )

    assert replanned_state is None


def test_build_reactive_bypass_points_returns_none_without_origin_or_for_zero_segment() -> None:
    """Защитные ветки квадратного обхода не строят некорректный маршрут."""
    service = _service(FakeL2Service(_state()))

    assert (
        service._build_reactive_bypass_points(  # type: ignore[attr-defined]
            current_state=_state(distance_cm=10.0),
            direction=service.BYPASS_DIRECTION_RIGHT,
        )
        is None
    )

    point = TargetPoint(x_cm=1.0, y_cm=2.0)
    service._bypass_origin = point  # type: ignore[attr-defined]
    service._bypass_goal = point  # type: ignore[attr-defined]

    assert (
        service._build_reactive_bypass_points(  # type: ignore[attr-defined]
            current_state=_state(distance_cm=10.0),
            direction=service.BYPASS_DIRECTION_RIGHT,
        )
        is None
    )


def test_is_new_obstacle_allows_distinct_dynamic_obstacles() -> None:
    """Динамическая карта не должна склеивать разные препятствия в одно."""
    service = _service(FakeL2Service(_state()))
    service._dynamic_obstacles = (  # type: ignore[attr-defined]
        Obstacle(x_cm=10.0, y_cm=0.0, radius_cm=8.0),
    )

    assert (  # type: ignore[attr-defined]
        service._is_new_obstacle(Obstacle(x_cm=40.0, y_cm=0.0, radius_cm=8.0)) is True
    )


def test_is_new_obstacle_rejects_overlapping_dynamic_obstacle() -> None:
    """Повторное близкое обнаружение не добавляется как новое препятствие."""
    service = _service(FakeL2Service(_state()))
    service._dynamic_obstacles = (  # type: ignore[attr-defined]
        Obstacle(x_cm=10.0, y_cm=0.0, radius_cm=8.0),
    )

    assert (  # type: ignore[attr-defined]
        service._is_new_obstacle(Obstacle(x_cm=12.0, y_cm=0.0, radius_cm=8.0)) is False
    )


def test_accept_planned_route_marks_unreachable_for_impossible_plan() -> None:
    """Невозможный план переводит L3 в unreachable."""
    service = _service(FakeL2Service(_state()))

    state = service._accept_planned_route(  # type: ignore[attr-defined]
        planned_route=PlannedRoute(points=(), status=L3_PLANNER_STATUS_IMPOSSIBLE),
        mode="point",
    )

    assert state.status == "unreachable"
    assert state.mode == "idle"
    assert state.planner_status == "impossible"
