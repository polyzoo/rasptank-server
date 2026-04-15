from __future__ import annotations

from src.application.services.goal_point_controller import GoalPointController
from src.application.services.l2_models import BodyVelocityCommand, L2State
from src.application.services.l3_models import KnownObstacle, L3State, TargetPoint, TargetRoute
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
        """Зафиксировать остановку нового контура."""
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


def test_mark_active_goal_reached_returns_without_current_target() -> None:
    """Защитная ветка: без активной точки достигнутую цель удалять не нужно."""
    service = _service(FakeL2Service(_state()))
    service._remaining_goal_points = (TargetPoint(x_cm=50.0, y_cm=0.0),)  # type: ignore[attr-defined]

    service._mark_active_goal_reached()  # type: ignore[attr-defined]

    assert service._remaining_goal_points == (TargetPoint(x_cm=50.0, y_cm=0.0),)  # type: ignore[attr-defined]


def test_set_target_point_uses_planner_to_insert_detour() -> None:
    """При известном препятствии L3 сохраняет уже спланированный обходной маршрут."""
    l2_service = FakeL2Service(_state())
    service = _service(l2_service)

    state = service.set_target_point(
        TargetPoint(x_cm=50.0, y_cm=0.0),
        obstacles=(KnownObstacle(x_cm=25.0, y_cm=0.0, radius_cm=6.0),),
    )

    assert state.status == "tracking"
    assert state.mode == "point"
    assert state.planner_status == "planned"
    assert state.total_points == 3
    assert state.target_y_cm != 0.0


def test_step_marks_target_blocked_when_same_unknown_obstacle_is_seen_again() -> None:
    """Повторно увиденное препятствие не вызывает новый replan и оставляет состояние blocked."""
    l2_service = FakeL2Service(_state(distance_cm=10.0))
    service = _service(l2_service)
    service.set_target_point(TargetPoint(x_cm=50.0, y_cm=0.0))
    first_step: L3State = service.step()
    assert first_step.status == "tracking"

    state: L3State = service.step()

    assert state.status == "blocked"
    assert state.mode == "point"
    assert state.target_x_cm == first_step.target_x_cm
    assert state.target_y_cm == first_step.target_y_cm
    assert state.detected_obstacle_x_cm == first_step.detected_obstacle_x_cm
    assert state.detected_obstacle_kind == "dynamic"
    assert l2_service.stop_calls == 2


def test_set_target_point_marks_goal_unreachable_when_planner_cannot_bypass() -> None:
    """Если обход невозможен в допустимом коридоре, L3 сообщает unreachable."""
    l2_service = FakeL2Service(_state())
    service = _service(l2_service, max_detour_offset_cm=5.0)

    state = service.set_target_point(
        TargetPoint(x_cm=50.0, y_cm=0.0),
        obstacles=(KnownObstacle(x_cm=25.0, y_cm=0.0, radius_cm=6.0),),
    )

    assert state.status == "unreachable"
    assert state.mode == "idle"
    assert state.planner_status == "impossible"


def test_step_marks_goal_unreachable_when_unknown_obstacle_cannot_be_bypassed() -> None:
    """Если внезапное препятствие не удаётся обойти, L3 сообщает unreachable."""
    l2_service = FakeL2Service(_state(distance_cm=10.0))
    service = _service(l2_service, max_detour_offset_cm=5.0)
    service.set_target_point(TargetPoint(x_cm=50.0, y_cm=0.0))

    state: L3State = service.step()

    assert state.status == "unreachable"
    assert state.mode == "idle"
    assert state.planner_status == "impossible"
    assert state.detected_obstacle_x_cm is not None
    assert state.detected_obstacle_kind == "dynamic"


def test_handle_unknown_obstacle_returns_none_without_distance() -> None:
    """Без измеренной дальности L3 не может оценить новое препятствие."""
    service = _service(FakeL2Service(_state(distance_cm=None)))

    replanned_state = service._handle_unknown_obstacle(  # type: ignore[attr-defined]
        current_state=_state(distance_cm=None),
        mode="point",
    )

    assert replanned_state is None
