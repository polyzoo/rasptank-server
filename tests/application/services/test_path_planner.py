from __future__ import annotations

from src.application.services.l3_models import (
    L3_PLANNER_STATUS_EMPTY,
    L3_PLANNER_STATUS_IMPOSSIBLE,
    L3_PLANNER_STATUS_PLANNED,
    KnownObstacle,
    TargetPoint,
    TargetRoute,
)
from src.application.services.path_planner import PathPlanner


def _planner(*, max_detour_offset_cm: float = 40.0) -> PathPlanner:
    """Создать планировщик обхода для тестов."""
    return PathPlanner(
        obstacle_clearance_cm=5.0,
        max_detour_offset_cm=max_detour_offset_cm,
        max_waypoints=24,
    )


def test_build_route_returns_straight_path_without_obstacles() -> None:
    """Без препятствий планировщик возвращает исходную цель без лишних точек."""
    planner = _planner()

    result = planner.build_route(
        start=TargetPoint(x_cm=0.0, y_cm=0.0),
        target=TargetPoint(x_cm=50.0, y_cm=0.0),
    )

    assert result.status == L3_PLANNER_STATUS_PLANNED
    assert result.points == (TargetPoint(x_cm=50.0, y_cm=0.0),)


def test_build_route_inserts_detour_around_known_obstacle() -> None:
    """Если прямая пересекает препятствие, планировщик добавляет обходную точку."""
    planner = _planner()

    result = planner.build_route(
        start=TargetPoint(x_cm=0.0, y_cm=0.0),
        target=TargetPoint(x_cm=50.0, y_cm=0.0),
        obstacles=(KnownObstacle(x_cm=25.0, y_cm=0.0, radius_cm=6.0),),
    )

    assert result.status == L3_PLANNER_STATUS_PLANNED
    assert len(result.points) == 3
    assert result.points[-1] == TargetPoint(x_cm=50.0, y_cm=0.0)
    assert result.points[0].y_cm != 0.0


def test_build_route_marks_target_impossible_when_detour_exceeds_corridor() -> None:
    """Если обход требует слишком большого смещения, цель объявляется недостижимой."""
    planner = _planner(max_detour_offset_cm=5.0)

    result = planner.build_route(
        start=TargetPoint(x_cm=0.0, y_cm=0.0),
        target=TargetPoint(x_cm=50.0, y_cm=0.0),
        obstacles=(KnownObstacle(x_cm=25.0, y_cm=0.0, radius_cm=6.0),),
    )

    assert result.status == L3_PLANNER_STATUS_IMPOSSIBLE


def test_build_route_plans_full_route_point_by_point() -> None:
    """Маршрут из нескольких точек сохраняет цели и вставляет обход только там, где нужно."""
    planner = _planner()

    result = planner.build_route(
        start=TargetPoint(x_cm=0.0, y_cm=0.0),
        target=TargetRoute(
            points=(
                TargetPoint(x_cm=50.0, y_cm=0.0),
                TargetPoint(x_cm=80.0, y_cm=0.0),
            )
        ),
        obstacles=(KnownObstacle(x_cm=25.0, y_cm=0.0, radius_cm=6.0),),
    )

    assert result.status == L3_PLANNER_STATUS_PLANNED
    assert result.points[-2:] == (
        TargetPoint(x_cm=50.0, y_cm=0.0),
        TargetPoint(x_cm=80.0, y_cm=0.0),
    )


def test_build_route_returns_empty_status_for_empty_route() -> None:
    """Пустой маршрут возвращает специальный статус empty."""
    planner = _planner()

    result = planner.build_route(
        start=TargetPoint(x_cm=0.0, y_cm=0.0),
        target=TargetRoute(points=()),
    )

    assert result.status == L3_PLANNER_STATUS_EMPTY
    assert result.points == ()


def test_plan_segment_returns_impossible_when_waypoint_limit_exceeded() -> None:
    """Если лимит обходных точек слишком мал, сегмент объявляется невозможным."""
    planner = PathPlanner(
        obstacle_clearance_cm=5.0,
        max_detour_offset_cm=40.0,
        max_waypoints=1,
    )

    result = planner._plan_segment(  # type: ignore[attr-defined]
        start=TargetPoint(x_cm=0.0, y_cm=0.0),
        goal=TargetPoint(x_cm=50.0, y_cm=0.0),
        obstacles=(KnownObstacle(x_cm=25.0, y_cm=0.0, radius_cm=6.0),),
    )

    assert result.status == L3_PLANNER_STATUS_IMPOSSIBLE


def test_build_detour_points_returns_none_for_zero_length_segment() -> None:
    """Для нулевого сегмента планировщик не строит обходные точки."""
    planner = _planner()

    result = planner._build_detour_points(  # type: ignore[attr-defined]
        start=TargetPoint(x_cm=10.0, y_cm=10.0),
        goal=TargetPoint(x_cm=10.0, y_cm=10.0),
        obstacle=KnownObstacle(x_cm=12.0, y_cm=10.0, radius_cm=2.0),
        obstacles=(KnownObstacle(x_cm=12.0, y_cm=10.0, radius_cm=2.0),),
    )

    assert result is None


def test_candidate_hits_any_obstacle_detects_all_three_segments() -> None:
    """Проверка обходного пути должна отлавливать пересечения на любом из трёх отрезков."""
    planner = _planner()
    start = TargetPoint(x_cm=0.0, y_cm=0.0)
    first = TargetPoint(x_cm=10.0, y_cm=0.0)
    second = TargetPoint(x_cm=20.0, y_cm=0.0)
    goal = TargetPoint(x_cm=30.0, y_cm=0.0)

    assert planner._candidate_hits_any_obstacle(  # type: ignore[attr-defined]
        start=start,
        first_waypoint=first,
        second_waypoint=second,
        goal=goal,
        obstacles=(KnownObstacle(x_cm=5.0, y_cm=0.0, radius_cm=2.0),),
    )
    assert planner._candidate_hits_any_obstacle(  # type: ignore[attr-defined]
        start=start,
        first_waypoint=first,
        second_waypoint=second,
        goal=goal,
        obstacles=(KnownObstacle(x_cm=15.0, y_cm=0.0, radius_cm=2.0),),
    )
    assert planner._candidate_hits_any_obstacle(  # type: ignore[attr-defined]
        start=start,
        first_waypoint=first,
        second_waypoint=second,
        goal=goal,
        obstacles=(KnownObstacle(x_cm=25.0, y_cm=0.0, radius_cm=2.0),),
    )


def test_projection_and_distance_helpers_cover_degenerate_segment() -> None:
    """Вспомогательные функции корректно работают на нулевом сегменте."""
    planner = _planner()
    start = TargetPoint(x_cm=10.0, y_cm=10.0)
    goal = TargetPoint(x_cm=10.0, y_cm=10.0)
    obstacle = KnownObstacle(x_cm=12.0, y_cm=10.0, radius_cm=2.0)

    assert (
        planner._segment_projection_to_obstacle(  # type: ignore[attr-defined]
            start=start,
            goal=goal,
            obstacle=obstacle,
        )
        == 0.0
    )
    assert (
        planner._distance_from_point_to_segment(  # type: ignore[attr-defined]
            point_x_cm=12.0,
            point_y_cm=10.0,
            start=start,
            goal=goal,
        )
        == 2.0
    )


def test_build_detour_points_skips_candidate_when_second_point_exceeds_corridor() -> None:
    """Если вторая обходная точка выходит за пределы коридора, кандидат отбрасывается."""

    class SecondPointOverflowPlanner(PathPlanner):
        """Заглушка, в которой только проверка второй точки превышает коридор."""

        def __init__(self) -> None:
            super().__init__(
                obstacle_clearance_cm=5.0,
                max_detour_offset_cm=10.5,
                max_waypoints=24,
            )
            object.__setattr__(self, "_calls", 0)

        def _perpendicular_offset_from_segment(self, *, start, goal, point):  # type: ignore[override]
            object.__setattr__(self, "_calls", self._calls + 1)
            return 0.0 if self._calls == 1 else 99.0

    planner = SecondPointOverflowPlanner()

    result = planner._build_detour_points(  # type: ignore[attr-defined]
        start=TargetPoint(x_cm=0.0, y_cm=0.0),
        goal=TargetPoint(x_cm=50.0, y_cm=0.0),
        obstacle=KnownObstacle(x_cm=25.0, y_cm=0.0, radius_cm=6.0),
        obstacles=(KnownObstacle(x_cm=25.0, y_cm=0.0, radius_cm=6.0),),
    )

    assert result is None


def test_build_detour_points_skips_blocked_side_and_selects_other_side() -> None:
    """Если один кандидат обхода пересекает другое препятствие, выбирается другая сторона."""
    planner = _planner()

    result = planner._build_detour_points(  # type: ignore[attr-defined]
        start=TargetPoint(x_cm=0.0, y_cm=0.0),
        goal=TargetPoint(x_cm=50.0, y_cm=0.0),
        obstacle=KnownObstacle(x_cm=25.0, y_cm=0.0, radius_cm=6.0),
        obstacles=(
            KnownObstacle(x_cm=25.0, y_cm=0.0, radius_cm=6.0),
            KnownObstacle(x_cm=25.0, y_cm=11.0, radius_cm=2.0),
        ),
    )

    assert result is not None
    assert result[0].y_cm < 0.0
    assert result[1].y_cm < 0.0


def test_candidate_hits_any_obstacle_detects_intersection_only_on_last_segment() -> None:
    """Пересечение на третьем отрезке тоже блокирует обходной путь."""
    planner = _planner()

    assert planner._candidate_hits_any_obstacle(  # type: ignore[attr-defined]
        start=TargetPoint(x_cm=0.0, y_cm=0.0),
        first_waypoint=TargetPoint(x_cm=10.0, y_cm=20.0),
        second_waypoint=TargetPoint(x_cm=20.0, y_cm=20.0),
        goal=TargetPoint(x_cm=30.0, y_cm=0.0),
        obstacles=(KnownObstacle(x_cm=25.0, y_cm=5.0, radius_cm=2.0),),
    )
