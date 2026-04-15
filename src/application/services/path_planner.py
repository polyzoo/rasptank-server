from __future__ import annotations

import math
from dataclasses import dataclass
from typing import ClassVar

from src.application.services.l3_models import (
    L3_PLANNER_STATUS_EMPTY,
    L3_PLANNER_STATUS_IMPOSSIBLE,
    L3_PLANNER_STATUS_PLANNED,
    KnownObstacle,
    PlannedRoute,
    TargetPoint,
    TargetRoute,
)


@dataclass(frozen=True, slots=True)
class PathPlanner:
    """Построить маршрут через точки с обходом заранее известных препятствий."""

    # Минимальная длина отрезка, при которой допустимо считать, что точка совпала.
    EPSILON_CM: ClassVar[float] = 1e-6

    # Нижняя граница безразмерной проекции точки на отрезок: начало отрезка.
    SEGMENT_PROJECTION_MIN: ClassVar[float] = 0.0

    # Верхняя граница безразмерной проекции точки на отрезок: конец отрезка.
    SEGMENT_PROJECTION_MAX: ClassVar[float] = 1.0

    obstacle_clearance_cm: float
    max_detour_offset_cm: float
    max_waypoints: int

    def build_route(
        self,
        start: TargetPoint,
        target: TargetPoint | TargetRoute,
        obstacles: tuple[KnownObstacle, ...] = (),
    ) -> PlannedRoute:
        """Построить маршрут от текущей точки к цели или набору точек."""
        target_points: tuple[TargetPoint, ...] = (
            target.points if isinstance(target, TargetRoute) else (target,)
        )
        if not target_points:
            return PlannedRoute(points=(), status=L3_PLANNER_STATUS_EMPTY)

        planned_points: list[TargetPoint] = []
        current_start: TargetPoint = start

        for target_point in target_points:
            segment_plan: PlannedRoute = self._plan_segment(
                start=current_start,
                goal=target_point,
                obstacles=obstacles,
            )
            if segment_plan.status != L3_PLANNER_STATUS_PLANNED:
                return segment_plan

            planned_points.extend(segment_plan.points)
            current_start: TargetPoint = target_point

        return PlannedRoute(points=tuple(planned_points), status=L3_PLANNER_STATUS_PLANNED)

    def _plan_segment(
        self,
        *,
        start: TargetPoint,
        goal: TargetPoint,
        obstacles: tuple[KnownObstacle, ...],
    ) -> PlannedRoute:
        """Построить путь между двумя точками с локальным обходом препятствий."""
        planned_points: list[TargetPoint] = [goal]

        while True:
            if len(planned_points) > self.max_waypoints:
                return PlannedRoute(
                    points=tuple(planned_points),
                    status=L3_PLANNER_STATUS_IMPOSSIBLE,
                )

            inserted: bool = False
            previous_point: TargetPoint = start

            for segment_index, next_point in enumerate(planned_points):
                obstacle_index: int | None = self._find_intersecting_obstacle_index(
                    start=previous_point,
                    goal=next_point,
                    obstacles=obstacles,
                )
                if obstacle_index is None:
                    previous_point = next_point
                    continue

                detour_points: tuple[TargetPoint, ...] | None = self._build_detour_points(
                    start=previous_point,
                    goal=next_point,
                    obstacle=obstacles[obstacle_index],
                    obstacles=obstacles,
                )
                if detour_points is None:
                    return PlannedRoute(
                        points=tuple(planned_points),
                        status=L3_PLANNER_STATUS_IMPOSSIBLE,
                        blocking_obstacle_index=obstacle_index,
                    )

                for detour_point in reversed(detour_points):
                    planned_points.insert(segment_index, detour_point)

                inserted: bool = True
                break

            if not inserted:
                return PlannedRoute(points=tuple(planned_points), status=L3_PLANNER_STATUS_PLANNED)

    def _find_intersecting_obstacle_index(
        self,
        *,
        start: TargetPoint,
        goal: TargetPoint,
        obstacles: tuple[KnownObstacle, ...],
    ) -> int | None:
        """Найти первое препятствие, пересекающее текущий отрезок."""
        closest_index: int | None = None
        closest_projection: float | None = None
        index: int
        obstacle: KnownObstacle

        for index, obstacle in enumerate(obstacles):
            if not self._segment_intersects_obstacle(start=start, goal=goal, obstacle=obstacle):
                continue

            projection: float = self._segment_projection_to_obstacle(
                start=start,
                goal=goal,
                obstacle=obstacle,
            )
            if closest_projection is None or projection < closest_projection:
                closest_projection: float = projection
                closest_index: int = index

        return closest_index

    def _build_detour_points(
        self,
        *,
        start: TargetPoint,
        goal: TargetPoint,
        obstacle: KnownObstacle,
        obstacles: tuple[KnownObstacle, ...],
    ) -> tuple[TargetPoint, ...] | None:
        """Построить две обходные точки с одной из сторон препятствия."""
        direction_x_cm: float = goal.x_cm - start.x_cm
        direction_y_cm: float = goal.y_cm - start.y_cm

        segment_length_cm: float = math.hypot(direction_x_cm, direction_y_cm)
        if segment_length_cm <= self.EPSILON_CM:
            return None

        unit_x: float = direction_x_cm / segment_length_cm
        unit_y: float = direction_y_cm / segment_length_cm
        normal_x: float = -unit_y
        normal_y: float = unit_x
        detour_radius_cm: float = obstacle.radius_cm + self.obstacle_clearance_cm
        candidates: tuple[tuple[TargetPoint, TargetPoint], tuple[TargetPoint, TargetPoint]] = (
            (
                TargetPoint(
                    x_cm=obstacle.x_cm - unit_x * detour_radius_cm + normal_x * detour_radius_cm,
                    y_cm=obstacle.y_cm - unit_y * detour_radius_cm + normal_y * detour_radius_cm,
                ),
                TargetPoint(
                    x_cm=obstacle.x_cm + unit_x * detour_radius_cm + normal_x * detour_radius_cm,
                    y_cm=obstacle.y_cm + unit_y * detour_radius_cm + normal_y * detour_radius_cm,
                ),
            ),
            (
                TargetPoint(
                    x_cm=obstacle.x_cm - unit_x * detour_radius_cm - normal_x * detour_radius_cm,
                    y_cm=obstacle.y_cm - unit_y * detour_radius_cm - normal_y * detour_radius_cm,
                ),
                TargetPoint(
                    x_cm=obstacle.x_cm + unit_x * detour_radius_cm - normal_x * detour_radius_cm,
                    y_cm=obstacle.y_cm + unit_y * detour_radius_cm - normal_y * detour_radius_cm,
                ),
            ),
        )

        best_candidate: tuple[TargetPoint, TargetPoint] | None = None
        best_length_cm: float | None = None
        for first_candidate, second_candidate in candidates:
            if (
                self._perpendicular_offset_from_segment(
                    start=start,
                    goal=goal,
                    point=first_candidate,
                )
                > self.max_detour_offset_cm
            ):
                continue

            if (
                self._perpendicular_offset_from_segment(
                    start=start,
                    goal=goal,
                    point=second_candidate,
                )
                > self.max_detour_offset_cm
            ):
                continue

            if self._candidate_hits_any_obstacle(
                start=start,
                first_waypoint=first_candidate,
                second_waypoint=second_candidate,
                goal=goal,
                obstacles=obstacles,
            ):
                continue

            candidate_length_cm: float = (
                math.hypot(first_candidate.x_cm - start.x_cm, first_candidate.y_cm - start.y_cm)
                + math.hypot(
                    second_candidate.x_cm - first_candidate.x_cm,
                    second_candidate.y_cm - first_candidate.y_cm,
                )
                + math.hypot(goal.x_cm - second_candidate.x_cm, goal.y_cm - second_candidate.y_cm)
            )
            if best_length_cm is None or candidate_length_cm < best_length_cm:
                best_length_cm: float = candidate_length_cm
                best_candidate: tuple[TargetPoint, TargetPoint] = (
                    first_candidate,
                    second_candidate,
                )

        return best_candidate

    def _candidate_hits_any_obstacle(
        self,
        *,
        start: TargetPoint,
        first_waypoint: TargetPoint,
        second_waypoint: TargetPoint,
        goal: TargetPoint,
        obstacles: tuple[KnownObstacle, ...],
    ) -> bool:
        """Проверить, пересекают ли новые отрезки какие-либо препятствия."""
        obstacle: KnownObstacle
        for obstacle in obstacles:
            if self._segment_intersects_obstacle(
                start=start,
                goal=first_waypoint,
                obstacle=obstacle,
            ):
                return True

            if self._segment_intersects_obstacle(
                start=first_waypoint,
                goal=second_waypoint,
                obstacle=obstacle,
            ):
                return True

            if self._segment_intersects_obstacle(
                start=second_waypoint,
                goal=goal,
                obstacle=obstacle,
            ):
                return True

        return False

    def _segment_intersects_obstacle(
        self,
        *,
        start: TargetPoint,
        goal: TargetPoint,
        obstacle: KnownObstacle,
    ) -> bool:
        """Проверить пересечение отрезка и расширенного препятствия."""
        return (
            self._distance_from_point_to_segment(
                point_x_cm=obstacle.x_cm,
                point_y_cm=obstacle.y_cm,
                start=start,
                goal=goal,
            )
            < obstacle.radius_cm + self.obstacle_clearance_cm - self.EPSILON_CM
        )

    def _segment_projection_to_obstacle(
        self,
        *,
        start: TargetPoint,
        goal: TargetPoint,
        obstacle: KnownObstacle,
    ) -> float:
        """Вернуть положение препятствия вдоль отрезка от старта до цели."""
        direction_x_cm: float = goal.x_cm - start.x_cm
        direction_y_cm: float = goal.y_cm - start.y_cm

        segment_length_squared: float = direction_x_cm**2 + direction_y_cm**2
        if segment_length_squared <= self.EPSILON_CM:
            return self.SEGMENT_PROJECTION_MIN

        return max(
            self.SEGMENT_PROJECTION_MIN,
            min(
                self.SEGMENT_PROJECTION_MAX,
                (
                    (obstacle.x_cm - start.x_cm) * direction_x_cm
                    + (obstacle.y_cm - start.y_cm) * direction_y_cm
                )
                / segment_length_squared,
            ),
        )

    def _distance_from_point_to_segment(
        self,
        *,
        point_x_cm: float,
        point_y_cm: float,
        start: TargetPoint,
        goal: TargetPoint,
    ) -> float:
        """Вернуть минимальное расстояние от точки до отрезка."""
        direction_x_cm: float = goal.x_cm - start.x_cm
        direction_y_cm: float = goal.y_cm - start.y_cm

        segment_length_squared: float = direction_x_cm**2 + direction_y_cm**2
        if segment_length_squared <= self.EPSILON_CM:
            return math.hypot(point_x_cm - start.x_cm, point_y_cm - start.y_cm)

        projection: float = max(
            self.SEGMENT_PROJECTION_MIN,
            min(
                self.SEGMENT_PROJECTION_MAX,
                (
                    (point_x_cm - start.x_cm) * direction_x_cm
                    + (point_y_cm - start.y_cm) * direction_y_cm
                )
                / segment_length_squared,
            ),
        )

        closest_x_cm: float = start.x_cm + projection * direction_x_cm
        closest_y_cm: float = start.y_cm + projection * direction_y_cm

        return math.hypot(point_x_cm - closest_x_cm, point_y_cm - closest_y_cm)

    def _perpendicular_offset_from_segment(
        self,
        *,
        start: TargetPoint,
        goal: TargetPoint,
        point: TargetPoint,
    ) -> float:
        """Вернуть боковое смещение точки от прямой между стартом и целью."""
        return self._distance_from_point_to_segment(
            point_x_cm=point.x_cm,
            point_y_cm=point.y_cm,
            start=start,
            goal=goal,
        )
