from __future__ import annotations

import math

from src.application.services.goal_point_controller import GoalPointController
from src.application.services.l2_models import BodyVelocityCommand, L2State
from src.application.services.l2_service import L2Service
from src.application.services.l3_models import (
    L3_DETECTED_OBSTACLE_KIND_DYNAMIC,
    L3_MODE_IDLE,
    L3_MODE_POINT,
    L3_MODE_ROUTE,
    L3_PLANNER_STATUS_IDLE,
    L3_PLANNER_STATUS_PLANNED,
    L3_PLANNER_STATUS_REPLANNED_DYNAMIC,
    L3_STATUS_BLOCKED,
    L3_STATUS_CANCELLED,
    L3_STATUS_IDLE,
    L3_STATUS_REACHED,
    L3_STATUS_TRACKING,
    L3_STATUS_UNREACHABLE,
    GoalTrackingCommand,
    KnownObstacle,
    L3State,
    PlannedRoute,
    TargetPoint,
    TargetRoute,
)
from src.application.services.path_planner import PathPlanner


class L3Service:
    """Изолированный верхний уровень нового контура движения."""

    POINT_MATCH_TOLERANCE_CM: float = 1e-6

    def __init__(
        self,
        goal_point_controller: GoalPointController,
        path_planner: PathPlanner,
        l2_service: L2Service,
        *,
        unknown_obstacle_radius_cm: float,
    ) -> None:
        """Сохранить составные части уровня L3."""
        self._goal_point_controller: GoalPointController = goal_point_controller
        self._path_planner: PathPlanner = path_planner
        self._l2_service: L2Service = l2_service
        self._unknown_obstacle_radius_cm: float = unknown_obstacle_radius_cm
        self._route_points: tuple[TargetPoint, ...] = ()
        self._remaining_goal_points: tuple[TargetPoint, ...] = ()
        self._active_point_index: int | None = None
        self._mode: str = L3_MODE_IDLE
        self._known_obstacles: tuple[KnownObstacle, ...] = ()
        self._dynamic_obstacles: tuple[KnownObstacle, ...] = ()
        self._last_detected_obstacle: KnownObstacle | None = None
        self._planner_status: str = L3_PLANNER_STATUS_IDLE
        self._last_state: L3State = L3State(
            status=L3_STATUS_IDLE,
            mode=L3_MODE_IDLE,
            planner_status=L3_PLANNER_STATUS_IDLE,
            target_x_cm=None,
            target_y_cm=None,
            active_point_index=None,
            total_points=0,
            distance_error_cm=None,
            heading_error_deg=None,
            target_heading_deg=None,
            linear_speed_cm_per_sec=0.0,
            angular_speed_deg_per_sec=0.0,
        )

    @property
    def unknown_obstacle_radius_cm(self) -> float:
        """Вернуть радиус, который L3 использует для неизвестного препятствия."""
        return self._unknown_obstacle_radius_cm

    def set_target_point(
        self,
        target: TargetPoint,
        obstacles: tuple[KnownObstacle, ...] = (),
    ) -> L3State:
        """Сохранить новую цель движения."""
        known_obstacles: tuple[KnownObstacle, ...] = obstacles
        empty_dynamic_obstacles: tuple[KnownObstacle, ...] = ()
        no_detected_obstacle: KnownObstacle | None = None
        remaining_goal_points: tuple[TargetPoint, ...] = (target,)
        point_mode: str = L3_MODE_POINT
        self._known_obstacles = known_obstacles
        self._dynamic_obstacles = empty_dynamic_obstacles
        self._last_detected_obstacle = no_detected_obstacle
        self._remaining_goal_points = remaining_goal_points
        self._mode = point_mode
        return self._plan_remaining_route(mode=point_mode)

    def set_route(
        self,
        route: TargetRoute,
        obstacles: tuple[KnownObstacle, ...] = (),
    ) -> L3State:
        """Сохранить новый маршрут из нескольких целевых точек."""
        if not route.points:
            return self.cancel()

        known_obstacles: tuple[KnownObstacle, ...] = obstacles
        empty_dynamic_obstacles: tuple[KnownObstacle, ...] = ()
        no_detected_obstacle: KnownObstacle | None = None
        remaining_goal_points: tuple[TargetPoint, ...] = route.points
        route_mode: str = L3_MODE_ROUTE
        self._known_obstacles = known_obstacles
        self._dynamic_obstacles = empty_dynamic_obstacles
        self._last_detected_obstacle = no_detected_obstacle
        self._remaining_goal_points = remaining_goal_points
        self._mode = route_mode
        return self._plan_remaining_route(mode=route_mode)

    def cancel(self) -> L3State:
        """Отменить текущую цель и остановить новый контур."""
        empty_route_points: tuple[TargetPoint, ...] = ()
        empty_goal_points: tuple[TargetPoint, ...] = ()
        empty_known_obstacles: tuple[KnownObstacle, ...] = ()
        empty_dynamic_obstacles: tuple[KnownObstacle, ...] = ()
        no_active_point_index: int | None = None
        no_detected_obstacle: KnownObstacle | None = None
        idle_mode: str = L3_MODE_IDLE
        idle_planner_status: str = L3_PLANNER_STATUS_IDLE

        self._route_points = empty_route_points
        self._remaining_goal_points = empty_goal_points
        self._active_point_index = no_active_point_index
        self._mode = idle_mode
        self._known_obstacles = empty_known_obstacles
        self._dynamic_obstacles = empty_dynamic_obstacles
        self._last_detected_obstacle = no_detected_obstacle
        self._planner_status = idle_planner_status

        stopped_state: L2State = self._l2_service.stop()
        cancelled_state: L3State = L3State(
            status=L3_STATUS_CANCELLED,
            mode=idle_mode,
            planner_status=idle_planner_status,
            target_x_cm=None,
            target_y_cm=None,
            active_point_index=no_active_point_index,
            total_points=0,
            distance_error_cm=None,
            heading_error_deg=None,
            target_heading_deg=None,
            linear_speed_cm_per_sec=stopped_state.linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=stopped_state.angular_speed_deg_per_sec,
        )

        self._last_state = cancelled_state
        return self._last_state

    def step(self) -> L3State:
        """Выполнить один шаг управления по текущей цели или активной точке маршрута."""
        current_target: TargetPoint | None = self._current_target_point()
        if current_target is None:
            idle_state: L2State = self._l2_service.get_state()
            idle_mode: str = L3_MODE_IDLE
            idle_planner_status: str = L3_PLANNER_STATUS_IDLE
            no_active_point_index: int | None = None
            new_idle_state: L3State = L3State(
                status=L3_STATUS_IDLE,
                mode=idle_mode,
                planner_status=idle_planner_status,
                target_x_cm=None,
                target_y_cm=None,
                active_point_index=no_active_point_index,
                total_points=0,
                distance_error_cm=None,
                heading_error_deg=None,
                target_heading_deg=None,
                linear_speed_cm_per_sec=idle_state.linear_speed_cm_per_sec,
                angular_speed_deg_per_sec=idle_state.angular_speed_deg_per_sec,
            )
            self._last_state = new_idle_state
            return self._last_state

        current_state: L2State = self._l2_service.get_state()
        tracking_command: GoalTrackingCommand = self._goal_point_controller.build_command(
            state=current_state,
            target=current_target,
        )
        tracking_status: str = tracking_command.status

        if tracking_status == L3_STATUS_BLOCKED:
            updated_state: L2State = self._l2_service.stop()
            current_mode: str = self._current_mode()
            replanned_state: L3State | None = self._handle_unknown_obstacle(
                current_state=current_state,
                mode=current_mode,
            )
            if replanned_state is not None:
                return replanned_state

            blocked_state: L3State = self._build_state(
                status=L3_STATUS_BLOCKED,
                linear_speed_cm_per_sec=updated_state.linear_speed_cm_per_sec,
                angular_speed_deg_per_sec=updated_state.angular_speed_deg_per_sec,
                distance_error_cm=tracking_command.distance_error_cm,
                heading_error_deg=tracking_command.heading_error_deg,
                target_heading_deg=tracking_command.target_heading_deg,
            )
            self._last_state = blocked_state
            return self._last_state

        if tracking_status == L3_STATUS_REACHED:
            updated_state: L2State = self._l2_service.stop()
            self._mark_active_goal_reached()
            if self._advance_route():
                continued_tracking_state: L3State = self._build_state(
                    status=L3_STATUS_TRACKING,
                    linear_speed_cm_per_sec=updated_state.linear_speed_cm_per_sec,
                    angular_speed_deg_per_sec=updated_state.angular_speed_deg_per_sec,
                    distance_error_cm=None,
                    heading_error_deg=None,
                    target_heading_deg=None,
                )
                self._last_state = continued_tracking_state
                return self._last_state

            empty_route_points: tuple[TargetPoint, ...] = ()
            no_active_point_index: int | None = None
            idle_mode: str = L3_MODE_IDLE
            idle_planner_status: str = L3_PLANNER_STATUS_IDLE
            no_detected_obstacle: KnownObstacle | None = None
            reached_state: L3State = L3State(
                status=L3_STATUS_REACHED,
                mode=idle_mode,
                planner_status=idle_planner_status,
                target_x_cm=None,
                target_y_cm=None,
                active_point_index=no_active_point_index,
                total_points=0,
                distance_error_cm=tracking_command.distance_error_cm,
                heading_error_deg=tracking_command.heading_error_deg,
                target_heading_deg=tracking_command.target_heading_deg,
                linear_speed_cm_per_sec=updated_state.linear_speed_cm_per_sec,
                angular_speed_deg_per_sec=updated_state.angular_speed_deg_per_sec,
                **self._detected_obstacle_fields(),
            )
            self._route_points = empty_route_points
            self._active_point_index = no_active_point_index
            self._mode = idle_mode
            self._planner_status = idle_planner_status
            self._last_detected_obstacle = no_detected_obstacle
            self._last_state = reached_state
            return self._last_state

        body_velocity_command: BodyVelocityCommand = BodyVelocityCommand(
            linear_speed_cm_per_sec=tracking_command.command.linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=tracking_command.command.angular_speed_deg_per_sec,
        )
        updated_state: L2State = self._l2_service.apply_body_velocity(body_velocity_command)
        new_tracking_state: L3State = self._build_state(
            status=L3_STATUS_TRACKING,
            linear_speed_cm_per_sec=updated_state.linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=updated_state.angular_speed_deg_per_sec,
            distance_error_cm=tracking_command.distance_error_cm,
            heading_error_deg=tracking_command.heading_error_deg,
            target_heading_deg=tracking_command.target_heading_deg,
        )
        self._last_state = new_tracking_state
        return self._last_state

    def get_state(self) -> L3State:
        """Вернуть текущее состояние верхнего уровня нового контура."""
        return self._last_state

    def _plan_remaining_route(self, *, mode: str) -> L3State:
        """Спланировать маршрут до оставшихся целевых точек."""
        start_point: TargetPoint = self._current_position_as_point()
        planned_route: PlannedRoute = self._path_planner.build_route(
            start=start_point,
            target=TargetRoute(points=self._remaining_goal_points),
            obstacles=self._all_obstacles(),
        )
        return self._accept_planned_route(planned_route=planned_route, mode=mode)

    def _build_tracking_state(self, *, mode: str) -> L3State:
        """Сформировать состояние после установки цели или маршрута."""
        current_state: L2State = self._l2_service.get_state()
        current_route_points: tuple[TargetPoint, ...] = self._route_points
        current_active_point_index: int | None = self._active_point_index
        target_x_cm: float | None = current_route_points[0].x_cm if current_route_points else None
        target_y_cm: float | None = current_route_points[0].y_cm if current_route_points else None
        total_points: int = len(current_route_points)
        tracking_state: L3State = L3State(
            status=L3_STATUS_TRACKING,
            mode=mode,
            planner_status=self._planner_status,
            target_x_cm=target_x_cm,
            target_y_cm=target_y_cm,
            active_point_index=current_active_point_index,
            total_points=total_points,
            distance_error_cm=None,
            heading_error_deg=None,
            target_heading_deg=None,
            linear_speed_cm_per_sec=current_state.linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=current_state.angular_speed_deg_per_sec,
            **self._detected_obstacle_fields(),
        )
        self._last_state = tracking_state
        return self._last_state

    def _build_state(
        self,
        *,
        status: str,
        linear_speed_cm_per_sec: float,
        angular_speed_deg_per_sec: float,
        distance_error_cm: float | None,
        heading_error_deg: float | None,
        target_heading_deg: float | None,
    ) -> L3State:
        """Собрать снимок состояния L3 по текущей активной цели."""
        current_target: TargetPoint | None = self._current_target_point()
        mode: str = self._current_mode()
        target_x_cm: float | None = current_target.x_cm if current_target is not None else None
        target_y_cm: float | None = current_target.y_cm if current_target is not None else None
        active_point_index: int | None = self._active_point_index
        total_points: int = len(self._route_points)
        state_snapshot: L3State = L3State(
            status=status,
            mode=mode,
            planner_status=self._planner_status,
            target_x_cm=target_x_cm,
            target_y_cm=target_y_cm,
            active_point_index=active_point_index,
            total_points=total_points,
            distance_error_cm=distance_error_cm,
            heading_error_deg=heading_error_deg,
            target_heading_deg=target_heading_deg,
            linear_speed_cm_per_sec=linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=angular_speed_deg_per_sec,
            **self._detected_obstacle_fields(),
        )
        self._last_state = state_snapshot
        return self._last_state

    def _current_target_point(self) -> TargetPoint | None:
        """Вернуть текущую активную точку маршрута."""
        if self._active_point_index is None:
            return None

        if self._active_point_index >= len(self._route_points):
            return None

        return self._route_points[self._active_point_index]

    def _advance_route(self) -> bool:
        """Переключиться на следующую точку маршрута, если она есть."""
        if self._active_point_index is None:
            return False

        next_index: int = self._active_point_index + 1
        if next_index >= len(self._route_points):
            return False

        self._active_point_index = next_index
        return True

    def _current_position_as_point(self) -> TargetPoint:
        """Преобразовать текущее состояние L2 в точку старта для планировщика."""
        current_state: L2State = self._l2_service.get_state()
        return TargetPoint(x_cm=current_state.x_cm, y_cm=current_state.y_cm)

    def _current_mode(self) -> str:
        """Вернуть режим текущей миссии верхнего уровня."""
        return self._mode

    def _all_obstacles(self) -> tuple[KnownObstacle, ...]:
        """Вернуть объединённый список известных и найденных по ходу препятствий."""
        all_obstacles: tuple[KnownObstacle, ...] = self._known_obstacles + self._dynamic_obstacles
        return all_obstacles

    def _mark_active_goal_reached(self) -> None:
        """Удалить из списка оставшихся целей уже достигнутую целевую точку."""
        current_target: TargetPoint | None = self._current_target_point()
        remaining_goal_points: tuple[TargetPoint, ...] = self._remaining_goal_points
        if current_target is None or not remaining_goal_points:
            return

        first_remaining_goal_point: TargetPoint = remaining_goal_points[0]
        if self._points_match(current_target, first_remaining_goal_point):
            updated_remaining_goal_points: tuple[TargetPoint, ...] = remaining_goal_points[1:]
            self._remaining_goal_points = updated_remaining_goal_points

    def _handle_unknown_obstacle(
        self,
        *,
        current_state: L2State,
        mode: str,
    ) -> L3State | None:
        """Добавить неизвестное препятствие и перестроить остаток маршрута."""
        inferred_obstacle: KnownObstacle | None = self._infer_unknown_obstacle(
            current_state=current_state
        )
        if inferred_obstacle is None:
            return None

        if not self._is_new_obstacle(inferred_obstacle):
            return None

        updated_dynamic_obstacles: tuple[KnownObstacle, ...] = self._dynamic_obstacles + (
            inferred_obstacle,
        )
        self._dynamic_obstacles = updated_dynamic_obstacles
        self._last_detected_obstacle = inferred_obstacle
        replanned_state: L3State = self._plan_remaining_route(mode=mode)
        replanned_status: str = replanned_state.status
        if replanned_status == L3_STATUS_TRACKING:
            self._planner_status = L3_PLANNER_STATUS_REPLANNED_DYNAMIC
            replanned_mode: str = self._current_mode()
            replanned_tracking_state: L3State = L3State(
                status=L3_STATUS_TRACKING,
                mode=replanned_mode,
                planner_status=L3_PLANNER_STATUS_REPLANNED_DYNAMIC,
                target_x_cm=replanned_state.target_x_cm,
                target_y_cm=replanned_state.target_y_cm,
                active_point_index=replanned_state.active_point_index,
                total_points=replanned_state.total_points,
                distance_error_cm=replanned_state.distance_error_cm,
                heading_error_deg=replanned_state.heading_error_deg,
                target_heading_deg=replanned_state.target_heading_deg,
                linear_speed_cm_per_sec=replanned_state.linear_speed_cm_per_sec,
                angular_speed_deg_per_sec=replanned_state.angular_speed_deg_per_sec,
                **self._detected_obstacle_fields(),
            )
            self._last_state = replanned_tracking_state
            return self._last_state

        return replanned_state

    def _infer_unknown_obstacle(self, *, current_state: L2State) -> KnownObstacle | None:
        """Оценить координаты неизвестного препятствия по текущей позе и дальности."""
        if current_state.distance_cm is None:
            return None

        heading_rad: float = math.radians(current_state.heading_deg)
        obstacle_center_distance_cm: float = (
            current_state.distance_cm + self._unknown_obstacle_radius_cm
        )

        return KnownObstacle(
            x_cm=current_state.x_cm + math.cos(heading_rad) * obstacle_center_distance_cm,
            y_cm=current_state.y_cm + math.sin(heading_rad) * obstacle_center_distance_cm,
            radius_cm=self._unknown_obstacle_radius_cm,
        )

    def _is_new_obstacle(self, obstacle: KnownObstacle) -> bool:
        """Проверить, что препятствие ещё не было добавлено в планировщике."""
        merge_distance_cm: float = max(
            obstacle.radius_cm,
            self._path_planner.obstacle_clearance_cm,
        )

        for known_obstacle in self._all_obstacles():
            if (
                math.hypot(
                    obstacle.x_cm - known_obstacle.x_cm,
                    obstacle.y_cm - known_obstacle.y_cm,
                )
                <= merge_distance_cm
            ):
                return False

        return True

    def _points_match(self, left: TargetPoint, right: TargetPoint) -> bool:
        """Проверить, совпадают ли две точки с малой вычислительной погрешностью."""
        return (
            math.hypot(left.x_cm - right.x_cm, left.y_cm - right.y_cm)
            <= self.POINT_MATCH_TOLERANCE_CM
        )

    def _detected_obstacle_fields(self) -> dict[str, float | str | None]:
        """Вернуть поля последнего обнаруженного неизвестного препятствия."""
        if self._last_detected_obstacle is None:
            return {
                "detected_obstacle_x_cm": None,
                "detected_obstacle_y_cm": None,
                "detected_obstacle_radius_cm": None,
                "detected_obstacle_kind": None,
            }

        return {
            "detected_obstacle_x_cm": self._last_detected_obstacle.x_cm,
            "detected_obstacle_y_cm": self._last_detected_obstacle.y_cm,
            "detected_obstacle_radius_cm": self._last_detected_obstacle.radius_cm,
            "detected_obstacle_kind": L3_DETECTED_OBSTACLE_KIND_DYNAMIC,
        }

    def _accept_planned_route(self, *, planned_route: PlannedRoute, mode: str) -> L3State:
        """Сохранить спланированный маршрут или перейти в невозможность достижения цели."""
        planner_status: str = planned_route.status
        planned_points: tuple[TargetPoint, ...] = planned_route.points

        if planner_status != L3_PLANNER_STATUS_PLANNED or not planned_points:
            empty_route_points: tuple[TargetPoint, ...] = ()
            no_active_point_index: int | None = None
            idle_mode: str = L3_MODE_IDLE
            self._route_points = empty_route_points
            self._active_point_index = no_active_point_index
            self._planner_status = planner_status
            stopped_state: L2State = self._l2_service.stop()
            unreachable_state: L3State = L3State(
                status=L3_STATUS_UNREACHABLE,
                mode=idle_mode,
                planner_status=planner_status,
                target_x_cm=None,
                target_y_cm=None,
                active_point_index=no_active_point_index,
                total_points=0,
                distance_error_cm=None,
                heading_error_deg=None,
                target_heading_deg=None,
                linear_speed_cm_per_sec=stopped_state.linear_speed_cm_per_sec,
                angular_speed_deg_per_sec=stopped_state.angular_speed_deg_per_sec,
                **self._detected_obstacle_fields(),
            )
            self._last_state = unreachable_state
            return self._last_state

        self._route_points = planned_points
        first_route_point_index: int = 0
        self._active_point_index = first_route_point_index
        self._planner_status = planner_status
        return self._build_tracking_state(mode=mode)
