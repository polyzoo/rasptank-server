from __future__ import annotations

import asyncio
from typing import Annotated

from fastapi import APIRouter, Depends, WebSocket, WebSocketDisconnect

from src.application.services.isolated_motion_service import IsolatedMotionService
from src.application.services.l3_models import KnownObstacle, L3State, TargetPoint, TargetRoute
from src.presentation.api.dependencies import get_isolated_motion_service
from src.presentation.api.v1.schemas.l3 import (
    KnownObstacleSchema,
    L3GoalRequestSchema,
    L3RouteRequestSchema,
    L3StateResponseSchema,
    TargetPointSchema,
)

router: APIRouter = APIRouter()


def _to_target_point(point: TargetPointSchema) -> TargetPoint:
    """Преобразовать API-схему точки в доменную модель."""
    return TargetPoint(x_cm=point.x_cm, y_cm=point.y_cm)


def _to_obstacles(obstacles: list[KnownObstacleSchema]) -> tuple[KnownObstacle, ...]:
    """Преобразовать список известных препятствий в доменные модели."""
    return tuple(
        KnownObstacle(x_cm=obstacle.x_cm, y_cm=obstacle.y_cm, radius_cm=obstacle.radius_cm)
        for obstacle in obstacles
    )


def _to_response(state: L3State) -> L3StateResponseSchema:
    """Преобразовать доменное состояние L3 в ответ API."""
    return L3StateResponseSchema(
        status=state.status,
        mode=state.mode,
        planner_status=state.planner_status,
        target_x_cm=state.target_x_cm,
        target_y_cm=state.target_y_cm,
        active_point_index=state.active_point_index,
        total_points=state.total_points,
        distance_error_cm=state.distance_error_cm,
        heading_error_deg=state.heading_error_deg,
        target_heading_deg=state.target_heading_deg,
        linear_speed_cm_per_sec=state.linear_speed_cm_per_sec,
        angular_speed_deg_per_sec=state.angular_speed_deg_per_sec,
        detected_obstacle_x_cm=state.detected_obstacle_x_cm,
        detected_obstacle_y_cm=state.detected_obstacle_y_cm,
        detected_obstacle_radius_cm=state.detected_obstacle_radius_cm,
        detected_obstacle_kind=state.detected_obstacle_kind,
    )


@router.get("/state", description="Текущее состояние уровня L3")
async def l3_state(
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L3StateResponseSchema:
    """Вернуть текущее состояние нового уровня L3."""
    return _to_response(isolated_motion.get_l3_state())


@router.post("/goal", description="Запустить движение к одной точке")
async def l3_goal(
    body: L3GoalRequestSchema,
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L3StateResponseSchema:
    """Передать в новый L3 цель в виде одной точки."""
    return _to_response(
        isolated_motion.set_l3_goal(
            target=_to_target_point(body.target),
            obstacles=_to_obstacles(body.obstacles),
        )
    )


@router.post("/route", description="Запустить движение по маршруту из нескольких точек")
async def l3_route(
    body: L3RouteRequestSchema,
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L3StateResponseSchema:
    """Передать в новый L3 маршрут из нескольких точек."""
    return _to_response(
        isolated_motion.set_l3_route(
            route=TargetRoute(points=tuple(_to_target_point(point) for point in body.points)),
            obstacles=_to_obstacles(body.obstacles),
        )
    )


@router.post("/step", description="Выполнить один шаг уровня L3 вручную")
async def l3_step(
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L3StateResponseSchema:
    """Выполнить один шаг нового уровня L3 и вернуть его состояние."""
    return _to_response(isolated_motion.step_l3())


@router.post("/cancel", description="Отменить текущую цель или маршрут")
async def l3_cancel(
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L3StateResponseSchema:
    """Отменить цель или маршрут нового уровня L3."""
    return _to_response(isolated_motion.cancel_l3())


@router.websocket("/ws")
async def l3_state_ws(websocket: WebSocket) -> None:
    """Поток состояния уровня L3."""
    await websocket.accept()

    isolated_motion: IsolatedMotionService = websocket.scope["app"].state.isolated_motion

    try:
        while True:
            await websocket.send_json(_to_response(isolated_motion.get_l3_state()).model_dump())
            await asyncio.sleep(isolated_motion.update_interval_sec)
    except WebSocketDisconnect:
        return
