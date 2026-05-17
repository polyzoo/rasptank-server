from __future__ import annotations

import asyncio
from typing import Annotated

from fastapi import APIRouter, Depends, WebSocket, WebSocketDisconnect

from src.application.services.isolated_motion_service import IsolatedMotionService
from src.application.services.l3_models import L3State, TargetPoint, TargetRoute
from src.presentation.api.dependencies import get_isolated_motion_service
from src.presentation.api.v1.schemas.l3 import (
    L3GoalRequestSchema,
    L3RouteRequestSchema,
    L3StateResponseSchema,
    TargetPointSchema,
)

router: APIRouter = APIRouter()


def _to_target_point(point: TargetPointSchema) -> TargetPoint:
    """Преобразовать API-точку в модель L3."""
    return TargetPoint(x_cm=point.x_cm, y_cm=point.y_cm)


def _to_response(state: L3State) -> L3StateResponseSchema:
    """Преобразовать состояние L3 в API-ответ."""
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
        desired_linear_speed_cm_per_sec=state.desired_linear_speed_cm_per_sec,
        desired_angular_speed_deg_per_sec=state.desired_angular_speed_deg_per_sec,
        detected_obstacle_x_cm=state.detected_obstacle_x_cm,
        detected_obstacle_y_cm=state.detected_obstacle_y_cm,
        detected_obstacle_radius_cm=state.detected_obstacle_radius_cm,
        detected_obstacle_kind=state.detected_obstacle_kind,
    )


@router.get("/state", summary="Текущее состояние уровня L3")
async def l3_state(
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L3StateResponseSchema:
    """Вернуть состояние L3."""
    state: L3State = await asyncio.to_thread(isolated_motion.get_l3_state)
    return _to_response(state)


@router.post("/goal", summary="Запустить движение к одной точке")
async def l3_goal(
    body: L3GoalRequestSchema,
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L3StateResponseSchema:
    """Передать целевую точку в L3."""
    state: L3State = await asyncio.to_thread(
        lambda: isolated_motion.set_l3_goal(
            target=_to_target_point(body.target),
        )
    )
    return _to_response(state)


@router.post("/route", summary="Запустить движение по маршруту из нескольких точек")
async def l3_route(
    body: L3RouteRequestSchema,
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L3StateResponseSchema:
    """Передать маршрут в L3."""
    state: L3State = await asyncio.to_thread(
        lambda: isolated_motion.set_l3_route(
            route=TargetRoute(points=tuple(_to_target_point(point) for point in body.points)),
        )
    )
    return _to_response(state)


@router.post("/step", summary="Выполнить один шаг уровня L3 вручную")
async def l3_step(
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L3StateResponseSchema:
    """Выполнить один шаг L3."""
    state: L3State = await asyncio.to_thread(isolated_motion.step_l3)
    return _to_response(state)


@router.post("/cancel", summary="Отменить текущую цель или маршрут")
async def l3_cancel(
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L3StateResponseSchema:
    """Отменить цель или маршрут L3."""
    state: L3State = await asyncio.to_thread(isolated_motion.cancel_l3)
    return _to_response(state)


@router.websocket("/ws")
async def l3_state_ws(websocket: WebSocket) -> None:
    """Отправлять поток состояния L3."""
    await websocket.accept()

    isolated_motion: IsolatedMotionService = websocket.scope["app"].state.isolated_motion

    try:
        while True:
            state: L3State = await asyncio.to_thread(isolated_motion.get_l3_state)
            await websocket.send_json(_to_response(state).model_dump())
            await asyncio.sleep(isolated_motion.update_interval_sec)
    except WebSocketDisconnect:
        return
