from __future__ import annotations

import asyncio
from typing import Annotated

from fastapi import APIRouter, Depends, WebSocket, WebSocketDisconnect

from src.application.services.isolated_motion_service import IsolatedMotionService
from src.application.services.l2_models import L2State
from src.presentation.api.dependencies import get_isolated_motion_service
from src.presentation.api.v1.schemas.l2 import (
    L2BodyVelocityRequestSchema,
    L2ResetStateRequestSchema,
    L2StateResponseSchema,
)

router: APIRouter = APIRouter()


def _to_response(state: L2State) -> L2StateResponseSchema:
    """Преобразовать доменное состояние L2 в ответ API."""
    return L2StateResponseSchema(
        x_cm=state.x_cm,
        y_cm=state.y_cm,
        heading_deg=state.heading_deg,
        linear_speed_cm_per_sec=state.linear_speed_cm_per_sec,
        angular_speed_deg_per_sec=state.angular_speed_deg_per_sec,
        left_percent=state.left_percent,
        right_percent=state.right_percent,
        distance_cm=state.distance_cm,
    )


@router.get("/state", description="Текущее состояние уровня L2")
async def l2_state(
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L2StateResponseSchema:
    """Вернуть текущее состояние нового уровня L2."""
    return _to_response(isolated_motion.get_l2_state())


@router.post("/cmd-vel", description="Команда скорости корпуса для уровня L2")
async def l2_cmd_vel(
    body: L2BodyVelocityRequestSchema,
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L2StateResponseSchema:
    """Передать желаемую линейную и угловую скорость в новый L2."""
    return _to_response(
        isolated_motion.apply_l2_body_velocity(
            linear_speed_cm_per_sec=body.linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=body.angular_speed_deg_per_sec,
        )
    )


@router.post("/stop", description="Остановить уровень L2")
async def l2_stop(
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L2StateResponseSchema:
    """Остановить новый уровень L2 и вернуть текущее состояние."""
    return _to_response(isolated_motion.stop_l2())


@router.post("/reset-state", description="Сбросить состояние уровня L2")
async def l2_reset_state(
    body: L2ResetStateRequestSchema,
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L2StateResponseSchema:
    """Сбросить состояние нового уровня L2."""
    return _to_response(
        isolated_motion.reset_l2_state(
            x_cm=body.x_cm,
            y_cm=body.y_cm,
            heading_deg=body.heading_deg,
            linear_speed_cm_per_sec=body.linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=body.angular_speed_deg_per_sec,
        )
    )


@router.websocket("/ws")
async def l2_state_ws(websocket: WebSocket) -> None:
    """Поток состояния уровня L2."""
    await websocket.accept()

    isolated_motion: IsolatedMotionService = websocket.scope["app"].state.isolated_motion

    try:
        while True:
            await websocket.send_json(_to_response(isolated_motion.get_l2_state()).model_dump())
            await asyncio.sleep(isolated_motion.update_interval_sec)
    except WebSocketDisconnect:
        return
