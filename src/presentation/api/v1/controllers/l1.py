from __future__ import annotations

import asyncio
from typing import Annotated

from fastapi import APIRouter, Depends, WebSocket, WebSocketDisconnect

from src.application.services.isolated_motion_service import IsolatedMotionService
from src.application.services.l1_models import L1SensorState
from src.presentation.api.dependencies import get_isolated_motion_service
from src.presentation.api.v1.schemas.l1 import (
    L1ActionResponseSchema,
    L1SensorStateResponseSchema,
    L1TrackCommandRequestSchema,
)

router: APIRouter = APIRouter()


def _to_response(state: L1SensorState) -> L1SensorStateResponseSchema:
    """Преобразовать доменное состояние L1 в ответ API."""
    return L1SensorStateResponseSchema(
        angular_speed_z_deg_per_sec=state.angular_speed_z_deg_per_sec,
        accel_x_m_s2=state.accel_x_m_s2,
        accel_y_m_s2=state.accel_y_m_s2,
        accel_z_m_s2=state.accel_z_m_s2,
        distance_cm=state.distance_cm,
    )


@router.get("/state", description="Текущий снимок датчиков уровня L1")
async def l1_state(
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L1SensorStateResponseSchema:
    """Вернуть снимок датчиков нового уровня L1."""
    return _to_response(isolated_motion.read_l1_state())


@router.post("/tracks", description="Прямая команда левому и правому борту")
async def l1_tracks(
    body: L1TrackCommandRequestSchema,
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L1ActionResponseSchema:
    """Передать сырую команду бортам через новый L1."""
    isolated_motion.apply_l1_track_command(
        left_percent=body.left_percent,
        right_percent=body.right_percent,
    )
    return L1ActionResponseSchema(status="accepted")


@router.post("/stop", description="Остановить оба борта через L1")
async def l1_stop(
    isolated_motion: Annotated[IsolatedMotionService, Depends(get_isolated_motion_service)],
) -> L1ActionResponseSchema:
    """Остановить борта нового контура на уровне L1."""
    isolated_motion.stop_l1()
    return L1ActionResponseSchema(status="stopped")


@router.websocket("/ws")
async def l1_state_ws(websocket: WebSocket) -> None:
    """Поток снимков датчиков уровня L1."""
    await websocket.accept()

    isolated_motion: IsolatedMotionService = websocket.scope["app"].state.isolated_motion

    try:
        while True:
            await websocket.send_json(_to_response(isolated_motion.read_l1_state()).model_dump())
            await asyncio.sleep(isolated_motion.update_interval_sec)
    except WebSocketDisconnect:
        return
