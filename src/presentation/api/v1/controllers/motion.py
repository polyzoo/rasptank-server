from __future__ import annotations

import asyncio
from queue import Empty, Queue
from typing import Annotated

from fastapi import APIRouter, Depends, WebSocket, WebSocketDisconnect

from src.application.services.motion_events import MotionEvent, MotionEventHub
from src.presentation.api.dependencies import get_motion_events

router: APIRouter = APIRouter()

# Время ожидания события из очереди
EVENT_QUEUE_TIMEOUT_SEC: float = 1.0


@router.get("/state", description="Последнее событие движения")
async def motion_state(
    motion_events: Annotated[MotionEventHub, Depends(get_motion_events)],
) -> dict[str, int | float | str | None]:
    """Вернуть последнее событие движения."""
    return motion_events.last_event.to_dict()


@router.websocket("/ws")
async def motion_events_ws(websocket: WebSocket) -> None:
    """WebSocket-поток событий позиции, статуса и ошибок."""
    await websocket.accept()

    motion_events: MotionEventHub = websocket.scope["app"].state.motion_events
    queue: Queue[MotionEvent] = motion_events.subscribe()

    try:
        while True:
            try:
                event: MotionEvent = await asyncio.to_thread(
                    queue.get,
                    block=True,
                    timeout=EVENT_QUEUE_TIMEOUT_SEC,
                )

            except Empty:
                await websocket.send_json(motion_events.last_event.to_dict())
                continue

            await websocket.send_json(event.to_dict())

    except WebSocketDisconnect:
        return

    finally:
        motion_events.unsubscribe(queue)
