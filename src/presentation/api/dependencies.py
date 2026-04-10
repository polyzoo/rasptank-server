from __future__ import annotations

from fastapi import HTTPException, Request, status

from src.application.protocols import DriveControllerProtocol
from src.application.services.motion_events import MotionEventHub


def get_drive_controller(request: Request) -> DriveControllerProtocol:
    """Получает контроллер для управления движением RaspTank."""
    if request.app.state.drive_controller:
        return request.app.state.drive_controller
    raise HTTPException(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail="Контроллер управления RaspTank не инициализирован.",
    )


def get_motion_events(request: Request) -> MotionEventHub:
    """Получает поток событий движения RaspTank."""
    if request.app.state.motion_events:
        return request.app.state.motion_events
    raise HTTPException(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail="Поток событий движения не инициализирован.",
    )
