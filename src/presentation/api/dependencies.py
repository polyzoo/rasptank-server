from __future__ import annotations

from fastapi import HTTPException, Request, status

from src.application.protocols import DriveControllerProtocol
from src.application.services.isolated_motion_service import IsolatedMotionService
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


def get_isolated_motion_service(request: Request) -> IsolatedMotionService:
    """Получить изолированный контур L1-L3."""
    if request.app.state.isolated_motion is not None:
        return request.app.state.isolated_motion
    raise HTTPException(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail="Изолированный контур L1-L3 не инициализирован.",
    )


def get_isolated_motion_optional(request: Request) -> IsolatedMotionService | None:
    """Тот же контур L1-L3 или None, если не поднят (для совместимости и legacy-ручек)."""
    return getattr(request.app.state, "isolated_motion", None)
