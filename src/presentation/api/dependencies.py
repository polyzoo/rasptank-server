from __future__ import annotations

from fastapi import HTTPException, Request, status

from src.application.protocols import DriveControllerProtocol


def get_drive_controller(request: Request) -> DriveControllerProtocol:
    """Получает контроллер для управления движением RaspTank."""
    if request.app.state.drive_controller:
        return request.app.state.drive_controller
    raise HTTPException(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail="Контроллер управления RaspTank не инициализирован.",
    )
