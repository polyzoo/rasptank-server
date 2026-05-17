from __future__ import annotations

from fastapi import HTTPException, Request, status

from src.application.services.isolated_motion_service import IsolatedMotionService


def get_isolated_motion_service(request: Request) -> IsolatedMotionService:
    """Вернуть координатор L1-L3 из состояния приложения."""
    if request.app.state.isolated_motion is not None:
        return request.app.state.isolated_motion
    raise HTTPException(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail="Изолированный контур L1-L3 не инициализирован.",
    )
