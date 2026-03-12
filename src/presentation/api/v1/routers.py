from __future__ import annotations

from fastapi import APIRouter

from src.presentation.api.v1.controllers import drive, health, websocket

router: APIRouter = APIRouter(prefix="/v1")

router.include_router(health.router, tags=["Общее"])
router.include_router(drive.router, prefix="/drive", tags=["Управление движением"])
router.include_router(websocket.router, prefix="/drive", tags=["WebSocket"])
