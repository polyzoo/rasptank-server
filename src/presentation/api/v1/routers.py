from fastapi import APIRouter

from src.presentation.api.v1.controllers import drive, health

router: APIRouter = APIRouter(prefix="/v1")

router.include_router(health.router, tags=["Общее"])
router.include_router(drive.router, prefix="/drive", tags=["Управление движением машинки"])
