from __future__ import annotations

from fastapi import APIRouter

from src.presentation.api.v1.controllers import drive, health, l1, l2, l3, motion

router: APIRouter = APIRouter(prefix="/v1")

router.include_router(health.router, tags=["Общее"])
router.include_router(drive.router, prefix="/drive", tags=["Программное движением"])
router.include_router(motion.router, prefix="/motion", tags=["События программного движения"])
router.include_router(l1.router, prefix="/l1", tags=["L1-уровень"])
router.include_router(l2.router, prefix="/l2", tags=["L2-уровень"])
router.include_router(l3.router, prefix="/l3", tags=["L3-уровень"])
