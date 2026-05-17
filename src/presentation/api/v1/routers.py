from __future__ import annotations

from fastapi import APIRouter

from src.presentation.api.v1.controllers import health, l1, l2, l3

router: APIRouter = APIRouter(prefix="/v1")

router.include_router(health.router, tags=["Общее"])
router.include_router(l1.router, prefix="/l1", tags=["L1-уровень"])
router.include_router(l2.router, prefix="/l2", tags=["L2-уровень"])
router.include_router(l3.router, prefix="/l3", tags=["L3-уровень"])
