from __future__ import annotations

from fastapi import APIRouter

from src.presentation.api.v1.schemas.health import HealthResponseSchema

router: APIRouter = APIRouter()


@router.get("/health", summary="Проверка работоспособности")
async def healthcheck() -> HealthResponseSchema:
    """Проверка работоспособности сервиса."""
    return HealthResponseSchema()
