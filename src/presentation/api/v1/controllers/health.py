from typing import Annotated

from fastapi import APIRouter, Depends

from src.application.protocols import HealthCheckProtocol
from src.presentation.api.dependencies import get_health_controller
from src.presentation.api.v1.schemas.health import HealthResponseSchema

router: APIRouter = APIRouter()


@router.get("/health", summary="Проверка работоспособности сервиса")
async def healthcheck(
    health: Annotated[HealthCheckProtocol, Depends(get_health_controller)],
) -> HealthResponseSchema:
    """Проверка работоспособности сервиса."""
    health.check()
    return HealthResponseSchema()
