from typing import Annotated

from fastapi import APIRouter, Depends

from src.application.protocols import DriveControllerProtocol
from src.presentation.api.dependencies import get_drive_controller
from src.presentation.api.v1.schemas.drive import ForwardRequestSchema

router: APIRouter = APIRouter()


@router.post("/forward", description="Движение вперед")
async def drive_forward(
    body: ForwardRequestSchema,
    drive: Annotated[DriveControllerProtocol, Depends(get_drive_controller)],
) -> None:
    """Запуск движения вперед."""
    drive.forward_cm(distance_cm=body.distance_cm, max_speed_percent=body.max_speed_percent)
    return


@router.post("/stop", description="Немедленная остановка")
async def drive_stop(
    drive: Annotated[DriveControllerProtocol, Depends(get_drive_controller)],
) -> None:
    """Немедленная остановка движения."""
    drive.stop()
    return
