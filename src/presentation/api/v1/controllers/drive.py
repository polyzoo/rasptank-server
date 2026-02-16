from typing import Annotated

from fastapi import APIRouter, Depends, status

from src.application.protocols import DriveControllerProtocol
from src.presentation.api.dependencies import get_drive_controller
from src.presentation.api.v1.schemas.drive import ForwardRequestSchema

router: APIRouter = APIRouter()


@router.post(
    path="/forward",
    status_code=status.HTTP_204_NO_CONTENT,
    description="Запуск машинки вперёд на заданное расстояние с ограничением скорости",
)
async def drive_forward(
    body: ForwardRequestSchema,
    drive: Annotated[DriveControllerProtocol, Depends(get_drive_controller)],
) -> None:
    """Запуск машинки вперёд на заданное расстояние с ограничением скорости."""
    drive.forward_cm(distance_cm=body.distance_cm, max_speed_percent=body.max_speed_percent)
    return


@router.post(
    path="/stop",
    status_code=status.HTTP_204_NO_CONTENT,
    description="Немедленная остановка машинки",
)
async def drive_stop(
    drive: Annotated[DriveControllerProtocol, Depends(get_drive_controller)],
) -> None:
    """Немедленная остановка машинки."""
    drive.stop()
    return
