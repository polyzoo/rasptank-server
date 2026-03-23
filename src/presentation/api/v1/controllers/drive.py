from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Depends

from src.application.models.route import (
    BackwardSegment,
    ForwardSegment,
    Route,
    TurnLeftSegment,
    TurnRightSegment,
)
from src.application.protocols import DriveControllerProtocol
from src.presentation.api.dependencies import get_drive_controller
from src.presentation.api.v1.schemas.drive import (
    BackwardSegmentSchema,
    DriveRouteResponseSchema,
    DriveStopResponseSchema,
    ForwardSegmentSchema,
    RouteRequestSchema,
    TurnLeftSegmentSchema,
    TurnRightSegmentSchema,
)

router: APIRouter = APIRouter()


def _schema_to_route(body: RouteRequestSchema) -> Route:
    """Преобразование API-схемы в доменную модель маршрута."""
    segments: list[ForwardSegment | BackwardSegment | TurnLeftSegment | TurnRightSegment] = []

    for s in body.segments:
        if isinstance(s, ForwardSegmentSchema):
            segments.append(ForwardSegment(distance_cm=s.distance_cm))

        elif isinstance(s, BackwardSegmentSchema):
            segments.append(BackwardSegment(distance_cm=s.distance_cm))

        elif isinstance(s, TurnLeftSegmentSchema):
            segments.append(TurnLeftSegment(angle_deg=s.angle_deg))

        elif isinstance(s, TurnRightSegmentSchema):
            segments.append(TurnRightSegment(angle_deg=s.angle_deg))

        else:
            raise ValueError(f"Неподдерживаемый сегмент маршрута: {type(s).__name__}")

    return Route(segments=segments)


@router.post("/route", description="Выполнение маршрута")
async def drive_route(
    body: RouteRequestSchema,
    drive: Annotated[DriveControllerProtocol, Depends(get_drive_controller)],
) -> DriveRouteResponseSchema:
    """Запуск движения по заданному маршруту с плавной остановкой при препятствиях."""
    route: Route = _schema_to_route(body)
    drive.execute_route(route=route)
    return DriveRouteResponseSchema()


@router.post("/stop", description="Немедленная остановка")
async def drive_stop(
    drive: Annotated[DriveControllerProtocol, Depends(get_drive_controller)],
) -> DriveStopResponseSchema:
    """Немедленная остановка движения."""
    drive.stop()
    return DriveStopResponseSchema()
