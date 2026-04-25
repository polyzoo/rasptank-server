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
from src.application.services.isolated_motion_service import IsolatedMotionService
from src.presentation.api.dependencies import (
    get_drive_controller,
    get_isolated_motion_optional,
)
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


def _disarm_l_stack_for_legacy_drive(isolated_motion: IsolatedMotionService | None) -> None:
    """Снять команды нового контура, чтобы моторы принадлежали только DriveController.

    Иначе фоновый цикл L3 и/или последние команды L2 продолжают вызывать set_tracks и
    наслаиваются на move_forward/move_backward — визуально «перепутываются» вперёд/назад и стоп.
    """
    if isolated_motion is None:
        return
    isolated_motion.cancel_l3()
    isolated_motion.stop_l2()
    isolated_motion.stop_l1()


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
    isolated_motion: Annotated[IsolatedMotionService | None, Depends(get_isolated_motion_optional)],
) -> DriveRouteResponseSchema:
    """Запуск движения по заданному маршруту с плавной остановкой при препятствиях."""
    _disarm_l_stack_for_legacy_drive(isolated_motion)
    route: Route = _schema_to_route(body)
    drive.execute_route(route=route)
    return DriveRouteResponseSchema()


@router.post("/stop", description="Немедленная остановка")
async def drive_stop(
    drive: Annotated[DriveControllerProtocol, Depends(get_drive_controller)],
    isolated_motion: Annotated[IsolatedMotionService | None, Depends(get_isolated_motion_optional)],
) -> DriveStopResponseSchema:
    """Немедленная остановка движения."""
    _disarm_l_stack_for_legacy_drive(isolated_motion)
    drive.stop()
    return DriveStopResponseSchema()
