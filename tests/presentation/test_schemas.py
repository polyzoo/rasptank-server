from __future__ import annotations

import pytest

from src.presentation.api.v1.schemas.drive import (
    DriveRouteResponseSchema,
    DriveStopResponseSchema,
    ForwardSegmentSchema,
    RouteRequestSchema,
    TurnLeftSegmentSchema,
)
from src.presentation.api.v1.schemas.health import HealthResponseSchema


def test_response_schemas_have_expected_defaults() -> None:
    """Response-схемы имеют ожидаемые значения по умолчанию."""
    assert HealthResponseSchema().status == "ok"
    assert DriveRouteResponseSchema().status == "accepted"
    assert DriveStopResponseSchema().status == "stopped"


def test_segment_schemas_reject_negative_values() -> None:
    """Схемы сегментов отклоняют отрицательные значения."""
    with pytest.raises(ValueError):
        ForwardSegmentSchema.model_validate({"action": "forward", "distance_cm": -1})

    with pytest.raises(ValueError):
        TurnLeftSegmentSchema.model_validate({"action": "turn_left", "angle_deg": -1})


def test_route_request_uses_discriminator() -> None:
    """RouteRequestSchema выбирает тип сегмента по discriminator action."""
    route: RouteRequestSchema = RouteRequestSchema.model_validate(
        {"segments": [{"action": "turn_left", "angle_deg": 30}]}
    )

    assert isinstance(route.segments[0], TurnLeftSegmentSchema)
