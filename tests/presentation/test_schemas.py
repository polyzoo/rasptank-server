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
from src.presentation.api.v1.schemas.l1 import L1ActionResponseSchema, L1TrackCommandRequestSchema
from src.presentation.api.v1.schemas.l2 import L2ResetStateRequestSchema
from src.presentation.api.v1.schemas.l3 import L3RouteRequestSchema


def test_response_schemas_have_expected_defaults() -> None:
    """Response-схемы имеют ожидаемые значения по умолчанию."""
    assert HealthResponseSchema().status == "ok"
    assert DriveRouteResponseSchema().status == "accepted"
    assert DriveStopResponseSchema().status == "stopped"
    assert L1ActionResponseSchema(status="accepted").status == "accepted"
    assert L2ResetStateRequestSchema().x_cm == 0.0


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


def test_new_l1_and_l3_schemas_validate_bounds() -> None:
    """Новые схемы L1 и L3 проверяют диапазоны команд и непустой маршрут."""
    with pytest.raises(ValueError):
        L1TrackCommandRequestSchema.model_validate({"left_percent": 101, "right_percent": 0})

    with pytest.raises(ValueError):
        L3RouteRequestSchema.model_validate({"points": [], "obstacles": []})
