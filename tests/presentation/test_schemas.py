from __future__ import annotations

import pytest

from src.presentation.api.v1.schemas.health import HealthResponseSchema
from src.presentation.api.v1.schemas.l1 import L1ActionResponseSchema, L1TrackCommandRequestSchema
from src.presentation.api.v1.schemas.l2 import L2ResetStateRequestSchema
from src.presentation.api.v1.schemas.l3 import L3GoalRequestSchema, L3RouteRequestSchema


def test_response_schemas_have_expected_defaults() -> None:
    """Response-схемы имеют ожидаемые значения по умолчанию."""
    assert HealthResponseSchema().status == "ok"
    assert L1ActionResponseSchema(status="accepted").status == "accepted"
    assert L2ResetStateRequestSchema().x_cm == 0.0


def test_new_l1_and_l3_schemas_validate_bounds() -> None:
    """Новые схемы L1 и L3 проверяют диапазоны команд и непустой маршрут."""
    with pytest.raises(ValueError):
        L1TrackCommandRequestSchema.model_validate({"left_percent": 101, "right_percent": 0})

    with pytest.raises(ValueError):
        L3RouteRequestSchema.model_validate({"points": []})

    with pytest.raises(ValueError):
        L3GoalRequestSchema.model_validate(
            {
                "target": {"x_cm": 1.0, "y_cm": 2.0},
                "obstacles": [{"x_cm": 3.0, "y_cm": 4.0, "radius_cm": 5.0}],
            }
        )
