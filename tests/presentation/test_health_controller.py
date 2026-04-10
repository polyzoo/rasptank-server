from __future__ import annotations

import anyio

from src.presentation.api.v1.controllers.health import healthcheck
from src.presentation.api.v1.schemas.health import HealthResponseSchema


def test_healthcheck_returns_ok() -> None:
    """Health endpoint handler возвращает статус ok."""

    async def run() -> None:
        """Выполнить async handler внутри anyio loop."""
        response: HealthResponseSchema = await healthcheck()

        assert response.status == "ok"

    anyio.run(run)
