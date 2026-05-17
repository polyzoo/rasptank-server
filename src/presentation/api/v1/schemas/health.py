from __future__ import annotations

from pydantic import BaseModel


class HealthResponseSchema(BaseModel):
    """API-ответ статуса сервиса."""

    status: str = "ok"
