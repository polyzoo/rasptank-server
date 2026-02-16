from pydantic import BaseModel


class HealthResponseSchema(BaseModel):
    """Схема ответа проверки работоспособности сервиса."""

    status: str = "ok"
