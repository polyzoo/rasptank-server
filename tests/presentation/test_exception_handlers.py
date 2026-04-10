from __future__ import annotations

from typing import Callable

import anyio
from fastapi import FastAPI, status
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse

from src.presentation.api.exception_handlers import setup_exception_handlers


def test_validation_exception_handler_returns_422_detail() -> None:
    """Обработчик валидационных ошибоку возвращает JSON с HTTP 422."""

    async def run() -> None:
        """Вызвать зарегистрированный обработчик напрямую."""
        app: FastAPI = FastAPI()
        setup_exception_handlers(app)
        handler: Callable = app.exception_handlers[RequestValidationError]
        exc: RequestValidationError = RequestValidationError(
            errors=[{"loc": ("body", "field"), "msg": "bad"}]
        )

        response: JSONResponse = await handler(None, exc)

        assert response.status_code == status.HTTP_422_UNPROCESSABLE_CONTENT
        assert b"body" in response.body
        assert b"field" in response.body

    anyio.run(run)


def test_generic_exception_handler_returns_500_message() -> None:
    """Обработчик неизвестных исключений возвращает HTTP 500."""

    async def run() -> None:
        """Вызвать зарегистрированный обработчик напрямую."""
        app: FastAPI = FastAPI()
        setup_exception_handlers(app)
        handler: Callable = app.exception_handlers[Exception]

        response: JSONResponse = await handler(None, RuntimeError("boom"))

        assert response.status_code == status.HTTP_500_INTERNAL_SERVER_ERROR
        assert response.body == b'{"message":"Internal server error"}'

    anyio.run(run)
