from __future__ import annotations

import logging

from fastapi import FastAPI, Request, status
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse

logger: logging.Logger = logging.getLogger(__name__)


def setup_exception_handlers(app: FastAPI) -> None:
    """Зарегистрировать обработчики исключений."""

    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(
        request: Request,
        exc: RequestValidationError,
    ) -> JSONResponse:
        """Вернуть ответ на ошибку валидации запроса."""
        return JSONResponse(
            status_code=status.HTTP_422_UNPROCESSABLE_CONTENT,
            content={"detail": exc.errors()},
        )

    @app.exception_handler(Exception)
    async def exception_handler(
        request: Request,
        exc: Exception,
    ) -> JSONResponse:
        """Вернуть ответ на непредвиденную ошибку."""
        logger.exception("Внутреняя ошибка сервера: %s", exc)
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={"message": "Internal server error"},
        )
