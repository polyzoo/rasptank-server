from fastapi import FastAPI, Request, status
from fastapi.responses import JSONResponse


def setup_exception_handlers(app: FastAPI) -> None:
    """Конфигурация обработчиков для исключений."""

    @app.exception_handler(Exception)
    async def exception_handler(request: Request, exc: Exception) -> JSONResponse:
        """Обработчик для всех ошибок."""
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={"message": str(exc)},
        )
