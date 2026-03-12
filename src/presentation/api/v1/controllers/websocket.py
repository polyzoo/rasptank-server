from __future__ import annotations

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from src.presentation.api.websocket_manager import ConnectionManager

router: APIRouter = APIRouter()

# Код закрытия для внутренних ошибок сервера WebSocket-соединения
CLOSE_CODE_SERVER_ERROR: int = 1011


@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    """WebSocket для получения событий в реальном времени."""
    manager: ConnectionManager | None = getattr(websocket.app.state, "ws_manager", None)
    if manager is None:
        await websocket.close(code=CLOSE_CODE_SERVER_ERROR, reason="WebSocket не инициализирован")
        return

    try:
        await manager.connect(websocket)
        while True:
            await websocket.receive_text()

    except WebSocketDisconnect:
        pass

    finally:
        manager.disconnect(websocket)
