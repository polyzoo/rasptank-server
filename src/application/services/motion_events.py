from __future__ import annotations

import time
from dataclasses import asdict, dataclass
from queue import Empty, Full, Queue
from threading import Lock
from typing import Literal, TypeAlias


MotionStatus: TypeAlias = Literal["idle", "moving", "turning", "blocked", "error", "stopped"]
MotionEventType: TypeAlias = Literal["position", "status", "error", "obstacle"]


@dataclass(frozen=True, slots=True)
class MotionEvent:
    """Событие движения для синхронизации машинки и интерфейса."""

    type: MotionEventType
    status: MotionStatus
    x_cm: float
    y_cm: float
    heading_deg: float
    message: str | None = None
    obstacle_cm: float | None = None
    obstacle_x_cm: float | None = None
    obstacle_y_cm: float | None = None
    created_at: float = 0.0

    def to_dict(self) -> dict:
        """Вернуть JSON-friendly представление события."""
        payload: dict = asdict(self)
        payload["x_cm"] = round(self.x_cm, 2)
        payload["y_cm"] = round(self.y_cm, 2)
        payload["heading_deg"] = round(self.heading_deg, 2)
        if self.obstacle_cm is not None:
            payload["obstacle_cm"] = round(self.obstacle_cm, 2)
        if self.obstacle_x_cm is not None:
            payload["obstacle_x_cm"] = round(self.obstacle_x_cm, 2)
        if self.obstacle_y_cm is not None:
            payload["obstacle_y_cm"] = round(self.obstacle_y_cm, 2)
        return payload


class MotionEventHub:
    """Простой потокобезопасный pub/sub для событий движения."""

    def __init__(self, queue_size: int = 200) -> None:
        self._subscribers: set[Queue[MotionEvent]] = set()
        self._lock: Lock = Lock()
        self._last_event: MotionEvent = MotionEvent(
            type="status",
            status="idle",
            x_cm=0.0,
            y_cm=0.0,
            heading_deg=0.0,
            message="Ожидание маршрута",
            created_at=time.time(),
        )
        self._queue_size: int = queue_size

    @property
    def last_event(self) -> MotionEvent:
        """Последнее опубликованное событие."""
        return self._last_event

    def subscribe(self) -> Queue[MotionEvent]:
        """Подписаться на поток событий."""
        queue: Queue[MotionEvent] = Queue(maxsize=self._queue_size)
        with self._lock:
            self._subscribers.add(queue)
            queue.put_nowait(self._last_event)
        return queue

    def unsubscribe(self, queue: Queue[MotionEvent]) -> None:
        """Отписаться от потока событий."""
        with self._lock:
            self._subscribers.discard(queue)

    def publish(self, event: MotionEvent) -> None:
        """Опубликовать событие всем текущим подписчикам."""
        if event.created_at <= 0.0:
            event = MotionEvent(
                type=event.type,
                status=event.status,
                x_cm=event.x_cm,
                y_cm=event.y_cm,
                heading_deg=event.heading_deg,
                message=event.message,
                obstacle_cm=event.obstacle_cm,
                obstacle_x_cm=event.obstacle_x_cm,
                obstacle_y_cm=event.obstacle_y_cm,
                created_at=time.time(),
            )

        with self._lock:
            self._last_event = event
            subscribers: tuple[Queue[MotionEvent], ...] = tuple(self._subscribers)

        for queue in subscribers:
            try:
                queue.put_nowait(event)
            except Full:
                try:
                    queue.get_nowait()
                    queue.put_nowait(event)
                except (Empty, Full):
                    continue
