from __future__ import annotations

from queue import Empty, Full, Queue

from src.application.services.motion_events import MotionEvent, MotionEventHub


def test_motion_event_to_dict_rounds_float_fields() -> None:
    """to_dict округляет координаты, курс и координаты препятствия."""
    event: MotionEvent = MotionEvent(
        type="obstacle",
        status="blocked",
        x_cm=1.234,
        y_cm=2.345,
        heading_deg=90.987,
        obstacle_cm=10.555,
        obstacle_x_cm=3.333,
        obstacle_y_cm=4.444,
        created_at=123.0,
    )

    payload: dict = event.to_dict()

    assert payload["x_cm"] == 1.23
    assert payload["y_cm"] == 2.35
    assert payload["heading_deg"] == 90.99
    assert payload["obstacle_cm"] == 10.55
    assert payload["obstacle_x_cm"] == 3.33
    assert payload["obstacle_y_cm"] == 4.44


def test_subscribe_receives_last_event_and_publish_updates_all_subscribers() -> None:
    """Подписчик сразу получает last_event и новые опубликованные события."""
    hub: MotionEventHub = MotionEventHub(queue_size=2)
    first_queue: Queue[MotionEvent] = hub.subscribe()
    second_queue: Queue[MotionEvent] = hub.subscribe()
    event: MotionEvent = MotionEvent(
        type="position",
        status="moving",
        x_cm=10.0,
        y_cm=5.0,
        heading_deg=30.0,
    )

    assert first_queue.get_nowait().status == "idle"
    assert second_queue.get_nowait().status == "idle"

    hub.publish(event)

    assert hub.last_event.created_at > 0.0
    assert first_queue.get_nowait().status == "moving"
    assert second_queue.get_nowait().status == "moving"


def test_unsubscribe_stops_delivery() -> None:
    """Отписанная очередь больше не получает новые события."""
    hub: MotionEventHub = MotionEventHub()
    queue: Queue[MotionEvent] = hub.subscribe()
    queue.get_nowait()

    hub.unsubscribe(queue)
    hub.publish(MotionEvent(type="status", status="stopped", x_cm=0.0, y_cm=0.0, heading_deg=0.0))

    try:
        delivered = queue.get_nowait()
    except Empty:
        delivered = None
    assert delivered is None


def test_publish_drops_oldest_event_when_queue_is_full() -> None:
    """При заполненной очереди publish удаляет старое событие и кладет новое."""
    hub: MotionEventHub = MotionEventHub(queue_size=1)
    queue: Queue[MotionEvent] = hub.subscribe()
    first_event: MotionEvent = MotionEvent(
        type="status",
        status="moving",
        x_cm=1.0,
        y_cm=1.0,
        heading_deg=0.0,
        created_at=1.0,
    )
    second_event: MotionEvent = MotionEvent(
        type="status",
        status="stopped",
        x_cm=2.0,
        y_cm=2.0,
        heading_deg=0.0,
        created_at=2.0,
    )

    hub.publish(first_event)
    hub.publish(second_event)

    assert queue.get_nowait().status == "stopped"


def test_publish_ignores_queue_when_drop_and_requeue_fail() -> None:
    """publish пропускает подписчика, если очередь не смогла освободиться."""

    class BrokenQueue:
        """Очередь, которая имитирует Full и Empty подряд."""

        def put_nowait(self, event: MotionEvent) -> None:
            """Сымитировать заполненную очередь."""
            raise Full

        def get_nowait(self) -> MotionEvent:
            """Сымитировать невозможность достать старое событие."""
            raise Empty

    hub: MotionEventHub = MotionEventHub()
    hub._subscribers.add(BrokenQueue())  # type: ignore[arg-type]
    event: MotionEvent = MotionEvent(
        type="status",
        status="moving",
        x_cm=1.0,
        y_cm=1.0,
        heading_deg=0.0,
        created_at=1.0,
    )

    hub.publish(event)

    assert hub.last_event is event
