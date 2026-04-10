from __future__ import annotations

from threading import current_thread

from src.application.services.motion_lifecycle import MotionLifecycle


class FakeThread:
    """Заглушка фонового потока."""

    def __init__(self, *, alive: bool) -> None:
        """Сохранить флаг активности."""
        self._alive: bool = alive
        self.join_timeout: float | None = None

    def is_alive(self) -> bool:
        """Вернуть состояние активности."""
        return self._alive

    def join(self, timeout: float | None = None) -> None:
        """Зафиксировать timeout ожидания."""
        self.join_timeout = timeout


def test_running_and_stopped_state() -> None:
    """set_running и set_stopped меняют состояние движения и stop event."""
    lifecycle: MotionLifecycle = MotionLifecycle()

    lifecycle.set_running(is_route_running=True)

    assert lifecycle.is_moving is True
    assert lifecycle.is_route_running is True
    assert lifecycle.should_keep_running() is True

    lifecycle.set_stopped()

    assert lifecycle.is_moving is False
    assert lifecycle.is_route_running is False
    assert lifecycle.should_keep_running() is False
    assert lifecycle.stop_event.is_set() is True


def test_attach_and_clear_thread_if_current() -> None:
    """clear_thread_if_current сбрасывает только текущий поток."""
    lifecycle: MotionLifecycle = MotionLifecycle()
    actual_current = current_thread()

    lifecycle.attach_thread(actual_current)
    lifecycle.clear_thread_if_current()

    assert lifecycle._movement_thread is None


def test_stop_and_join_stops_motor_and_joins_alive_foreign_thread() -> None:
    """stop_and_join останавливает движение, вызывает motor_stop и join для чужого потока."""
    lifecycle: MotionLifecycle = MotionLifecycle()
    thread: FakeThread = FakeThread(alive=True)
    motor_stop_calls: list[None] = []
    lifecycle.set_running(is_route_running=True)
    lifecycle.attach_thread(thread)  # type: ignore[arg-type]

    lifecycle.stop_and_join(lambda: motor_stop_calls.append(None), timeout_sec=0.5)

    assert lifecycle.is_moving is False
    assert lifecycle.is_route_running is False
    assert lifecycle.stop_event.is_set() is True
    assert motor_stop_calls == [None]
    assert thread.join_timeout == 0.5
    assert lifecycle._movement_thread is None


def test_stop_and_join_does_not_join_dead_thread() -> None:
    """stop_and_join не вызывает join для неактивного потока."""
    lifecycle: MotionLifecycle = MotionLifecycle()
    thread: FakeThread = FakeThread(alive=False)
    lifecycle.attach_thread(thread)  # type: ignore[arg-type]

    lifecycle.stop_and_join(lambda: None, timeout_sec=0.5)

    assert thread.join_timeout is None
