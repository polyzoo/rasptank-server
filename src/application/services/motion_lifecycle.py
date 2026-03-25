from __future__ import annotations

from collections.abc import Callable
from threading import Event, Lock, Thread, current_thread


class MotionLifecycle:
    """Потокобезопасное состояние движения и фонового потока."""

    def __init__(self) -> None:
        """Инициализация состояния движения."""
        self._is_moving: bool = False
        self._is_route_running: bool = False
        self._movement_thread: Thread | None = None
        self._stop_event: Event = Event()
        self._lock: Lock = Lock()

    def set_running(self, *, is_route_running: bool) -> None:
        """Перевести движение в рабочее состояние."""
        with self._lock:
            self._is_moving: bool = True
            self._is_route_running: bool = is_route_running
            self._stop_event.clear()

    def set_stopped(self) -> None:
        """Перевести движение в остановленное состояние."""
        with self._lock:
            self._is_moving: bool = False
            self._is_route_running: bool = False
            self._stop_event.set()

    def should_keep_running(self) -> bool:
        """Проверить, что движение не было остановлено."""
        with self._lock:
            return self._is_moving and not self._stop_event.is_set()

    @property
    def is_moving(self) -> bool:
        """Текущее состояние активного движения."""
        with self._lock:
            return self._is_moving

    @property
    def is_route_running(self) -> bool:
        """Текущее состояние исполнения маршрута."""
        with self._lock:
            return self._is_route_running

    @property
    def stop_event(self) -> Event:
        """Сигнал остановки движения для low-level исполнителей."""
        return self._stop_event

    def attach_thread(self, thread: Thread) -> None:
        """Запомнить фоновый поток движения."""
        with self._lock:
            self._movement_thread: Thread | None = thread

    def clear_thread_if_current(self) -> None:
        """Сбросить сохранённый поток, если он завершился именно сейчас."""
        with self._lock:
            if self._movement_thread is current_thread():
                self._movement_thread: Thread | None = None

    def stop_and_join(self, motor_stop: Callable[[], None], timeout_sec: float) -> None:
        """Остановить движение и дождаться завершения фонового потока."""
        thread_to_join: Thread | None = None
        with self._lock:
            self._is_moving: bool = False
            self._is_route_running: bool = False
            self._stop_event.set()

            if (
                self._movement_thread is not None
                and self._movement_thread.is_alive()
                and self._movement_thread is not current_thread()
            ):
                thread_to_join: Thread | None = self._movement_thread

            self._movement_thread: Thread | None = None

        motor_stop()
        if thread_to_join is not None:
            thread_to_join.join(timeout=timeout_sec)
