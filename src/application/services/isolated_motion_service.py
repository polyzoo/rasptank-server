from __future__ import annotations

import time
from threading import Event, RLock, Thread
from typing import Callable

from src.application.services.l1_models import L1SensorState, L1TrackCommand
from src.application.services.l1_service import L1Service
from src.application.services.l2_models import BodyVelocityCommand, L1SensorSnapshot, L2State
from src.application.services.l2_service import L2Service
from src.application.services.l3_models import KnownObstacle, L3State, TargetPoint, TargetRoute
from src.application.services.l3_service import L3Service


class IsolatedMotionService:
    """Координатор нового изолированного контура L1-L3."""

    # Сколько периодов обновления даём фоновой нити на корректное завершение.
    LOOP_THREAD_JOIN_TIMEOUT_MULTIPLIER: float = 2.0

    def __init__(
        self,
        l1_service: L1Service,
        l2_service: L2Service,
        l3_service: L3Service,
        *,
        update_interval_sec: float,
        time_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        """Сохранить уровни нового контура и параметры цикла синхронизации."""
        self._l1_service: L1Service = l1_service
        self._l2_service: L2Service = l2_service
        self._l3_service: L3Service = l3_service
        self._update_interval_sec: float = update_interval_sec
        self._time_fn: Callable[[], float] = time_fn
        self._state_lock: RLock = RLock()
        self._stop_event: Event = Event()
        self._loop_thread: Thread | None = None
        self._last_sync_time_sec: float | None = None

    @property
    def update_interval_sec(self) -> float:
        """Вернуть период фонового обновления нового контура."""
        return self._update_interval_sec

    def start(self, *, calibrate_imu: bool = True) -> None:
        """Запустить IMU и фоновый цикл нового контура."""
        with self._state_lock:
            self._l1_service.start_imu(calibrate=calibrate_imu)
            self._last_sync_time_sec: float = self._time_fn()
            if self._loop_thread is not None and self._loop_thread.is_alive():
                return

            self._stop_event.clear()
            self._loop_thread = Thread(target=self._background_loop, daemon=True)
            self._loop_thread.start()

    def stop(self) -> None:
        """Остановить фоновый цикл нового контура."""
        self._stop_event.set()
        if self._loop_thread is not None:
            self._loop_thread.join(
                timeout=self._update_interval_sec * self.LOOP_THREAD_JOIN_TIMEOUT_MULTIPLIER
            )

        self._loop_thread: Thread | None = None

    def destroy(self, *, release_hardware: bool = True) -> None:
        """Остановить цикл и освободить ресурсы нового контура."""
        self.stop()
        with self._state_lock:
            self._l3_service.cancel()
            self._l2_service.stop()
            self._l1_service.stop_motion()
            self._l1_service.stop_imu()
            self._l1_service.destroy(release_devices=release_hardware)

    def read_l1_state(self) -> L1SensorState:
        """Вернуть последний снимок датчиков нижнего уровня."""
        with self._state_lock:
            return self._l1_service.read_sensors()

    def apply_l1_track_command(self, left_percent: int, right_percent: int) -> None:
        """Передать сырую команду непосредственно в L1."""
        with self._state_lock:
            self._l1_service.apply_track_command(
                L1TrackCommand(left_percent=left_percent, right_percent=right_percent)
            )

    def stop_l1(self) -> None:
        """Остановить борта через L1."""
        with self._state_lock:
            self._l1_service.stop_motion()

    def sync_l2_from_l1(self, *, dt_sec: float | None = None) -> L2State:
        """Считать датчики L1 и обновить состояние L2."""
        with self._state_lock:
            actual_dt_sec: float = self._compute_dt_sec(dt_sec)
            l1_state: L1SensorState = self._l1_service.read_sensors()
            return self._l2_service.update_from_l1(
                L1SensorSnapshot(
                    angular_speed_z_deg_per_sec=l1_state.angular_speed_z_deg_per_sec,
                    longitudinal_acceleration_m_s2=l1_state.accel_x_m_s2,
                    accel_x_m_s2=l1_state.accel_x_m_s2,
                    accel_y_m_s2=l1_state.accel_y_m_s2,
                    accel_z_m_s2=l1_state.accel_z_m_s2,
                    distance_cm=l1_state.distance_cm,
                ),
                actual_dt_sec,
            )

    def get_l2_state(self) -> L2State:
        """Вернуть текущее состояние L2."""
        with self._state_lock:
            return self._l2_service.get_state()

    def apply_l2_body_velocity(
        self,
        linear_speed_cm_per_sec: float,
        angular_speed_deg_per_sec: float,
    ) -> L2State:
        """Передать желаемую скорость корпуса в L2."""
        with self._state_lock:
            return self._l2_service.apply_body_velocity(
                BodyVelocityCommand(
                    linear_speed_cm_per_sec=linear_speed_cm_per_sec,
                    angular_speed_deg_per_sec=angular_speed_deg_per_sec,
                )
            )

    def stop_l2(self) -> L2State:
        """Остановить уровень L2."""
        with self._state_lock:
            return self._l2_service.stop()

    def reset_l2_state(
        self,
        *,
        x_cm: float = 0.0,
        y_cm: float = 0.0,
        heading_deg: float = 0.0,
        linear_speed_cm_per_sec: float = 0.0,
        angular_speed_deg_per_sec: float = 0.0,
    ) -> L2State:
        """Сбросить состояние L2."""
        with self._state_lock:
            return self._l2_service.reset_state(
                x_cm=x_cm,
                y_cm=y_cm,
                heading_deg=heading_deg,
                linear_speed_cm_per_sec=linear_speed_cm_per_sec,
                angular_speed_deg_per_sec=angular_speed_deg_per_sec,
            )

    def set_l3_goal(
        self,
        target: TargetPoint,
        obstacles: tuple[KnownObstacle, ...] = (),
    ) -> L3State:
        """Передать целевую точку в L3."""
        with self._state_lock:
            return self._l3_service.set_target_point(target=target, obstacles=obstacles)

    def set_l3_route(
        self,
        route: TargetRoute,
        obstacles: tuple[KnownObstacle, ...] = (),
    ) -> L3State:
        """Передать маршрут в L3."""
        with self._state_lock:
            return self._l3_service.set_route(route=route, obstacles=obstacles)

    def step_l3(self) -> L3State:
        """Выполнить один шаг верхнего уровня после синхронизации L2 с датчиками."""
        with self._state_lock:
            self.sync_l2_from_l1()
            return self._l3_service.step()

    def cancel_l3(self) -> L3State:
        """Отменить текущую цель или маршрут верхнего уровня."""
        with self._state_lock:
            return self._l3_service.cancel()

    def get_l3_state(self) -> L3State:
        """Вернуть текущее состояние L3."""
        with self._state_lock:
            return self._l3_service.get_state()

    def _background_loop(self) -> None:
        """Фоновый цикл синхронизации L2 и продвижения L3."""
        while not self._stop_event.wait(self._update_interval_sec):
            with self._state_lock:
                self.sync_l2_from_l1()
                self._l3_service.step()

    def _compute_dt_sec(self, dt_sec: float | None) -> float:
        """Вычислить шаг времени между обновлениями."""
        if dt_sec is not None:
            self._last_sync_time_sec = self._time_fn()
            return dt_sec

        now_sec: float = self._time_fn()
        if self._last_sync_time_sec is None:
            self._last_sync_time_sec: float = now_sec
            return 0.0

        actual_dt_sec: float = max(0.0, now_sec - self._last_sync_time_sec)
        self._last_sync_time_sec: float = now_sec
        return actual_dt_sec
