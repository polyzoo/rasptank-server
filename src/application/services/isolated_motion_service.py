from __future__ import annotations

import logging
import threading
import time
from threading import Event, RLock, Thread
from typing import Callable, ClassVar

from src.application.services.l1_models import L1SensorState, L1TrackCommand
from src.application.services.l1_service import L1Service
from src.application.services.l2_models import BodyVelocityCommand, L1SensorSnapshot, L2State
from src.application.services.l2_service import L2Service
from src.application.services.l3_models import (
    L3_MODE_IDLE,
    KnownObstacle,
    L3State,
    TargetPoint,
    TargetRoute,
)
from src.application.services.l3_service import L3Service

logger: logging.Logger = logging.getLogger(__name__)


class IsolatedMotionService:
    """Координатор нового изолированного контура L1-L3."""

    # Сколько периодов обновления даём фоновой нити на корректное завершение.
    LOOP_THREAD_JOIN_TIMEOUT_MULTIPLIER: float = 2.0

    # WARNING, если полный sync L1→L2 занял столько миллисекунд и больше.
    SLOW_SYNC_L2_TOTAL_MS: ClassVar[float] = 350.0
    MAX_SYNC_DT_SEC: ClassVar[float] = 0.25

    def __init__(
        self,
        l1_service: L1Service,
        l2_service: L2Service,
        l3_service: L3Service,
        *,
        update_interval_sec: float,
        debug_trace_enabled: bool = False,
        debug_trace_every_n_steps: int = 1,
        time_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        """Сохранить уровни нового контура и параметры цикла синхронизации."""
        self._l1_service: L1Service = l1_service
        self._l2_service: L2Service = l2_service
        self._l3_service: L3Service = l3_service
        self._update_interval_sec: float = update_interval_sec
        self._debug_trace_enabled: bool = debug_trace_enabled
        self._debug_trace_every_n_steps: int = max(1, debug_trace_every_n_steps)
        self._debug_step_counter: int = 0
        self._time_fn: Callable[[], float] = time_fn
        self._state_lock: RLock = RLock()
        self._stop_event: Event = Event()
        self._loop_thread: Thread | None = None
        self._last_sync_time_sec: float | None = None
        # Пока > 0, фоновый контур не трогает датчики и L3 — отдать IMU/УЗ legacy DriveController.
        self._legacy_drive_exclusive_depth: int = 0

    @property
    def update_interval_sec(self) -> float:
        """Вернуть период фонового обновления нового контура."""
        return self._update_interval_sec

    def begin_legacy_drive_exclusive(self) -> None:
        """Пометить, что идёт движение по старому DriveController (маршрут / forward_cm_sync)."""
        with self._state_lock:
            self._legacy_drive_exclusive_depth += 1

    def end_legacy_drive_exclusive(self) -> None:
        """Завершить режим эксклюзива для legacy-движения."""
        with self._state_lock:
            if self._legacy_drive_exclusive_depth > 0:
                self._legacy_drive_exclusive_depth -= 1

    def reset_legacy_drive_exclusive(self) -> None:
        """Сбросить счётчик (например при destroy), чтобы фон снова работал."""
        with self._state_lock:
            self._legacy_drive_exclusive_depth = 0

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
        """Вернуть снимок датчиков нижнего уровня.

        Чтение выполняется без ``_state_lock``: оно может блокироваться на I²C/GPIO,
        и удержание общего замка здесь блокировало все остальные ручки L1–L3.
        """
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
        """Считать датчики L1 и обновить состояние L2.

        Датчики читаются вне ``_state_lock``, чтобы фоновый цикл не блокировал HTTP
        на время ``read_sensors()`` (IMU, ультразвук).
        """
        t0: float = time.perf_counter()
        with self._state_lock:
            actual_dt_sec: float = self._compute_dt_sec(dt_sec)
        t_after_dt_lock: float = time.perf_counter()

        l1_state: L1SensorState = self._l1_service.read_sensors()
        t_after_read: float = time.perf_counter()

        with self._state_lock:
            l2_state: L2State = self._l2_service.update_from_l1(
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
            self._trace_l1_l2_math(l1_state=l1_state, l2_state=l2_state, dt_sec=actual_dt_sec)
            self._log_l2_diagnostics(l2_state=l2_state, dt_sec=actual_dt_sec)

        t_end: float = time.perf_counter()
        dt_lock_ms: float = (t_after_dt_lock - t0) * 1000.0
        read_ms: float = (t_after_read - t_after_dt_lock) * 1000.0
        apply_lock_ms: float = (t_end - t_after_read) * 1000.0
        total_ms: float = (t_end - t0) * 1000.0

        logger.debug(
            "IsolatedMotion.sync_l2_from_l1 thread=%s dt_lock_ms=%.2f read_l1_ms=%.2f "
            "apply_l2_lock_ms=%.2f total_ms=%.2f dt_sec=%.4f",
            threading.current_thread().name,
            dt_lock_ms,
            read_ms,
            apply_lock_ms,
            total_ms,
            actual_dt_sec,
        )

        if total_ms >= self.SLOW_SYNC_L2_TOTAL_MS:
            logger.warning(
                "IsolatedMotion.sync_l2_from_l1 медленно thread=%s total_ms=%.1f "
                "dt_lock_ms=%.1f read_l1_ms=%.1f apply_l2_lock_ms=%.1f "
                "(read_l1_ms: датчики L1; apply_*: конкуренция за lock)",
                threading.current_thread().name,
                total_ms,
                dt_lock_ms,
                read_ms,
                apply_lock_ms,
            )

        return l2_state

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

    def configure_l2_state_space(self, enabled: bool, t_v: float, t_w: float) -> None:
        """Настроить параметры LQR/МПС на уровне L2."""
        with self._state_lock:
            self._l2_service.configure_state_space(enabled, t_v, t_w)

    def set_l3_goal(
        self,
        target: TargetPoint,
        obstacles: tuple[KnownObstacle, ...] = (),
    ) -> L3State:
        """Передать целевую точку в L3."""
        with self._state_lock:
            self._l2_service.reset_state()
            return self._l3_service.set_target_point(target=target, obstacles=obstacles)

    def set_l3_route(
        self,
        route: TargetRoute,
        obstacles: tuple[KnownObstacle, ...] = (),
    ) -> L3State:
        """Передать маршрут в L3."""
        with self._state_lock:
            self._l2_service.reset_state()
            return self._l3_service.set_route(route=route, obstacles=obstacles)

    def step_l3(self) -> L3State:
        """Выполнить один шаг верхнего уровня после синхронизации L2 с датчиками."""
        self.sync_l2_from_l1()
        with self._state_lock:
            l3_state: L3State = self._l3_service.step()
            self._trace_l3_math(l3_state=l3_state)
            return l3_state

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
                if self._legacy_drive_exclusive_depth > 0:
                    continue

            self.sync_l2_from_l1()

            with self._state_lock:
                l3_state: L3State = self._l3_service.step()
                self._trace_l3_math(l3_state=l3_state)

    def _compute_dt_sec(self, dt_sec: float | None) -> float:
        """Вычислить шаг времени между обновлениями."""
        if dt_sec is not None:
            self._last_sync_time_sec = self._time_fn()
            return dt_sec

        now_sec: float = self._time_fn()
        if self._last_sync_time_sec is None:
            self._last_sync_time_sec: float = now_sec
            return 0.0

        actual_dt_sec: float = min(
            self.MAX_SYNC_DT_SEC,
            max(0.0, now_sec - self._last_sync_time_sec),
        )
        self._last_sync_time_sec: float = now_sec
        return actual_dt_sec

    def _trace_l1_l2_math(
        self, *, l1_state: L1SensorState, l2_state: L2State, dt_sec: float
    ) -> None:
        """Записать отладочный след преобразования L1 -> L2."""
        if not self._should_trace():
            return

        logger.info(
            (
                "L1->L2 dt=%.3f sensors=(w_gyro=%.3f, a_xyz=(%.3f, %.3f, %.3f), dist=%.2f) | "
                "L2 state_hat=(x=%.2f, y=%.2f, theta=%.2f, v_hat=%.2f, w_hat=%.2f) "
                "tracks=(U_L=%.1f, U_R=%.1f)"
            ),
            dt_sec,
            l1_state.angular_speed_z_deg_per_sec,
            l1_state.accel_x_m_s2,
            l1_state.accel_y_m_s2,
            l1_state.accel_z_m_s2,
            l1_state.distance_cm,
            l2_state.x_cm,
            l2_state.y_cm,
            l2_state.heading_deg,
            l2_state.linear_speed_cm_per_sec,
            l2_state.angular_speed_deg_per_sec,
            l2_state.left_percent,
            l2_state.right_percent,
        )

    def _log_l2_diagnostics(self, *, l2_state: L2State, dt_sec: float) -> None:
        """Записать компактную диагностику L2 для ручных тестов без дашборда."""
        control_u = l2_state.state_space_control_u or (None, None)
        logger.info(
            (
                "L2_DIAG dt=%.3f pose=(x=%.2f,y=%.2f,theta=%.2f) "
                "vel=(v=%.2f,w=%.2f) tracks=(L=%.1f,R=%.1f) "
                "mps_u=(v=%s,w=%s)"
            ),
            dt_sec,
            l2_state.x_cm,
            l2_state.y_cm,
            l2_state.heading_deg,
            l2_state.linear_speed_cm_per_sec,
            l2_state.angular_speed_deg_per_sec,
            l2_state.left_percent,
            l2_state.right_percent,
            _fmt_optional(control_u[0]),
            _fmt_optional(control_u[1]),
        )

    def _trace_l3_math(self, *, l3_state: L3State) -> None:
        """Записать отладочный след шага L3 -> L2 -> L1 в терминах формул."""
        if not self._should_trace():
            return

        l2_state: L2State = self._l2_service.get_state()
        logger.info(
            (
                "L3->L2 status=%s mode=%s planner=%s "
                "X*=(x=%s, y=%s) err=(dist=%s, dtheta=%s, theta*=%s) "
                "cmd_des=(v_des=%s, w_des=%s) | "
                "L2->L1 cmd_tracks=(U_L=%.1f, U_R=%.1f) "
                "state_hat=(x=%.2f, y=%.2f, theta=%.2f, v_hat=%.2f, w_hat=%.2f)"
            ),
            l3_state.status,
            l3_state.mode,
            l3_state.planner_status,
            _fmt_optional(l3_state.target_x_cm),
            _fmt_optional(l3_state.target_y_cm),
            _fmt_optional(l3_state.distance_error_cm),
            _fmt_optional(l3_state.heading_error_deg),
            _fmt_optional(l3_state.target_heading_deg),
            _fmt_optional(l3_state.desired_linear_speed_cm_per_sec),
            _fmt_optional(l3_state.desired_angular_speed_deg_per_sec),
            l2_state.left_percent,
            l2_state.right_percent,
            l2_state.x_cm,
            l2_state.y_cm,
            l2_state.heading_deg,
            l2_state.linear_speed_cm_per_sec,
            l2_state.angular_speed_deg_per_sec,
        )

    def _should_trace(self) -> bool:
        """Определить, нужно ли печатать текущий шаг debug-трейса."""
        if not self._debug_trace_enabled:
            return False
        if self._l3_service.get_state().mode == L3_MODE_IDLE:
            return False
        self._debug_step_counter += 1
        return self._debug_step_counter % self._debug_trace_every_n_steps == 0


def _fmt_optional(value: float | None) -> str:
    """Сформатировать optional float для логов без 'NoneType'."""
    if value is None:
        return "-"
    return f"{value:.2f}"
