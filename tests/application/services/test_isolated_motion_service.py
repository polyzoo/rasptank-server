from __future__ import annotations

from typing import Any

from src.application.services.isolated_motion_service import IsolatedMotionService
from src.application.services.l1_models import L1SensorState, L1TrackCommand
from src.application.services.l2_models import BodyVelocityCommand, L1SensorSnapshot, L2State
from src.application.services.l3_models import KnownObstacle, L3State, TargetPoint, TargetRoute


class FakeL1Service:
    """Заглушка нового уровня L1 для тестов координатора."""

    def __init__(self) -> None:
        """Подготовить состояние вызовов."""
        self.start_calls: list[bool] = []
        self.stop_imu_calls: int = 0
        self.track_commands: list[L1TrackCommand] = []
        self.stop_motion_calls: int = 0
        self.destroy_calls: int = 0
        self.sensor_state: L1SensorState = L1SensorState(
            angular_speed_z_deg_per_sec=12.0,
            accel_x_m_s2=1.5,
            accel_y_m_s2=2.5,
            accel_z_m_s2=3.5,
            distance_cm=44.0,
        )

    def start_imu(self, *, calibrate: bool = True) -> None:
        """Сохранить факт запуска IMU."""
        self.start_calls.append(calibrate)

    def stop_imu(self) -> None:
        """Сохранить факт остановки IMU."""
        self.stop_imu_calls += 1

    def read_sensors(self) -> L1SensorState:
        """Вернуть заранее подготовленные данные датчиков."""
        return self.sensor_state

    def apply_track_command(self, command: L1TrackCommand) -> None:
        """Запомнить сырую команду бортам."""
        self.track_commands.append(command)

    def stop_motion(self) -> None:
        """Запомнить остановку бортов."""
        self.stop_motion_calls += 1

    def destroy(self, *, release_devices: bool = True) -> None:
        """Запомнить освобождение ресурсов."""
        self.destroy_calls += 1


class FakeL2Service:
    """Заглушка нового уровня L2 для тестов координатора."""

    def __init__(self) -> None:
        """Подготовить состояние и историю вызовов."""
        self.update_snapshots: list[tuple[L1SensorSnapshot, float]] = []
        self.body_velocity_commands: list[BodyVelocityCommand] = []
        self.stop_calls: int = 0
        self.reset_calls: list[dict[str, float]] = []
        self.state: L2State = L2State(
            x_cm=1.0,
            y_cm=2.0,
            heading_deg=3.0,
            linear_speed_cm_per_sec=4.0,
            angular_speed_deg_per_sec=5.0,
            left_percent=6.0,
            right_percent=7.0,
            distance_cm=8.0,
        )

    def update_from_l1(self, snapshot: L1SensorSnapshot, dt_sec: float) -> L2State:
        """Сохранить снимок L1 и вернуть текущее состояние."""
        self.update_snapshots.append((snapshot, dt_sec))
        return self.state

    def get_state(self) -> L2State:
        """Вернуть состояние L2."""
        return self.state

    def apply_body_velocity(self, command: BodyVelocityCommand) -> L2State:
        """Сохранить команду скорости корпуса."""
        self.body_velocity_commands.append(command)
        return self.state

    def stop(self) -> L2State:
        """Сохранить факт остановки L2."""
        self.stop_calls += 1
        return self.state

    def reset_state(self, **kwargs: float) -> L2State:
        """Сохранить параметры сброса состояния."""
        self.reset_calls.append(kwargs)
        return self.state


class FakeL3Service:
    """Заглушка нового уровня L3 для тестов координатора."""

    def __init__(self) -> None:
        """Подготовить состояние и историю вызовов."""
        self.goal_calls: list[tuple[TargetPoint, tuple[KnownObstacle, ...]]] = []
        self.route_calls: list[tuple[TargetRoute, tuple[KnownObstacle, ...]]] = []
        self.step_calls: int = 0
        self.cancel_calls: int = 0
        self.state: L3State = L3State(
            status="idle",
            mode="idle",
            planner_status="idle",
            target_x_cm=None,
            target_y_cm=None,
            active_point_index=None,
            total_points=0,
            distance_error_cm=None,
            heading_error_deg=None,
            target_heading_deg=None,
            linear_speed_cm_per_sec=0.0,
            angular_speed_deg_per_sec=0.0,
        )

    def set_target_point(
        self,
        target: TargetPoint,
        obstacles: tuple[KnownObstacle, ...] = (),
    ) -> L3State:
        """Сохранить цель L3."""
        self.goal_calls.append((target, obstacles))
        return self.state

    def set_route(
        self,
        route: TargetRoute,
        obstacles: tuple[KnownObstacle, ...] = (),
    ) -> L3State:
        """Сохранить маршрут L3."""
        self.route_calls.append((route, obstacles))
        return self.state

    def step(self) -> L3State:
        """Зафиксировать шаг L3."""
        self.step_calls += 1
        return self.state

    def cancel(self) -> L3State:
        """Зафиксировать отмену L3."""
        self.cancel_calls += 1
        return self.state

    def get_state(self) -> L3State:
        """Вернуть состояние L3."""
        return self.state


class FakeThread:
    """Упрощённая заглушка потока для проверки запуска координатора."""

    def __init__(self, target: Any, daemon: bool) -> None:
        """Сохранить целевую функцию и флаг демона."""
        self.target: Any = target
        self.daemon: bool = daemon
        self.started: bool = False
        self.join_timeout: float | None = None
        self._alive: bool = False

    def start(self) -> None:
        """Зафиксировать запуск потока."""
        self.started = True
        self._alive = True

    def is_alive(self) -> bool:
        """Вернуть признак активного потока."""
        return self._alive

    def join(self, timeout: float | None = None) -> None:
        """Зафиксировать ожидание завершения потока."""
        self.join_timeout = timeout
        self._alive = False


class FakeStopEvent:
    """Заглушка события остановки для одного прохода фонового цикла."""

    def __init__(self) -> None:
        """Подготовить последовательность ответов wait."""
        self.calls: int = 0

    def wait(self, timeout: float) -> bool:
        """Вернуть False на первом шаге и True на втором."""
        self.calls += 1
        return self.calls > 1


def _service(
    *, time_values: tuple[float, ...] = (10.0, 10.2)
) -> tuple[
    IsolatedMotionService,
    FakeL1Service,
    FakeL2Service,
    FakeL3Service,
]:
    """Собрать координатор нового контура и заглушки уровней."""
    l1_service: FakeL1Service = FakeL1Service()
    l2_service: FakeL2Service = FakeL2Service()
    l3_service: FakeL3Service = FakeL3Service()
    values: list[float] = list(time_values)

    def time_fn() -> float:
        """Вернуть очередное значение времени."""
        return values.pop(0)

    return (
        IsolatedMotionService(
            l1_service=l1_service,  # type: ignore[arg-type]
            l2_service=l2_service,  # type: ignore[arg-type]
            l3_service=l3_service,  # type: ignore[arg-type]
            update_interval_sec=0.1,
            time_fn=time_fn,
        ),
        l1_service,
        l2_service,
        l3_service,
    )


def test_start_stop_and_destroy_manage_new_contour_lifecycle(monkeypatch: Any) -> None:
    """Координатор запускает цикл, затем корректно останавливает и освобождает ресурсы."""
    service, l1_service, l2_service, l3_service = _service()
    created_threads: list[FakeThread] = []

    def fake_thread(target: Any, daemon: bool) -> FakeThread:
        """Создать заглушку потока и сохранить её для проверки."""
        thread: FakeThread = FakeThread(target=target, daemon=daemon)
        created_threads.append(thread)
        return thread

    monkeypatch.setattr("src.application.services.isolated_motion_service.Thread", fake_thread)

    service.start(calibrate_imu=False)
    service.start(calibrate_imu=True)

    assert l1_service.start_calls == [False, True]
    assert created_threads[0].started is True
    assert service.update_interval_sec == 0.1

    service.stop()
    service.destroy()

    assert created_threads[0].join_timeout == 0.2
    assert l3_service.cancel_calls == 1
    assert l2_service.stop_calls == 1
    assert l1_service.stop_motion_calls == 1
    assert l1_service.stop_imu_calls == 1
    assert l1_service.destroy_calls == 1


def test_public_methods_forward_commands_and_states() -> None:
    """Координатор прокидывает команды между уровнями и возвращает состояния."""
    service, l1_service, l2_service, l3_service = _service(time_values=(1.0, 1.5, 2.0))

    assert service.read_l1_state() == l1_service.sensor_state

    service.apply_l1_track_command(left_percent=10, right_percent=-20)
    service.stop_l1()
    synced_state = service.sync_l2_from_l1()
    l2_state = service.get_l2_state()
    after_cmd_state = service.apply_l2_body_velocity(12.0, 34.0)
    stopped_l2_state = service.stop_l2()
    reset_state = service.reset_l2_state(x_cm=9.0, y_cm=8.0, heading_deg=7.0)
    goal_state = service.set_l3_goal(TargetPoint(x_cm=1.0, y_cm=2.0))
    route_state = service.set_l3_route(TargetRoute(points=(TargetPoint(x_cm=3.0, y_cm=4.0),)))
    stepped_l3_state = service.step_l3()
    cancelled_l3_state = service.cancel_l3()
    current_l3_state = service.get_l3_state()

    assert l1_service.track_commands[0] == L1TrackCommand(left_percent=10, right_percent=-20)
    assert l1_service.stop_motion_calls == 1
    assert l2_service.update_snapshots[0][0].longitudinal_acceleration_m_s2 == 1.5
    assert l2_service.update_snapshots[0][0].distance_cm == 44.0
    assert l2_service.update_snapshots[0][1] == 0.0
    assert l2_service.body_velocity_commands[0] == BodyVelocityCommand(
        linear_speed_cm_per_sec=12.0,
        angular_speed_deg_per_sec=34.0,
    )
    assert l2_service.reset_calls[0]["x_cm"] == 9.0
    assert l3_service.goal_calls[0][0] == TargetPoint(x_cm=1.0, y_cm=2.0)
    assert l3_service.route_calls[0][0] == TargetRoute(points=(TargetPoint(x_cm=3.0, y_cm=4.0),))
    assert l3_service.step_calls == 1
    assert l3_service.cancel_calls == 1
    assert l2_service.update_snapshots[1][1] == 0.5
    assert (
        synced_state
        == l2_state
        == after_cmd_state
        == stopped_l2_state
        == reset_state
        == l2_service.state
    )
    assert (
        goal_state
        == route_state
        == stepped_l3_state
        == cancelled_l3_state
        == current_l3_state
        == l3_service.state
    )


def test_legacy_exclusive_depth_begin_end_reset() -> None:
    """Счётчик эксклюзива legacy-драйва инкрементируется, уменьшается и сбрасывается."""
    service, _, _, _ = _service()

    service.begin_legacy_drive_exclusive()
    service.begin_legacy_drive_exclusive()
    assert service._legacy_drive_exclusive_depth == 2
    service.end_legacy_drive_exclusive()
    assert service._legacy_drive_exclusive_depth == 1
    service.reset_legacy_drive_exclusive()
    assert service._legacy_drive_exclusive_depth == 0


def test_end_legacy_drive_exclusive_noop_when_depth_zero() -> None:
    """end_legacy при нулевой глубине не уходит в отрицательные значения."""
    service, _, _, _ = _service()

    service.end_legacy_drive_exclusive()

    assert service._legacy_drive_exclusive_depth == 0


def test_background_loop_skips_sync_when_legacy_exclusive_active() -> None:
    """Пока legacy держит эксклюзив, фон не вызывает sync L2 и step L3."""
    service, _l1, l2, l3 = _service(time_values=(5.0, 8.0))
    service.begin_legacy_drive_exclusive()
    service._stop_event = FakeStopEvent()  # type: ignore[assignment]
    service._background_loop()

    assert len(l2.update_snapshots) == 0
    assert l3.step_calls == 0


def test_sync_l2_respects_explicit_dt_and_background_loop_runs_one_iteration() -> None:
    """Координатор умеет брать явный шаг времени и выполнять проход фонового цикла."""
    service, l1_service, l2_service, l3_service = _service(time_values=(5.0, 8.0))

    service.sync_l2_from_l1(dt_sec=0.25)
    assert l2_service.update_snapshots[0][1] == 0.25

    service._stop_event = FakeStopEvent()  # type: ignore[assignment]
    service._background_loop()

    assert len(l2_service.update_snapshots) == 2
    assert l3_service.step_calls == 1
    assert l2_service.update_snapshots[1][1] == 3.0
    assert l1_service.sensor_state.angular_speed_z_deg_per_sec == 12.0
