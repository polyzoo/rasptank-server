from __future__ import annotations

import pytest

from src.application.services.goal_point_controller import GoalPointController
from src.application.services.l2_models import L2State
from src.application.services.l3_models import TargetPoint


def _controller() -> GoalPointController:
    """Создать контроллер цели с удобными для тестов коэффициентами."""
    return GoalPointController(
        position_tolerance_cm=5.0,
        linear_speed_gain=1.0,
        angular_speed_gain=2.0,
        max_linear_speed_cm_per_sec=20.0,
        max_angular_speed_deg_per_sec=120.0,
        obstacle_stop_distance_cm=20.0,
        obstacle_slowdown_distance_cm=40.0,
    )


def _state(**overrides: float | None) -> L2State:
    """Собрать состояние L2 для теста."""
    return L2State(
        x_cm=overrides.get("x_cm", 0.0) or 0.0,
        y_cm=overrides.get("y_cm", 0.0) or 0.0,
        heading_deg=overrides.get("heading_deg", 0.0) or 0.0,
        linear_speed_cm_per_sec=overrides.get("linear_speed_cm_per_sec", 0.0) or 0.0,
        angular_speed_deg_per_sec=overrides.get("angular_speed_deg_per_sec", 0.0) or 0.0,
        left_percent=overrides.get("left_percent", 0.0) or 0.0,
        right_percent=overrides.get("right_percent", 0.0) or 0.0,
        distance_cm=overrides.get("distance_cm"),
    )


def test_build_command_tracks_target_in_front() -> None:
    """Цель впереди даёт поступательное движение без лишнего поворота."""
    controller = _controller()

    result = controller.build_command(
        state=_state(),
        target=TargetPoint(x_cm=50.0, y_cm=0.0),
    )

    assert result.status == "tracking"
    assert result.distance_error_cm == pytest.approx(50.0)
    assert result.heading_error_deg == pytest.approx(0.0)
    assert result.command.linear_speed_cm_per_sec == pytest.approx(20.0)
    assert result.command.angular_speed_deg_per_sec == pytest.approx(0.0)


def test_build_command_slows_forward_motion_when_heading_error_large() -> None:
    """При большом отклонении по курсу линейная скорость снижается."""
    controller = _controller()

    result = controller.build_command(
        state=_state(heading_deg=90.0),
        target=TargetPoint(x_cm=50.0, y_cm=0.0),
    )

    assert result.status == "tracking"
    assert result.heading_error_deg == pytest.approx(-90.0)
    assert result.command.linear_speed_cm_per_sec == pytest.approx(0.0, abs=1e-6)
    assert result.command.angular_speed_deg_per_sec == pytest.approx(-120.0)


def test_build_command_reports_reached_inside_tolerance() -> None:
    """Цель внутри допуска переводит уровень в состояние reached."""
    controller = _controller()

    result = controller.build_command(
        state=_state(x_cm=9.0, y_cm=9.0),
        target=TargetPoint(x_cm=10.0, y_cm=10.0),
    )

    assert result.status == "reached"
    assert result.command.linear_speed_cm_per_sec == pytest.approx(0.0)
    assert result.command.angular_speed_deg_per_sec == pytest.approx(0.0)


def test_build_command_reports_blocked_when_obstacle_too_close() -> None:
    """Близкое препятствие переводит уровень в состояние blocked."""
    controller = _controller()

    result = controller.build_command(
        state=_state(distance_cm=10.0),
        target=TargetPoint(x_cm=50.0, y_cm=0.0),
    )

    assert result.status == "blocked"
    assert result.command.linear_speed_cm_per_sec == pytest.approx(0.0)
    assert result.command.angular_speed_deg_per_sec == pytest.approx(0.0)


def test_build_command_smoothly_reduces_linear_speed_near_obstacle() -> None:
    """Перед препятствием линейная скорость плавно снижается, а не обрубается сразу."""
    controller = _controller()

    result = controller.build_command(
        state=_state(distance_cm=30.0),
        target=TargetPoint(x_cm=50.0, y_cm=0.0),
    )

    assert result.status == "tracking"
    assert result.command.linear_speed_cm_per_sec == pytest.approx(10.0)


def test_obstacle_speed_ratio_handles_boundary_cases() -> None:
    """Коэффициент торможения покрывает все защитные ветки."""
    controller = _controller()

    assert controller._obstacle_speed_ratio(None) == pytest.approx(1.0)  # type: ignore[attr-defined]
    assert controller._obstacle_speed_ratio(20.0) == pytest.approx(0.0)  # type: ignore[attr-defined]
    assert controller._obstacle_speed_ratio(40.0) == pytest.approx(1.0)  # type: ignore[attr-defined]


def test_obstacle_speed_ratio_returns_zero_when_slowdown_range_invalid() -> None:
    """Если граница плавного торможения не больше границы остановки, скорость гасится в ноль."""
    controller = GoalPointController(
        position_tolerance_cm=5.0,
        linear_speed_gain=1.0,
        angular_speed_gain=2.0,
        max_linear_speed_cm_per_sec=20.0,
        max_angular_speed_deg_per_sec=120.0,
        obstacle_stop_distance_cm=20.0,
        obstacle_slowdown_distance_cm=20.0,
    )

    assert controller._obstacle_speed_ratio(25.0) == pytest.approx(0.0)  # type: ignore[attr-defined]
