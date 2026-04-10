from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from src.application.services.motion_config import MotionConfig


def test_motion_config_is_frozen_slots_dataclass() -> None:
    """MotionConfig хранит настройки движения и запрещает изменение."""
    config: MotionConfig = MotionConfig(
        min_obstacle_distance_cm=20.0,
        deceleration_distance_cm=40.0,
        base_speed_percent=50,
        turn_speed_percent=45,
        turn_slowdown_remaining_deg=15.0,
        turn_creep_speed_percent=20,
        turn_angle_trim_deg=1.0,
        last_turn_angle_trim_deg=2.0,
        max_speed_cm_per_sec=100.0,
        update_interval_sec=0.1,
        avoidance_scan_angle_deg=45.0,
        avoidance_side_step_cm=10.0,
        avoidance_forward_step_cm=15.0,
        avoidance_rejoin_step_cm=20.0,
        avoidance_max_attempts=3,
        avoidance_confirm_readings=5,
        avoidance_min_side_clearance_cm=25.0,
        avoidance_max_lateral_offset_cm=40.0,
        avoidance_max_bypass_distance_cm=100.0,
        heading_hold_enabled=True,
        heading_hold_kp=1.5,
        heading_hold_steer_max=30,
        heading_hold_deadband_deg=2.0,
        heading_hold_steer_speed_ratio=0.5,
        heading_hold_min_speed_percent=10.0,
        heading_hold_steer_cap_min_spd_percent=20.0,
        heading_hold_steer_trim=1,
        heading_hold_invert_steer=False,
        forward_soft_start_sec=0.2,
        turn_check_interval_sec=0.01,
        turn_obstacle_check_interval_sec=0.1,
        turn_timeout_per_deg=0.05,
        turn_timeout_min=1.0,
    )

    assert config.base_speed_percent == 50
    assert config.heading_hold_enabled is True
    with pytest.raises(FrozenInstanceError):
        config.base_speed_percent = 60  # type: ignore[misc]
